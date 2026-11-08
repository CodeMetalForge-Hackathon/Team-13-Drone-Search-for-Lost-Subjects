#!/usr/bin/env python3
"""ROS side of the live pipeline (runs inside the cf_vio/openvins container, Python 3.8 + rospy).

  streamer.py --TCP--> live_node.py --/imu0, /cam0/image_raw--> OpenVINS (run_subscribe_msckf)
                                     --/scan (LaserScan, RPLIDAR C1, frame lidar0)--> lio_node.py (2D LIO)
                                     <--/ov_msckf/poseimu, /ov_msckf/trackhist--
                                     <--/lio/odom, /lio/scan_world--
                       live_node.py --HTTP :8080--> any browser (laptop, phone, projector)

The web page shows, frame by frame: the raw camera, the OpenVINS feature tracks, a top-down map (lidar
submap, current scan, LIO path, VIO path aligned into the LIO frame) and, while both odometries run, the
VIO-vs-LIO error over time (ATE after a rigid 2D alignment + lever arm, see traj_align.py). Without a lidar
it falls back to the VIO-only view. Plain HTTP, so it works from any browser on the same network.

Usage (live/run_live.sh starts it):
  python3 live_node.py --host 127.0.0.1 --port 5600 --http 8080 [--title NAME]
"""
import argparse
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, LaserScan, PointCloud

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traj_align as TA  # noqa: E402

MSG_HDR = struct.Struct("<BI")
IMU_MSG = struct.Struct("<q6f")
CAM_HDR = struct.Struct("<qHH")
SCAN_HDR = struct.Struct("<q7fH")
MAP_CELL = 0.05  # m, viewer map resolution


def stamp(t_ns):
    return rospy.Time(int(t_ns // 1_000_000_000), int(t_ns % 1_000_000_000))


def r3(v):
    return round(float(v), 3)


class Frames:
    """Latest JPEG-able image + a condition for MJPEG streaming (one per stream)."""

    def __init__(self):
        self.cond = threading.Condition()
        self.raw = None   # (bytes, w, h) for lazily encoded raw camera frames
        self.jpeg = None
        self.id = 0

    def put_raw(self, data, w, h):
        with self.cond:
            self.raw, self.jpeg = (data, w, h), None
            self.id += 1
            self.cond.notify_all()

    def put_jpeg(self, jpg):
        with self.cond:
            self.raw, self.jpeg = None, jpg
            self.id += 1
            self.cond.notify_all()

    def wait(self, last, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.id != last, timeout=timeout)
            fid, raw, jpg = self.id, self.raw, self.jpeg
        if jpg is None and raw is not None:  # encode on the HTTP thread, only when someone is watching
            img = np.frombuffer(raw[0], np.uint8).reshape(raw[2], raw[1])
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            jpg = buf.tobytes() if ok else None
        return fid, jpg


class State:
    """Everything the web page shows, shared between ROS callbacks and HTTP threads."""

    def __init__(self, title=""):
        self.lock = threading.Lock()
        self.title = title
        self.rates = {"imu": 0.0, "cam": 0.0, "pose": 0.0, "lidar": 0.0, "lio": 0.0}
        self.counts = {"imu": 0, "cam": 0, "pose": 0, "lidar": 0, "lio": 0}
        self.connected = False
        self.t_first = None  # first sensor stamp [s]
        self.t_last = None
        self.reset()

    def reset(self):
        with self.lock:
            # VIO (OpenVINS poseimu): full rate for the comparison, ~2 cm steps for drawing
            self.vio_t, self.vio_xyz, self.vio_yaw = [], [], []
            self.path, self.travelled, self.last, self.start = [], 0.0, None, None
            # LIO
            self.lio_t, self.lio_xy, self.lio_yaw, self.lio_diag = [], [], [], {}
            self.lio_path = []          # decimated for drawing, append-only
            self.scan = []
            self.map_keys, self.map_cells = set(), []
            self.lio_bad = 0
            self.cmp_json = b'{"ok":false}'
            self.cmp_n = 0
            self.epoch = time.time()    # lets the page notice a reset / new run and drop its incremental data

    def add_pose(self, p, yaw, t):
        with self.lock:
            self.counts["pose"] += 1
            self.vio_t.append(t)
            self.vio_xyz.append(p)
            self.vio_yaw.append(yaw)
            if self.start is None:
                self.start, self.path, self.last = p, [p], p
            step = math.dist(p, self.last)
            if step > 1e-4:
                self.travelled += step
                self.last = p
            if math.dist(p, self.path[-1]) > 0.02:
                self.path.append(p)

    def add_lio(self, t, x, y, yaw, diag):
        with self.lock:
            self.counts["lio"] += 1
            self.lio_t.append(t)
            self.lio_xy.append((x, y))
            self.lio_yaw.append(yaw)
            self.lio_diag = diag
            self.lio_bad += 0 if diag["ok"] else 1
            if not self.lio_path or math.dist((x, y), self.lio_path[-1]) > 0.02:
                self.lio_path.append((r3(x), r3(y)))

    def add_scan_world(self, pts):
        """Current scan for drawing; scans LIO accepted also go into the viewer map (5 cm cells)."""
        with self.lock:
            self.scan = pts
            if not self.lio_diag.get("ok", False) or not len(pts):
                return
            for k in np.unique(np.round(pts / MAP_CELL).astype(np.int64) @ np.array([1, 1 << 20])).tolist():
                if k not in self.map_keys:
                    self.map_keys.add(k)
                    iy = (k + (1 << 19)) >> 20
                    ix = k - (iy << 20)
                    self.map_cells.append((round(ix * MAP_CELL, 2), round(iy * MAP_CELL, 2)))

    def snapshot(self, m0=0, p0=0, epoch=None):
        with self.lock:
            if epoch is not None and abs(epoch - self.epoch) > 1e-6:
                m0 = p0 = 0
            d = {"connected": self.connected, "rates": self.rates, "initialised": self.start is not None,
                 "travelled": self.travelled, "title": self.title, "epoch": self.epoch,
                 "t": (self.t_last - self.t_first) if self.t_first is not None and self.t_last else 0.0}
            if self.start is not None:
                d["from_start"] = math.dist(self.last, self.start)
                d["pos"] = [a - b for a, b in zip(self.last, self.start)]
                d["path"] = [[r3(a - b) for a, b in zip(p, self.start)] for p in self.path[-5000:]]
            if self.lio_t:
                x, y = self.lio_xy[-1]
                d["lio"] = {"pose": [r3(x), r3(y), r3(self.lio_yaw[-1])], "diag": self.lio_diag,
                            "bad": self.lio_bad, "n": len(self.lio_t),
                            "path": self.lio_path[p0:], "p_total": len(self.lio_path),
                            "map": self.map_cells[m0:], "m_total": len(self.map_cells),
                            "scan": np.round(self.scan, 2).tolist() if len(self.scan) else []}
            return d


STATE = State()
CAM = Frames()
TRACKS = Frames()


# ---------------------------------------------------------------- TCP -> ROS
def bridge(host, port):
    pub_imu = rospy.Publisher("/imu0", Imu, queue_size=1000)
    pub_img = rospy.Publisher("/cam0/image_raw", Image, queue_size=5)
    pub_scan = rospy.Publisher("/scan", LaserScan, queue_size=10)
    while not rospy.is_shutdown():
        try:
            s = socket.create_connection((host, port), timeout=2.0)
        except OSError:
            time.sleep(0.5)
            continue
        s.settimeout(None)
        rospy.loginfo("connected to streamer %s:%d", host, port)
        with STATE.lock:
            STATE.connected = True
        f = s.makefile("rb", buffering=1 << 20)
        seq = scan_seq = 0
        try:
            while not rospy.is_shutdown():
                hdr = f.read(MSG_HDR.size)
                if len(hdr) < MSG_HDR.size:
                    break
                kind, n = MSG_HDR.unpack(hdr)
                body = f.read(n)
                if len(body) < n:
                    break
                if kind == ord("I"):
                    t, gx, gy, gz, ax, ay, az = IMU_MSG.unpack(body)
                    m = Imu()
                    m.header.stamp = stamp(t)
                    m.header.frame_id = "imu0"
                    m.orientation.w = 1.0
                    m.orientation_covariance[0] = -1.0
                    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gx, gy, gz
                    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax, ay, az
                    pub_imu.publish(m)
                    with STATE.lock:
                        STATE.counts["imu"] += 1
                        STATE.t_last = t * 1e-9
                        if STATE.t_first is None:
                            STATE.t_first = STATE.t_last
                elif kind == ord("C"):
                    t, w, h = CAM_HDR.unpack_from(body)
                    m = Image()
                    m.header.stamp = stamp(t)
                    m.header.seq = seq
                    m.header.frame_id = "cam0"
                    m.height, m.width, m.encoding, m.step = h, w, "mono8", w
                    m.data = body[CAM_HDR.size:]
                    pub_img.publish(m)
                    CAM.put_raw(body[CAM_HDR.size:], w, h)
                    seq += 1
                    with STATE.lock:
                        STATE.counts["cam"] += 1
                elif kind == ord("L"):
                    t, a0, a1, da, dt, st, rmin, rmax, n = SCAN_HDR.unpack_from(body)
                    arr = np.frombuffer(body, "<f4", 2 * n, SCAN_HDR.size)
                    m = LaserScan()
                    m.header.stamp = stamp(t)
                    m.header.seq = scan_seq
                    m.header.frame_id = "lidar0"
                    m.angle_min, m.angle_max, m.angle_increment = a0, a1, da
                    m.time_increment, m.scan_time, m.range_min, m.range_max = dt, st, rmin, rmax
                    m.ranges, m.intensities = arr[:n].tolist(), arr[n:].tolist()
                    pub_scan.publish(m)
                    scan_seq += 1
                    with STATE.lock:
                        STATE.counts["lidar"] += 1
        except OSError:
            pass
        finally:
            with STATE.lock:
                STATE.connected = False
            s.close()
            rospy.logwarn("streamer disconnected; reconnecting")


# ---------------------------------------------------------------- OpenVINS / LIO -> web state
def on_pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    STATE.add_pose((p.x, p.y, p.z), TA.yaw_from_quat(q.x, q.y, q.z, q.w), msg.header.stamp.to_sec())


def on_lio(msg):
    p, q, c = msg.pose.pose.position, msg.pose.pose.orientation, msg.pose.covariance
    diag = {"rms_cm": round(c[0] * 100, 2), "matches": int(c[7]), "ok": c[14] > 0.5, "degenerate": c[21] > 0.5,
            "ms": round(c[28], 1)}
    STATE.add_lio(msg.header.stamp.to_sec(), p.x, p.y, TA.yaw_from_quat(q.x, q.y, q.z, q.w), diag)


def on_scan_world(msg):
    STATE.add_scan_world(np.array([(p.x, p.y) for p in msg.points], np.float64).reshape(-1, 2))


def on_tracks(msg):
    try:
        img = np.frombuffer(msg.data, np.uint8)
        ch = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(msg.encoding, 3)
        img = img.reshape(msg.height, msg.step // 1)[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]
        if img.shape[1] > 960:   # OpenVINS draws several views side by side; keep the page light
            s = 960 / img.shape[1]
            img = cv2.resize(img, (960, int(img.shape[0] * s)))
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    except Exception as e:  # noqa: BLE001  (never let the viewer kill the node)
        rospy.logwarn_throttle(10, "trackhist decode failed: %s", e)
        return
    if ok:
        TRACKS.put_jpeg(jpg.tobytes())


def compare_loop():
    """Every 0.5 s: align VIO to LIO over everything so far and cache the error series for /cmp."""
    while not rospy.is_shutdown():
        time.sleep(0.5)
        with STATE.lock:
            if len(STATE.lio_t) < 10 or len(STATE.vio_t) < 10 or len(STATE.lio_t) == STATE.cmp_n:
                continue
            lt, lxy, lyaw = list(STATE.lio_t), list(STATE.lio_xy), list(STATE.lio_yaw)
            vt, vxyz, vyaw = list(STATE.vio_t), list(STATE.vio_xyz), list(STATE.vio_yaw)
            t0 = STATE.t_first or lt[0]
            n_now = len(lt)
        try:
            r = TA.compare(lt, lxy, lyaw, vt, vxyz, vyaw)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn_throttle(10, "compare failed: %s", e)
            continue
        if r is None:
            out = {"ok": False}
        else:
            k = max(1, len(r["t"]) // 900)  # keep the series light
            vio_path = TA.transform_vio(r["align"], np.asarray(vxyz)[:, :2])
            kv = max(1, len(vio_path) // 3000)
            yaw_now = vyaw[-1] + r["align"]["theta"] + r["yaw_offset"]
            out = {
                "ok": True,
                "series": {"t": np.round(r["t"][::k] - t0, 2).tolist(),
                           "pos": np.round(r["e_pos"][::k], 3).tolist(),
                           "yaw": np.round(np.degrees(r["e_yaw"][::k]), 2).tolist(),
                           "dz": np.round(r["dz"][::k], 3).tolist()},
                "vio_path": np.round(vio_path[::kv], 3).tolist(),
                "vio_pose": [r3(vio_path[-1][0]), r3(vio_path[-1][1]), r3(yaw_now)],
                "stats": {k2: (round(v, 4) if isinstance(v, float) else v) for k2, v in r.items()
                          if isinstance(v, (int, float))},
                "lever": np.round(r["align"]["l"], 3).tolist(), "scale": round(r["align"]["scale"], 4),
                "theta_deg": round(math.degrees(r["align"]["theta"]), 2),
                "cur": {"pos": r3(r["e_pos"][-1]), "yaw": round(math.degrees(r["e_yaw"][-1]), 2),
                        "dz": r3(r["dz"][-1])},
            }
        with STATE.lock:
            STATE.cmp_json = json.dumps(out).encode()
            STATE.cmp_n = n_now


def rate_loop():
    prev, t_prev = dict(STATE.counts), time.monotonic()
    while not rospy.is_shutdown():
        time.sleep(1.0)
        now = time.monotonic()
        with STATE.lock:
            for k in STATE.counts:
                STATE.rates[k] = (STATE.counts[k] - prev[k]) / (now - t_prev)
            prev = dict(STATE.counts)
        t_prev = now


# ---------------------------------------------------------------- HTTP viewer
PAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "viewer.html")


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _mjpeg(self, src):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = -1
        try:
            while not rospy.is_shutdown():
                last, jpg = src.wait(last)
                if jpg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            with open(PAGE_PATH, "rb") as f:  # read per request: edit the page without restarting
                self._send(f.read(), "text/html; charset=utf-8")
        elif u.path == "/state":
            q = parse_qs(u.query)
            m0 = int(q.get("m", ["0"])[0])
            p0 = int(q.get("p", ["0"])[0])
            ep = float(q["e"][0]) if "e" in q else None
            self._send(json.dumps(STATE.snapshot(m0, p0, ep)).encode(), "application/json")
        elif u.path == "/cmp":
            with STATE.lock:
                body = STATE.cmp_json
            self._send(body, "application/json")
        elif u.path.startswith("/camera.mjpg"):
            self._mjpeg(CAM)
        elif u.path.startswith("/tracks.mjpg"):
            self._mjpeg(TRACKS)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/reset":
            STATE.reset()
            self.send_response(204)
            self.end_headers()
        else:
            self.send_error(404)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1", help="streamer address")
    ap.add_argument("--port", type=int, default=5600)
    ap.add_argument("--http", type=int, default=8080, help="web viewer port (0 = off)")
    ap.add_argument("--title", default="", help="shown in the page header (e.g. the recording name)")
    args = ap.parse_args(rospy.myargv()[1:])
    STATE.title = args.title

    rospy.init_node("cf_vio_live", disable_signals=False)
    rospy.Subscriber("/ov_msckf/poseimu", PoseWithCovarianceStamped, on_pose, queue_size=50)
    rospy.Subscriber("/ov_msckf/trackhist", Image, on_tracks, queue_size=1, buff_size=1 << 24)
    rospy.Subscriber("/lio/odom", Odometry, on_lio, queue_size=50)
    rospy.Subscriber("/lio/scan_world", PointCloud, on_scan_world, queue_size=5)
    threading.Thread(target=bridge, args=(args.host, args.port), daemon=True).start()
    threading.Thread(target=rate_loop, daemon=True).start()
    threading.Thread(target=compare_loop, daemon=True).start()
    if args.http:
        srv = ThreadingHTTPServer(("0.0.0.0", args.http), Handler)
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        rospy.loginfo("viewer on http://<this machine>:%d/", args.http)
    rospy.spin()


if __name__ == "__main__":
    main()
