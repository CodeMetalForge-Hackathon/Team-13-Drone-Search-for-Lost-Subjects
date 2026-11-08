#!/usr/bin/env python3
"""ROS side of the live pipeline (runs inside the cf_vio/openvins container, Python 3.8 + rospy).

  streamer.py --TCP--> live_node.py --/imu0, /cam0/image_raw--> OpenVINS (run_subscribe_msckf)
                                     <--/ov_msckf/poseimu, /ov_msckf/trackhist--
                       live_node.py --HTTP :8080--> any browser (laptop, phone, projector)

The web page shows the OpenVINS feature-tracking image and the trajectory from above, updated live,
with distance travelled and distance from the start. No ROS networking between machines is needed:
the viewer is plain HTTP, so it works from Windows, a phone, or anything on the same network.

Usage (live/run_live.sh starts it):
  python3 live_node.py --host 127.0.0.1 --port 5600 --http 8080
"""
import argparse
import json
import math
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, Imu

MSG_HDR = struct.Struct("<BI")
IMU_MSG = struct.Struct("<q6f")
CAM_HDR = struct.Struct("<qHH")


def stamp(t_ns):
    return rospy.Time(int(t_ns // 1_000_000_000), int(t_ns % 1_000_000_000))


class State:
    """Everything the web page shows, shared between ROS callbacks and HTTP threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.path = []            # [x, y, z] decimated to ~10 cm steps for drawing
        self.travelled = 0.0
        self.last = None
        self.start = None
        self.pose_t = None
        self.jpeg = None
        self.jpeg_id = 0
        self.rates = {"imu": 0.0, "cam": 0.0, "pose": 0.0}
        self.counts = {"imu": 0, "cam": 0, "pose": 0}
        self.connected = False
        self.cond = threading.Condition(self.lock)

    def add_pose(self, p, t):
        with self.lock:
            self.counts["pose"] += 1
            if self.start is None:
                self.start = p
                self.path = [p]
                self.last = p
            step = math.dist(p, self.last)
            if step > 1e-4:
                self.travelled += step
                self.last = p
            if math.dist(p, self.path[-1]) > 0.02:
                self.path.append(p)
            self.pose_t = t

    def reset(self):
        with self.lock:
            self.path, self.travelled, self.last, self.start = [], 0.0, None, None

    def snapshot(self):
        with self.lock:
            d = {"connected": self.connected, "rates": self.rates, "initialised": self.start is not None,
                 "travelled": self.travelled, "path": self.path[-5000:]}
            if self.start is not None and self.last is not None:
                d["from_start"] = math.dist(self.last, self.start)
                d["pos"] = [a - b for a, b in zip(self.last, self.start)]
            d["path"] = [[a - b for a, b in zip(p, self.start)] for p in d["path"]] if self.start else []
            return d


STATE = State()


# ---------------------------------------------------------------- TCP -> ROS
def bridge(host, port):
    pub_imu = rospy.Publisher("/imu0", Imu, queue_size=1000)
    pub_img = rospy.Publisher("/cam0/image_raw", Image, queue_size=5)
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
        seq = 0
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
                elif kind == ord("C"):
                    t, w, h = CAM_HDR.unpack_from(body)
                    m = Image()
                    m.header.stamp = stamp(t)
                    m.header.seq = seq
                    m.header.frame_id = "cam0"
                    m.height, m.width, m.encoding, m.step = h, w, "mono8", w
                    m.data = body[CAM_HDR.size:]
                    pub_img.publish(m)
                    seq += 1
                    with STATE.lock:
                        STATE.counts["cam"] += 1
        except OSError:
            pass
        finally:
            with STATE.lock:
                STATE.connected = False
            s.close()
            rospy.logwarn("streamer disconnected; reconnecting")


# ---------------------------------------------------------------- OpenVINS -> web state
def on_pose(msg):
    p = msg.pose.pose.position
    STATE.add_pose((p.x, p.y, p.z), msg.header.stamp.to_sec())


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
        with STATE.cond:
            STATE.jpeg = jpg.tobytes()
            STATE.jpeg_id += 1
            STATE.cond.notify_all()


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
PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Live VIO</title>
<style>
 :root{--bg:#0f1115;--fg:#e8eaed;--mut:#9aa0a6;--acc:#4c8dff;--ok:#2fbf71;--bad:#ff5d5d;--card:#181b22}
 body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,sans-serif}
 header{display:flex;gap:16px;align-items:baseline;padding:12px 16px;flex-wrap:wrap}
 h1{font-size:18px;margin:0}.st{color:var(--mut)}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}
 main{display:grid;grid-template-columns:minmax(0,1.2fr) minmax(0,1fr);gap:12px;padding:0 16px 16px}
 @media(max-width:820px){main{grid-template-columns:1fr}}
 .card{background:var(--card);border-radius:10px;padding:10px;min-width:0}
 img{width:100%;border-radius:6px;background:#000;min-height:120px}
 canvas{width:100%;aspect-ratio:1;display:block}
 .nums{display:flex;gap:24px;margin:6px 4px 0;flex-wrap:wrap}.big{font-size:28px;font-weight:600}
 .lbl{color:var(--mut);font-size:12px;text-transform:uppercase;letter-spacing:.04em}
 button{background:#2a2f3a;color:var(--fg);border:0;border-radius:6px;padding:6px 12px;cursor:pointer}
</style></head><body>
<header><h1>Live visual-inertial odometry</h1><span class="st" id="st">connecting...</span>
<button onclick="fetch('/reset',{method:'POST'})">Reset path</button></header>
<main>
 <div class="card"><img src="/tracks.mjpg" alt="feature tracks"><div class="st" style="margin:6px 4px 0">
 Tracked features (OpenVINS)</div></div>
 <div class="card"><canvas id="c" width="800" height="800"></canvas>
  <div class="nums"><div><div class="lbl">Travelled</div><div class="big" id="trav">-</div></div>
  <div><div class="lbl">From start</div><div class="big" id="from">-</div></div>
  <div><div class="lbl">Height</div><div class="big" id="ht">-</div></div></div></div>
</main>
<script>
const c=document.getElementById('c'),g=c.getContext('2d');
function fmt(m){return m<10?m.toFixed(2)+' m':m.toFixed(1)+' m'}
function nice(x){const p=Math.pow(10,Math.floor(Math.log10(x)));for(const k of [1,2,5,10])if(k*p>=x)return k*p}
let span=1;
function draw(s){
 const W=c.width,H=c.height;g.fillStyle='#181b22';g.fillRect(0,0,W,H);
 const P=s.path;let mx=1;for(const p of P)mx=Math.max(mx,Math.abs(p[0]),Math.abs(p[1]));
 span=Math.max(span*0.98,mx*1.15,1);const sc=(W/2-30)/span,cx=W/2,cy=H/2;
 const grid=nice(span/4);g.strokeStyle='#262b35';g.lineWidth=1;
 for(let v=-Math.ceil(span/grid)*grid;v<=span;v+=grid){g.beginPath();g.moveTo(cx+v*sc,0);g.lineTo(cx+v*sc,H);g.stroke();
  g.beginPath();g.moveTo(0,cy-v*sc);g.lineTo(W,cy-v*sc);g.stroke();}
 g.fillStyle='#9aa0a6';g.font='22px system-ui';g.fillText('grid '+(grid<1?grid*100+' cm':grid+' m'),16,H-16);
 if(P.length){g.strokeStyle='#4c8dff';g.lineWidth=4;g.lineJoin='round';g.beginPath();
  P.forEach((p,i)=>{const x=cx+p[0]*sc,y=cy-p[1]*sc;i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke();
  g.fillStyle='#2fbf71';g.beginPath();g.arc(cx,cy,9,0,7);g.fill();
  const q=P[P.length-1];g.fillStyle='#ff5d5d';g.beginPath();g.arc(cx+q[0]*sc,cy-q[1]*sc,9,0,7);g.fill();}
}
async function tick(){
 try{const s=await (await fetch('/state')).json();
  const r=s.rates;const ok=s.connected&&r.imu>100&&r.cam>5;
  document.getElementById('st').innerHTML=`<span class="dot" style="background:${ok?'#2fbf71':'#ff5d5d'}"></span>`+
   (s.connected?`IMU ${r.imu.toFixed(0)} Hz · camera ${r.cam.toFixed(0)} fps · VIO ${r.pose.toFixed(0)} Hz`:'waiting for sensors')+
   (s.connected&&!s.initialised?' · hold still, then move to start':'');
  document.getElementById('trav').textContent=s.initialised?fmt(s.travelled):'-';
  document.getElementById('from').textContent=s.initialised?fmt(s.from_start):'-';
  document.getElementById('ht').textContent=s.initialised?(s.pos[2]>=0?'+':'')+s.pos[2].toFixed(2)+' m':'-';
  draw(s);}catch(e){document.getElementById('st').textContent='viewer offline';}
 setTimeout(tick,200);}
tick();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/state":
            body = json.dumps(STATE.snapshot()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/tracks.mjpg"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            try:
                while not rospy.is_shutdown():
                    with STATE.cond:
                        STATE.cond.wait_for(lambda: STATE.jpeg_id != last, timeout=2.0)
                        jpg, last = STATE.jpeg, STATE.jpeg_id
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
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
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("cf_vio_live", disable_signals=False)
    rospy.Subscriber("/ov_msckf/poseimu", PoseWithCovarianceStamped, on_pose, queue_size=50)
    rospy.Subscriber("/ov_msckf/trackhist", Image, on_tracks, queue_size=1, buff_size=1 << 24)
    threading.Thread(target=bridge, args=(args.host, args.port), daemon=True).start()
    threading.Thread(target=rate_loop, daemon=True).start()
    if args.http:
        srv = ThreadingHTTPServer(("0.0.0.0", args.http), Handler)
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        rospy.loginfo("viewer on http://<this machine>:%d/", args.http)
    rospy.spin()


if __name__ == "__main__":
    main()
