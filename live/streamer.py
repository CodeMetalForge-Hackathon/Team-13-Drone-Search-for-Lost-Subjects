#!/usr/bin/env python3
"""Live sensor streamer: camera + Crazyflie IMU + RPLIDAR C1 -> OpenVINS (via live_node.py in Docker).

Runs on the host (Pi 5 with the camera and the Crazyflie UART), or on the laptop in --replay mode,
which plays a recording from pi/recorder.py back in real time so the whole live path can be tested
without hardware.

What it does
  * reads the camera (SensorTimestamp, CLOCK_BOOTTIME ns) and the raw IMU UART stream
  * parses IMU packets and maps the Crazyflie clock onto the Pi clock with a running lower-envelope
    fit (the live version of offline/convert.py fit_clock), so camera and IMU share one clock
  * reads the RPLIDAR C1 (if connected): samples come at a fixed rate, so the sample index is the lidar's
    clock and is mapped onto the Pi clock with the same running lower-envelope fit; each revolution is
    sent as one LaserScan (pi/lidar.py bin_scan, same as offline/convert.py)
  * like pi/recorder.py, waits for the lidar motor to settle, then starts every stream at one common
    time (t_start_ns); nothing before it is streamed or recorded
  * serves everything over TCP (default 127.0.0.1:5600) to live_node.py, which publishes ROS topics
  * also records the session in the recorder.py format (recordings/<stamp>/), so any live run can be
    replayed or reprocessed with ./demo.sh afterwards

Wire format (little endian), one message after another:
  u8 type, u32 payload length, payload
  type 'I': i64 t_ns, f32 gyro[3] rad/s, f32 acc[3] m/s^2
  type 'C': i64 t_ns, u16 width, u16 height, width*height bytes (mono8)
  type 'L': i64 t_ns (first sample), f32 angle_min, angle_max, angle_increment, time_increment, scan_time,
            range_min, range_max, u16 n, f32 ranges[n], f32 intensities[n]   (sensor_msgs/LaserScan)

Usage
  python3 live/streamer.py                                  # Pi: camera + GPIO UART (auto: /dev/serial0, or /dev/ttyAMA0 on a Pi 5)
  python3 live/streamer.py --port /dev/ttyAMA0              # Pi 5 GPIO UART, see README
  python3 live/streamer.py --replay recordings/20261107_233223   # laptop test, no hardware
  python3 live/streamer.py --no-lidar                       # camera + IMU only
"""
import argparse
import collections
import csv
import json
import math
import os
import queue
import signal
import socket
import struct
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "pi"))
import imu_check  # noqa: E402  (packet parser shared with the Pi tools)
import lidar  # noqa: E402  (RPLIDAR protocol, spin-up detector, scan binning)

G = 9.80665
DEG2RAD = math.pi / 180.0
MSG_HDR = struct.Struct("<BI")
IMU_MSG = struct.Struct("<q6f")
CAM_HDR = struct.Struct("<qHH")
CHUNK_HDR = struct.Struct("<QI")
SCAN_HDR = struct.Struct("<q7fH")
LIDAR_NOMINAL_HZ = 5100.0  # C1 sample rate; only a starting point, the clock fit estimates the real one


def now_ns():
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


# ---------------------------------------------------------------- clock sync
class OnlineClock:
    """Map Crazyflie microseconds to Pi ns: lower envelope of (arrival - cf time), refit as data comes in.

    Each `window_s` window keeps the packet with the smallest arrival delay; a line through the last
    `history` window minima (with late outliers dropped) gives offset + drift. Output times are forced to
    increase so OpenVINS never sees IMU going backwards when the fit is updated.
    """

    def __init__(self, window_s=1.0, history=60, min_windows=3):
        self.window_s, self.history, self.min_windows = window_s, history, min_windows
        self.t0_us = self.a0_ns = None
        self.us_hi = 0                # unwrap of the 32-bit microsecond counter
        self.last_us = None
        self.points = collections.deque(maxlen=history)   # (x, r) per finished window
        self.cur_win, self.cur_min = None, None
        self.a, self.b = 0.0, 0.0
        self.last_out = 0

    @property
    def ready(self):
        return len(self.points) >= self.min_windows

    def unwrap(self, t_us):
        if self.last_us is not None and t_us < self.last_us - (1 << 31):
            self.us_hi += 1 << 32
        self.last_us = t_us
        return t_us + self.us_hi

    def add(self, t_us, arr_ns):
        t_us = self.unwrap(t_us)
        if self.t0_us is None:
            self.t0_us, self.a0_ns = t_us, arr_ns
        x = (t_us - self.t0_us) * 1e-6
        r = (arr_ns - self.a0_ns) * 1e-9 - x
        w = int(x // self.window_s)
        if w != self.cur_win:
            if self.cur_min is not None:
                self.points.append(self.cur_min)
                self._fit()
            self.cur_win, self.cur_min = w, (x, r)
        elif r < self.cur_min[1]:
            self.cur_min = (x, r)
        return t_us

    def _fit(self):
        pts = list(self.points)
        if len(pts) == 1:
            self.a, self.b = pts[0][1], 0.0
            return
        keep = pts
        for _ in range(3):
            n = len(keep)
            sx = sum(p[0] for p in keep); sy = sum(p[1] for p in keep)
            sxx = sum(p[0] ** 2 for p in keep); sxy = sum(p[0] * p[1] for p in keep)
            den = n * sxx - sx * sx
            if den <= 0:
                return
            b = (n * sxy - sx * sy) / den
            a = (sy - b * sx) / n
            e = sorted(p[1] - (a + b * p[0]) for p in keep)
            med = e[len(e) // 2]
            mad = sorted(abs(v - med) for v in e)[len(e) // 2] + 1e-6
            new = [p for p in pts if p[1] - (a + b * p[0]) < med + 4 * 1.4826 * mad]
            if len(new) < 2:
                break
            keep = new
        self.a, self.b = a, b

    def to_pi_ns(self, t_us_unwrapped):
        x = (t_us_unwrapped - self.t0_us) * 1e-6
        t = self.a0_ns + int(round((x + self.a + self.b * x) * 1e9))
        t = max(t, self.last_out + 1000)        # strictly increasing, at least 1 us apart
        self.last_out = t
        return t

    def info(self):
        return f"drift {self.b * 1e6:+.0f} ppm over {len(self.points)} s"


class LidarTimer:
    """Raw lidar bytes -> timed samples -> one LaserScan dict per complete revolution.

    The C1 sends no timestamps but samples at a fixed rate, so the running sample index is its clock
    (like the Crazyflie's microsecond counter) and OnlineClock maps it onto the Pi clock. A glitch in the
    byte stream (resync) loses an unknown number of samples, so the index clock restarts there."""

    def __init__(self):
        self.buf = bytearray()
        self.resyncs = 0
        self._reset()

    def _reset(self):
        self.clock = OnlineClock()
        self.k = 0
        self.rev = None  # samples of the current revolution; None until the first start flag

    def add(self, arr_ns, data):
        self.buf += data
        samples, bad = lidar.parse_scan(self.buf)
        if bad:
            self.resyncs += 1
            self._reset()
        out = []
        for start, q, ang, mm in samples:
            tu = self.clock.add(int(round(self.k * 1e6 / LIDAR_NOMINAL_HZ)), arr_ns)
            self.k += 1
            if start:
                if self.rev and len(self.rev) > 100:
                    t, a, r, qq = zip(*self.rev)
                    out.append(lidar.bin_scan(a, r, qq, t))
                self.rev = [] if self.clock.ready else None
            if self.rev is not None:
                self.rev.append((self.clock.to_pi_ns(tu), ang, mm / 1000.0, q))
        return out


# ---------------------------------------------------------------- TCP output
class Sender:
    """One client at a time. IMU is never dropped; camera frames are dropped if the client falls behind."""

    def __init__(self, bind, port, max_frames=3):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((bind, port))
        self.srv.listen(1)
        self.q = collections.deque()
        self.frames_queued = 0
        self.max_frames = max_frames
        self.cv = threading.Condition()
        self.conn = None
        self.dropped_frames = 0
        self.sent_imu = self.sent_frames = self.sent_scans = 0
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._send, daemon=True).start()

    @property
    def connected(self):
        return self.conn is not None

    def _accept(self):
        while True:
            c, addr = self.srv.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.cv:
                if self.conn is not None:
                    c.close()
                    continue
                self.q.clear()
                self.frames_queued = 0
                self.conn = c
            print(f"[streamer] live_node connected from {addr[0]}")

    def put_imu(self, t_ns, gyro, acc):
        if self.conn is None:
            return
        body = IMU_MSG.pack(t_ns, *gyro, *acc)
        with self.cv:
            self.q.append((False, MSG_HDR.pack(ord("I"), len(body)) + body))
            self.cv.notify()

    def put_scan(self, f):
        if self.conn is None:
            return
        n = len(f["ranges"])
        body = (SCAN_HDR.pack(f["stamp_ns"], f["angle_min"], f["angle_max"], f["angle_increment"],
                              f["time_increment"], f["scan_time"], f["range_min"], f["range_max"], n)
                + f["ranges"].astype("<f4").tobytes() + f["intensities"].astype("<f4").tobytes())
        with self.cv:
            self.q.append((False, MSG_HDR.pack(ord("L"), len(body)) + body))
            self.cv.notify()

    def put_frame(self, t_ns, img_bytes, w, h):
        if self.conn is None:
            return
        with self.cv:
            if self.frames_queued >= self.max_frames:
                self.dropped_frames += 1
                return
            body = CAM_HDR.pack(t_ns, w, h) + img_bytes
            self.q.append((True, MSG_HDR.pack(ord("C"), len(body)) + body))
            self.frames_queued += 1
            self.cv.notify()

    def _send(self):
        while True:
            with self.cv:
                while not self.q:
                    self.cv.wait()
                is_frame, msg = self.q.popleft()
                if is_frame:
                    self.frames_queued -= 1
                c = self.conn
            if c is None:
                continue
            try:
                c.sendall(msg)
                if is_frame:
                    self.sent_frames += 1
                elif msg[0] == ord("L"):
                    self.sent_scans += 1
                else:
                    self.sent_imu += 1
            except OSError:
                print("[streamer] live_node disconnected")
                with self.cv:
                    self.conn = None
                    self.q.clear()
                    self.frames_queued = 0
                try:
                    c.close()
                except OSError:
                    pass


# ---------------------------------------------------------------- recording (recorder.py format)
class Recorder:
    def __init__(self, out_dir, meta):
        os.makedirs(out_dir)
        self.dir = out_dir
        self.meta = meta
        self.write_meta()
        self.imu = open(os.path.join(out_dir, "imu.bin"), "wb", buffering=1 << 20)
        self.lidar = (open(os.path.join(out_dir, "lidar.bin"), "wb", buffering=1 << 20)
                      if meta.get("lidar_port") else None)
        self.fbin = open(os.path.join(out_dir, "frames.bin"), "wb", buffering=1 << 22)
        self.fcsv = open(os.path.join(out_dir, "frames.csv"), "w", newline="")
        self.csv = csv.writer(self.fcsv)
        self.csv.writerow(["idx", "sensor_ts_ns", "recv_ns", "exposure_us",
                           "analogue_gain", "frame_duration_us", "lens_position"])
        self.q = queue.Queue(maxsize=90)
        self.dropped = 0
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def write_meta(self):
        with open(os.path.join(self.dir, "meta.json"), "w") as f:
            json.dump(self.meta, f, indent=2)

    def imu_chunk(self, arr_ns, data):
        self.q.put(("i", arr_ns, data))

    def lidar_chunk(self, arr_ns, data):
        if self.lidar:
            self.q.put(("l", arr_ns, data))

    def frame(self, img_bytes, row):
        try:
            self.q.put_nowait(("c", img_bytes, row))
        except queue.Full:
            self.dropped += 1

    def _run(self):
        while True:
            item = self.q.get()
            if item is None:
                break
            if item[0] in "il":
                f = self.imu if item[0] == "i" else self.lidar
                f.write(CHUNK_HDR.pack(item[1], len(item[2])))
                f.write(item[2])
            else:
                self.fbin.write(item[1])
                self.csv.writerow(item[2])
        for f in (self.imu, self.lidar, self.fbin, self.fcsv):
            if f:
                f.close()

    def close(self):
        self.q.put(None)
        self.t.join(timeout=5)
        self.write_meta()


# ---------------------------------------------------------------- sources
def pi_source(args, events, stop):
    """Camera + UART + lidar on the Pi. Pushes ('meta', dict), ('start', t_ns), ('imu', arr_ns, bytes),
    ('lidar', arr_ns, bytes) and ('cam', sensor_ns, recv_ns, bytes, row). main() drops data before 'start'."""
    import serial
    from picamera2 import Picamera2
    from libcamera import Transform
    try:
        from libcamera import controls as lc_controls
    except ImportError:
        lc_controls = None

    lid_ser = lid_info = None
    if not args.no_lidar:
        lid_ser = serial.Serial(args.lidar_port, lidar.BAUD, timeout=0.002)
        lid_ser.dtr = False
        lid_info = lidar.info(lid_ser)
        if not lid_info:
            events.put(("error", f"no answer from lidar on {args.lidar_port} (override: --no-lidar)"))
            return
        lid_info["health"] = lidar.health(lid_ser)
        print(f"[streamer] lidar: {lid_info}")
    lid_ready = threading.Event()

    ser = serial.Serial(args.port, args.baud, timeout=0.002)
    ser.reset_input_buffer()

    def uart():
        while not stop.is_set():
            data = ser.read(max(1, ser.in_waiting))
            if data:
                events.put(("imu", now_ns(), data))
        ser.close()
    threading.Thread(target=uart, daemon=True).start()

    def lidar_reader():
        """Spin-up: decode and wait for steady revolutions, then announce 'start' and forward raw bytes."""
        t_cmd = now_ns()
        pending = bytearray(lidar.start_scan(lid_ser))
        spin = lidar.SpinupDetector()
        while not stop.is_set():
            data = lid_ser.read(max(1, lid_ser.in_waiting))
            if not data:
                continue
            t = now_ns()
            if not lid_ready.is_set():
                pending += data
                samples, _ = lidar.parse_scan(pending)  # keeps only a partial sample in `pending`
                if spin.feed(t, samples):
                    print(f"[streamer] lidar steady at ~{spin.rev_hz:.1f} rev/s after {(t - t_cmd) / 1e9:.1f} s")
                    events.put(("start", t, (t - t_cmd) / 1e9))
                    lid_ready.set()
                    if pending:  # starts on a sample boundary
                        events.put(("lidar", t, bytes(pending)))
                continue
            events.put(("lidar", t, data))
        lidar.stop(lid_ser)
        lid_ser.close()
    if lid_ser:
        threading.Thread(target=lidar_reader, daemon=True).start()

    w, h = args.width, args.height
    sw, sh = (int(v) for v in args.sensor_size.split("x"))
    frame_us = int(round(1e6 / args.fps))
    cam = Picamera2()
    ctrl = {"AeEnable": False, "AwbEnable": False, "ExposureTime": args.exposure_us,
            "AnalogueGain": args.gain, "FrameDurationLimits": (frame_us, frame_us)}
    if "AfMode" in cam.camera_controls and lc_controls is not None:
        ctrl["AfMode"] = lc_controls.AfModeEnum.Manual
        ctrl["LensPosition"] = args.lens_position
    cam.configure(cam.create_video_configuration(
        main={"size": (w, h), "format": "YUV420"}, raw={"size": (sw, sh)}, buffer_count=6,
        controls=ctrl, transform=Transform(hflip=args.rotate == 180, vflip=args.rotate == 180)))
    meta = {"width": w, "height": h, "fps": args.fps, "exposure_us": args.exposure_us, "gain": args.gain,
            "lens_position": args.lens_position, "rotate": args.rotate, "sensor_size": [sw, sh],
            "camera": cam.camera_properties.get("Model"), "imu_port": args.port, "imu_baud": args.baud,
            "clock": "CLOCK_BOOTTIME", "source": "live/streamer.py", "t_start_ns": None, "t_end_ns": None,
            "lidar_port": args.lidar_port if lid_ser else None, "lidar_baud": lidar.BAUD, "lidar_info": lid_info,
            "lidar_format": "RPLIDAR standard scan, 5 B/sample: S|!S|quality6, C|angle_q6[6:0], angle_q6[14:7], dist_q2 u16",
            "imu_packet": "A5 5A | u16 seq | u32 t_us | f32 gyro[3] deg/s | f32 acc[3] g | u16 crc"}
    events.put(("meta", meta))
    cam.start()
    if lid_ser:
        print("[streamer] waiting for the lidar to spin up ...")
        t_wait = time.monotonic()
    else:
        events.put(("start", now_ns(), None))
    n = 0
    while not stop.is_set():
        if lid_ser and not lid_ready.is_set() and time.monotonic() - t_wait > args.lidar_timeout:
            events.put(("error", f"lidar did not reach steady rotation within {args.lidar_timeout:.0f} s"))
            break
        req = cam.capture_request()
        try:
            recv = now_ns()
            md = req.get_metadata()
            y = req.make_array("main")[:h, :w].tobytes()
        finally:
            req.release()
        if lid_ser and not lid_ready.is_set():
            continue  # keep the camera pipeline draining during lidar spin-up
        row = [n, md.get("SensorTimestamp", 0), recv, md.get("ExposureTime", 0),
               f"{md.get('AnalogueGain', 0):.3f}", md.get("FrameDuration", 0), f"{md.get('LensPosition', -1):.3f}"]
        events.put(("cam", md.get("SensorTimestamp", 0), recv, y, row))
        n += 1
    cam.stop()
    events.put(("end",))


def replay_source(args, events, stop):
    """Play a recording back in real time (by arrival times), exactly as the Pi would have produced it."""
    rec = args.replay
    with open(os.path.join(rec, "meta.json")) as f:
        meta = json.load(f)
    w, h = meta["width"], meta["height"]

    def read_chunks(name):
        path = os.path.join(rec, name)
        if not os.path.exists(path):
            return []
        raw = open(path, "rb").read()
        out, p = [], 0
        while p + CHUNK_HDR.size <= len(raw):
            t, n = CHUNK_HDR.unpack_from(raw, p)
            p += CHUNK_HDR.size
            if p + n > len(raw):
                break
            out.append((t, raw[p:p + n]))
            p += n
        return out
    chunks = read_chunks("imu.bin")
    lchunks = [] if args.no_lidar else read_chunks("lidar.bin")
    with open(os.path.join(rec, "frames.csv")) as f:
        rows = list(csv.reader(f))[1:]
    fbin = open(os.path.join(rec, "frames.bin"), "rb")
    nfr = min(len(rows), os.path.getsize(os.path.join(rec, "frames.bin")) // (w * h))
    evs = ([(t, 0, i) for i, (t, _) in enumerate(chunks)] + [(int(rows[k][2]), 1, k) for k in range(nfr)]
           + [(t, 2, i) for i, (t, _) in enumerate(lchunks)])
    evs.sort()
    events.put(("meta", dict(meta, source=f"replay of {rec}")))
    # recordings made with the synchronised start contain nothing before t_start_ns; older ones start at
    # their first event
    events.put(("start", meta.get("t_start_ns") or evs[0][0], meta.get("lidar_spinup_s")))
    t_rec0, t_wall0 = evs[0][0], time.monotonic()
    for t, kind, i in evs:
        if stop.is_set():
            break
        delay = (t - t_rec0) * 1e-9 / args.rate - (time.monotonic() - t_wall0)
        if delay > 0:
            time.sleep(delay)
        if kind == 0:
            events.put(("imu", t, chunks[i][1]))
        elif kind == 2:
            events.put(("lidar", t, lchunks[i][1]))
        else:
            fbin.seek(i * w * h)
            events.put(("cam", int(rows[i][1]), int(rows[i][2]), fbin.read(w * h), rows[i]))
    events.put(("end",))


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--replay", help="recording folder to play back instead of the live camera/UART")
    ap.add_argument("--rate", type=float, default=1.0, help="replay speed factor")
    ap.add_argument("--bind", default="127.0.0.1", help="address for live_node to connect to")
    ap.add_argument("--tcp-port", type=int, default=5600)
    ap.add_argument("--out", default="recordings", help="where live sessions are recorded")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--wait-client", type=float, default=60.0,
                    help="seconds to wait for live_node before starting (replay); 0 = don't wait")
    # Pi camera / UART (same defaults as pi/recorder.py, so the Kalibr calibration still applies)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--exposure-us", type=int, default=3000)
    ap.add_argument("--gain", type=float, default=4.0)
    ap.add_argument("--lens-position", type=float, default=0.5)
    ap.add_argument("--sensor-size", default="2304x1296")
    ap.add_argument("--rotate", type=int, choices=(0, 180), default=180)
    ap.add_argument("--port", default=imu_check.default_port(), help="GPIO UART (auto: ttyAMA0 on a Pi 5)")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--skip-imu-check", action="store_true")
    ap.add_argument("--lidar-port", default=lidar.default_port(), help="RPLIDAR C1 USB serial port")
    ap.add_argument("--no-lidar", action="store_true", help="run without the lidar (live or replay)")
    ap.add_argument("--lidar-timeout", type=float, default=10.0,
                    help="seconds to wait for steady lidar rotation before giving up")
    args = ap.parse_args()

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    if not args.replay and not args.skip_imu_check:
        import serial
        with serial.Serial(args.port, args.baud, timeout=0.01) as s:
            v, pkts, bad = imu_check.check(s, 1.0)
        print(f"[streamer] IMU check: {v} ({len(pkts)} packets/s)")
        if v != "OK":
            raise SystemExit("[streamer] no usable IMU data; run pi/imu_check.py (override: --skip-imu-check)")

    sender = Sender(args.bind, args.tcp_port)
    print(f"[streamer] serving on {args.bind}:{args.tcp_port}")
    if args.replay and args.wait_client > 0:
        t_end = time.monotonic() + args.wait_client
        print("[streamer] waiting for live_node to connect ...")
        while not sender.connected and time.monotonic() < t_end and not stop.is_set():
            time.sleep(0.1)

    events = queue.Queue()
    src = replay_source if args.replay else pi_source
    threading.Thread(target=src, args=(args, events, stop), daemon=True).start()

    clock = OnlineClock()
    frame_wh = [args.width, args.height]
    buf = bytearray()
    recorder = None
    n = {"imu": 0, "cam": 0, "scan": 0}
    t_start = None
    pre_start = collections.deque(maxlen=5000)  # data that raced ahead of the 'start' event
    lid = LidarTimer()
    announced = False

    def handle(ev):
        nonlocal announced
        kind = ev[0]
        if kind == "lidar":
            _, arr, data = ev
            if recorder:
                recorder.lidar_chunk(arr, data)
            for f in lid.add(arr, data):
                sender.put_scan(f)
                n["scan"] += 1
        elif kind == "imu":
            _, arr, data = ev
            if recorder:
                recorder.imu_chunk(arr, data)
            buf.extend(data)
            pkts, _ = imu_check.parse(buf)
            for seq, t_us, g, a in pkts:
                tu = clock.add(t_us, arr)
                if not clock.ready:
                    continue
                if not announced:
                    announced = True
                    print(f"[streamer] clock sync ready ({clock.info()}). "
                          "Hold the rig still for 3 s, then start moving.")
                sender.put_imu(clock.to_pi_ns(tu), [v * DEG2RAD for v in g], [v * G for v in a])
                n["imu"] += 1
        elif kind == "cam":
            _, sensor_ns, recv, img, row = ev
            if recorder:
                recorder.frame(img, row)
            if clock.ready:
                sender.put_frame(sensor_ns, img, *frame_wh)
                n["cam"] += 1

    t_print = time.monotonic()
    while True:
        try:
            ev = events.get(timeout=0.5)
        except queue.Empty:
            if stop.is_set():
                break
            continue
        kind = ev[0]
        if kind == "end":
            break
        if kind == "error":
            print(f"[streamer] error: {ev[1]}")
            break
        if kind == "meta":
            frame_wh = [ev[1]["width"], ev[1]["height"]]
            if not args.no_record and not args.replay:
                recorder = Recorder(os.path.join(args.out, time.strftime("%Y%m%d_%H%M%S")), ev[1])
                print(f"[streamer] recording to {recorder.dir}")
            continue
        if kind == "start":
            t_start = ev[1]
            if recorder:
                recorder.meta["t_start_ns"] = t_start
                recorder.meta["lidar_spinup_s"] = ev[2]
                recorder.write_meta()
            print("[streamer] synchronised start: streaming and recording from here")
            for e in pre_start:  # queued before 'start' but timed after it
                if e[1] >= t_start:
                    handle(e)
            pre_start.clear()
            continue
        if t_start is None:
            pre_start.append(ev)  # ev[1] is arrival time (imu/lidar) or sensor time (cam)
            continue
        if ev[1] < t_start:
            continue
        handle(ev)
        now = time.monotonic()
        if now - t_print > 2.0:
            t_print = now
            print(f"[streamer] imu {n['imu']}  frames {n['cam']}  scans {n['scan']}  "
                  f"sent {sender.sent_imu}/{sender.sent_frames}/{sender.sent_scans}  "
                  f"dropped frames {sender.dropped_frames}  {clock.info()}"
                  + (f"  lidar {lid.clock.info()}" if n["scan"] else "")
                  + ("" if sender.connected else "  (no live_node connected)"))
    stop.set()
    if recorder:
        recorder.meta["t_end_ns"] = now_ns()
        recorder.close()
        print(f"[streamer] saved {recorder.dir}  (reprocess: ./demo.sh {os.path.basename(recorder.dir)})")


if __name__ == "__main__":
    main()
