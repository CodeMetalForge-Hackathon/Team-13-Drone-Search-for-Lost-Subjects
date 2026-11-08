#!/usr/bin/env python3
"""Record camera frames + Crazyflie IMU stream + RPLIDAR C1 scans on a Raspberry Pi.

Everything is stored raw so the Pi does almost no work; offline/convert.py
turns a recording into a EuRoC dataset / ROS1 bag.

Output folder (recordings/<YYYYmmdd_HHMMSS>/):
  frames.bin   concatenated 8-bit grayscale images (W*H bytes each)
  frames.csv   idx,sensor_ts_ns,recv_ns,exposure_us,analogue_gain,frame_duration_us,lens_position
  imu.bin      raw UART chunks: [uint64 recv_ns][uint32 len][len bytes] ...
  lidar.bin    raw RPLIDAR standard-scan bytes, same chunk format (decode with lidar.parse_scan)
  meta.json    settings used, plus t_start_ns / t_end_ns

All Pi-side times are CLOCK_BOOTTIME ns, the clock libcamera uses for SensorTimestamp.

Before recording it checks 1 s of IMU data and refuses to start if there is none or it is all zeros,
and refuses if the lidar does not answer (use --no-lidar to record without it).

Synchronised start: camera, IMU and lidar are all started first, but nothing is written until the lidar
motor has spun up and gives steady revolutions (~2 s). That moment becomes t_start_ns, and every
stream keeps only data from t_start_ns on (camera: SensorTimestamp, IMU/lidar: chunk arrival time).
Stopping works the same way with t_end_ns, so all three files cover the same time window.

Usage:
  python3 recorder.py                      # 640x360 @ 30 fps, Ctrl+C to stop
  python3 recorder.py --duration 120 --exposure-us 2000 --gain 6
"""
import argparse
import csv
import json
import os
import queue
import shutil
import signal
import struct
import threading
import time

import serial

import imu_check  # same folder: packet parser + UART auto-detect
import lidar  # same folder: RPLIDAR C1 protocol
from picamera2 import Picamera2

from libcamera import Transform

try:
    from libcamera import controls as lc_controls
except ImportError:  # older images
    lc_controls = None

CHUNK = struct.Struct("<QI")


def now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


class Gate:
    """Shared recording window [start, end] on CLOCK_BOOTTIME. Data outside it is discarded."""

    def __init__(self):
        self.start = None  # set once, when every stream is ready
        self.end = None    # set on stop

    def open(self, t):
        return self.start is not None and t >= self.start and (self.end is None or t <= self.end)


class ImuLogger(threading.Thread):
    """Reads the UART as fast as possible and stores each chunk (inside the gate) with its arrival time."""

    def __init__(self, port: str, baud: int, path: str, stop: threading.Event, gate: Gate):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=0.002)
        self.ser.reset_input_buffer()
        self.f = open(path, "wb", buffering=1 << 20)
        self.stop, self.gate = stop, gate
        self.nbytes = 0

    def run(self):
        while not self.stop.is_set():
            data = self.ser.read(max(1, self.ser.in_waiting))
            if data:
                t = now_ns()
                if self.gate.open(t):
                    self.f.write(CHUNK.pack(t, len(data)))
                    self.f.write(data)
                    self.nbytes += len(data)
        self.f.close()
        self.ser.close()


class LidarLogger(threading.Thread):
    """Starts a standard scan, waits for steady rotation, then defines the common start time.

    Once lidar.SpinupDetector says the motor has settled, gate.start is set to the arrival time of the
    current chunk and `ready` is signalled. Decoding consumes whole samples only, so writing starts on a
    sample boundary."""

    def __init__(self, port: str, path: str, stop: threading.Event, gate: Gate):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, lidar.BAUD, timeout=0.002)
        self.ser.dtr = False
        self.path = path
        self.stop, self.gate = stop, gate
        self.spin = lidar.SpinupDetector()
        self.ready = threading.Event()
        self.nbytes = 0
        self.t_scan_cmd = now_ns()
        self._pending = bytearray(lidar.start_scan(self.ser))  # bytes that came with the SCAN ack

    def _wait_steady(self, f):
        while not self.stop.is_set():
            data = self.ser.read(max(1, self.ser.in_waiting))
            if not data:
                continue
            t = now_ns()
            self._pending += data
            samples, _ = lidar.parse_scan(self._pending)  # leaves only a partial sample in _pending
            if self.spin.feed(t, samples):
                self.gate.start = t
                if self._pending:  # partial sample after the last decoded one: keep it
                    f.write(CHUNK.pack(t, len(self._pending)))
                    f.write(self._pending)
                    self.nbytes += len(self._pending)
                self.ready.set()
                return True
        return False

    def run(self):
        with open(self.path, "wb", buffering=1 << 20) as f:
            if self._wait_steady(f):
                while not self.stop.is_set():
                    data = self.ser.read(max(1, self.ser.in_waiting))
                    if data:
                        t = now_ns()
                        if self.gate.open(t):
                            f.write(CHUNK.pack(t, len(data)))
                            f.write(data)
                            self.nbytes += len(data)
        lidar.stop(self.ser)
        self.ser.close()


class FrameWriter(threading.Thread):
    """Writes frames off the capture thread so an SD-card stall doesn't drop frames."""

    def __init__(self, out_dir: str, q: "queue.Queue"):
        super().__init__(daemon=True)
        self.q = q
        self.fbin = open(os.path.join(out_dir, "frames.bin"), "wb", buffering=1 << 22)
        self.fcsv = open(os.path.join(out_dir, "frames.csv"), "w", newline="")
        self.csv = csv.writer(self.fcsv)
        self.csv.writerow(["idx", "sensor_ts_ns", "recv_ns", "exposure_us",
                           "analogue_gain", "frame_duration_us", "lens_position"])

    def run(self):
        while True:
            item = self.q.get()
            if item is None:
                break
            img, row = item
            self.fbin.write(img)
            self.csv.writerow(row)
        self.fbin.close()
        self.fcsv.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="recordings")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--exposure-us", type=int, default=3000,
                    help="fixed exposure; keep short to limit motion blur")
    ap.add_argument("--gain", type=float, default=4.0, help="fixed analogue gain")
    ap.add_argument("--lens-position", type=float, default=0.5,
                    help="Camera Module 3 focus in dioptres (0=infinity, 0.5=2 m). "
                         "Must be the same for calibration and flights.")
    ap.add_argument("--sensor-size", default="2304x1296",
                    help="sensor mode; 2304x1296 is the full-FOV 2x2 binned IMX708 mode")
    ap.add_argument("--rotate", type=int, choices=(0, 180), default=180,
                    help="image rotation done in the sensor (no CPU cost); 180 = camera mounted "
                         "upside down. Use the same value for calibration and flights.")
    ap.add_argument("--port", default=imu_check.default_port(), help="GPIO UART (auto: ttyAMA0 on a Pi 5)")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--lidar-port", default=lidar.default_port(), help="RPLIDAR C1 USB serial port")
    ap.add_argument("--no-lidar", action="store_true", help="record without the lidar")
    ap.add_argument("--lidar-timeout", type=float, default=10.0,
                    help="seconds to wait for steady lidar rotation before giving up")
    ap.add_argument("--duration", type=float, default=0,
                    help="seconds after the synchronised start, 0 = until Ctrl+C")
    ap.add_argument("--skip-imu-check", action="store_true",
                    help="record even if the 1 s IMU check fails (camera-only recordings)")
    args = ap.parse_args()

    # --- refuse to record without real IMU data (see imu_check.py) ------------
    if not args.skip_imu_check:
        with serial.Serial(args.port, args.baud, timeout=0.01) as ser:
            v, pkts, bad = imu_check.check(ser, 1.0)
        print(f"IMU check: {v} ({len(pkts)} packets/s, {bad} CRC errors)")
        if v != "OK":
            hint = {"ALL ZEROS": "Crazyflie sensors not calibrated: put it flat and still, or power-cycle it lying flat.",
                    "NO DATA": "nothing on the UART: check the TX2 wire, GND and the firmware app."}[v]
            raise SystemExit(f"Not recording: {hint}  Live view: python3 imu_check.py  (override: --skip-imu-check)")

    # --- lidar must answer GET_INFO ------------------------------------------
    lidar_info = None
    if not args.no_lidar:
        try:
            with serial.Serial(args.lidar_port, lidar.BAUD, timeout=0.01) as ser:
                ser.dtr = False
                lidar_info = lidar.info(ser)
                lidar_health = lidar.health(ser) if lidar_info else None
        except serial.SerialException as e:
            raise SystemExit(f"Not recording: cannot open lidar port {args.lidar_port} ({e}). "
                             "Check the USB cable carries data (lsusb should list a CP210x). Override: --no-lidar")
        if not lidar_info:
            raise SystemExit(f"Not recording: no answer from lidar on {args.lidar_port}. Override: --no-lidar")
        print(f"Lidar: {lidar_info} health={lidar_health}")
        lidar_info["health"] = lidar_health

    out_dir = os.path.join(args.out, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir)
    stop = threading.Event()   # tells the logger threads to exit
    quit_ = threading.Event()  # Ctrl+C / SIGTERM: main closes the gate first, then sets `stop`
    signal.signal(signal.SIGINT, lambda *_: quit_.set())
    signal.signal(signal.SIGTERM, lambda *_: quit_.set())
    gate = Gate()

    # --- camera -----------------------------------------------------------
    w, h = args.width, args.height
    sw, sh = (int(v) for v in args.sensor_size.split("x"))
    frame_us = int(round(1e6 / args.fps))
    cam = Picamera2()
    ctrl = {
        "AeEnable": False,
        "AwbEnable": False,
        "ExposureTime": args.exposure_us,
        "AnalogueGain": args.gain,
        "FrameDurationLimits": (frame_us, frame_us),
    }
    if "AfMode" in cam.camera_controls and lc_controls is not None:
        ctrl["AfMode"] = lc_controls.AfModeEnum.Manual
        ctrl["LensPosition"] = args.lens_position
    cfg = cam.create_video_configuration(
        main={"size": (w, h), "format": "YUV420"},
        raw={"size": (sw, sh)},
        buffer_count=6,
        controls=ctrl,
        transform=Transform(hflip=args.rotate == 180, vflip=args.rotate == 180),
    )
    cam.configure(cfg)

    # --- start everything; nothing is written until the gate opens -----------
    lid = None
    if not args.no_lidar:
        lid = LidarLogger(args.lidar_port, os.path.join(out_dir, "lidar.bin"), stop, gate)
    imu = ImuLogger(args.port, args.baud, os.path.join(out_dir, "imu.bin"), stop, gate)
    q: "queue.Queue" = queue.Queue(maxsize=90)
    writer = FrameWriter(out_dir, q)
    if lid:
        lid.start()
    imu.start()
    writer.start()
    cam.start()

    def grab():
        """Capture one frame; returns (sensor_ts, recv_ns, metadata, Y-plane bytes)."""
        req = cam.capture_request()
        try:
            recv = now_ns()
            md = req.get_metadata()
            y = req.make_array("main")[:h, :w].tobytes()  # Y plane of YUV420
        finally:
            req.release()
        return md.get("SensorTimestamp", 0), recv, md, y

    if lid:
        print("Waiting for the lidar to spin up ...")
        t_wait = time.monotonic()
        # keep the camera pipeline draining while we wait
        while not lid.ready.is_set() and not quit_.is_set():
            if time.monotonic() - t_wait > args.lidar_timeout:
                break
            grab()
        if not lid.ready.is_set():
            stop.set()
            cam.stop()
            q.put(None)
            writer.join()
            imu.join(timeout=1.0)
            lid.join(timeout=1.0)
            shutil.rmtree(out_dir, ignore_errors=True)
            raise SystemExit("Not recording: interrupted" if time.monotonic() - t_wait <= args.lidar_timeout else
                             f"Not recording: lidar did not reach steady rotation within {args.lidar_timeout:.0f} s")
        print(f"Lidar steady at ~{lid.spin.rev_hz:.1f} rev/s ({lid.spin.samples_per_rev} samples/rev) after {(gate.start - lid.t_scan_cmd) / 1e9:.1f} s")
    else:
        gate.start = now_ns()

    meta = {
        "width": w, "height": h, "fps": args.fps, "exposure_us": args.exposure_us,
        "gain": args.gain, "lens_position": args.lens_position, "rotate": args.rotate,
        "sensor_size": [sw, sh], "camera": cam.camera_properties.get("Model"),
        "imu_port": args.port, "imu_baud": args.baud, "clock": "CLOCK_BOOTTIME",
        "t_start_ns": gate.start, "t_end_ns": None,
        "lidar_port": None if args.no_lidar else args.lidar_port, "lidar_baud": lidar.BAUD,
        "lidar_info": lidar_info,
        "lidar_spinup_s": None if not lid else (gate.start - lid.t_scan_cmd) / 1e9,
        "lidar_format": "RPLIDAR standard scan, 5 B/sample: S|!S|quality6, C|angle_q6[6:0], angle_q6[14:7], dist_q2 u16",
        "imu_packet": "A5 5A | u16 seq | u32 t_us | f32 gyro[3] deg/s | f32 acc[3] g | u16 crc",
    }

    def write_meta():
        with open(os.path.join(out_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
    write_meta()

    print(f"Recording to {out_dir}  (Ctrl+C to stop)")
    t_start = time.monotonic()
    t_print = t_start
    n = dropped_q = 0
    nbytes_prev = 0
    while not quit_.is_set():
        ts, recv, md, y = grab()
        if ts < gate.start:  # frame exposed before the synchronised start
            continue
        row = [n, ts, recv, md.get("ExposureTime", 0),
               f"{md.get('AnalogueGain', 0):.3f}", md.get("FrameDuration", 0),
               f"{md.get('LensPosition', -1):.3f}"]
        try:
            q.put_nowait((y, row))
            n += 1
        except queue.Full:
            dropped_q += 1
        now = time.monotonic()
        if now - t_print > 2.0:
            # 500 packets/s x 34 bytes = 17 kB/s; much less means IMU packets are being lost
            imu_rate = (imu.nbytes - nbytes_prev) / (now - t_print) / (500 * 34)
            nbytes_prev = imu.nbytes
            t_print = now
            warn = f"  <-- IMU at {imu_rate:.0%}: check the TX2/GND wires" if imu_rate < 0.9 else ""
            print(f"  {now - t_start:6.1f}s  frames={n}  writer_drops={dropped_q}  "
                  f"imu={imu.nbytes / 1024:.0f} KiB  lidar={(lid.nbytes if lid else 0) / 1024:.0f} KiB{warn}")
        if args.duration and now - t_start >= args.duration:
            break

    gate.end = now_ns()  # common end: IMU/lidar chunks after this are dropped
    time.sleep(0.05)     # let IMU/lidar read what arrived up to gate.end (USB batching ~3 ms)
    stop.set()
    cam.stop()
    q.put(None)
    writer.join()
    imu.join(timeout=1.0)
    if lid:
        lid.join(timeout=1.0)
    meta["t_end_ns"] = gate.end
    write_meta()
    print(f"Done: {n} frames, {dropped_q} dropped by writer, {imu.nbytes} IMU bytes, "
          f"{lid.nbytes if lid else 0} lidar bytes -> {out_dir}")
    if imu.nbytes == 0:
        print("WARNING: no IMU bytes received; check wiring (CF TX2 -> Pi GPIO15/RXD) and UART setup")


if __name__ == "__main__":
    main()
