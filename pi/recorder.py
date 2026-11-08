#!/usr/bin/env python3
"""Record camera frames + Crazyflie IMU stream on a Raspberry Pi Zero 2 W.

Everything is stored raw so the Pi does almost no work; offline/convert.py
turns a recording into a EuRoC dataset / ROS1 bag.

Output folder (recordings/<YYYYmmdd_HHMMSS>/):
  frames.bin   concatenated 8-bit grayscale images (W*H bytes each)
  frames.csv   idx,sensor_ts_ns,recv_ns,exposure_us,analogue_gain,frame_duration_us,lens_position
  imu.bin      raw UART chunks: [uint64 recv_ns][uint32 len][len bytes] ...
  meta.json    settings used

All Pi-side times are CLOCK_BOOTTIME ns, the clock libcamera uses for SensorTimestamp.

Before recording it checks 1 s of IMU data and refuses to start if there is none or it is all zeros.

Usage:
  python3 recorder.py                      # 640x360 @ 30 fps, Ctrl+C to stop
  python3 recorder.py --duration 120 --exposure-us 2000 --gain 6
"""
import argparse
import csv
import json
import os
import queue
import signal
import struct
import threading
import time

import serial
from picamera2 import Picamera2

from libcamera import Transform

try:
    from libcamera import controls as lc_controls
except ImportError:  # older images
    lc_controls = None


def now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


class ImuLogger(threading.Thread):
    """Reads the UART as fast as possible and stores each chunk with its arrival time."""

    def __init__(self, port: str, baud: int, path: str, stop: threading.Event):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=0.002)
        self.ser.reset_input_buffer()
        self.f = open(path, "wb", buffering=1 << 20)
        self.stop = stop
        self.nbytes = 0

    def run(self):
        hdr = struct.Struct("<QI")
        while not self.stop.is_set():
            data = self.ser.read(max(1, self.ser.in_waiting))
            if data:
                t = now_ns()
                self.f.write(hdr.pack(t, len(data)))
                self.f.write(data)
                self.nbytes += len(data)
        self.f.close()
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
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--duration", type=float, default=0, help="seconds, 0 = until Ctrl+C")
    ap.add_argument("--skip-imu-check", action="store_true",
                    help="record even if the 1 s IMU check fails (camera-only recordings)")
    args = ap.parse_args()

    # --- refuse to record without real IMU data (see imu_check.py) ------------
    if not args.skip_imu_check:
        import imu_check
        with serial.Serial(args.port, args.baud, timeout=0.01) as ser:
            v, pkts, bad = imu_check.check(ser, 1.0)
        print(f"IMU check: {v} ({len(pkts)} packets/s, {bad} CRC errors)")
        if v != "OK":
            hint = {"ALL ZEROS": "Crazyflie sensors not calibrated: put it flat and still, or power-cycle it lying flat.",
                    "NO DATA": "nothing on the UART: check the TX2 wire, GND and the firmware app."}[v]
            raise SystemExit(f"Not recording: {hint}  Live view: python3 imu_check.py  (override: --skip-imu-check)")

    out_dir = os.path.join(args.out, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

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

    meta = {
        "width": w, "height": h, "fps": args.fps, "exposure_us": args.exposure_us,
        "gain": args.gain, "lens_position": args.lens_position, "rotate": args.rotate,
        "sensor_size": [sw, sh], "camera": cam.camera_properties.get("Model"),
        "imu_port": args.port, "imu_baud": args.baud, "clock": "CLOCK_BOOTTIME",
        "imu_packet": "A5 5A | u16 seq | u32 t_us | f32 gyro[3] deg/s | f32 acc[3] g | u16 crc",
    }
    with open(os.path.join(out_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # --- start logging ------------------------------------------------------
    imu = ImuLogger(args.port, args.baud, os.path.join(out_dir, "imu.bin"), stop)
    q: "queue.Queue" = queue.Queue(maxsize=90)
    writer = FrameWriter(out_dir, q)
    imu.start()
    writer.start()
    cam.start()

    print(f"Recording to {out_dir}  (Ctrl+C to stop)")
    t_start = time.monotonic()
    t_print = t_start
    n = dropped_q = 0
    while not stop.is_set():
        req = cam.capture_request()
        try:
            recv = now_ns()
            md = req.get_metadata()
            y = req.make_array("main")[:h, :w].tobytes()  # Y plane of YUV420
        finally:
            req.release()
        row = [n, md.get("SensorTimestamp", 0), recv, md.get("ExposureTime", 0),
               f"{md.get('AnalogueGain', 0):.3f}", md.get("FrameDuration", 0),
               f"{md.get('LensPosition', -1):.3f}"]
        try:
            q.put_nowait((y, row))
            n += 1
        except queue.Full:
            dropped_q += 1
        now = time.monotonic()
        if now - t_print > 2.0:
            t_print = now
            print(f"  {now - t_start:6.1f}s  frames={n}  writer_drops={dropped_q}  "
                  f"imu={imu.nbytes / 1024:.0f} KiB")
        if args.duration and now - t_start >= args.duration:
            stop.set()

    cam.stop()
    q.put(None)
    writer.join()
    imu.join(timeout=1.0)
    print(f"Done: {n} frames, {dropped_q} dropped by writer, {imu.nbytes} IMU bytes -> {out_dir}")
    if imu.nbytes == 0:
        print("WARNING: no IMU bytes received; check wiring (CF TX2 -> Pi GPIO15/RXD) and UART setup")


if __name__ == "__main__":
    main()
