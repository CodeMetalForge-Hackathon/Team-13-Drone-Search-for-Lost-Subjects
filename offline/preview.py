#!/usr/bin/env python3
"""Turn a raw Pi recording into an MP4 you can play on the laptop.

Each frame gets a text overlay (time, frame number, exposure, dropped-frame
warning) and, when imu.bin is present, a strip underneath showing the gyro
over a +-2 s window with a line at the current frame. If the camera and IMU
are in sync, motion blur / image shake lines up with the gyro peaks at the line.

Usage
  python preview.py recordings/20261107_120000                 # -> recordings/.../preview.mp4
  python preview.py recordings/20261107_120000 --speed 0.5 --no-imu
"""
import argparse
import csv
import json
import os

import cv2
import numpy as np

import convert  # same folder: IMU parsing + clock fit

STRIP_H = 140
WINDOW_S = 2.0
COLORS = [(80, 80, 255), (80, 200, 80), (255, 160, 60)]  # BGR for gyro x, y, z


def load_imu(rec):
    path = os.path.join(rec, "imu.bin")
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return None
    stream, ends, times = convert.read_uart_log(path)
    imu, _ = convert.parse_packets(stream, ends, times)
    cf_to_pi, _, _ = convert.fit_clock(imu["t_cf_us"], imu["arr_ns"])
    return cf_to_pi(imu["t_cf_us"]) * 1e-9, imu["gyro"]


def draw_strip(w, t_now, t_imu, gyro, scale):
    strip = np.full((STRIP_H, w, 3), 25, np.uint8)
    mid = STRIP_H // 2
    cv2.line(strip, (0, mid), (w, mid), (70, 70, 70), 1)
    lo, hi = np.searchsorted(t_imu, [t_now - WINDOW_S, t_now + WINDOW_S])
    if hi - lo > 1:
        x = ((t_imu[lo:hi] - (t_now - WINDOW_S)) / (2 * WINDOW_S) * (w - 1)).astype(np.int32)
        for c in range(3):
            y = (mid - gyro[lo:hi, c] / scale * (mid - 6)).clip(2, STRIP_H - 3).astype(np.int32)
            cv2.polylines(strip, [np.c_[x, y].reshape(-1, 1, 2)], False, COLORS[c], 1, cv2.LINE_AA)
    cv2.line(strip, (w // 2, 0), (w // 2, STRIP_H), (255, 255, 255), 1)
    for c, name in enumerate("xyz"):
        cv2.putText(strip, f"gyro {name}", (8 + 85 * c, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLORS[c], 1, cv2.LINE_AA)
    cv2.putText(strip, f"+-{scale:.1f} rad/s   window +-{WINDOW_S:.0f} s", (w - 250, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    return strip


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rec", help="recording folder from pi/recorder.py")
    ap.add_argument("--out", help="output .mp4 (default: <rec>/preview.mp4)")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor")
    ap.add_argument("--no-imu", action="store_true")
    args = ap.parse_args()

    meta = json.load(open(os.path.join(args.rec, "meta.json")))
    w, h = meta["width"], meta["height"]
    with open(os.path.join(args.rec, "frames.csv")) as f:
        rows = list(csv.DictReader(f))
    n = min(len(rows), os.path.getsize(os.path.join(args.rec, "frames.bin")) // (w * h))
    imgs = np.memmap(os.path.join(args.rec, "frames.bin"), np.uint8, "r", shape=(n, h, w))
    t_cam = np.array([int(r["sensor_ts_ns"]) for r in rows[:n]], np.int64) * 1e-9
    period = float(np.median(np.diff(t_cam)))

    imu = None if args.no_imu else load_imu(args.rec)
    if imu is not None:
        t_imu, gyro = imu
        scale = max(0.5, float(np.percentile(np.abs(gyro), 99.5)))

    out = args.out or os.path.join(args.rec, "preview.mp4")
    out_h = h + (STRIP_H if imu is not None else 0)
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), args.speed / period, (w, out_h))
    if not vw.isOpened():
        raise SystemExit(f"Could not open video writer for {out}")

    for k in range(n):
        frame = cv2.cvtColor(np.asarray(imgs[k]), cv2.COLOR_GRAY2BGR)
        gap = 0 if k == 0 else round((t_cam[k] - t_cam[k - 1]) / period) - 1
        txt = f"t={t_cam[k] - t_cam[0]:7.2f}s  frame {k}/{n - 1}  exp {rows[k]['exposure_us']}us"
        cv2.putText(frame, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        if gap > 0:
            cv2.putText(frame, f"{gap} DROPPED", (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
        if imu is not None:
            frame = np.vstack([frame, draw_strip(w, t_cam[k], t_imu, gyro, scale)])
        vw.write(frame)
    vw.release()
    print(f"Wrote {out}  ({n} frames, {n * period / args.speed:.1f} s at {args.speed}x)")


if __name__ == "__main__":
    main()
