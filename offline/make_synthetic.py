#!/usr/bin/env python3
"""Generate a fake recording (same format as pi/recorder.py) with known answers,
to test offline/convert.py without hardware.

Truth baked in: CF clock drift +40 ppm, 32-bit µs counter wraps mid-recording,
random UART latency/chunking, ~1% corrupted bytes, and camera timestamps that are
15 ms EARLY (so convert.py --check-sync should suggest about +15 ms).
"""
import argparse
import binascii
import json
import os
import struct

import cv2
import numpy as np

CAM_EARLY_S = 0.015
DRIFT = 40e-6


def gyro_z_dps(t):
    # a few rotational "shakes" plus slow yaw
    shake = sum(250 * np.exp(-((t - c) / 0.4) ** 2) * np.sin(2 * np.pi * 3.0 * (t - c))
                for c in (3.0, 9.0, 16.0, 24.0))
    return 20 * np.sin(2 * np.pi * 0.1 * t) + shake


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--duration", type=float, default=30.0)
    args = ap.parse_args()
    rng = np.random.default_rng(0)
    os.makedirs(args.out, exist_ok=True)
    w, h, fps = 640, 360, 30.0
    boot0 = 5_000_000_000  # Pi BOOTTIME at t=0

    # ---- IMU packets ----
    t = np.arange(0, args.duration, 0.002)
    gz = gyro_z_dps(t)
    t_cf_us = (np.uint64((1 << 32) - 8_000_000) + (t * 1e6).astype(np.uint64)) & np.uint64(0xFFFFFFFF)
    t_pi = boot0 + (t * (1 + DRIFT)) * 1e9                       # true Pi time of sample
    arrival = t_pi + 0.4e6 + rng.exponential(0.8e6, len(t))     # ns
    arrival = np.maximum.accumulate(arrival)
    pkts = []
    for k in range(len(t)):
        body = struct.pack("<HI6f", k & 0xFFFF, int(t_cf_us[k]),
                           rng.normal(0, 0.3), rng.normal(0, 0.3), gz[k] + rng.normal(0, 0.3),
                           rng.normal(0, 0.01), rng.normal(0, 0.01), 1 + rng.normal(0, 0.01))

        pkts.append(b"\xA5\x5A" + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF)))
    with open(os.path.join(args.out, "imu.bin"), "wb") as f:
        k = 0
        while k < len(pkts):
            n = int(rng.integers(1, 5))
            chunk = bytearray(b"".join(pkts[k:k + n]))
            if rng.random() < 0.01:
                chunk[int(rng.integers(len(chunk)))] ^= 0xFF
            f.write(struct.pack("<QI", int(arrival[min(k + n, len(t)) - 1]), len(chunk)))
            f.write(chunk)
            k += n

    # ---- camera: textured scene rotating with integrated gyro z ----
    theta = np.cumsum(gz) * 0.002                                # deg
    tex = np.zeros((900, 900), np.uint8)
    for _ in range(600):
        cv2.circle(tex, tuple(int(v) for v in rng.integers(0, 900, 2)), int(rng.integers(3, 15)),
                   int(rng.integers(40, 255)), -1)
    tex = cv2.GaussianBlur(tex, (3, 3), 0)
    t_frames = np.arange(0.5, args.duration - 0.5, 1 / fps)
    with open(os.path.join(args.out, "frames.bin"), "wb") as fb, \
            open(os.path.join(args.out, "frames.csv"), "w") as fc:
        fc.write("idx,sensor_ts_ns,recv_ns,exposure_us,analogue_gain,frame_duration_us,lens_position\n")
        for i, tf in enumerate(t_frames):
            ang = np.interp(tf, t, theta)
            M = cv2.getRotationMatrix2D((450, 450), ang, 1.0)
            img = cv2.warpAffine(tex, M, (900, 900))[270:270 + h, 130:130 + w]
            ts = int(boot0 + tf * (1 + DRIFT) * 1e9 - CAM_EARLY_S * 1e9)
            fb.write(np.ascontiguousarray(img).tobytes())
            fc.write(f"{i},{ts},{ts + 30_000_000},3000,4.000,33333,0.500\n")

    json.dump({"width": w, "height": h, "fps": fps, "clock": "CLOCK_BOOTTIME", "synthetic": True},
              open(os.path.join(args.out, "meta.json"), "w"), indent=2)
    print(f"wrote {args.out}: {len(pkts)} IMU packets, {len(t_frames)} frames")


if __name__ == "__main__":
    main()
