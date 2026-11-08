#!/usr/bin/env python3
"""Check the Crazyflie IMU stream on the Pi before recording. Needs only python3-serial.

Prints, twice a second: packet rate, CRC errors, latest gyro [deg/s] and accel [g], and a verdict:
  OK            real data (at rest |acc| is about 1.0 g; tilt the drone and the gyro should move)
  ALL ZEROS     packets arrive but carry zeros: the Crazyflie's sensors have not been calibrated, so its
                stabilizer is not running. Put it flat and still; if that does not fix it, power-cycle it
                lying flat and watch its LEDs (see README "IMU check").
  NO DATA       nothing valid on the UART: check the TX2 -> GPIO15 wire, GND, the firmware app, the baud rate

Usage
  python3 imu_check.py            # live, Ctrl+C to stop
  python3 imu_check.py --once     # 1 s check, exit code 0 only if OK (recorder.py uses this)
"""
import argparse
import binascii
import math
import struct
import sys
import time

SYNC = b"\xA5\x5A"
PKT = struct.Struct("<2sHI6fH")  # same 34-byte packet as firmware/app_imu_uart


def default_port():
    """The UART on GPIO14/15. /dev/serial0 is right on the Zero 2 W, but on a Pi 5 it points to the
    separate debug connector (ttyAMA10); there the GPIO UART is /dev/ttyAMA0 (dtparam=uart0=on)."""
    import os
    s0 = os.path.realpath("/dev/serial0")
    if s0.endswith("ttyAMA10") and os.path.exists("/dev/ttyAMA0"):
        return "/dev/ttyAMA0"
    return "/dev/serial0"


def parse(buf):
    """Parse complete packets from a bytearray (consumed in place). Returns (packets, crc_errors)."""
    out, bad = [], 0
    p = buf.find(SYNC)
    while p >= 0 and p + PKT.size <= len(buf):
        body = bytes(buf[p + 2:p + 32])
        crc = struct.unpack_from("<H", buf, p + 32)[0]
        if binascii.crc_hqx(body, 0xFFFF) == crc:
            _, seq, t_us, gx, gy, gz, ax, ay, az, _ = PKT.unpack_from(buf, p)
            out.append((seq, t_us, (gx, gy, gz), (ax, ay, az)))
            p = buf.find(SYNC, p + PKT.size)
        else:
            bad += 1
            p = buf.find(SYNC, p + 1)
    del buf[:p if p >= 0 else max(0, len(buf) - 1)]
    return out, bad


def verdict(pkts):
    if not pkts:
        return "NO DATA"
    zeros = sum(1 for _, _, g, a in pkts if not any(g) and not any(a))
    return "ALL ZEROS" if zeros > 0.5 * len(pkts) else "OK"


def check(ser, seconds):
    """Read for `seconds`; return (verdict, packets, crc_errors)."""
    buf, pkts, bad = bytearray(), [], 0
    ser.reset_input_buffer()
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        buf += ser.read(max(1, ser.in_waiting))
        p, b = parse(buf)
        pkts += p
        bad += b
    return verdict(pkts), pkts, bad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=default_port(), help="GPIO UART (auto: ttyAMA0 on a Pi 5)")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--once", action="store_true", help="check for 1 s, exit 0 only if OK")
    args = ap.parse_args()
    import serial
    ser = serial.Serial(args.port, args.baud, timeout=0.01)

    if args.once:
        v, pkts, bad = check(ser, 1.0)
        print(f"IMU check: {v} ({len(pkts)} packets/s, {bad} CRC errors)")
        sys.exit(0 if v == "OK" else 1)

    print("Ctrl+C to stop. Tilt the drone: the gyro should move; at rest |acc| should be about 1.00 g.")
    try:
        while True:
            v, pkts, bad = check(ser, 0.5)
            if pkts:
                _, _, g, a = pkts[-1]
                na = math.sqrt(sum(x * x for x in a))
                line = (f"{v:9s} {len(pkts) * 2:4d} Hz  crc_err {bad:2d}  "
                        f"gyro [{g[0]:7.1f} {g[1]:7.1f} {g[2]:7.1f}] deg/s  "
                        f"acc [{a[0]:5.2f} {a[1]:5.2f} {a[2]:5.2f}] g  |acc| {na:4.2f}")
            else:
                line = f"{v:9s}    0 Hz  crc_err {bad:2d}"
            print(line, flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
