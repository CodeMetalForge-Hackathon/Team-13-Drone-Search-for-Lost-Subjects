#!/usr/bin/env python3
"""RPLIDAR C1 helpers: port auto-detect, device info/health, raw scan start/stop, scan parser.

The C1 talks the standard Slamtec protocol at 460800 baud (USB adapter board = CP2102).
recorder.py stores the raw scan byte stream; parse_scan() decodes it offline.

Usage
  python3 lidar.py              # print info + health, then 2 s of scan stats
  python3 lidar.py --port /dev/ttyUSB0
"""
import argparse
import glob
import os
import struct
import time

BAUD = 460800
CMD_STOP, CMD_RESET, CMD_SCAN, CMD_INFO, CMD_HEALTH = 0x25, 0x40, 0x20, 0x50, 0x52
SCAN_DESC = b"\xA5\x5A\x05\x00\x00\x40\x81"
NODE = 5  # bytes per standard-scan sample


def default_port():
    """First Silicon Labs / CH340 USB-serial adapter, else /dev/ttyUSB0."""
    ids = sorted(glob.glob("/dev/serial/by-id/*"))
    for p in ids:
        if any(k in p for k in ("CP210", "Silicon_Labs", "1a86", "CH34", "Slamtec")):
            return os.path.realpath(p)
    return "/dev/ttyUSB0"


def _cmd(ser, cmd):
    ser.write(bytes([0xA5, cmd]))


def _response(ser, timeout=1.0):
    """Read a 7-byte response descriptor + payload. Returns (type, payload) or None."""
    t_end = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < t_end:
        buf += ser.read(max(1, ser.in_waiting))
        p = buf.find(b"\xA5\x5A")
        if p >= 0 and len(buf) >= p + 7:
            size = struct.unpack_from("<I", buf, p + 2)[0] & 0x3FFFFFFF
            typ = buf[p + 6]
            while len(buf) < p + 7 + size and time.monotonic() < t_end:
                buf += ser.read(p + 7 + size - len(buf))
            if len(buf) >= p + 7 + size:
                return typ, bytes(buf[p + 7:p + 7 + size])
    return None


def stop(ser):
    _cmd(ser, CMD_STOP)
    time.sleep(0.05)
    ser.reset_input_buffer()


def info(ser):
    """Returns dict(model, firmware, hardware, serial) or None if the lidar doesn't answer."""
    stop(ser)
    _cmd(ser, CMD_INFO)
    r = _response(ser)
    if not r or r[0] != 0x04 or len(r[1]) != 20:
        return None
    d = r[1]
    return {"model": d[0], "firmware": f"{d[2]}.{d[1]:02d}", "hardware": d[3], "serial": d[4:].hex().upper()}


def health(ser):
    """Returns 'Good' / 'Warning' / 'Error' (+ code) or None."""
    _cmd(ser, CMD_HEALTH)
    r = _response(ser)
    if not r or r[0] != 0x06 or len(r[1]) != 3:
        return None
    status, code = r[1][0], struct.unpack_from("<H", r[1], 1)[0]
    return ["Good", "Warning", "Error"][status] + ("" if status == 0 else f" (code {code})")


def start_scan(ser):
    """Send SCAN and consume the response descriptor; the motor spins up on its own on the C1."""
    stop(ser)
    _cmd(ser, CMD_SCAN)
    t_end = time.monotonic() + 2.0
    buf = bytearray()
    while time.monotonic() < t_end:
        buf += ser.read(max(1, ser.in_waiting))
        p = buf.find(SCAN_DESC)
        if p >= 0:
            return bytes(buf[p + len(SCAN_DESC):])  # leftover sample bytes, keep them
    raise RuntimeError("lidar did not acknowledge SCAN")


def parse_scan(buf):
    """Decode standard-scan samples from a bytearray (consumed in place).

    Returns (samples, bad) with samples = [(start_flag, quality, angle_deg, dist_mm), ...].
    Resyncs byte by byte when the S/!S or check bit is wrong."""
    out, bad, i, n = [], 0, 0, len(buf)
    while i + NODE <= n:
        b0, b1 = buf[i], buf[i + 1]
        s = b0 & 1
        if s == ((b0 >> 1) & 1) or not (b1 & 1):
            bad += 1
            i += 1
            continue
        angle = ((buf[i + 2] << 7) | (b1 >> 1)) / 64.0
        dist = (buf[i + 3] | (buf[i + 4] << 8)) / 4.0
        out.append((s, b0 >> 2, angle, dist))
        i += NODE
    del buf[:i]
    return out, bad


class SpinupDetector:
    """Decides when the motor has settled, from the number of samples per revolution.

    The C1 motor overshoots while spinning up (~88 ms revolutions settling to 100 ms over ~1 s). The sample
    rate is fixed, so samples per revolution measure the period exactly, unlike arrival times, which are
    blurred by USB batching. Steady = the last `steady_revs` revolutions all within `tol` of their median."""

    def __init__(self, steady_revs=6, tol=0.01):
        self.steady_revs, self.tol = steady_revs, tol
        self.rev_t, self.rev_n, self.since = [], [], 0
        self.samples_per_rev = self.rev_hz = None

    def feed(self, t_ns, samples):
        """Feed decoded samples that arrived at t_ns. Returns True once steady (and stays True)."""
        if self.samples_per_rev is not None:
            return True
        for s in samples:
            self.since += 1
            if s[0]:
                if self.since > 100:  # ignore start flags of near-empty revolutions
                    self.rev_t.append(t_ns)
                    self.rev_n.append(self.since)
                self.since = 0
        k = self.steady_revs
        if len(self.rev_n) > k:  # rev_n[0] may be a partial first revolution
            last = self.rev_n[-k:]
            med = sorted(last)[len(last) // 2]
            if all(abs(c - med) <= self.tol * med for c in last):
                self.samples_per_rev = med
                self.rev_hz = k * 1e9 / (self.rev_t[-1] - self.rev_t[-1 - k])
                return True
        return False


RANGE_MIN, RANGE_MAX = 0.05, 12.0  # RPLIDAR C1 spec [m]


def bin_scan(angle_deg, range_m, quality, t_ns):
    """One revolution (samples in time order) -> sensor_msgs/LaserScan fields, ranges in time order.

    Frame convention matches rplidar_ros: ROS angle = pi - device angle (device angles run clockwise).
    The device rotates clockwise, so angle_increment is negative (laser_geometry/rviz handle that) and
    time_increment is exact. Each sample goes to the bin of its angle relative to the first sample;
    empty bins and out-of-spec ranges are inf. Returns a dict, plus 'stamp_ns' = first sample time."""
    import numpy as np
    ang = np.asarray(angle_deg, np.float64)
    r = np.asarray(range_m, np.float64)
    t = np.asarray(t_ns, np.int64)
    n = len(ang)
    inc = 360.0 / n
    b = np.clip(np.round(((ang - ang[0]) % 360.0) / inc).astype(np.int64), 0, n - 1)
    ranges = np.full(n, np.inf, np.float32)
    inten = np.zeros(n, np.float32)
    ok = (r >= RANGE_MIN) & (r <= RANGE_MAX)
    ranges[b[ok]] = r[ok]
    inten[b[ok]] = np.asarray(quality)[ok]
    scan_time = (t[-1] - t[0]) * 1e-9 * n / max(n - 1, 1)
    a0 = (np.pi - np.deg2rad(ang[0]) + np.pi) % (2 * np.pi) - np.pi
    d = -np.deg2rad(inc)
    return {"stamp_ns": int(t[0]), "angle_min": float(a0), "angle_max": float(a0 + d * (n - 1)),
            "angle_increment": float(d), "time_increment": float(scan_time / n), "scan_time": float(scan_time),
            "range_min": RANGE_MIN, "range_max": RANGE_MAX, "ranges": ranges, "intensities": inten}


def main():
    import serial
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=default_port())
    ap.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args()
    with serial.Serial(args.port, BAUD, timeout=0.01) as ser:
        ser.dtr = False
        i = info(ser)
        print(f"{args.port}: info={i} health={health(ser) if i else None}")
        if not i:
            raise SystemExit("no answer from the lidar")
        buf = bytearray(start_scan(ser))
        samples, bad, t_end = [], 0, time.monotonic() + args.seconds
        while time.monotonic() < t_end:
            buf += ser.read(max(1, ser.in_waiting))
            s, b = parse_scan(buf)
            samples += s
            bad += b
        stop(ser)
    revs = sum(s[0] for s in samples)
    valid = [s for s in samples if s[3] > 0]
    print(f"{len(samples) / args.seconds:.0f} samples/s, {revs / args.seconds:.1f} rev/s, "
          f"{len(valid)}/{len(samples)} with range, {bad} resync bytes")
    if valid:
        d = sorted(s[3] for s in valid)
        print(f"range mm: min={d[0]:.0f} median={d[len(d) // 2]:.0f} max={d[-1]:.0f}")


if __name__ == "__main__":
    main()
