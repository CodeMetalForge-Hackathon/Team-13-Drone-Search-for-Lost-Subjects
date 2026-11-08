#!/usr/bin/env python3
"""Convert a Pi recording (pi/recorder.py) into a VIO dataset.

Steps
  1. Parse the Crazyflie IMU packets out of the raw UART log (CRC-checked).
  2. Fit the Crazyflie clock to the Pi clock (offset + drift) using the
     lower envelope of packet arrival times.
  3. If the recording has lidar.bin (RPLIDAR C1), decode the scan samples and time them the same way:
     the lidar sends no timestamps, but samples come at a constant rate, so sample index is its clock.
  4. Put IMU, camera and lidar on the Pi clock (ns) and write
       <out>/mav0/cam0/data/<ts>.png, <out>/mav0/cam0/data.csv, <out>/mav0/imu0/data.csv  (EuRoC/ASL)
       <out>/mav0/lidar0/data.csv   every lidar sample: timestamp, angle, range, quality, start flag
       <out>/<name>.bag  (ROS1: /cam0/image_raw mono8, /imu0, /scan LaserScan per revolution)   with --bag
  5. Optionally (--check-sync) estimate the remaining camera-IMU time offset by
     correlating optical-flow speed with gyro rate. Shake the drone to make this work.

Usage
  python convert.py recordings/20261107_120000 --out data/flight1 --bag --check-sync
  python convert.py <rec> --out data/flight1 --bag --cam-shift-ms 4.2   # apply Kalibr timeshift_cam_imu
"""
import argparse
import binascii
import csv
import json
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pi"))
import lidar as lidar_proto  # noqa: E402  (scan binning shared with the live streamer; no hardware deps)

G = 9.80665
DEG2RAD = np.pi / 180.0
PKT = struct.Struct("<2sHI6fH")  # 34 bytes
SYNC = b"\xA5\x5A"


# ---------------------------------------------------------------- IMU parsing
def read_uart_log(path):
    """Return (stream bytes, end offset of each chunk, arrival ns of each chunk)."""
    raw = open(path, "rb").read()
    hdr = struct.Struct("<QI")
    parts, ends, times = [], [], []
    p = total = 0
    while p + hdr.size <= len(raw):
        t, n = hdr.unpack_from(raw, p)
        p += hdr.size
        if p + n > len(raw):
            break  # truncated final chunk
        parts.append(raw[p:p + n])
        p += n
        total += n
        ends.append(total)
        times.append(t)
    return b"".join(parts), np.asarray(ends, np.int64), np.asarray(times, np.int64)


def parse_packets(stream, ends, times):
    rows, bad = [], 0
    p = stream.find(SYNC)
    while p >= 0 and p + PKT.size <= len(stream):
        body = stream[p + 2:p + 32]
        crc = struct.unpack_from("<H", stream, p + 32)[0]
        if binascii.crc_hqx(body, 0xFFFF) == crc:
            _, seq, t_us, gx, gy, gz, ax, ay, az, _ = PKT.unpack_from(stream, p)
            rows.append((p + PKT.size - 1, seq, t_us, gx, gy, gz, ax, ay, az))
            p = stream.find(SYNC, p + PKT.size)
        else:
            bad += 1
            p = stream.find(SYNC, p + 1)
    if not rows:
        sys.exit("No valid IMU packets found in imu.bin")
    a = np.asarray(rows, dtype=np.float64)
    last_byte = a[:, 0].astype(np.int64)
    chunk = np.searchsorted(ends, last_byte, side="right")  # chunk holding the packet's last byte
    imu = {
        "arr_ns": times[chunk],
        "seq": a[:, 1].astype(np.int64),
        "t_cf_us": np.asarray([r[2] for r in rows], dtype=np.int64),
        "gyro": a[:, 3:6] * DEG2RAD,  # rad/s
        "acc": a[:, 6:9] * G,         # m/s^2
    }
    # unwrap 32-bit microsecond counter and 16-bit sequence
    d = np.diff(imu["t_cf_us"])
    imu["t_cf_us"] = imu["t_cf_us"] + np.concatenate([[0], np.cumsum(d < -(1 << 31))]) * (1 << 32)
    ds = np.diff(imu["seq"])
    imu["seq"] = imu["seq"] + np.concatenate([[0], np.cumsum(ds < -(1 << 15))]) * (1 << 16)
    return imu, bad


# ---------------------------------------------------------------- clock fit
def fit_clock(t_cf_us, arr_ns, window_s=1.0):
    """Fit arr = t0 + (1+drift)*t_cf + latency, latency >= 0, via the lower envelope.

    Returns a function mapping CF microseconds -> Pi ns, and diagnostics.
    """
    x = (t_cf_us - t_cf_us[0]) * 1e-6               # s, CF clock
    y = (arr_ns - arr_ns[0]) * 1e-9                  # s, Pi clock
    r = y - x
    win = np.floor(x / window_s).astype(np.int64)
    order = np.lexsort((r, win))
    first = np.concatenate([[True], win[order][1:] != win[order][:-1]])
    idx = order[first]                               # min residual per window
    keep = np.ones(len(idx), bool)
    for _ in range(3):
        A = np.c_[np.ones(keep.sum()), x[idx][keep]]
        coef, *_ = np.linalg.lstsq(A, r[idx][keep], rcond=None)
        e = r[idx] - (coef[0] + coef[1] * x[idx])
        mad = np.median(np.abs(e[keep] - np.median(e[keep]))) + 1e-6
        keep = e < np.median(e[keep]) + 4 * 1.4826 * mad  # one-sided: drop late outliers
    a, b = coef
    latency = r - (a + b * x)
    diag = {
        "drift_ppm": float(b * 1e6),
        "offset_s": float(a),
        "envelope_points": int(keep.sum()),
        "envelope_rms_us": float(np.sqrt(np.mean(e[keep] ** 2)) * 1e6),
        "latency_ms_p50": float(np.percentile(latency, 50) * 1e3),
        "latency_ms_p99": float(np.percentile(latency, 99) * 1e3),
    }
    t_cf0, arr0 = int(t_cf_us[0]), int(arr_ns[0])

    def cf_to_pi_ns(t_us):
        xx = (np.asarray(t_us, np.int64) - t_cf0) * 1e-6
        return arr0 + np.round((xx + a + b * xx) * 1e9).astype(np.int64)

    return cf_to_pi_ns, diag, (x, r, x[idx][keep], a + b * x[idx][keep])


# ---------------------------------------------------------------- lidar
LIDAR_NODE = 5         # bytes per RPLIDAR standard-scan sample
LIDAR_MIN_SEGMENT = 2000  # samples (~0.4 s); shorter runs between stream glitches are dropped


def parse_lidar(stream, ends, times):
    """Decode RPLIDAR standard-scan samples. Returns dict of arrays + number of resync bytes.

    Sample layout: b0 = quality<<2 | !S<<1 | S, b1 = angle_q6[6:0]<<1 | 1, b2 = angle_q6[14:7], b3..4 = dist_q2."""
    a = np.frombuffer(stream, np.uint8)
    n = len(a)
    off, rows, bad, i = [], [], 0, 0
    while i + LIDAR_NODE <= n:
        b0, b1 = a[i], a[i + 1]
        if (b0 & 1) == ((b0 >> 1) & 1) or not (b1 & 1):
            bad += 1
            i += 1
            continue
        off.append(i)
        i += LIDAR_NODE
    if not off:
        return None, bad
    o = np.asarray(off, np.int64)
    b0, b1, b2 = a[o].astype(np.int64), a[o + 1].astype(np.int64), a[o + 2].astype(np.int64)
    dist = a[o + 3].astype(np.int64) | (a[o + 4].astype(np.int64) << 8)
    chunk = np.searchsorted(ends, o + LIDAR_NODE - 1, side="right")
    return {
        "offset": o,
        "arr_ns": times[chunk],
        "start": (b0 & 1).astype(bool),
        "quality": b0 >> 2,
        "angle_deg": ((b2 << 7) | (b1 >> 1)) / 64.0,
        "range_m": dist / 4000.0,  # 0 = no return
    }, bad


def time_lidar(lid):
    """Timestamp each sample on the Pi clock.

    Within an unbroken run of samples, sample k was measured at t0 + k / rate. Treat k as the lidar's clock
    and fit it to arrival times with the same lower-envelope fit as the IMU (removes USB/CP2102 batching
    latency). A gap in the byte stream starts a new run with its own fit."""
    brk = np.flatnonzero(np.diff(lid["offset"]) != LIDAR_NODE) + 1
    segs = np.split(np.arange(len(lid["offset"])), brk)
    t = np.full(len(lid["offset"]), -1, np.int64)
    diags = []
    total_s = (lid["arr_ns"][-1] - lid["arr_ns"][0]) * 1e-9
    nominal_rate = len(lid["offset"]) / max(total_s, 1e-3)
    for seg in segs:
        if len(seg) < LIDAR_MIN_SEGMENT:
            continue
        k_us = np.round(np.arange(len(seg)) * (1e6 / nominal_rate)).astype(np.int64)
        f, diag, _ = fit_clock(k_us, lid["arr_ns"][seg])
        t[seg] = f(k_us)
        diag["samples"] = int(len(seg))
        diag["sample_rate_hz"] = float(nominal_rate / (1 + diag["drift_ppm"] * 1e-6))
        diags.append(diag)
    keep = t >= 0
    out = {k: v[keep] for k, v in lid.items()}
    out["t_ns"] = t[keep]
    return out, {"segments": len(segs), "segments_used": len(diags),
                 "samples_dropped_short_segments": int((~keep).sum()), "fits": diags}


def lidar_scans(lid):
    """Split samples into full revolutions at the start flags. Returns list of index arrays."""
    starts = np.flatnonzero(lid["start"])
    scans = []
    for a, b in zip(starts[:-1], starts[1:]):
        # a revolution must be contiguous in time (no dropped segment in between)
        dt = np.diff(lid["t_ns"][a:b])
        if b - a > 50 and (len(dt) == 0 or dt.max() < 20 * np.median(dt)):
            scans.append(np.arange(a, b))
    return scans


# ---------------------------------------------------------------- camera
def read_frames(rec, meta):
    with open(os.path.join(rec, "frames.csv")) as f:
        rows = list(csv.DictReader(f))
    ts = np.array([int(r["sensor_ts_ns"]) for r in rows], np.int64)
    recv = np.array([int(r["recv_ns"]) for r in rows], np.int64)
    w, h = meta["width"], meta["height"]
    n_bin = os.path.getsize(os.path.join(rec, "frames.bin")) // (w * h)
    n = min(len(rows), n_bin)
    imgs = np.memmap(os.path.join(rec, "frames.bin"), np.uint8, "r", shape=(n, h, w))
    return ts[:n], recv[:n], imgs


# ---------------------------------------------------------------- sync check
def flow_speed(imgs, step=1):
    """Median feature displacement per frame pair (pixels), on a downscaled image."""
    import cv2
    out = np.full(len(imgs), np.nan)
    prev = None
    for k in range(0, len(imgs), step):
        g = cv2.resize(np.asarray(imgs[k]), None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        if prev is not None:
            pts = cv2.goodFeaturesToTrack(prev, 150, 0.01, 8)
            if pts is not None and len(pts) >= 10:
                nxt, st, _ = cv2.calcOpticalFlowPyrLK(prev, g, pts, None, winSize=(21, 21), maxLevel=3)
                ok = st.ravel() == 1
                if ok.sum() >= 10:
                    out[k] = np.median(np.linalg.norm((nxt - pts)[ok].reshape(-1, 2), axis=1))
        prev = g
    return out


def check_sync(t_cam, imgs, t_imu, gyro, max_lag_s=0.3):
    """Return lag (s) to ADD to camera timestamps so they line up with the IMU."""
    speed = flow_speed(imgs)
    dt = np.diff(t_cam) * 1e-9
    v = speed[1:] / dt                                    # px/s over interval k-1..k
    tm = (t_cam[1:] + t_cam[:-1]) * 0.5e-9                 # interval midpoints, s
    good = np.isfinite(v)
    v, tm = v[good], tm[good]
    ti = t_imu * 1e-9
    wn = np.linalg.norm(gyro, axis=1)
    # smooth gyro over one frame interval to match the flow integration window
    k = max(1, int(round(np.median(dt) / np.median(np.diff(ti)))))
    wn = np.convolve(wn, np.ones(k) / k, mode="same")
    lags = np.arange(-max_lag_s, max_lag_s + 1e-9, 0.001)
    corr = np.array([np.corrcoef(v, np.interp(tm + L, ti, wn))[0, 1] for L in lags])
    i = int(np.nanargmax(corr))
    return float(lags[i]), float(corr[i]), lags, corr


# ---------------------------------------------------------------- writers
def write_euroc(out, t_cam, imgs, t_imu, gyro, acc, write_png=True, lid=None):
    import cv2
    cam_dir = os.path.join(out, "mav0", "cam0", "data")
    imu_dir = os.path.join(out, "mav0", "imu0")
    os.makedirs(cam_dir, exist_ok=True)
    os.makedirs(imu_dir, exist_ok=True)
    with open(os.path.join(out, "mav0", "cam0", "data.csv"), "w", newline="") as f:
        f.write("#timestamp [ns],filename\n")
        for k, t in enumerate(t_cam):
            name = f"{t}.png"
            f.write(f"{t},{name}\n")
            if write_png:
                cv2.imwrite(os.path.join(cam_dir, name), np.asarray(imgs[k]))
    with open(os.path.join(imu_dir, "data.csv"), "w", newline="") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for t, g, a in zip(t_imu, gyro, acc):
            f.write(f"{t},{g[0]:.9f},{g[1]:.9f},{g[2]:.9f},{a[0]:.9f},{a[1]:.9f},{a[2]:.9f}\n")
    if lid is not None:
        lid_dir = os.path.join(out, "mav0", "lidar0")
        os.makedirs(lid_dir, exist_ok=True)
        with open(os.path.join(lid_dir, "data.csv"), "w", newline="") as f:
            f.write("#timestamp [ns],angle [deg clockwise from lidar 0],range [m] (0 = no return),"
                    "quality,scan_start\n")
            for t, ang, r, q, st in zip(lid["t_ns"], lid["angle_deg"], lid["range_m"], lid["quality"],
                                        lid["start"]):
                f.write(f"{t},{ang:.4f},{r:.4f},{q},{int(st)}\n")


def write_bag(path, t_cam, imgs, t_imu, gyro, acc, lid=None, scans=()):
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)
    T = ts.types
    Image, Imu = T["sensor_msgs/msg/Image"], T["sensor_msgs/msg/Imu"]
    Header, Time = T["std_msgs/msg/Header"], T["builtin_interfaces/msg/Time"]
    V3, Q = T["geometry_msgs/msg/Vector3"], T["geometry_msgs/msg/Quaternion"]
    LaserScan = T["sensor_msgs/msg/LaserScan"]
    zero9 = np.zeros(9)
    cov_unknown = np.array([-1.0] + [0.0] * 8)

    def hdr(seq, t, frame):
        return Header(seq=seq, stamp=Time(sec=int(t // 1_000_000_000), nanosec=int(t % 1_000_000_000)),
                      frame_id=frame)

    if os.path.exists(path):
        os.remove(path)
    h, w = imgs.shape[1:]
    events = sorted([(int(t), 0, k) for k, t in enumerate(t_cam)] +
                    [(int(t), 1, k) for k, t in enumerate(t_imu)] +
                    [(int(lid["t_ns"][idx[0]]), 2, k) for k, idx in enumerate(scans)])
    with Writer(path) as wr:
        c_img = wr.add_connection("/cam0/image_raw", Image.__msgtype__, typestore=ts)
        c_imu = wr.add_connection("/imu0", Imu.__msgtype__, typestore=ts)
        c_scan = wr.add_connection("/scan", LaserScan.__msgtype__, typestore=ts) if scans else None
        for t, kind, k in events:
            if kind == 0:
                msg = Image(header=hdr(k, t, "cam0"), height=h, width=w, encoding="mono8",
                            is_bigendian=0, step=w, data=np.ascontiguousarray(imgs[k]).reshape(-1))
                wr.write(c_img, t, ts.serialize_ros1(msg, Image.__msgtype__))
            elif kind == 2:
                wr.write(c_scan, t, ts.serialize_ros1(laser_scan(LaserScan, hdr(k, t, "lidar0"), lid, scans[k]),
                                                      LaserScan.__msgtype__))
            else:
                g, a = gyro[k], acc[k]
                msg = Imu(header=hdr(k, t, "imu0"), orientation=Q(x=0.0, y=0.0, z=0.0, w=1.0),
                          orientation_covariance=cov_unknown,
                          angular_velocity=V3(x=float(g[0]), y=float(g[1]), z=float(g[2])),
                          angular_velocity_covariance=zero9,
                          linear_acceleration=V3(x=float(a[0]), y=float(a[1]), z=float(a[2])),
                          linear_acceleration_covariance=zero9)
                wr.write(c_imu, t, ts.serialize_ros1(msg, Imu.__msgtype__))


def laser_scan(LaserScan, header, lid, idx):
    """One revolution as a LaserScan (see pi/lidar.py bin_scan for the frame convention)."""
    f = lidar_proto.bin_scan(lid["angle_deg"][idx], lid["range_m"][idx], lid["quality"][idx], lid["t_ns"][idx])
    f.pop("stamp_ns")
    return LaserScan(header=header, **f)


def save_plot(path, clock_dbg, sync):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    x, r, xe, fe = clock_dbg
    n = 2 if sync else 1
    fig, ax = plt.subplots(n, 1, figsize=(9, 3.5 * n))
    ax = np.atleast_1d(ax)
    sub = slice(None, None, max(1, len(x) // 20000))
    ax[0].plot(x[sub], (r[sub] - r.min()) * 1e3, ".", ms=1, alpha=0.4, label="arrival - CF time")
    ax[0].plot(xe, (fe - r.min()) * 1e3, "r-", label="lower-envelope fit")
    ax[0].set(xlabel="CF time [s]", ylabel="ms (shifted)", title="IMU clock fit")
    ax[0].legend()
    if sync:
        lag, c, lags, corr = sync
        ax[1].plot(lags * 1e3, corr)
        ax[1].axvline(lag * 1e3, color="r")
        ax[1].set(xlabel="shift added to camera time [ms]", ylabel="corr",
                  title=f"flow speed vs |gyro|: best {lag * 1e3:+.1f} ms (r={c:.2f})")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    return True


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rec", help="recording folder from pi/recorder.py")
    ap.add_argument("--out", required=True)
    ap.add_argument("--bag", action="store_true", help="also write a ROS1 bag (Kalibr / OpenVINS)")
    ap.add_argument("--no-png", action="store_true", help="skip the EuRoC PNG export")
    ap.add_argument("--cam-shift-ms", type=float, default=0.0,
                    help="added to camera timestamps (Kalibr: t_imu = t_cam + timeshift_cam_imu)")
    ap.add_argument("--check-sync", action="store_true")
    ap.add_argument("--no-lidar", action="store_true", help="ignore lidar.bin")
    ap.add_argument("--trim-s", type=float, default=0.0, help="drop this many seconds at both ends")
    args = ap.parse_args()

    meta = json.load(open(os.path.join(args.rec, "meta.json")))
    report = {"recording": os.path.abspath(args.rec)}

    # IMU
    stream, ends, times = read_uart_log(os.path.join(args.rec, "imu.bin"))
    imu, bad = parse_packets(stream, ends, times)
    seq_gaps = np.diff(imu["seq"])
    report["imu"] = {
        "packets": int(len(imu["seq"])), "crc_failures": int(bad),
        "lost_packets": int(np.sum(seq_gaps[seq_gaps > 1] - 1)),
        "rate_hz": float(1e6 / np.median(np.diff(imu["t_cf_us"]))),
    }
    # The Crazyflie only fills the gyro/acc log variables once its sensors are calibrated, which needs
    # it to sit still after power-on. If it never was, every packet is valid but contains zeros.
    zero = np.all(imu["gyro"] == 0, axis=1) & np.all(imu["acc"] == 0, axis=1)
    report["imu"]["zero_samples"] = int(zero.sum())
    if zero.mean() > 0.5:
        sys.exit(f"error: {zero.sum()} of {len(zero)} IMU samples are all zeros. The Crazyflie's sensors were "
                 "not calibrated: power it on lying flat and still for ~3 s before picking it up, then record again.")
    if zero.any():
        print(f"warning: {zero.sum()} IMU samples are all zeros (sensors not yet calibrated at the start?)",
              file=sys.stderr)
    lost = report["imu"]["lost_packets"]
    expected = lost + len(imu["seq"])
    if lost > 0.02 * expected:
        print(f"WARNING: {lost} of {expected} IMU packets ({lost / expected:.0%}) were lost. VIO will drift or "
              "diverge. Losses only while moving usually mean a loose TX2/GND wire: solder or secure it.",
              file=sys.stderr)
    cf_to_pi, clock, clock_dbg = fit_clock(imu["t_cf_us"], imu["arr_ns"])
    report["clock_fit"] = clock
    t_imu = cf_to_pi(imu["t_cf_us"])

    # lidar (optional)
    lid = None
    lidar_path = os.path.join(args.rec, "lidar.bin")
    if not args.no_lidar and os.path.exists(lidar_path) and os.path.getsize(lidar_path) > 0:
        lid_raw, lid_bad = parse_lidar(*read_uart_log(lidar_path))
        if lid_raw is None:
            print("warning: lidar.bin has no valid samples; continuing without lidar", file=sys.stderr)
        else:
            lid, lid_fit = time_lidar(lid_raw)
            report["lidar"] = {"samples": int(len(lid_raw["offset"])), "resync_bytes": int(lid_bad),
                               "info": meta.get("lidar_info"), **lid_fit}
            if len(lid["t_ns"]) == 0:
                print("warning: no usable lidar segment; continuing without lidar", file=sys.stderr)
                lid = None

    # camera
    t_cam, recv, imgs = read_frames(args.rec, meta)
    lag_recv = (recv - t_cam) * 1e-6
    report["camera"] = {
        "frames": int(len(t_cam)),
        "fps": float(1e9 / np.median(np.diff(t_cam))),
        "dropped_frames": int(np.sum(np.round(np.diff(t_cam) / np.median(np.diff(t_cam))) - 1)),
        "recv_minus_sensor_ms_median": float(np.median(lag_recv)),
    }
    if not 0 < np.median(lag_recv) < 500:
        print("WARNING: SensorTimestamp does not look like CLOCK_BOOTTIME "
              f"(recv - sensor = {np.median(lag_recv):.1f} ms). Check libcamera version.")
    t_cam = t_cam + int(round(args.cam_shift_ms * 1e6))

    # trim to the overlap of both streams
    lo = max(t_cam[0], t_imu[0] + 100_000_000) + int(args.trim_s * 1e9)
    hi = min(t_cam[-1], t_imu[-1] - 100_000_000) - int(args.trim_s * 1e9)
    cam_sel = np.flatnonzero((t_cam >= lo) & (t_cam <= hi))
    imu_sel = np.flatnonzero((t_imu >= lo - 500_000_000) & (t_imu <= hi + 500_000_000))
    t_cam, imgs = t_cam[cam_sel], imgs[cam_sel[0]:cam_sel[-1] + 1]
    t_imu, gyro, acc = t_imu[imu_sel], imu["gyro"][imu_sel], imu["acc"][imu_sel]
    report["output"] = {"frames": int(len(t_cam)), "imu": int(len(t_imu)),
                        "duration_s": float((t_cam[-1] - t_cam[0]) * 1e-9)}
    scans = []
    if lid is not None:
        # the lidar starts ~2 s late (motor spin-up), so it does not limit the camera/IMU window
        sel = (lid["t_ns"] >= t_cam[0] - 500_000_000) & (lid["t_ns"] <= t_cam[-1] + 500_000_000)
        lid = {k: v[sel] for k, v in lid.items()}
        scans = lidar_scans(lid)
        if scans:
            per = np.array([(lid["t_ns"][i[-1]] - lid["t_ns"][i[0]]) * 1e-9 * len(i) / (len(i) - 1) for i in scans])
            starts_after = (lid["t_ns"][scans[0][0]] - t_cam[0]) * 1e-9
            report["output"].update({
                "lidar_samples": int(len(lid["t_ns"])), "lidar_scans": len(scans),
                "lidar_scan_hz": float(1 / np.median(per)),
                "lidar_samples_per_scan": float(np.median([len(i) for i in scans])),
                "lidar_valid_fraction": float(np.mean(lid["range_m"] > 0)),
                "lidar_first_scan_after_first_frame_s": float(starts_after),
            })
            if starts_after > 0.5:
                print(f"note: first lidar scan is {starts_after:.1f} s after the first camera frame "
                      "(motor spin-up); use --trim-s if you need all three sensors from the start",
                      file=sys.stderr)
        else:
            print("warning: no complete lidar revolutions inside the output window", file=sys.stderr)

    sync = None
    if args.check_sync:
        sync = check_sync(t_cam, imgs, t_imu, gyro)
        report["sync_check"] = {"suggested_extra_cam_shift_ms": sync[0] * 1e3, "corr": sync[1]}

    os.makedirs(args.out, exist_ok=True)
    write_euroc(args.out, t_cam, imgs, t_imu, gyro, acc, write_png=not args.no_png, lid=lid)
    if args.bag:
        write_bag(os.path.join(args.out, os.path.basename(os.path.normpath(args.out)) + ".bag"),
                  t_cam, imgs, t_imu, gyro, acc, lid=lid, scans=scans)
    save_plot(os.path.join(args.out, "sync.png"), clock_dbg, sync)
    with open(os.path.join(args.out, "report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
