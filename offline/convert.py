#!/usr/bin/env python3
"""Convert a Pi recording (pi/recorder.py) into a VIO dataset.

Steps
  1. Parse the Crazyflie IMU packets out of the raw UART log (CRC-checked).
  2. Fit the Crazyflie clock to the Pi clock (offset + drift) using the
     lower envelope of packet arrival times.
  3. Put IMU and camera on the Pi clock (ns) and write
       <out>/mav0/cam0/data/<ts>.png, <out>/mav0/cam0/data.csv, <out>/mav0/imu0/data.csv  (EuRoC/ASL)
       <out>/<name>.bag  (ROS1: /cam0/image_raw mono8, /imu0)   with --bag
  4. Optionally (--check-sync) estimate the remaining camera-IMU time offset by
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
def write_euroc(out, t_cam, imgs, t_imu, gyro, acc, write_png=True):
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


def write_bag(path, t_cam, imgs, t_imu, gyro, acc):
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)
    T = ts.types
    Image, Imu = T["sensor_msgs/msg/Image"], T["sensor_msgs/msg/Imu"]
    Header, Time = T["std_msgs/msg/Header"], T["builtin_interfaces/msg/Time"]
    V3, Q = T["geometry_msgs/msg/Vector3"], T["geometry_msgs/msg/Quaternion"]
    zero9 = np.zeros(9)
    cov_unknown = np.array([-1.0] + [0.0] * 8)

    def hdr(seq, t, frame):
        return Header(seq=seq, stamp=Time(sec=int(t // 1_000_000_000), nanosec=int(t % 1_000_000_000)),
                      frame_id=frame)

    if os.path.exists(path):
        os.remove(path)
    h, w = imgs.shape[1:]
    events = sorted([(int(t), 0, k) for k, t in enumerate(t_cam)] +
                    [(int(t), 1, k) for k, t in enumerate(t_imu)])
    with Writer(path) as wr:
        c_img = wr.add_connection("/cam0/image_raw", Image.__msgtype__, typestore=ts)
        c_imu = wr.add_connection("/imu0", Imu.__msgtype__, typestore=ts)
        for t, kind, k in events:
            if kind == 0:
                msg = Image(header=hdr(k, t, "cam0"), height=h, width=w, encoding="mono8",
                            is_bigendian=0, step=w, data=np.ascontiguousarray(imgs[k]).reshape(-1))
                wr.write(c_img, t, ts.serialize_ros1(msg, Image.__msgtype__))
            else:
                g, a = gyro[k], acc[k]
                msg = Imu(header=hdr(k, t, "imu0"), orientation=Q(x=0.0, y=0.0, z=0.0, w=1.0),
                          orientation_covariance=cov_unknown,
                          angular_velocity=V3(x=float(g[0]), y=float(g[1]), z=float(g[2])),
                          angular_velocity_covariance=zero9,
                          linear_acceleration=V3(x=float(a[0]), y=float(a[1]), z=float(a[2])),
                          linear_acceleration_covariance=zero9)
                wr.write(c_imu, t, ts.serialize_ros1(msg, Imu.__msgtype__))


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
    cf_to_pi, clock, clock_dbg = fit_clock(imu["t_cf_us"], imu["arr_ns"])
    report["clock_fit"] = clock
    t_imu = cf_to_pi(imu["t_cf_us"])

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

    sync = None
    if args.check_sync:
        sync = check_sync(t_cam, imgs, t_imu, gyro)
        report["sync_check"] = {"suggested_extra_cam_shift_ms": sync[0] * 1e3, "corr": sync[1]}

    os.makedirs(args.out, exist_ok=True)
    write_euroc(args.out, t_cam, imgs, t_imu, gyro, acc, write_png=not args.no_png)
    if args.bag:
        write_bag(os.path.join(args.out, os.path.basename(os.path.normpath(args.out)) + ".bag"),
                  t_cam, imgs, t_imu, gyro, acc)
    save_plot(os.path.join(args.out, "sync.png"), clock_dbg, sync)
    with open(os.path.join(args.out, "report.json"), "w") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
