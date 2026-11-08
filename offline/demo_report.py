#!/usr/bin/env python3
"""Demo outputs for one converted run: VIO stats, IMU-only comparison, plots and a side-by-side video.

Reads a dataset folder written by convert.py (+ traj_est.txt from OpenVINS, if it ran):
  <run>/mav0/imu0/data.csv, <run>/mav0/cam0/data.csv + data/*.png, <run>/traj_est.txt

Writes into the same folder
  summary.json      path length, start-to-end gap, drift %, IMU-only end error
  traj_topdown.png  VIO trajectory seen from above, start/end marked
  imu_vs_vio.png    distance from the start over time: IMU-only dead reckoning vs VIO
  demo.mp4          (--video) camera on the left, trajectory drawing itself on the right

IMU-only dead reckoning: gyro bias and gravity are taken from the first still period, then gyro and
accelerometer are integrated with nothing else. It is the "no camera" baseline and drifts within seconds.
Distances from the start are compared rather than positions, so the two estimates' different yaw
frames do not matter. For loop runs (end where you started) the end distance is the loop-closure error.

Usage
  python offline/demo_report.py data/walk1 --video
"""
import argparse
import json
import os
import sys
import warnings

import numpy as np

STILL_WIN_S = 0.5
STILL_GYRO_STD = 0.03   # rad/s, std of |gyro| inside a window to count as still
STILL_ACC_STD = 0.10    # m/s^2, std of |acc|
MIN_AFTER_S = 3.0       # a still period must leave at least this much data after it
MAX_CAL_S = 2.0         # use at most this much of the still period for gyro bias / gravity


# ---------------------------------------------------------------- loading
def load_imu(run):
    d = np.loadtxt(os.path.join(run, "mav0", "imu0", "data.csv"), delimiter=",", comments="#")
    return d[:, 0] * 1e-9, d[:, 1:4], d[:, 4:7]


def load_traj(run):
    path = os.path.join(run, "traj_est.txt")
    if not os.path.isfile(path):
        return None
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")   # header-only file when OpenVINS never initialised
        d = np.loadtxt(path, comments="#", ndmin=2)
    if d.size == 0 or len(d) < 2:
        return None
    return d[:, 0], d[:, 1:4]


def load_frames(run):
    d = np.genfromtxt(os.path.join(run, "mav0", "cam0", "data.csv"), delimiter=",", comments="#",
                      dtype=None, encoding=None)
    ts = np.array([int(r[0]) for r in d], dtype=np.int64) * 1e-9
    files = [os.path.join(run, "mav0", "cam0", "data", str(r[1]).strip()) for r in d]
    return ts, files


# ---------------------------------------------------------------- IMU-only baseline
def find_still(t, gyro, acc):
    """Bias/gravity window (start_idx, end_idx) in the first still period, or None.

    Uses at most MAX_CAL_S of that period (like OpenVINS' static init window) and needs at least
    MIN_AFTER_S of data after it, so a fully still bench recording still shows how the IMU drifts."""
    dt = float(np.median(np.diff(t)))
    n = max(2, int(round(STILL_WIN_S / dt)))
    max_cal = int(round(MAX_CAL_S / dt))
    gn, an = np.linalg.norm(gyro, axis=1), np.linalg.norm(acc, axis=1)
    starts = list(range(0, len(t) - n, n))
    still = [np.std(gn[i:i + n]) < STILL_GYRO_STD and np.std(an[i:i + n]) < STILL_ACC_STD for i in starts]
    k = 0
    while k < len(still):
        if not still[k]:
            k += 1
            continue
        j = k
        while j + 1 < len(still) and still[j + 1]:
            j += 1
        end = min(starts[j] + n, starts[k] + max_cal, len(t) - 1)
        if t[-1] - t[end] >= MIN_AFTER_S:   # enough data left to integrate
            return starts[k], end
        k = j + 1
    return None


def so3_exp(w):
    th = np.linalg.norm(w)
    K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    if th < 1e-9:
        return np.eye(3) + K
    return np.eye(3) + np.sin(th) / th * K + (1 - np.cos(th)) / th ** 2 * K @ K


def dead_reckon(t, gyro, acc, i0, i1):
    """Integrate from i1 on, with bias/gravity from samples i0:i1. Returns (t, p) from i1."""
    bg = gyro[i0:i1].mean(axis=0)
    a0 = acc[i0:i1].mean(axis=0)
    g = np.linalg.norm(a0)
    z = a0 / g                                   # body-frame "up" while still
    x = np.cross([0.0, 1.0, 0.0], z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross([1.0, 0.0, 0.0], z)
    x /= np.linalg.norm(x)
    R = np.vstack([x, np.cross(z, x), z])        # world <- body, with world z up
    gw = np.array([0.0, 0.0, g])
    v, p = np.zeros(3), np.zeros(3)
    out = [p.copy()]
    for k in range(i1, len(t) - 1):
        dt = t[k + 1] - t[k]
        a_w = R @ acc[k] - gw
        p = p + v * dt + 0.5 * a_w * dt * dt
        v = v + a_w * dt
        R = R @ so3_exp((gyro[k] - bg) * dt)
        out.append(p.copy())
    return t[i1:], np.array(out)


# ---------------------------------------------------------------- stats
def path_length(t, p, hz=10.0):
    """Length of the path resampled at `hz`, so estimator jitter does not inflate it."""
    ts = np.arange(t[0], t[-1], 1.0 / hz)
    if len(ts) < 2:
        return 0.0
    q = np.column_stack([np.interp(ts, t, p[:, i]) for i in range(3)])
    return float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())


def summarize(traj, imu_dr):
    s = {}
    if traj is not None:
        t, p = traj
        L = path_length(t, p)
        gap = float(np.linalg.norm(p[-1] - p[0]))
        s["vio"] = {"initialised_at_s": float(t[0]), "duration_s": float(t[-1] - t[0]),
                    "path_length_m": round(L, 3), "start_end_gap_m": round(gap, 3),
                    "drift_pct_if_loop": round(100 * gap / L, 2) if L > 0.5 else None,
                    "max_distance_from_start_m": round(float(np.linalg.norm(p - p[0], axis=1).max()), 3)}
    if imu_dr is not None:
        t, p = imu_dr
        s["imu_only"] = {"start_s": float(t[0]), "duration_s": float(t[-1] - t[0]),
                         "end_distance_from_start_m": round(float(np.linalg.norm(p[-1])), 2)}
        for sec in (5, 10, 30):
            if t[-1] - t[0] >= sec:
                k = np.searchsorted(t, t[0] + sec)
                s["imu_only"][f"distance_after_{sec}s_m"] = round(float(np.linalg.norm(p[k])), 2)
    return s


# ---------------------------------------------------------------- plots
def plot_all(run, traj, imu_dr, s):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if traj is not None:
        t, p = traj
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(p[:, 0], p[:, 1], color="#2a6fdb", lw=2, label="VIO (OpenVINS)")
        ax.plot(*p[0, :2], "o", color="#1a9e55", ms=10, label="start")
        ax.plot(*p[-1, :2], "s", color="#d94b3d", ms=9, label="end")
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        v = s["vio"]
        sub = f"path {v['path_length_m']:.1f} m, start-end gap {v['start_end_gap_m'] * 100:.0f} cm"
        if v["drift_pct_if_loop"] is not None:
            sub += f" ({v['drift_pct_if_loop']:.1f} %)"
        ax.set_title("Trajectory from above\n" + sub, fontsize=11)
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=9)
        fig.tight_layout()
        fig.savefig(os.path.join(run, "traj_topdown.png"), dpi=150)
        plt.close(fig)

    if imu_dr is not None:
        fig, ax = plt.subplots(figsize=(8, 4.5))
        ti, pi = imu_dr
        t0 = min(ti[0], traj[0][0]) if traj is not None else ti[0]
        ax.plot(ti - t0, np.linalg.norm(pi, axis=1), color="#d94b3d", lw=2, label="IMU only (no camera)")
        if traj is not None:
            t, p = traj
            ax.plot(t - t0, np.linalg.norm(p - p[0], axis=1), color="#2a6fdb", lw=2, label="VIO (camera + IMU)")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("distance from start [m]")
        ax.set_title("How far each estimate thinks the drone is from where it started")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left")
        fig.tight_layout()
        fig.savefig(os.path.join(run, "imu_vs_vio.png"), dpi=150)
        plt.close(fig)


# ---------------------------------------------------------------- video
def render_video(run, traj, imu_dr, out_path, speed=1.0):
    import cv2

    ts, files = load_frames(run)
    first = cv2.imread(files[0], cv2.IMREAD_GRAYSCALE)
    h, w = first.shape
    pw = h * 4 // 3                                         # trajectory panel width
    fps = 1.0 / np.median(np.diff(ts)) * speed
    vw = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w + pw, h))

    if traj is not None:
        tt, pp = traj
        xy = pp[:, :2] - pp[0, :2]
        lo, hi = xy.min(axis=0), xy.max(axis=0)
        span = max(float((hi - lo).max()), 0.5)
        margin = 30
        sc = (min(pw, h) - 2 * margin) / span
        ctr = (lo + hi) / 2

        def to_px(q):
            return (int(pw / 2 + (q[0] - ctr[0]) * sc), int(h / 2 - (q[1] - ctr[1]) * sc))
        bar_m = max([b for b in (0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50) if b <= span / 4] or [0.05])
    font = cv2.FONT_HERSHEY_SIMPLEX
    for k, (t, f) in enumerate(zip(ts, files)):
        img = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
        left = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.putText(left, f"t = {t - ts[0]:5.1f} s", (10, 25), font, 0.6, (0, 255, 255), 2)
        panel = np.full((h, pw, 3), 255, np.uint8)
        if traj is None:
            cv2.putText(panel, "no VIO trajectory", (20, h // 2), font, 0.7, (60, 60, 60), 2)
        elif t < tt[0]:
            cv2.putText(panel, "VIO initialising...", (20, h // 2 - 10), font, 0.7, (60, 60, 60), 2)
            cv2.putText(panel, "(hold still, then move)", (20, h // 2 + 20), font, 0.5, (120, 120, 120), 1)
        else:
            n = int(np.searchsorted(tt, t, side="right"))
            pts = np.array([to_px(q) for q in xy[:n]], np.int32)
            cv2.circle(panel, to_px(xy[0]), 7, (85, 158, 26), -1)
            if len(pts) > 1:
                cv2.polylines(panel, [pts], False, (219, 111, 42), 2, cv2.LINE_AA)
            cv2.circle(panel, tuple(pts[-1]), 6, (61, 75, 217), -1)
            L = path_length(tt[:n], pp[:n]) if n > 2 else 0.0
            d = float(np.linalg.norm(pp[n - 1] - pp[0]))
            cv2.putText(panel, f"travelled {L:5.1f} m", (12, 24), font, 0.55, (40, 40, 40), 1, cv2.LINE_AA)
            cv2.putText(panel, f"from start {d:5.2f} m", (12, 46), font, 0.55, (40, 40, 40), 1, cv2.LINE_AA)
            if imu_dr is not None and t >= imu_dr[0][0]:
                ki = min(int(np.searchsorted(imu_dr[0], t)), len(imu_dr[0]) - 1)
                di = float(np.linalg.norm(imu_dr[1][ki]))
                cv2.putText(panel, f"IMU-only says {di:6.1f} m", (12, 68), font, 0.55, (61, 75, 217), 1,
                            cv2.LINE_AA)
            bx, by = 12, h - 15
            cv2.line(panel, (bx, by), (bx + int(bar_m * sc), by), (40, 40, 40), 2)
            label = f"{bar_m:g} m" if bar_m >= 1 else f"{bar_m * 100:g} cm"
            cv2.putText(panel, label, (bx, by - 6), font, 0.45, (40, 40, 40), 1, cv2.LINE_AA)
        cv2.putText(panel, "top view", (pw - 85, 24), font, 0.5, (120, 120, 120), 1, cv2.LINE_AA)
        vw.write(np.hstack([left, panel]))
    vw.release()


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="dataset folder from convert.py (data/<name>)")
    ap.add_argument("--video", action="store_true", help="also render demo.mp4")
    ap.add_argument("--speed", type=float, default=1.0, help="video playback speed factor")
    args = ap.parse_args()

    t, gyro, acc = load_imu(args.run)
    traj = load_traj(args.run)
    still = find_still(t, gyro, acc)
    imu_dr = None
    if still is None:
        print("warning: no still period found; skipping the IMU-only baseline", file=sys.stderr)
    else:
        imu_dr = dead_reckon(t, gyro, acc, *still)
    if traj is None:
        print("warning: no VIO trajectory (traj_est.txt missing or empty: OpenVINS did not initialise)",
              file=sys.stderr)

    s = summarize(traj, imu_dr)
    if still is not None:
        s["still_period_s"] = [round(float(t[still[0]] - t[0]), 2), round(float(t[still[1] - 1] - t[0]), 2)]
    with open(os.path.join(args.run, "summary.json"), "w") as f:
        json.dump(s, f, indent=2)
    plot_all(args.run, traj, imu_dr, s)
    if args.video:
        render_video(args.run, traj, imu_dr, os.path.join(args.run, "demo.mp4"), args.speed)
    print(json.dumps(s, indent=2))


if __name__ == "__main__":
    main()
