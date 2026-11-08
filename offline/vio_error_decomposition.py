#!/usr/bin/env python3
"""Where does the VIO error come from: scale or rotation? And is the IMU itself the problem?

For each validation run (live/validate.sh output; LIO = reference, see live/traj_align.py):

1. IMU sanity, straight from the raw recording (recordings/<stamp>):
     |g| at rest      -> accelerometer scale (should be 9.81)
     gyro bias at rest
     gyro yaw-rate scale vs LIO yaw rate (1 s windows, least squares through 0) -> should be 1.000
     VIO yaw-rate scale vs LIO yaw rate                                          -> should be 1.000
   If the IMU checks out, a VIO scale error is the estimator (initialisation / calibration / observability),
   not a mis-scaled sensor.

2. Correction ladder: VIO error after removing progressively more (each with a free 2D pose + lever arm):
     rigid             nothing (validate.sh)
     +scale            one constant scale
     +scale+yawdrift   plus heading drifting linearly in time (what a gyro bias / extrinsic tilt would do)
     +drifts           plus scale drifting linearly in time (log-linear)

3. "Perfect" reconstructions from 1 s steps of the overlap:
     perfect scale     VIO step directions (VIO's rotation) x LIO step lengths -> only rotation error left
     perfect rotation  LIO step directions x VIO step lengths                  -> only scale error left
     perfect rotation + constant scale   as above, VIO lengths times the best constant scale
   and per-step diagnostics: step-length ratio VIO/LIO and step-direction error over time.

Writes data/error_decomposition.json and data/error_decomposition.png.

Usage (inside cf_vio/openvins: numpy + matplotlib):
  python3 offline/vio_error_decomposition.py [data/validate_* ...]
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "live"))
sys.path.insert(0, HERE)
import convert as C  # noqa: E402
import traj_align as TA  # noqa: E402
from compare_vio_lio import load_tum  # noqa: E402


# ---------------------------------------------------------------- helpers
def nelder_mead(f, x0, step, iters=200, tol=1e-7):
    n = len(x0)
    pts = [np.array(x0, float)] + [np.array(x0, float) + np.eye(n)[i] * step[i] for i in range(n)]
    vals = [f(p) for p in pts]
    for _ in range(iters):
        o = np.argsort(vals)
        pts, vals = [pts[i] for i in o], [vals[i] for i in o]
        if abs(vals[-1] - vals[0]) < tol * (abs(vals[0]) + 1e-12):
            break
        c = np.mean(pts[:-1], 0)
        xr = c + (c - pts[-1])
        fr = f(xr)
        if fr < vals[0]:
            xe = c + 2 * (c - pts[-1])
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = c + 0.5 * (pts[-1] - c)
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                pts = [pts[0]] + [pts[0] + 0.5 * (p - pts[0]) for p in pts[1:]]
                vals = [vals[0]] + [f(p) for p in pts[1:]]
    i = int(np.argmin(vals))
    return pts[i], vals[i]


def warp(a, tt, ls0=0.0, w=0.0, k=0.0):
    """VIO xy with scale exp(ls0 + k t) and an extra heading rotation w t applied about the first pose."""
    a0 = a[0]
    s = np.exp(ls0 + k * tt)[:, None]
    th = w * tt
    c, sn = np.cos(th), np.sin(th)
    d = (a - a0) * s
    return np.stack([c * d[:, 0] - sn * d[:, 1], sn * d[:, 0] + c * d[:, 1]], 1)


def ate(a, b, psi):
    al = TA.align(a, b, psi)
    e = TA.errors(a, b, psi, np.zeros(len(a)), al)[0]
    return e, al


def fit_model(a, b, psi, tt, params):
    """params: subset of ('ls0','w','k'). Returns (best dict, errors)."""
    def unpack(x):
        d = {"ls0": 0.0, "w": 0.0, "k": 0.0}
        d.update(dict(zip(params, x)))
        return d

    def cost(x):
        return float(np.mean(ate(warp(a, tt, **unpack(x)), b, psi)[0] ** 2))
    x0 = [{"ls0": math.log(1.5), "w": 0.0, "k": 0.0}[p] for p in params]
    st = [{"ls0": 0.3, "w": 0.002, "k": 0.005}[p] for p in params]
    best = None
    for start in ([x0, [0.0 if p == "ls0" else v for p, v in zip(params, x0)]] if "ls0" in params else [x0]):
        x, v = nelder_mead(cost, start, st)
        if best is None or v < best[1]:
            best = (x, v)
    d = unpack(best[0])
    e, _ = ate(warp(a, tt, **d), b, psi)
    return d, e


def steps(t, xy, dt=1.0):
    """Indices of samples ~dt apart."""
    idx = [0]
    for i in range(1, len(t)):
        if t[i] - t[idx[-1]] >= dt:
            idx.append(i)
    return np.array(idx)


def rebuild(dirs, lengths, start):
    d = np.stack([np.cos(dirs), np.sin(dirs)], 1) * lengths[:, None]
    return np.vstack([start, start + np.cumsum(d, 0)])


# ---------------------------------------------------------------- IMU checks
def imu_checks(rec, lio_t, lio_yaw, vio_t, vio_yaw):
    s, e, t = C.read_uart_log(os.path.join(rec, "imu.bin"))
    imu, _ = C.parse_packets(s, e, t)
    f, _, _ = C.fit_clock(imu["t_cf_us"], imu["arr_ns"])
    ti = f(imu["t_cf_us"]) * 1e-9
    rest = ti < ti[0] + 5.0  # recordings start with the rig still for ~10 s
    g = imu["acc"][rest].mean(0)
    up = g / np.linalg.norm(g)
    bias = imu["gyro"][rest].mean(0)
    wz = (imu["gyro"] - bias) @ up
    yaw_gyro = np.concatenate([[0], np.cumsum(0.5 * (wz[1:] + wz[:-1]) * np.diff(ti))])
    # 1 s yaw increments on the overlap of all three
    t0, t1 = max(lio_t[0], vio_t[0], ti[0]), min(lio_t[-1], vio_t[-1], ti[-1])
    tw = np.arange(t0, t1, 1.0)
    dl = np.diff(np.interp(tw, lio_t, np.unwrap(lio_yaw)))
    dg = np.diff(np.interp(tw, ti, yaw_gyro))
    dv = np.diff(np.interp(tw, vio_t, np.unwrap(vio_yaw)))
    m = np.abs(dl) > math.radians(3)  # windows with real turning
    k_g = float(dg[m] @ dl[m] / (dl[m] @ dl[m])) if m.sum() > 3 else float("nan")
    k_v = float(dv[m] @ dl[m] / (dl[m] @ dl[m])) if m.sum() > 3 else float("nan")
    tot_l = float(np.sum(dl))
    return {"g_norm_rest": float(np.linalg.norm(g)), "accel_scale": float(np.linalg.norm(g) / 9.80665),
            "accel_std_rest": float(np.linalg.norm(imu["acc"][rest].std(0))),
            "gyro_bias_dps": np.degrees(bias).round(3).tolist(),
            "gyro_yaw_scale_vs_lio": k_g, "vio_yaw_scale_vs_lio": k_v, "turn_windows": int(m.sum()),
            "total_yaw_lio_deg": math.degrees(tot_l), "total_yaw_gyro_deg": math.degrees(float(np.sum(dg))),
            "total_yaw_vio_deg": math.degrees(float(np.sum(dv)))}


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--out", default="data")
    args = ap.parse_args()
    dirs = args.runs or sorted(glob.glob("data/validate_*"))
    results = []
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(len(dirs), 4, figsize=(21, 4.6 * len(dirs)),
                            gridspec_kw={"width_ratios": [1, 1.25, 0.9, 0.9]})
    axs = np.atleast_2d(axs)
    for d, ax in zip(dirs, axs):
        name = os.path.basename(os.path.normpath(d))
        stamp = "_".join(name.split("_")[1:3])
        vt, vxyz, vyaw = load_tum(os.path.join(d, "traj_est.txt"))
        lt, lxyz, lyaw = load_tum(os.path.join(d, "traj_lio.txt"))
        t, b, psi, xyz, vy = TA.overlap(lt, lxyz[:, :2], lyaw, vt, vxyz, vyaw)
        a, tt = xyz[:, :2], t - t[0]
        res = {"run": name}
        rec = os.path.join("recordings", stamp)
        if os.path.exists(os.path.join(rec, "imu.bin")):
            res["imu"] = imu_checks(rec, lt, lyaw, vt, vyaw)

        # ---- correction ladder
        ladder = {}
        e0, _ = ate(a, b, psi)
        ladder["rigid"] = ({}, e0)
        ladder["+scale"] = fit_model(a, b, psi, tt, ("ls0",))
        ladder["+scale+yawdrift"] = fit_model(a, b, psi, tt, ("ls0", "w"))
        ladder["+drifts"] = fit_model(a, b, psi, tt, ("ls0", "w", "k"))

        # ---- step reconstructions (1 s steps)
        si = steps(t, b)
        db, da = np.diff(b[si], axis=0), np.diff(a[si], axis=0)
        Lb, La = np.linalg.norm(db, axis=1), np.linalg.norm(da, axis=1)
        th_b = np.arctan2(db[:, 1], db[:, 0])
        th_a = np.arctan2(da[:, 1], da[:, 0])
        moving = Lb > 0.15
        # rotate VIO directions into the LIO frame with the rigid alignment rotation
        _, al_r = ate(a, b, psi)
        th_a_l = th_a + al_r["theta"]
        dir_err = np.degrees(TA.wrap(th_a_l - th_b))
        dir_err = dir_err - np.degrees(math.atan2(np.sin(np.radians(dir_err[moving])).mean(),
                                                  np.cos(np.radians(dir_err[moving])).mean()))
        ratio = np.where(moving, La / np.maximum(Lb, 1e-9), np.nan)
        s_const = math.exp(ladder["+scale"][0]["ls0"])
        recon = {}
        # perfect scale: VIO directions, LIO lengths (still-steps keep VIO direction, length is LIO's ~0)
        recon["perfect scale (VIO rotation only)"] = rebuild(th_a_l, Lb, b[si[0]])
        recon["perfect rotation (VIO lengths)"] = rebuild(th_b, La, b[si[0]])
        recon["perfect rotation + const scale"] = rebuild(th_b, La * s_const, b[si[0]])
        rec_err = {}
        for k, p in recon.items():
            e, al = ate(p, b[si], psi[si])
            rec_err[k] = (e, TA.transform_vio(al, p))

        # ---- summarise
        res["ladder"] = {k: {"params": {"scale": round(math.exp(v[0].get("ls0", 0.0)), 3),
                                        "yaw_drift_deg_per_min": round(math.degrees(v[0].get("w", 0.0)) * 60, 2),
                                        "scale_drift_pct_per_min": round((math.exp(v[0].get("k", 0.0) * 60) - 1) * 100, 1)},
                             "ate_rmse": float(np.sqrt(np.mean(v[1] ** 2))), "ate_max": float(v[1].max()),
                             "final": float(v[1][-1])} for k, v in ladder.items()}
        res["reconstructions"] = {k: {"ate_rmse": float(np.sqrt(np.mean(v[0] ** 2))), "ate_max": float(v[0].max()),
                                      "final": float(v[0][-1])} for k, v in rec_err.items()}
        res["steps"] = {"step_s": 1.0, "n_moving": int(moving.sum()),
                        "length_ratio_vio_over_lio_median": float(np.nanmedian(ratio)),
                        "length_ratio_p10_p90": [float(np.nanpercentile(ratio, 10)), float(np.nanpercentile(ratio, 90))],
                        "direction_error_deg_rms": float(np.sqrt(np.mean(dir_err[moving] ** 2))),
                        "direction_error_deg_p90_abs": float(np.percentile(np.abs(dir_err[moving]), 90))}
        res["len_lio"] = TA.path_length(b)
        results.append(res)

        # ---- plot
        ax[0].plot(b[:, 0], b[:, 1], color="#d95926", lw=2.6, label="LIO (reference)")
        cols = {"perfect scale (VIO rotation only)": "#3987e5", "perfect rotation (VIO lengths)": "#a3a29a",
                "perfect rotation + const scale": "#1baf7a"}
        for k, (e, p) in rec_err.items():
            ax[0].plot(p[:, 0], p[:, 1], color=cols[k], lw=1.4,
                       label=f"{k}: ATE {np.sqrt(np.mean(e ** 2)):.2f} m")
        ax[0].set_aspect("equal", adjustable="datalim")
        ax[0].grid(alpha=0.3)
        ax[0].legend(fontsize=7.5)
        ax[0].set_title(name)
        lc = {"rigid": "#a3a29a", "+scale": "#3987e5", "+scale+yawdrift": "#d95926", "+drifts": "#1baf7a"}
        for k, (p, e) in ladder.items():
            ax[1].plot(tt, e, color=lc[k], lw=1.5, label=f"{k}: ATE {np.sqrt(np.mean(e ** 2)):.2f} m")
        ax[1].set(xlabel="time since VIO start [s]", ylabel="position error vs LIO [m]", title="correction ladder")
        ax[1].grid(alpha=0.3)
        ax[1].legend(fontsize=8)
        ts = tt[si[1:]]
        ax[2].plot(ts[moving], ratio[moving], ".", color="#3987e5", ms=5)
        ax[2].axhline(1.0, color="#a3a29a", ls=":")
        ax[2].axhline(1 / s_const, color="#1baf7a", ls="--", lw=1, label=f"1 / best const scale ({1 / s_const:.2f})")
        ax[2].set(xlabel="time [s]", ylabel="1 s step length VIO / LIO", ylim=(0, 2), title="scale per step")
        ax[2].grid(alpha=0.3)
        ax[2].legend(fontsize=8)
        ax[3].plot(ts[moving], dir_err[moving], ".", color="#d95926", ms=5)
        ax[3].axhline(0, color="#a3a29a", ls=":")
        ax[3].set(xlabel="time [s]", ylabel="step direction error [deg]", title="rotation per step (aligned)")
        ax[3].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "error_decomposition.png"), dpi=90)
    with open(os.path.join(args.out, "error_decomposition.json"), "w") as f:
        json.dump(results, f, indent=2)

    for r in results:
        print(f"\n== {r['run']}  (LIO path {r['len_lio']:.1f} m)")
        if "imu" in r:
            i = r["imu"]
            print(f"  IMU: |g| at rest {i['g_norm_rest']:.3f} m/s^2 (accel scale {i['accel_scale']:.4f}), "
                  f"gyro bias {i['gyro_bias_dps']} deg/s")
            print(f"       yaw-rate scale vs LIO: gyro {i['gyro_yaw_scale_vs_lio']:.3f}, VIO {i['vio_yaw_scale_vs_lio']:.3f} "
                  f"({i['turn_windows']} turning windows); total yaw LIO {i['total_yaw_lio_deg']:.1f}, "
                  f"gyro {i['total_yaw_gyro_deg']:.1f}, VIO {i['total_yaw_vio_deg']:.1f} deg")
        for k, v in r["ladder"].items():
            p = v["params"]
            print(f"  {k:18s} ATE {v['ate_rmse']:5.2f} m  max {v['ate_max']:5.2f}  final {v['final']:5.2f}   "
                  f"scale {p['scale']:.3f}  yaw drift {p['yaw_drift_deg_per_min']:+.2f} deg/min  "
                  f"scale drift {p['scale_drift_pct_per_min']:+.1f} %/min")
        for k, v in r["reconstructions"].items():
            print(f"  {k:36s} ATE {v['ate_rmse']:5.2f} m  max {v['ate_max']:5.2f}  final {v['final']:5.2f}")
        s = r["steps"]
        print(f"  1 s steps: length ratio VIO/LIO median {s['length_ratio_vio_over_lio_median']:.2f} "
              f"(p10-p90 {s['length_ratio_p10_p90'][0]:.2f}-{s['length_ratio_p10_p90'][1]:.2f}), "
              f"direction error rms {s['direction_error_deg_rms']:.1f} deg (p90 {s['direction_error_deg_p90_abs']:.1f})")


if __name__ == "__main__":
    main()
