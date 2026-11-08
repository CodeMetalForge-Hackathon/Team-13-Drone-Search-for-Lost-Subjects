#!/usr/bin/env python3
"""Is the VIO error just a scale error? Re-score validation runs with the VIO scale corrected.

For each run folder (live/validate.sh output: traj_est.txt + traj_lio.txt) computes the VIO-vs-LIO error
(live/traj_align.py, LIO = reference) three ways:
  rigid    SE(2) + lever arm, VIO scale untouched (what validate.sh reports)
  per-run  best constant scale for that run alone (Sim(2) ATE)
  common   one scale shared by all runs of the same OpenVINS config (folder suffix), each run keeping its own
           rotation/translation/lever arm. If this is close to per-run, the scale error is a stable
           calibration/initialisation bias rather than something random per run.
and the scale fitted on 15 s windows, which shows whether the scale is constant or drifts during a run.

Writes <first run's parent>/scale_study.json and scale_study.png.

Usage (numpy + matplotlib, e.g. in the cf_vio/openvins container):
  python3 offline/vio_scale_study.py data/validate_*
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "live"))
import traj_align as TA  # noqa: E402
from compare_vio_lio import load_tum  # noqa: E402


def load_run(d):
    vt, vxyz, vyaw = load_tum(os.path.join(d, "traj_est.txt"))
    lt, lxyz, lyaw = load_tum(os.path.join(d, "traj_lio.txt"))
    return dict(name=os.path.basename(os.path.normpath(d)), config=os.path.basename(os.path.normpath(d)).split("_")[-1],
                lio=(lt, lxyz[:, :2], lyaw), vio=(vt, vxyz, vyaw))


def sse_at(run, s):
    t, b, psi, xyz, _ = run["pairs"]
    a = xyz[:, :2] * s
    al = TA.align(a, b, psi)
    return float(np.sum(TA.errors(a, b, psi, np.zeros(len(a)), al)[0] ** 2))


def common_scale(runs, lo=0.2, hi=5.0, iters=40):
    g = (math.sqrt(5) - 1) / 2
    x0, x1 = math.log(lo), math.log(hi)
    f = lambda ls: sum(sse_at(r, math.exp(ls)) for r in runs)  # noqa: E731
    c, d = x1 - g * (x1 - x0), x0 + g * (x1 - x0)
    fc, fd = f(c), f(d)
    for _ in range(iters):
        if fc < fd:
            x1, d, fd = d, c, fc
            c = x1 - g * (x1 - x0)
            fc = f(c)
        else:
            x0, c, fc = c, d, fd
            d = x0 + g * (x1 - x0)
            fd = f(d)
    return math.exp((x0 + x1) / 2)


def window_scales(run, win=15.0, min_len=2.0):
    """Scale fitted on consecutive windows (needs >= min_len m of LIO motion in the window)."""
    t, b, psi, xyz, _ = run["pairs"]
    out = []
    for t0 in np.arange(t[0], t[-1] - win / 2, win):
        m = (t >= t0) & (t < t0 + win)
        if m.sum() < 20 or TA.path_length(b[m]) < min_len:
            continue
        a = xyz[m, :2]
        # within a short window, a rigid fit of the scaled VIO is enough (no lever arm: too little rotation)
        s = TA.fit_scale(a, b[m], psi[m])
        out.append((float(t0 + win / 2 - t[0]), s, TA.path_length(b[m])))
    return out


def score(run, scale):
    r = TA.compare(*run["lio"], *run["vio"], scale=scale)
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", help="run folders (default: data/validate_*)")
    ap.add_argument("--out", default=None, help="output folder (default: parent of the first run)")
    args = ap.parse_args()
    dirs = args.runs or sorted(glob.glob("data/validate_*"))
    runs = [load_run(d) for d in dirs if os.path.exists(os.path.join(d, "traj_lio.txt"))]
    for r in runs:
        r["pairs"] = TA.overlap(*r["lio"], *r["vio"])

    configs = sorted({r["config"] for r in runs})
    common = {c: common_scale([r for r in runs if r["config"] == c]) for c in configs}
    rows = []
    for r in runs:
        res = {"rigid": score(r, 1.0), "per-run": score(r, "fit"), "common": score(r, common[r["config"]])}
        r["res"] = res
        r["win"] = window_scales(r)
        ws = np.array([w[1] for w in r["win"]])
        row = {"run": r["name"], "config": r["config"], "len_lio_m": res["rigid"]["len_lio"]}
        for k, v in res.items():
            row[k] = {"scale": v["scale_applied"], "ate_rmse": v["ate_rmse"], "ate_max": v["ate_max"],
                      "final": v["ate_final"], "drift_pct": v["drift_pct"], "yaw_rmse_deg": v["yaw_rmse_deg"],
                      "len_vio_scaled": v["len_vio_xy"], "z_range_scaled": v["z_range"]}
        row["window_scale"] = {"t_mid_s": [round(w[0], 1) for w in r["win"]], "scale": [round(w[1], 3) for w in r["win"]],
                               "median": float(np.median(ws)) if len(ws) else None,
                               "spread_p10_p90": [float(np.percentile(ws, 10)), float(np.percentile(ws, 90))] if len(ws) else None}
        rows.append(row)

    print(f"{'run':38s} {'mode':8s} {'scale':>6s} {'ATE rmse':>9s} {'max':>7s} {'final':>7s} {'drift':>7s} {'yaw rmse':>8s}")
    for row in rows:
        for k in ("rigid", "per-run", "common"):
            v = row[k]
            print(f"{row['run']:38s} {k:8s} {v['scale']:6.3f} {v['ate_rmse']:8.2f}m {v['ate_max']:6.2f}m "
                  f"{v['final']:6.2f}m {v['drift_pct']:6.1f}% {v['yaw_rmse_deg']:7.1f}°")
        w = row["window_scale"]
        print(f"{'':38s} 15 s window scales: {w['scale']}  (median {w['median']:.2f})")
    print("common scale per config:", {k: round(v, 3) for k, v in common.items()})

    out = args.out or os.path.dirname(os.path.normpath(dirs[0]))
    with open(os.path.join(out, "scale_study.json"), "w") as f:
        json.dump({"common_scale": common, "runs": rows}, f, indent=2)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    col = {"rigid": "#a3a29a", "per-run": "#3987e5", "common": "#1baf7a"}
    fig, axs = plt.subplots(len(runs), 3, figsize=(17, 4.6 * len(runs)), gridspec_kw={"width_ratios": [1, 1.3, 0.9]})
    axs = np.atleast_2d(axs)
    for r, ax in zip(runs, axs):
        lt, lxy, _ = r["lio"]
        vt, vxyz, _ = r["vio"]
        ax[0].plot(lxy[:, 0], lxy[:, 1], color="#d95926", lw=2.4, label="LIO (reference)")
        for k in ("rigid", "common"):
            v = r["res"][k]
            p = TA.transform_vio(v["align"], vxyz[:, :2], v["scale_applied"])
            ax[0].plot(p[:, 0], p[:, 1], color=col[k], lw=1.5, label=f"VIO {k} (scale {v['scale_applied']:.2f})")
        ax[0].set_aspect("equal", adjustable="datalim")
        ax[0].grid(alpha=0.3)
        ax[0].legend(fontsize=8)
        ax[0].set_title(r["name"])
        for k in ("rigid", "per-run", "common"):
            v = r["res"][k]
            ax[1].plot(v["t"] - lt[0], v["e_pos"], color=col[k], lw=1.6,
                       label=f"{k}: scale {v['scale_applied']:.2f}, ATE {v['ate_rmse']:.2f} m")
        ax[1].set(xlabel="time [s]", ylabel="position error vs LIO [m]")
        ax[1].grid(alpha=0.3)
        ax[1].legend(fontsize=8)
        if r["win"]:
            w = np.array(r["win"])
            ax[2].plot(w[:, 0] + (r["pairs"][0][0] - lt[0]), w[:, 1], "o-", color="#3987e5", lw=1.5, ms=6)
        ax[2].axhline(common[r["config"]], color=col["common"], ls="--", lw=1.2, label="common scale")
        ax[2].axhline(1.0, color=col["rigid"], ls=":", lw=1.2, label="no correction")
        ax[2].set(xlabel="time [s]", ylabel="scale fitted per 15 s window", ylim=(0, None))
        ax[2].grid(alpha=0.3)
        ax[2].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "scale_study.png"), dpi=95)
    print("wrote", os.path.join(out, "scale_study.json"), os.path.join(out, "scale_study.png"))


if __name__ == "__main__":
    main()
