#!/usr/bin/env python3
"""VIO (OpenVINS traj_est.txt) vs LIO (live/lio_node.py traj_lio.txt) report for one run folder.

Same alignment and error definitions as the live viewer (live/traj_align.py): rigid 2D alignment of VIO into
the LIO frame + IMU->lidar lever arm, LIO as the reference. Writes into the run folder:
  vio_lio_summary.json   ATE (rmse/mean/median/max/final), heading error, path lengths, scale, drift, lever arm
  vio_lio_errors.csv     t, position error, heading error, VIO dz, per LIO pose
  vio_lio.png            top-down paths + error over time

Usage (needs numpy + matplotlib, e.g. inside the cf_vio/openvins container):
  python3 offline/compare_vio_lio.py data/validate_20261108_061640
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "live"))
import traj_align as TA  # noqa: E402


def load_tum(path):
    a = np.loadtxt(path, comments="#", ndmin=2)
    if a.size == 0:
        return None
    yaw = np.array([TA.yaw_from_quat(*q) for q in a[:, 4:8]])
    return a[:, 0], a[:, 1:4], yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="folder with traj_est.txt and traj_lio.txt")
    args = ap.parse_args()
    vio = load_tum(os.path.join(args.run, "traj_est.txt"))
    lio = load_tum(os.path.join(args.run, "traj_lio.txt"))
    if vio is None or lio is None:
        sys.exit(f"missing trajectory: VIO poses {0 if vio is None else len(vio[0])}, "
                 f"LIO poses {0 if lio is None else len(lio[0])}")
    vt, vxyz, vyaw = vio
    lt, lxyz, lyaw = lio
    r = TA.compare(lt, lxyz[:, :2], lyaw, vt, vxyz, vyaw)
    if r is None:
        sys.exit("VIO and LIO do not overlap enough to compare")
    t0 = lt[0]
    summary = {k: v for k, v in r.items() if isinstance(v, (int, float))}
    summary.update({
        "lever_arm_m": r["align"]["l"].tolist(), "sim2_scale": r["align"]["scale"],
        "theta_deg": math.degrees(r["align"]["theta"]), "mount_yaw_offset_deg": math.degrees(r["yaw_offset"]),
        "vio_poses": int(len(vt)), "lio_poses": int(len(lt)),
        "vio_start_after_lio_s": float(vt[0] - t0), "duration_s": float(lt[-1] - t0),
    })
    st = os.path.join(args.run, "lio_stats.csv")
    if os.path.exists(st):
        s = np.genfromtxt(st, delimiter=",", names=True)
        summary["lio"] = {"scans": int(len(s)), "rejected_scans": int((s["ok"] == 0).sum()),
                          "icp_rms_cm_median": float(np.nanmedian(s["rms_m"]) * 100),
                          "reanchors": int(s["reanchors"].max()) if len(s) else 0,
                          "ms_mean": float(s["ms"].mean()), "ms_p95": float(np.percentile(s["ms"], 95))}
    with open(os.path.join(args.run, "vio_lio_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    with open(os.path.join(args.run, "vio_lio_errors.csv"), "w") as f:
        f.write("t_s,pos_err_m,yaw_err_deg,vio_dz_m\n")
        for t, e, y, z in zip(r["t"] - t0, r["e_pos"], np.degrees(r["e_yaw"]), r["dz"]):
            f.write(f"{t:.3f},{e:.4f},{y:.3f},{z:.4f}\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    vio_xy = TA.transform_vio(r["align"], vxyz[:, :2])
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.1, 1])
    ax = fig.add_subplot(gs[:, 0])
    ax.plot(lxyz[:, 0], lxyz[:, 1], color="#d95926", lw=2.2, label="LIO (reference)")
    ax.plot(vio_xy[:, 0], vio_xy[:, 1], color="#3987e5", lw=1.6, label="VIO (aligned)")
    ax.plot(lxyz[0, 0], lxyz[0, 1], "ko", ms=6)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="best")
    ax.set(xlabel="x [m]", ylabel="y [m]", title=f"{os.path.basename(os.path.normpath(args.run))}: "
           f"ATE RMSE {r['ate_rmse']:.2f} m over {r['len_lio']:.1f} m")
    tt = r["t"] - t0
    for i, (y, lab) in enumerate([(r["e_pos"], "position error [m]"), (np.degrees(r["e_yaw"]), "heading error [deg]"),
                                  (r["dz"], "VIO height change [m]")]):
        a = fig.add_subplot(gs[i, 1])
        a.plot(tt, y, color="#3987e5", lw=1.5)
        a.set_ylabel(lab)
        a.grid(alpha=0.3)
        if i == 2:
            a.set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(args.run, "vio_lio.png"), dpi=110)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
