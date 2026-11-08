#!/usr/bin/env python3
"""VIO vs LIO comparison, shared by the live viewer (live_node.py) and the report (offline/compare_vio_lio.py).

LIO is the reference: a 2D scan-matcher against walls drifts far less than monocular VIO. The two run in
different world frames (VIO: gravity-aligned, yaw from its initialisation; LIO: the lidar frame at the first
scan), and they track different points on the rig (VIO: the IMU; LIO: the lidar centre). So, per pair i of
time-matched poses (VIO position a_i in x/y, LIO position b_i and heading psi_i):

    b_i ~= R(theta) a_i + t + R(psi_i) l

theta, t: the rigid 2D transform VIO world -> LIO world; l: the IMU->lidar lever arm in the lidar frame
(observable once the rig has turned; ridge-regularised towards 0 before that). Solved by alternating a
closed-form 2D Umeyama for theta with linear least squares for (t, l). Fitting over all pairs gives the
usual ATE (absolute trajectory error). Heading error = psi_i - (vio_yaw_i + theta) minus its circular mean
(the constant IMU/lidar mounting yaw).
"""
import math

import numpy as np


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def rot2(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def _umeyama2(a, b, with_scale=False):
    ma, mb = a.mean(0), b.mean(0)
    A, B = a - ma, b - mb
    C = B.T @ A                                  # sum b a^T
    th = math.atan2(C[1, 0] - C[0, 1], C[0, 0] + C[1, 1])
    R = rot2(th)
    s = 1.0
    if with_scale:
        va = (A ** 2).sum()
        s = float(np.trace(R.T @ C) / va) if va > 1e-9 else 1.0
    return th, mb - s * R @ ma, s


def interp_vio(vio_t, vio_xyz, vio_yaw, t):
    """VIO position and (unwrapped) yaw at times t (all arrays). Returns xyz (n,3), yaw (n,)."""
    xyz = np.stack([np.interp(t, vio_t, vio_xyz[:, k]) for k in range(3)], 1)
    yaw = np.interp(t, vio_t, np.unwrap(vio_yaw))
    return xyz, yaw


def align(a, b, psi, lever=True, ridge=2.0, max_lever=0.5):
    """a: VIO xy (n,2), b: LIO xy (n,2), psi: LIO yaw (n,). Returns dict(theta, t, l, scale)."""
    n = len(a)
    l = np.zeros(2)
    th, t, _ = _umeyama2(a, b)
    if lever and n >= 10:
        c, s = np.cos(psi), np.sin(psi)
        for _ in range(3):
            # fix theta: unknowns z = [tx, ty, lx, ly];  b - R a = t + R(psi) l
            y = (b - a @ rot2(th).T).reshape(-1)
            A = np.zeros((2 * n, 4))
            A[0::2, 0] = 1
            A[1::2, 1] = 1
            A[0::2, 2], A[0::2, 3] = c, -s
            A[1::2, 2], A[1::2, 3] = s, c
            reg = np.diag([0, 0, ridge, ridge])
            z = np.linalg.solve(A.T @ A + reg, A.T @ y)
            l = z[2:]
            nl = np.linalg.norm(l)
            if nl > max_lever:
                l = l * (max_lever / nl)
            bl = b - np.stack([c * l[0] - s * l[1], s * l[0] + c * l[1]], 1)
            th, t, _ = _umeyama2(a, bl)
    else:
        bl = b
    _, _, scale = _umeyama2(a, bl if lever and n >= 10 else b, with_scale=True)
    return {"theta": float(th), "t": t, "l": l, "scale": float(scale)}


def errors(a, b, psi, vio_yaw, al):
    """Per-pair position error [m] (VIO prediction of the lidar position vs LIO) and heading error [rad]."""
    R = rot2(al["theta"])
    l = al["l"]
    c, s = np.cos(psi), np.sin(psi)
    pred = a @ R.T + al["t"] + np.stack([c * l[0] - s * l[1], s * l[0] + c * l[1]], 1)
    e_pos = np.linalg.norm(pred - b, axis=1)
    d = wrap(psi - (vio_yaw + al["theta"]))
    off = math.atan2(np.sin(d).mean(), np.cos(d).mean())
    e_yaw = wrap(d - off)
    return e_pos, e_yaw, float(off), pred


def path_length(xy):
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()) if len(xy) > 1 else 0.0


def fit_scale(a, b, psi, lo=0.2, hi=5.0, iters=40):
    """Best constant VIO scale s (Sim(2) + lever arm): golden-section search on log s of the ATE RMSE."""
    def cost(ls):
        sa = a * math.exp(ls)
        al = align(sa, b, psi)
        return float(np.mean(errors(sa, b, psi, np.zeros(len(a)), al)[0] ** 2))
    g = (math.sqrt(5) - 1) / 2
    x0, x1 = math.log(lo), math.log(hi)
    c, d = x1 - g * (x1 - x0), x0 + g * (x1 - x0)
    fc, fd = cost(c), cost(d)
    for _ in range(iters):
        if fc < fd:
            x1, d, fd = d, c, fc
            c = x1 - g * (x1 - x0)
            fc = cost(c)
        else:
            x0, c, fc = c, d, fd
            d = x0 + g * (x1 - x0)
            fd = cost(d)
    return math.exp((x0 + x1) / 2)


def overlap(lio_t, lio_xy, lio_yaw, vio_t, vio_xyz, vio_yaw):
    """Time-matched pairs: (t, LIO xy, LIO yaw, VIO xyz, VIO yaw) at the LIO times inside the VIO span."""
    lio_t, vio_t = np.asarray(lio_t, np.float64), np.asarray(vio_t, np.float64)
    m = (lio_t >= vio_t[0]) & (lio_t <= vio_t[-1])
    t = lio_t[m]
    xyz, vyaw = interp_vio(vio_t, np.asarray(vio_xyz), np.asarray(vio_yaw), t)
    return t, np.asarray(lio_xy)[m], np.asarray(lio_yaw)[m], xyz, vyaw


def compare(lio_t, lio_xy, lio_yaw, vio_t, vio_xyz, vio_yaw, min_spread=1.0, scale=1.0):
    """Full comparison on the overlap. Returns None until there is enough motion to fix the rotation.

    scale: VIO positions are multiplied by this before aligning (1 = rigid / SE(2) ATE as usual,
    'fit' = best constant scale for this run, i.e. Sim(2) ATE)."""
    lio_t, vio_t = np.asarray(lio_t, np.float64), np.asarray(vio_t, np.float64)
    if len(vio_t) < 5 or len(lio_t) < 5:
        return None
    m = (lio_t >= vio_t[0]) & (lio_t <= vio_t[-1])
    if m.sum() < 10:
        return None
    t, b, psi = lio_t[m], np.asarray(lio_xy)[m], np.asarray(lio_yaw)[m]
    xyz, vyaw = interp_vio(vio_t, np.asarray(vio_xyz), np.asarray(vio_yaw), t)
    if np.linalg.norm(b - b.mean(0), axis=1).max() < min_spread:
        return None
    if scale == "fit":
        scale = fit_scale(xyz[:, :2], b, psi)
    xyz = xyz * scale  # about the VIO origin; the alignment translation absorbs the choice of centre
    a = xyz[:, :2]
    al = align(a, b, psi)
    e_pos, e_yaw, yaw_off, pred = errors(a, b, psi, vyaw, al)
    L_lio, L_vio = path_length(b), path_length(a)
    return {
        "t": t, "e_pos": e_pos, "e_yaw": e_yaw, "dz": xyz[:, 2] - xyz[0, 2], "vio_in_lio": pred,
        "align": al, "yaw_offset": yaw_off,
        "ate_rmse": float(np.sqrt(np.mean(e_pos ** 2))), "ate_mean": float(e_pos.mean()),
        "ate_median": float(np.median(e_pos)), "ate_max": float(e_pos.max()), "ate_final": float(e_pos[-1]),
        "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(e_yaw ** 2)))),
        "yaw_max_deg": float(np.degrees(np.abs(e_yaw).max())),
        "len_lio": L_lio, "len_vio_xy": L_vio,
        "len_ratio": L_vio / L_lio if L_lio > 0 else float("nan"),
        "drift_pct": 100.0 * float(e_pos[-1]) / L_lio if L_lio > 0 else float("nan"),
        "z_range": float(np.ptp(xyz[:, 2])),
        "n": int(len(t)), "scale_applied": float(scale),
    }


def transform_vio(al, xy, scale=1.0):
    """VIO xy (IMU) into the LIO world frame (for drawing); scale = compare()'s scale_applied."""
    return (np.asarray(xy) * scale) @ rot2(al["theta"]).T + al["t"]
