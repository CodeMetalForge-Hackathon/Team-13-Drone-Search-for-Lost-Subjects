#!/usr/bin/env python3
"""2D lidar-inertial odometry for the RPLIDAR C1 + Crazyflie IMU (numpy + OpenCV only, Python 3.8 OK).

Loosely coupled, one revolution (~100 ms, ~500 samples) at a time:
  1. IMU: gravity direction and gyro bias from the first stationary second (recordings start still).
     Yaw rate = bias-corrected gyro projected on the up axis, so it works without a lidar-IMU extrinsic
     as long as the lidar spins about the vertical (rig roughly level).
  2. Predict: yaw from the integrated gyro, position from constant velocity.
  3. Deskew: each sample is moved into the frame at the end of the revolution using the gyro yaw at its
     own timestamp and the current velocity estimate.
  4. Correct: ICP (Gauss-Newton, Huber) against a submap of recent keyframes: point-to-line where the map
     has a wall, point-to-point for isolated returns (chair/desk legs). The prediction enters as a prior,
     so directions the lidar cannot see (along a corridor) follow the IMU/constant-velocity prediction.
     After 5 bad scans in a row the submap is re-anchored at the predicted pose. Nearest neighbours come from an OpenCV distance transform of the
     rasterised submap (O(1) lookups, rebuilt only when a keyframe is added).

Frames: the LIO world frame is the lidar frame at the first scan (x/y in the scan plane, CCW yaw).
Scan convention is sensor_msgs/LaserScan as produced by pi/lidar.py bin_scan (same live and offline).

  lio = LIO2D()
  lio.add_imu(t_ns, gyro_xyz, acc_xyz)            # every IMU sample, in time order
  out = lio.add_scan(stamp_ns, angle_min, angle_increment, time_increment, ranges)
  # out: dict(t_ns, x, y, yaw, pts_world (N,2), ok, n_match, rms, ms) or None while initialising
"""
import math
import time
from collections import deque

import cv2
import numpy as np


def rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class LIO2D:
    def __init__(self, yaw_sign=1.0, min_range=0.35, max_range=12.0, res=0.03, keyframes=40,
                 kf_dist=0.20, kf_yaw_deg=10.0, init_s=1.0, max_iter=15, yaw_prior_sigma_deg=1.0,
                 trans_prior_sigma=0.05):
        self.yaw_sign = yaw_sign
        self.min_range, self.max_range = min_range, max_range
        self.res = res
        self.kf_dist, self.kf_yaw = kf_dist, math.radians(kf_yaw_deg)
        self.init_s, self.max_iter = init_s, max_iter
        self.yaw_prior_sigma = math.radians(yaw_prior_sigma_deg)  # gyro yaw over one revolution
        self.trans_prior_sigma = trans_prior_sigma  # constant-velocity position over one revolution [m]
        # IMU
        self.imu_t = deque()          # ns
        self.imu_w = deque()          # yaw rate about up, rad/s (bias-corrected once initialised)
        self.init_buf = []            # (t, gyro, acc) before initialisation
        self.up = None
        self.bias = None
        # state
        self.pose = None              # x, y, yaw (world) at self.t
        self.t = None
        self.vel = np.zeros(2)        # world m/s
        self.kfs = deque(maxlen=keyframes)   # (pose, world points, world normals)
        self.kf_pose = None
        self.field = None
        self.n_scans = 0
        self.n_bad = 0
        self.n_reanchor = 0

    # ---------------------------------------------------------------- IMU
    @property
    def ready(self):
        return self.up is not None

    def add_imu(self, t_ns, gyro, acc):
        if self.up is None:
            self.init_buf.append((t_ns, gyro, acc))
            if (t_ns - self.init_buf[0][0]) * 1e-9 >= self.init_s:
                g = np.array([b[1] for b in self.init_buf])
                a = np.array([b[2] for b in self.init_buf])
                am = a.mean(0)
                self.up = am / np.linalg.norm(am)
                self.bias = g.mean(0)
                for t, gy, _ in self.init_buf:
                    self._push_w(t, gy)
                self.init_buf = []
            return
        self._push_w(t_ns, gyro)

    def _push_w(self, t_ns, gyro):
        w = self.yaw_sign * float(np.dot(np.asarray(gyro) - self.bias, self.up))
        self.imu_t.append(int(t_ns))
        self.imu_w.append(w)
        while len(self.imu_t) > 5000:  # 10 s at 500 Hz
            self.imu_t.popleft()
            self.imu_w.popleft()

    def _yaw_at(self, t0, ts):
        """Integrated gyro yaw from t0 to each time in ts (ns array), trapezoid on the IMU samples."""
        it = np.fromiter(self.imu_t, np.int64, len(self.imu_t))
        iw = np.fromiter(self.imu_w, np.float64, len(self.imu_w))
        if len(it) < 2:
            return np.zeros(len(ts))
        tt = (it - t0) * 1e-9
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (iw[1:] + iw[:-1]) * np.diff(tt))])
        # cum is the integral from it[0]; shift so the integral is 0 at t0 (np.interp holds the ends
        # constant, so extrapolate with the last rate instead for times past the newest IMU sample)
        x = (np.asarray(ts, np.int64) - t0) * 1e-9
        y = np.interp(x, tt, cum) - np.interp(0.0, tt, cum)
        late = x > tt[-1]
        y[late] += (x[late] - tt[-1]) * iw[-1]
        early = x < tt[0]
        y[early] += (x[early] - tt[0]) * iw[0]
        return y

    # ---------------------------------------------------------------- scan helpers
    @staticmethod
    def normals(p, max_gap=0.15):
        """Normals from neighbours i-2, i+2 in sample order (consecutive samples are adjacent in space).

        The allowed gap grows with range (C1: ~0.7 deg between samples, so ~1.2 cm per metre)."""
        n = np.full_like(p, np.nan)
        if len(p) < 5:
            return n
        a, b, c = p[:-4], p[4:], p[2:-2]
        tng = b - a
        gap = np.maximum(max_gap, 0.04 * np.linalg.norm(c, axis=1))
        ok = (np.linalg.norm(b - c, axis=1) < gap) & (np.linalg.norm(c - a, axis=1) < gap)
        L = np.linalg.norm(tng, axis=1)
        ok &= L > 1e-6
        nn = np.stack([-tng[:, 1], tng[:, 0]], 1) / np.maximum(L, 1e-9)[:, None]
        n[2:-2][ok] = nn[ok]
        return n

    def _build_field(self):
        P = np.concatenate([k[1] for k in self.kfs])
        N = np.concatenate([k[2] for k in self.kfs])
        lo = P.min(0) - 1.0
        hi = P.max(0) + 1.0
        W, H = (np.ceil((hi - lo) / self.res).astype(int) + 1).tolist()
        if W * H > 2500 * 2500:  # keep the raster bounded; points far away do not matter for ICP
            c = np.array(self.pose[:2])
            keep = np.all(np.abs(P - c) < 30.0, axis=1)
            P, N = P[keep], N[keep]
            lo, hi = P.min(0) - 1.0, P.max(0) + 1.0
            W, H = (np.ceil((hi - lo) / self.res).astype(int) + 1).tolist()
        px = np.round((P - lo) / self.res).astype(np.int64)
        # one point per cell (voxel downsample)
        key = px[:, 1] * W + px[:, 0]
        _, first = np.unique(key, return_index=True)
        P, N, px = P[first], N[first], px[first]
        img = np.full((H, W), 255, np.uint8)
        img[px[:, 1], px[:, 0]] = 0
        _, labels = cv2.distanceTransformWithLabels(img, cv2.DIST_L2, cv2.DIST_MASK_5,
                                                   labelType=cv2.DIST_LABEL_PIXEL)
        lab2idx = np.full(int(labels.max()) + 1, -1, np.int64)
        lab2idx[labels[px[:, 1], px[:, 0]]] = np.arange(len(P))
        self.field = (lo, W, H, labels, lab2idx, P, N)

    def _nearest(self, pw):
        lo, W, H, labels, lab2idx, P, N = self.field
        px = np.round((pw - lo) / self.res).astype(np.int64)
        inb = (px[:, 0] >= 0) & (px[:, 0] < W) & (px[:, 1] >= 0) & (px[:, 1] < H)
        idx = np.full(len(pw), -1, np.int64)
        idx[inb] = lab2idx[labels[px[inb, 1], px[inb, 0]]]
        return idx

    def map_points(self):
        return None if self.field is None else self.field[5]

    # ---------------------------------------------------------------- main step
    def add_scan(self, stamp_ns, angle_min, angle_inc, time_inc, ranges):
        if not self.ready or len(self.imu_t) < 2:
            return None
        tic = time.monotonic()
        r = np.asarray(ranges, np.float64)
        n = len(r)
        i = np.arange(n)
        ts = stamp_ns + np.round(i * time_inc * 1e9).astype(np.int64)
        t_end = int(ts[-1])
        ang = angle_min + i * angle_inc
        ok = np.isfinite(r) & (r >= self.min_range) & (r <= self.max_range)
        if ok.sum() < 50:
            return None
        q = np.stack([r * np.cos(ang), r * np.sin(ang)], 1)[ok]
        ts = ts[ok]

        # gyro yaw: from the previous pose time to the end of this scan, and per sample relative to the end
        t_prev = self.t if self.t is not None else t_end
        yaw_end_from_prev = float(self._yaw_at(t_prev, [t_end])[0])
        dyaw = self._yaw_at(t_end, ts)  # yaw(t_i) - yaw(t_end)  (<= 0 ... for CCW motion)
        # deskew into the frame at t_end: rotation from gyro, translation from the velocity estimate
        th_pred = (self.pose[2] if self.pose is not None else 0.0) + yaw_end_from_prev
        v_body = rot(th_pred).T @ self.vel
        c, s = np.cos(dyaw), np.sin(dyaw)
        dt = (ts - t_end) * 1e-9
        qd = np.stack([c * q[:, 0] - s * q[:, 1] + v_body[0] * dt,
                       s * q[:, 0] + c * q[:, 1] + v_body[1] * dt], 1)
        nq = self.normals(qd)

        if self.pose is None:  # first scan defines the world frame
            self.pose, self.t = np.array([0.0, 0.0, 0.0]), t_end
            self._add_kf(qd, nq)
            self.n_scans += 1
            return dict(t_ns=t_end, x=0.0, y=0.0, yaw=0.0, pts_world=qd, ok=True, n_match=len(qd), rms=0.0,
                        ms=(time.monotonic() - tic) * 1e3)

        dtp = (t_end - self.t) * 1e-9
        pred = np.array([self.pose[0] + self.vel[0] * dtp, self.pose[1] + self.vel[1] * dtp, th_pred])
        x = pred.copy()
        qs = qd
        gate = 0.5
        n_match, rms, frac, converged = 0, float("nan"), 0.0, False
        # information (1/sigma^2) of the prediction, in units of the per-point noise sigma_pt: in a corridor
        # the along-axis direction has no lidar information and follows the prediction instead of drifting
        sig_pt = 0.03
        prior = np.diag([(sig_pt / self.trans_prior_sigma) ** 2] * 2 + [(sig_pt / self.yaw_prior_sigma) ** 2])
        H_lidar = np.zeros((3, 3))
        k = 0.05  # Huber threshold [m]
        for it in range(self.max_iter):
            R = rot(x[2])
            pw = qs @ R.T + x[:2]
            idx = self._nearest(pw)
            m = idx >= 0
            P, N = self.field[5][idx[m]], self.field[6][idx[m]]
            d = pw[m] - P
            sel = np.linalg.norm(d, axis=1) < gate
            if sel.sum() < 20:
                break
            d, N, rq = d[sel], N[sel], (pw[m] - x[:2])[sel]
            dR = np.stack([-rq[:, 1], rq[:, 0]], 1)  # d(R q)/d yaw
            # walls (map point has a normal): point-to-line; posts / chair legs (no normal): point-to-point
            ln = np.isfinite(N[:, 0])
            pp = ~ln
            res = np.concatenate([np.einsum("ij,ij->i", d[ln], N[ln]), d[pp, 0], d[pp, 1]])
            J = np.concatenate([
                np.stack([N[ln, 0], N[ln, 1], np.einsum("ij,ij->i", N[ln], dR[ln])], 1),
                np.stack([np.ones(pp.sum()), np.zeros(pp.sum()), dR[pp, 0]], 1),
                np.stack([np.zeros(pp.sum()), np.ones(pp.sum()), dR[pp, 1]], 1)])
            wt = np.concatenate([np.ones(ln.sum()), np.full(2 * pp.sum(), 0.5)])
            w = wt * np.where(np.abs(res) < k, 1.0, k / np.maximum(np.abs(res), 1e-9))
            e = x - pred
            e[2] = wrap(e[2])
            H_lidar = (J * w[:, None]).T @ J
            Hm = H_lidar + prior
            g = (J * w[:, None]).T @ res + prior @ e
            try:
                dx = -np.linalg.solve(Hm, g)
            except np.linalg.LinAlgError:
                break
            x += dx
            n_match = int(sel.sum())
            inl = np.abs(res) < 2 * k
            rms = float(np.sqrt(np.mean(res[inl] ** 2))) if inl.any() else float("nan")
            frac = float(inl.mean())
            gate = max(0.15, gate * 0.7)
            if np.linalg.norm(dx[:2]) < 1e-4 and abs(dx[2]) < 1e-4:
                converged = True
                break
        # degeneracy: how well the lidar alone constrains the weakest translation direction
        ev = np.linalg.eigvalsh(H_lidar[:2, :2])
        degenerate = bool(ev[0] < 0.05 * max(ev[1], 1e-9))
        good = n_match >= 30 and np.isfinite(rms) and frac > 0.5
        self.n_bad = 0 if good else self.n_bad + 1
        if not good:  # keep going on the IMU yaw + constant-velocity prediction through bad scans
            x = pred
        x[2] = wrap(x[2])
        if dtp > 0:
            v_new = (x[:2] - self.pose[:2]) / dtp
            self.vel = 0.6 * self.vel + 0.4 * v_new if good else 0.9 * self.vel
        self.pose, self.t = x, t_end
        R = rot(x[2])
        pw_all = qd @ R.T + x[:2]
        moved = (np.linalg.norm(x[:2] - self.kf_pose[:2]) > self.kf_dist
                 or abs(wrap(x[2] - self.kf_pose[2])) > self.kf_yaw)
        # after several bad scans in a row, re-anchor the submap at the predicted pose: otherwise the map
        # stays behind and matching never recovers
        reanchor = self.n_bad >= 5
        if (good and moved) or reanchor:
            if reanchor:
                self.n_bad = 0
                self.n_reanchor += 1
            self._add_kf(qd, nq)
        self.n_scans += 1
        return dict(t_ns=t_end, x=float(x[0]), y=float(x[1]), yaw=float(x[2]), pts_world=pw_all, ok=bool(good),
                    n_match=n_match, rms=rms, converged=converged, degenerate=degenerate, reanchors=self.n_reanchor,
                    ms=(time.monotonic() - tic) * 1e3)

    def _add_kf(self, q_body, n_body):
        R = rot(self.pose[2])
        self.kfs.append((self.pose.copy(), q_body @ R.T + self.pose[:2], n_body @ R.T))
        self.kf_pose = self.pose.copy()
        self._build_field()
