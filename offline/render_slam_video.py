#!/usr/bin/env python3
"""Side-by-side SLAM video: for each run, camera (left) + top-down LIO vs calibration-corrected VIO (right).

Rows = runs, played in sync from their first camera frame. The right panel replays, frame by frame:
the lidar map building up, the current scan, the LIO path (reference) and the VIO path after correcting
the calibration error -- the best constant scale plus rotation (alignment + linear heading drift) fitted over
the whole run (offline/vio_error_decomposition.py model "+scale+yawdrift") -- and the remaining position
error over time underneath.

LIO is re-run offline here with live/lio2d.py (same code as the live node) to get the scan points.
Writes raw BGR frames (1920x1080, 30 fps) to stdout or a FIFO for ffmpeg; see usage.

Usage (inside cf_vio/openvins, encoder on the host):
  mkfifo /tmp/frames.raw   # under a folder mounted into the container, e.g. data/
  ffmpeg -f rawvideo -pix_fmt bgr24 -s 1920x1080 -r 30 -i data/frames.raw -c:v libx264 -pix_fmt yuv420p out.mp4 &
  docker run --rm -v $PWD:/data -w /data --entrypoint python3 cf_vio/openvins offline/render_slam_video.py \
    --runs 20261108_061442 20261108_061640 --config crazyflie --out data/frames.raw
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "live"))
sys.path.insert(0, os.path.join(HERE, "..", "pi"))
sys.path.insert(0, HERE)
import convert as C  # noqa: E402
import lidar as LP  # noqa: E402
import traj_align as TA  # noqa: E402
from compare_vio_lio import load_tum  # noqa: E402
from lio2d import LIO2D  # noqa: E402
from vio_error_decomposition import ate, fit_model, warp  # noqa: E402

W, H = 1920, 1080
FPS = 30
BG = (26, 26, 25)
GRID = (44, 42, 40)
LIO_C = (38, 89, 217)      # #d95926
VIO_C = (229, 135, 57)     # #3987e5
SCAN_C = (103, 103, 230)   # #e66767
MAP_C = (170, 172, 172)
FG = (236, 239, 240)
MUT = (154, 162, 163)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def log(*a):
    print(*a, file=sys.stderr, flush=True)


def text(img, s, org, scale=0.6, col=FG, th=1):
    cv2.putText(img, s, org, FONT, scale, (0, 0, 0), th + 3, cv2.LINE_AA)
    cv2.putText(img, s, org, FONT, scale, col, th, cv2.LINE_AA)


# ---------------------------------------------------------------- per-run data
def prepare(stamp, config):
    rec = os.path.join("recordings", stamp)
    meta = json.load(open(os.path.join(rec, "meta.json")))
    s, e, t = C.read_uart_log(os.path.join(rec, "imu.bin"))
    imu, _ = C.parse_packets(s, e, t)
    f, _, _ = C.fit_clock(imu["t_cf_us"], imu["arr_ns"])
    ti = f(imu["t_cf_us"])
    L, _ = C.parse_lidar(*C.read_uart_log(os.path.join(rec, "lidar.bin")))
    L, _ = C.time_lidar(L)
    scans = C.lidar_scans(L)
    # LIO, offline, same code as the live node
    lio = LIO2D()
    ev = sorted([(int(ti[k]), 0, k) for k in range(len(ti))] + [(int(L["t_ns"][i[-1]]), 1, j) for j, i in enumerate(scans)])
    out = []
    for tt, kind, k in ev:
        if kind == 0:
            lio.add_imu(tt, imu["gyro"][k], imu["acc"][k])
            continue
        i = scans[k]
        b = LP.bin_scan(L["angle_deg"][i], L["range_m"][i], L["quality"][i], L["t_ns"][i])
        o = lio.add_scan(b["stamp_ns"], b["angle_min"], b["angle_increment"], b["time_increment"], b["ranges"])
        if o:
            out.append(o)
    lt = np.array([o["t_ns"] * 1e-9 for o in out])
    lxy = np.array([(o["x"], o["y"]) for o in out])
    lyaw = np.array([o["yaw"] for o in out])
    pts = [o["pts_world"][::2] if o["ok"] else np.zeros((0, 2)) for o in out]
    log(f"{stamp}: LIO {len(out)} scans, {sum(not o['ok'] for o in out)} rejected")

    # VIO + calibration correction (scale + rotation incl. linear heading drift), fitted over the run
    vt, vxyz, vyaw = load_tum(os.path.join("data", f"validate_{stamp}_{config}", "traj_est.txt"))
    tp, b, psi, xyz, _ = TA.overlap(lt, lxy, lyaw, vt, vxyz, vyaw)
    a = xyz[:, :2]
    prm, err = fit_model(a, b, psi, tp - tp[0], ("ls0", "w"))
    _, al = ate(warp(a, tp - tp[0], **prm), b, psi)
    # same warp for every VIO pose (origin = VIO at the first matched time, time from that instant)
    va = vxyz[:, :2]
    a0 = a[0]
    sc = math.exp(prm["ls0"])
    th = prm["w"] * (vt - tp[0])
    d = (va - a0) * sc
    vw = np.stack([np.cos(th) * d[:, 0] - np.sin(th) * d[:, 1], np.sin(th) * d[:, 0] + np.cos(th) * d[:, 1]], 1)
    vcorr = vw @ TA.rot2(al["theta"]).T + al["t"]
    rigid_e, _ = ate(a, b, psi)
    log(f"{stamp}: scale {sc:.3f}, heading drift {math.degrees(prm['w']) * 60:+.2f} deg/min, "
        f"ATE {np.sqrt(np.mean(err ** 2)):.2f} m (uncorrected {np.sqrt(np.mean(rigid_e ** 2)):.2f} m)")

    # camera
    rows = [l.split(",") for l in open(os.path.join(rec, "frames.csv")).read().splitlines()[1:]]
    w, h = meta["width"], meta["height"]
    n = min(len(rows), os.path.getsize(os.path.join(rec, "frames.bin")) // (w * h))
    cam_t = np.array([int(r[1]) for r in rows[:n]]) * 1e-9
    frames = np.memmap(os.path.join(rec, "frames.bin"), np.uint8, "r", shape=(n, h, w))
    return dict(stamp=stamp, cam_t=cam_t, frames=frames, lt=lt, lxy=lxy, lyaw=lyaw, pts=pts,
                vt=vt, vcorr=vcorr, ep_t=tp, ep=err, scale=sc, drift=math.degrees(prm["w"]) * 60,
                ate=float(np.sqrt(np.mean(err ** 2))), ate_rigid=float(np.sqrt(np.mean(rigid_e ** 2))),
                len_lio=TA.path_length(b))


# ---------------------------------------------------------------- SLAM panel
class Panel:
    """Live top-down view: auto-zooms (smoothly) to what has been mapped so far."""

    def __init__(self, run, w=960, h=540, strip=130):
        self.r, self.w, self.h, self.mh = run, w, h, h - strip
        lxy = run["lxy"]
        # display rotation: long axis of the LIO path horizontal (panel is wide)
        c = lxy - lxy.mean(0)
        _, evec = np.linalg.eigh(c.T @ c)
        ax = evec[:, 1]
        self.R = np.array([[ax[0], ax[1]], [-ax[1], ax[0]]])
        self.cells, self.keys = [], set()
        self.map_xy = np.zeros((0, 2))
        self.k_scan = 0
        self.view = None  # (cx, cy, metres per pixel) in the rotated frame
        self.emax = max(0.5, float(np.percentile(run["ep"], 99.5)) * 1.15)

    def px(self, xy):
        q = np.asarray(xy) @ self.R.T
        cx, cy, mpp = self.view
        return np.stack([self.w / 2 + (q[:, 0] - cx) / mpp, (self.mh + 40) / 2 - (q[:, 1] - cy) / mpp], 1)

    def _update_view(self, k, kv):
        r = self.r
        pts = [r["lxy"][:k + 1]]
        if kv:
            pts.append(r["vcorr"][:kv])
        if k >= 0 and len(r["pts"][k]):
            sc = r["pts"][k]
            pts.append(sc[np.linalg.norm(sc - r["lxy"][k], axis=1) < 6.0])
        q = np.vstack(pts) @ self.R.T
        lo, hi = q.min(0), q.max(0)
        span_x = max(hi[0] - lo[0], 8.0) * 1.12
        span_y = max(hi[1] - lo[1], 4.5) * 1.12
        tgt = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, max(span_x / self.w, span_y / (self.mh - 70)))
        if self.view is None:
            self.view = tgt
        else:
            a = 0.08
            self.view = tuple(v + (t - v) * a for v, t in zip(self.view, tgt))

    def _grid(self, img):
        cx, cy, mpp = self.view
        step = next(s for s in (1.0, 2.0, 5.0, 10.0, 20.0) if s / mpp >= 70)
        x0, x1 = cx - self.w / 2 * mpp, cx + self.w / 2 * mpp
        y0, y1 = cy - self.mh / 2 * mpp - 1, cy + self.mh / 2 * mpp + 1
        for v in np.arange(math.floor(x0 / step) * step, x1, step):
            X = int(round(self.w / 2 + (v - cx) / mpp))
            cv2.line(img, (X, 0), (X, self.mh), GRID, 1)
        for v in np.arange(math.floor(y0 / step) * step, y1, step):
            Y = int(round((self.mh + 40) / 2 - (v - cy) / mpp))
            cv2.line(img, (0, Y), (self.w, Y), GRID, 1)
        text(img, f"grid {step:g} m", (12, self.mh - 12), 0.5, MUT)

    def render(self, t_abs):
        r = self.r
        new = []
        while self.k_scan < len(r["lt"]) and r["lt"][self.k_scan] <= t_abs:
            if len(r["pts"][self.k_scan]):
                new.append(r["pts"][self.k_scan])
            self.k_scan += 1
        if new:  # map = unique 5 cm cells
            P = np.vstack(new)
            for kk, p in zip(map(tuple, np.round(P / 0.05).astype(np.int64)), P):
                if kk not in self.keys:
                    self.keys.add(kk)
                    self.cells.append(p)
            self.map_xy = np.array(self.cells)
        k = self.k_scan - 1
        kv = int(np.searchsorted(r["vt"], t_abs, side="right"))
        img = np.full((self.h, self.w, 3), BG, np.uint8)
        if k < 0:
            text(img, "waiting for lidar...", (20, 80), 0.6, MUT)
            self._strip(img, t_abs)
            return img
        self._update_view(k, kv)
        self._grid(img)
        # lidar map (2x2 px dots)
        if len(self.map_xy):
            q = np.round(self.px(self.map_xy)).astype(int)
            ok = (q[:, 0] >= 0) & (q[:, 0] < self.w - 1) & (q[:, 1] >= 0) & (q[:, 1] < self.mh - 1)
            q = q[ok]
            for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
                img[q[:, 1] + dy, q[:, 0] + dx] = MAP_C
        # current scan
        if len(r["pts"][k]):
            for x, y in np.round(self.px(r["pts"][k])).astype(int):
                if 0 <= x < self.w and 0 <= y < self.mh:
                    cv2.circle(img, (int(x), int(y)), 3, SCAN_C, -1, cv2.LINE_AA)
        # paths
        if k >= 1:
            cv2.polylines(img, [np.round(self.px(r["lxy"][:k + 1])).astype(np.int32)], False, LIO_C, 4, cv2.LINE_AA)
        if kv >= 2:
            cv2.polylines(img, [np.round(self.px(r["vcorr"][:kv])).astype(np.int32)], False, VIO_C, 2, cv2.LINE_AA)
        s0 = self.px(r["lxy"][:1])[0]
        cv2.circle(img, (int(s0[0]), int(s0[1])), 6, FG, -1, cv2.LINE_AA)
        p = self.px(r["lxy"][k:k + 1])[0]
        hd = self.R @ np.array([math.cos(r["lyaw"][k]), math.sin(r["lyaw"][k])])
        cv2.line(img, (int(p[0]), int(p[1])), (int(p[0] + 28 * hd[0]), int(p[1] - 28 * hd[1])), LIO_C, 3, cv2.LINE_AA)
        cv2.circle(img, (int(p[0]), int(p[1])), 9, LIO_C, -1, cv2.LINE_AA)
        cv2.circle(img, (int(p[0]), int(p[1])), 9, BG, 2, cv2.LINE_AA)
        if kv >= 1:
            q = self.px(r["vcorr"][kv - 1:kv])[0]
            cv2.circle(img, (int(q[0]), int(q[1])), 7, VIO_C, -1, cv2.LINE_AA)
            cv2.circle(img, (int(q[0]), int(q[1])), 7, BG, 2, cv2.LINE_AA)
        # legend
        cv2.rectangle(img, (0, 0), (self.w, 60), BG, -1)
        cv2.line(img, (14, 20), (40, 20), LIO_C, 4, cv2.LINE_AA)
        text(img, "LIO (lidar + IMU, reference)", (48, 25), 0.55)
        cv2.line(img, (14, 44), (40, 44), VIO_C, 3, cv2.LINE_AA)
        text(img, f"VIO, calibration-corrected (x{r['scale']:.2f} scale, rotation fit"
                  f" {r['drift']:+.1f} deg/min)", (48, 49), 0.55)
        if kv == 0:
            text(img, "VIO initialising...", (520, 25), 0.55, MUT)
        cv2.circle(img, (self.w - 170, 20), 4, SCAN_C, -1, cv2.LINE_AA)
        text(img, "scan", (self.w - 160, 25), 0.5, MUT)
        cv2.rectangle(img, (self.w - 108, 16), (self.w - 100, 24), MAP_C, -1)
        text(img, "lidar map", (self.w - 94, 25), 0.5, MUT)
        self._strip(img, t_abs)
        return img

    def _strip(self, img, t_abs):
        r = self.r
        y0, y1, x0, x1 = self.mh + 26, self.h - 22, 70, self.w - 16
        cv2.rectangle(img, (0, self.mh), (self.w, self.h), (32, 33, 33), -1)
        t = r["ep_t"] - r["cam_t"][0]
        T = r["cam_t"][-1] - r["cam_t"][0]
        X = lambda v: (x0 + v / T * (x1 - x0)).astype(np.int32) if isinstance(v, np.ndarray) else int(x0 + v / T * (x1 - x0))  # noqa: E731
        Y = lambda e: (y1 - np.clip(e / self.emax, 0, 1) * (y1 - y0)).astype(np.int32)  # noqa: E731
        for g in np.linspace(0, self.emax, 3):
            yy = int(y1 - g / self.emax * (y1 - y0))
            cv2.line(img, (x0, yy), (x1, yy), GRID, 1)
            text(img, f"{g:.1f}", (22, yy + 5), 0.42, MUT)
        pts = np.stack([X(t), Y(r["ep"])], 1)
        cv2.polylines(img, [pts], False, (90, 70, 50), 1, cv2.LINE_AA)  # whole run, faint
        n = int(np.searchsorted(r["ep_t"], t_abs, side="right"))
        if n >= 2:
            cv2.polylines(img, [pts[:n]], False, VIO_C, 2, cv2.LINE_AA)
        cx = X(t_abs - r["cam_t"][0])
        cv2.line(img, (cx, y0 - 4), (cx, y1), MUT, 1)
        cur = r["ep"][n - 1] if n else None
        so_far = float(np.sqrt(np.mean(r["ep"][:n] ** 2))) if n else None
        text(img, "position error vs LIO [m]" + (f"   now {cur:.2f} m   RMSE so far {so_far:.2f} m" if n else ""),
             (x0, self.mh + 18), 0.5, FG)
        text(img, f"run: RMSE {r['ate']:.2f} m over {r['len_lio']:.0f} m (uncorrected {r['ate_rigid']:.2f} m)",
             (self.w - 430, self.h - 6), 0.45, MUT)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", nargs="+", default=["20261108_061442", "20261108_061640"])
    ap.add_argument("--config", default="crazyflie")
    ap.add_argument("--max-s", type=float, default=0, help="limit duration (testing)")
    ap.add_argument("--out", default="-", help="raw BGR frame sink: '-' = stdout, or a file / FIFO path "
                                               "(much faster than stdout through docker)")
    args = ap.parse_args()
    runs = [prepare(s, args.config) for s in args.runs]
    panels = [Panel(r, 960, H // len(runs)) for r in runs]
    T = max(r["cam_t"][-1] - r["cam_t"][0] for r in runs)
    if args.max_s:
        T = min(T, args.max_s)
    nfr = int(T * FPS)
    rh = H // len(runs)
    log(f"rendering {nfr} frames ({T:.1f} s)")
    out = sys.stdout.buffer if args.out == "-" else open(args.out, "wb", buffering=1 << 24)
    for fi in range(nfr):
        tv = fi / FPS
        frame = np.zeros((H, W, 3), np.uint8)
        for k, (r, p) in enumerate(zip(runs, panels)):
            ta = r["cam_t"][0] + tv
            ci = min(int(np.searchsorted(r["cam_t"], ta)), len(r["cam_t"]) - 1)
            cam = cv2.cvtColor(cv2.resize(np.asarray(r["frames"][ci]), (960, rh), interpolation=cv2.INTER_LINEAR),
                               cv2.COLOR_GRAY2BGR)
            text(cam, f"{r['stamp']}  camera", (14, 30), 0.7, FG, 2)
            text(cam, f"t = {min(tv, r['cam_t'][-1] - r['cam_t'][0]):5.1f} s", (14, rh - 16), 0.65, FG, 2)
            frame[k * rh:(k + 1) * rh, :960] = cam
            frame[k * rh:(k + 1) * rh, 960:] = p.render(ta)
        cv2.line(frame, (0, rh), (W, rh), (70, 70, 70), 2)
        cv2.line(frame, (960, 0), (960, H), (70, 70, 70), 2)
        out.write(frame.tobytes())
        if fi % 300 == 0:
            log(f"  frame {fi}/{nfr}")
    out.flush()


if __name__ == "__main__":
    main()
