"""Live matplotlib display: Doppler spectrogram, motion metric, breathing waveform, state."""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from .detector import EMPTY, MOVING, STILL, Pipeline, Status, format_status
from .dsp import doppler_spectrogram
from .radio import Source

STATE_COLORS = {EMPTY: "#6b7280", MOVING: "#dc2626", STILL: "#16a34a", "STILL?": "#ca8a04",
                "PRESENT": "#15803d"}
SPEC_SECONDS = 20.0


class LiveView:
    def __init__(self, source: Source, pipe: Pipeline, on_status=None, interval_ms: int = 200):
        self.src, self.pipe, self.on_status = source, pipe, on_status
        cfg = pipe.cfg
        self.last: Status | None = None

        self.fig = plt.figure(figsize=(12, 8))
        self.fig.canvas.manager.set_window_title("Pluto presence radar")
        gs = self.fig.add_gridspec(3, 2, height_ratios=[0.5, 2.2, 1.6], hspace=0.45, wspace=0.25)

        self.ax_state = self.fig.add_subplot(gs[0, :])
        self.ax_state.axis("off")
        self.state_txt = self.ax_state.text(0.5, 0.5, "starting...", ha="center", va="center",
                                            fontsize=26, fontweight="bold", color="white",
                                            transform=self.ax_state.transAxes,
                                            bbox=dict(boxstyle="round,pad=0.4", fc="#6b7280", ec="none"))

        self.ax_spec = self.fig.add_subplot(gs[1, :])
        n_spec = int(SPEC_SECONDS * pipe.fs)
        f, t, P = doppler_spectrogram(np.zeros(n_spec, complex) + 1e-6, pipe.fs)
        self.im = self.ax_spec.imshow(P, aspect="auto", origin="lower", cmap="magma",
                                      extent=[-SPEC_SECONDS, 0, f[0], f[-1]], interpolation="nearest")
        self.ax_spec.set_ylim(-60, 60)
        self.ax_spec.set_title("Doppler spectrogram (clutter removed)")
        self.ax_spec.set_ylabel("Doppler (Hz)")
        self.ax_spec.set_xlabel("time (s)")
        self.ax_spec.text(1.0, -0.12, f"{cfg.wavelength / 2 * 100:.1f} cm/s per Hz",
                          ha="right", transform=self.ax_spec.transAxes, fontsize=8, color="gray")

        left = gs[2, 0].subgridspec(2, 1, hspace=0.15)
        self.ax_mot = self.fig.add_subplot(left[0])
        (self.mot_line,) = self.ax_mot.plot([], [], lw=1.2, color="#2563eb", label="motion")
        (self.thr_line,) = self.ax_mot.plot([], [], color="#dc2626", ls="--", lw=1, label="threshold")
        self.ax_mot.set_title("motion vs noise (dB) / auto threshold", fontsize=9)
        self.ax_mot.set_xlim(-60, 0)
        self.ax_mot.tick_params(labelbottom=False, labelsize=8)

        self.ax_abs = self.fig.add_subplot(left[1], sharex=self.ax_mot)
        (self.abs_line,) = self.ax_abs.plot([], [], lw=1.2, color="#7c3aed")
        b = pipe.th.motion_abs_db
        lab = "boundary" + (f" ~{pipe.th.boundary_m:g} m" if pipe.th.boundary_m else "")
        self.bnd_line = self.ax_abs.axhline(b if b is not None else np.nan, color="#ea580c", ls="--", lw=1)
        self.ax_abs.text(0.01, 0.92, lab if b is not None else "no range boundary set",
                         transform=self.ax_abs.transAxes, fontsize=8, va="top", color="#ea580c")
        # proximity mode: near micro-motion energy and its threshold
        sb = pipe.th.still_abs_db
        (self.near_line,) = self.ax_abs.plot([], [], lw=1.2, color="#0891b2")
        self.near_thr = self.ax_abs.axhline(sb if sb is not None else np.nan, color="#0891b2", ls=":", lw=1)
        if sb is not None:
            self.ax_abs.text(0.99, 0.92, "near energy (presence)", transform=self.ax_abs.transAxes,
                             fontsize=8, va="top", ha="right", color="#0891b2")
        self.ax_abs.set_ylabel("dBFS", fontsize=8)
        self.ax_abs.set_xlabel("time (s)")
        self.ax_abs.tick_params(labelsize=8)

        self.ax_br = self.fig.add_subplot(gs[2, 1])
        (self.br_line,) = self.ax_br.plot([], [], lw=1.4, color="#16a34a")
        self.ax_br.set_title("breathing")
        self.ax_br.set_xlim(-cfg.breath_win_s, 0)
        self.ax_br.set_xlabel("time (s)")
        self.br_txt = self.ax_br.text(0.02, 0.95, "", transform=self.ax_br.transAxes, va="top", fontsize=10)

        self.anim = FuncAnimation(self.fig, self._frame, interval=interval_ms, cache_frame_data=False)

    def _frame(self, _):
        x = self.src.read(timeout=0.05)
        for s in self.pipe.feed(x):
            self.last = s
            if self.on_status:
                self.on_status(s)
        if self.src.finished and not len(x):
            self.anim.event_source.stop()
        s = self.last
        if s is None:
            return

        # state banner
        label = s.state if s.state != STILL or s.confirmed else ("PRESENT" if s.proximity else "STILL?")
        far = "   (far motion ignored)" if s.far_motion else ""
        text = {EMPTY: "NO ONE DETECTED", MOVING: "PERSON MOVING",
                STILL: "PERSON STILL (breathing)", "STILL?": "PERSON STILL? (acquiring breathing)",
                "PRESENT": "PERSON PRESENT (still)"}[label]
        if label == STILL and s.breath is not None:
            text += f"  {s.breath.bpm:.0f} bpm"
        text += far
        self.state_txt.set_text(text)
        self.state_txt.get_bbox_patch().set_facecolor(STATE_COLORS[label])

        # spectrogram
        n_spec = int(SPEC_SECONDS * self.pipe.fs)
        xc = self.pipe.hp.last(n_spec)
        filled = len(xc) / n_spec
        if len(xc) < n_spec:
            xc = np.concatenate([np.zeros(n_spec - len(xc), complex), xc])
        _, _, P = doppler_spectrogram(xc, self.pipe.fs)
        # colour scale from the columns that hold data (ignore the zero padding at start-up)
        valid = P[:, int(P.shape[1] * (1 - filled)):] if filled < 1 else P
        P = np.maximum(P, valid.min()) if valid.size else P
        lo = np.percentile(valid if valid.size else P, 30)
        self.im.set_data(P)
        self.im.set_clim(lo, max(lo + 25, P.max()))

        # motion history
        h = np.array(self.pipe.history)
        if len(h):
            tt = h[:, 0] - s.t
            self.mot_line.set_data(tt, h[:, 1])
            self.thr_line.set_data(tt, h[:, 3])
            self.ax_mot.set_ylim(min(-3, h[:, 1].min()), max(h[:, 3].max() + 6, h[:, 1].max() + 2))
            moving = h[:, 1] > h[:, 3]
            lv = np.where(moving, h[:, 2], np.nan)          # level only meaningful while moving
            self.abs_line.set_data(tt, lv)
            ref = [v for v in [self.pipe.th.motion_abs_db, self.pipe.th.still_abs_db] if v is not None]
            near = h[:, 4]
            if self.pipe.th.still_abs_db is not None:
                self.near_line.set_data(tt, np.where(near > -150, near, np.nan))
                ref += list(near[near > -150][-150:])
            vals = np.concatenate([lv[np.isfinite(lv)], ref])
            if len(vals):
                self.ax_abs.set_ylim(vals.min() - 6, vals.max() + 6)

        # breathing
        b = s.breath
        if b is not None:
            tb = (np.arange(len(b.wave)) - len(b.wave)) / self.pipe.cfg.breath_fs
            self.br_line.set_data(tb, b.wave)
            m = np.abs(b.wave).max() * 1.2 + 1e-9
            self.ax_br.set_ylim(-m, m)
            self.ax_br.set_ylabel("mm" if b.method == "arc" else "a.u.")
            ok = self.pipe.detector.breath_ok(b)
            self.br_txt.set_text(f"{b.bpm:.1f} bpm   SNR {b.snr_db:.1f} dB   ACF {b.acf:.2f}"
                                 f"   {'LOCK' if ok else 'no lock'}")
            self.br_txt.set_color("#16a34a" if ok else "gray")
        else:
            self.br_line.set_data([], [])
            self.br_txt.set_text("waiting for stillness..." if s.state != EMPTY else "")
            self.br_txt.set_color("gray")

    def show(self):
        plt.show()


def _ago(sec: float) -> str:
    sec = int(sec)
    return f"{sec} s ago" if sec < 60 else f"{sec // 60} min {sec % 60:02d} s ago"


class HeartView:
    """Heart-rate focused live view (run --heart)."""

    def __init__(self, source: Source, pipe: Pipeline, on_status=None, interval_ms: int = 200,
                 ref_bpm: float | None = None):
        self.src, self.pipe, self.on_status, self.ref = source, pipe, on_status, ref_bpm
        self.last: Status | None = None
        self.trend: list[tuple[float, float | None, float | None]] = []   # (t, locked, raw)
        cfg = pipe.cfg

        self.fig = plt.figure(figsize=(12, 8))
        self.fig.canvas.manager.set_window_title("Pluto heart-rate radar")
        gs = self.fig.add_gridspec(3, 2, height_ratios=[0.6, 1.6, 1.6], hspace=0.5, wspace=0.25)
        self.ax_state = self.fig.add_subplot(gs[0, :])
        self.ax_state.axis("off")
        self.state_txt = self.ax_state.text(0.5, 0.55, "starting...", ha="center", va="center", fontsize=28,
                                            fontweight="bold", color="white", transform=self.ax_state.transAxes,
                                            bbox=dict(boxstyle="round,pad=0.4", fc="#6b7280", ec="none"))
        self.sub_txt = self.ax_state.text(0.5, -0.25, "", ha="center", va="center", fontsize=11,
                                          color="#374151", transform=self.ax_state.transAxes)

        self.ax_wave = self.fig.add_subplot(gs[1, 0])
        (self.wave_line,) = self.ax_wave.plot([], [], color="#dc2626", lw=1.3)
        self.ax_wave.set_title("heartbeat waveform (breathing removed)", fontsize=10)
        self.ax_wave.set_xlim(-cfg.heart_win_s, 0)
        self.ax_wave.set_xlabel("time (s)")
        self.ax_wave.set_yticks([])

        self.ax_spec = self.fig.add_subplot(gs[1, 1])
        (self.spec_line,) = self.ax_spec.plot([], [], color="#7c3aed", lw=1.3)
        self.thr_line = self.ax_spec.axhline(cfg.heart_snr_db, color="#9ca3af", ls="--", lw=1)
        (self.peak_mark,) = self.ax_spec.plot([], [], "o", color="#dc2626")
        self.breath_marks = [self.ax_spec.axvline(np.nan, color="#16a34a", ls=":", lw=1) for _ in range(8)]
        if ref_bpm:
            self.ax_spec.axvline(ref_bpm, color="#0891b2", lw=1.5, alpha=0.6)
        self.ax_spec.set_xlim(42, 180)
        self.ax_spec.set_title("heart spectrum (dB over median)  : breathing harmonics", fontsize=10)
        self.ax_spec.set_xlabel("bpm")

        self.ax_trend = self.fig.add_subplot(gs[2, 0])
        (self.raw_pts,) = self.ax_trend.plot([], [], ".", color="#c4b5fd", ms=4, label="per-second estimate")
        (self.lock_line,) = self.ax_trend.plot([], [], color="#dc2626", lw=2, label="locked")
        if ref_bpm:
            self.ax_trend.axhline(ref_bpm, color="#0891b2", lw=1.5, alpha=0.6, label="reference")
        self.ax_trend.set_xlim(-120, 0)
        self.ax_trend.set_ylim(45, 125)
        self.ax_trend.set_title("heart rate (bpm)", fontsize=10)
        self.ax_trend.set_xlabel("time (s)")
        self.ax_trend.legend(fontsize=7, loc="upper left")

        self.ax_br = self.fig.add_subplot(gs[2, 1])
        (self.br_line,) = self.ax_br.plot([], [], color="#16a34a", lw=1.2)
        self.ax_br.set_xlim(-cfg.breath_win_s, 0)
        self.ax_br.set_title("breathing", fontsize=10)
        self.ax_br.set_xlabel("time (s)")
        self.ax_br.set_yticks([])
        self.br_txt = self.ax_br.text(0.02, 0.95, "", transform=self.ax_br.transAxes, va="top", fontsize=9)

        self.anim = FuncAnimation(self.fig, self._frame, interval=interval_ms, cache_frame_data=False)

    def _frame(self, _):
        x = self.src.read(timeout=0.05)
        for s in self.pipe.feed(x):
            self.last = s
            if self.on_status:
                self.on_status(s)
            h = s.heart
            if not self.trend or s.t - self.trend[-1][0] >= 0.99:
                self.trend.append((s.t, s.heart_bpm, h.bpm if h is not None else None))
        if self.src.finished and not len(x):
            self.anim.event_source.stop()
        s = self.last
        if s is None:
            return
        self.trend = [r for r in self.trend if s.t - r[0] <= 120]

        moving = s.motion_db > s.motion_thr and not s.far_motion
        if s.heart_bpm is not None:
            avg = s.heart_last_bpm if s.heart_last_bpm is not None else s.heart_bpm
            txt, col = f"♥  {avg:.0f} bpm", "#dc2626"
        elif s.heart_last_bpm is not None:
            # hold the last reading so the subject can walk up and read it
            txt, col = f"♥  {s.heart_last_bpm:.0f} bpm   (last reading, {_ago(s.heart_last_age_s)})", "#991b1b"
        elif moving:
            txt, col = "MOTION IN RANGE - hold still", "#6b7280"
        else:
            txt, col = "measuring heart rate...", "#ca8a04"
        self.state_txt.set_text(txt)
        self.state_txt.get_bbox_patch().set_facecolor(col)
        sub = s.heart_wait or ""
        if s.far_motion:
            sub += "   (motion beyond range boundary ignored)"
        if s.heart_last_bpm is not None and s.heart_bpm is None:
            sub = (f"average of {max(s.heart_last_span_s, 1):.0f} s of locked readings   |   for a new reading "
                   f"sit still 0.3-0.5 m in front" + (f"   ({s.heart_wait})" if s.heart_wait else ""))
        elif s.heart is not None:
            sub = f"estimate {s.heart.bpm:.0f} bpm, confidence {s.heart.snr_db:.1f} dB (lock needs >= " \
                  f"{self.pipe.cfg.heart_snr_db:.0f} dB for {self.pipe.cfg.heart_lock_s:.0f} s)"
            if self.ref:
                sub += f"   |   reference {self.ref:.0f} bpm"
        self.sub_txt.set_text(sub)

        h = s.heart
        if h is not None:
            fs = self.pipe.cfg.breath_fs
            tw = (np.arange(len(h.wave)) - len(h.wave)) / fs
            self.wave_line.set_data(tw, h.wave)
            m = np.abs(h.wave).max() * 1.2 + 1e-12
            self.ax_wave.set_ylim(-m, m)
            self.spec_line.set_data(60 * h.freqs, h.score)
            self.peak_mark.set_data([h.bpm], [h.snr_db])
            self.ax_spec.set_ylim(min(-5, h.score.min()), max(h.snr_db + 4, self.pipe.cfg.heart_snr_db + 4))
            for k, ln in enumerate(self.breath_marks, start=1):
                ln.set_xdata([60 * k * h.breath_hz] * 2 if h.breath_hz else [np.nan] * 2)
        else:
            self.wave_line.set_data([], [])
            self.spec_line.set_data([], [])
            self.peak_mark.set_data([], [])

        if self.trend:
            tr = np.array([(r[0] - s.t, np.nan if r[1] is None else r[1], np.nan if r[2] is None else r[2])
                           for r in self.trend])
            self.raw_pts.set_data(tr[:, 0], tr[:, 2])
            self.lock_line.set_data(tr[:, 0], tr[:, 1])

        b = s.breath
        if b is not None:
            tb = (np.arange(len(b.wave)) - len(b.wave)) / self.pipe.cfg.breath_fs
            self.br_line.set_data(tb, b.wave)
            m = np.abs(b.wave).max() * 1.2 + 1e-12
            self.ax_br.set_ylim(-m, m)
            self.br_txt.set_text(f"{b.bpm:.1f} breaths/min" + ("" if self.pipe.detector.breath_ok(b) else " (weak)"))
        else:
            self.br_line.set_data([], [])
            self.br_txt.set_text("")

    def show(self):
        plt.show()
