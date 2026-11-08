"""Live matplotlib display: Doppler spectrogram, motion metric, breathing waveform, state."""
from __future__ import annotations

import threading

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyBboxPatch

from .detector import EMPTY, MOVING, STILL, Pipeline, Status, format_status
from .dsp import doppler_spectrogram
from .radio import Source

STATE_COLORS = {EMPTY: "#6b7280", MOVING: "#dc2626", STILL: "#16a34a", "STILL?": "#ca8a04",
                "PRESENT": "#15803d"}
SPEC_SECONDS = 20.0
STOP = threading.Event()   # set from a signal handler; checked by the animation timer (main thread)


class LiveView:
    def __init__(self, source: Source, pipe: Pipeline, on_status=None, interval_ms: int = 200):
        self.src, self.pipe, self.on_status = source, pipe, on_status
        cfg = pipe.cfg
        self.last: Status | None = None

        self.fig = plt.figure(figsize=(12, 8))
        self.fig.canvas.manager.set_window_title("bladeRF presence radar")
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
        if STOP.is_set():               # SIGINT / SIGTERM: close the window -> normal shutdown path
            plt.close(self.fig)
            return
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


GREY, ORANGE, YELLOW, GREEN, DKGREEN, RED, DKRED = ("#6b7280", "#ea580c", "#ca8a04", "#16a34a",
                                                     "#166534", "#dc2626", "#991b1b")


def _style(ax, title):
    ax.set_title(title, fontsize=10, loc="left", color="#374151")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.tick_params(labelsize=8, colors="#4b5563")
    ax.grid(alpha=0.25, lw=0.6)


class _Card:
    """Fixed-size coloured card: small heading, big value, one caption line."""

    def __init__(self, fig, spec, heading):
        ax = fig.add_subplot(spec)
        ax.axis("off")
        self.box = FancyBboxPatch((0.02, 0.04), 0.96, 0.92, boxstyle="round,pad=0,rounding_size=0.06",
                                  transform=ax.transAxes, fc=GREY, ec="none", mutation_aspect=0.3)
        ax.add_patch(self.box)
        kw = dict(transform=ax.transAxes, ha="center", va="center", color="white")
        ax.text(0.5, 0.80, heading, fontsize=10, alpha=0.85, **kw)
        self.value = ax.text(0.5, 0.48, "--", fontsize=23, fontweight="bold", **kw)
        self.caption = ax.text(0.5, 0.16, "", fontsize=9, **kw)

    def set(self, value, caption, color):
        self.value.set_text(value)
        self.caption.set_text(caption)
        self.box.set_facecolor(color)


class HeartView:
    """Heart-rate focused live view (run --heart)."""

    def __init__(self, source: Source, pipe: Pipeline, on_status=None, interval_ms: int = 200,
                 ref_bpm: float | None = None):
        self.src, self.pipe, self.on_status, self.ref = source, pipe, on_status, ref_bpm
        self.last: Status | None = None
        self.trend: list[tuple[float, float | None, float | None]] = []   # (t, locked, raw)
        cfg = pipe.cfg

        self.fig = plt.figure(figsize=(13, 9), facecolor="white")
        self.fig.canvas.manager.set_window_title("bladeRF heart-rate radar")
        outer = self.fig.add_gridspec(2, 1, height_ratios=[0.8, 4.4], hspace=0.12,
                                      left=0.06, right=0.97, top=0.98, bottom=0.06)
        cards = outer[0].subgridspec(1, 3, wspace=0.05)
        gs = outer[1].subgridspec(3, 2, height_ratios=[1.35, 1.05, 1.2], hspace=0.55, wspace=0.18)
        self.c_status = _Card(self.fig, cards[0], "STATUS")
        self.c_breath = _Card(self.fig, cards[1], "BREATHING")
        self.c_heart = _Card(self.fig, cards[2], "HEART RATE")

        # Doppler spectrogram (all motion, incl. beyond the range boundary)
        self.ax_dop = self.fig.add_subplot(gs[0, :])
        n_spec = int(SPEC_SECONDS * pipe.fs)
        f, _, P = doppler_spectrogram(np.zeros(n_spec, complex) + 1e-6, pipe.fs)
        self.im = self.ax_dop.imshow(P, aspect="auto", origin="lower", cmap="magma",
                                     extent=[-SPEC_SECONDS, 0, f[0], f[-1]], interpolation="nearest")
        self.ax_dop.set_ylim(-60, 60)
        _style(self.ax_dop, "Doppler spectrogram  (motion; "
               f"{cfg.wavelength / 2 * 100:.1f} cm/s per Hz)")
        self.ax_dop.grid(False)
        self.ax_dop.set_ylabel("Hz", fontsize=8)

        self.ax_br = self.fig.add_subplot(gs[1, 0])
        (self.br_line,) = self.ax_br.plot([], [], color=GREEN, lw=1.4)
        self.ax_br.set_xlim(-cfg.breath_win_s, 0)
        self.ax_br.set_yticks([])
        _style(self.ax_br, "breathing waveform")
        self.br_txt = self.ax_br.text(0.99, 1.04, "", transform=self.ax_br.transAxes, ha="right",
                                      fontsize=8, color="#6b7280")

        self.ax_wave = self.fig.add_subplot(gs[1, 1])
        (self.wave_line,) = self.ax_wave.plot([], [], color=RED, lw=1.2)
        self.ax_wave.set_xlim(-cfg.heart_win_s, 0)
        self.ax_wave.set_yticks([])
        _style(self.ax_wave, "heartbeat waveform (breathing removed)")

        self.ax_trend = self.fig.add_subplot(gs[2, 0])
        (self.raw_pts,) = self.ax_trend.plot([], [], ".", color="#c4b5fd", ms=4, label="estimate")
        (self.lock_line,) = self.ax_trend.plot([], [], color=RED, lw=2, label="locked")
        if ref_bpm:
            self.ax_trend.axhline(ref_bpm, color="#0891b2", lw=1.5, alpha=0.6, label="reference")
        self.ax_trend.set_xlim(-120, 0)
        self.ax_trend.set_ylim(45, 125)
        self.ax_trend.set_xlabel("time (s)", fontsize=8)
        _style(self.ax_trend, "heart rate history (bpm)")
        self.ax_trend.legend(fontsize=7, loc="upper left", frameon=False, ncol=3)

        self.ax_spec = self.fig.add_subplot(gs[2, 1])
        (self.spec_line,) = self.ax_spec.plot([], [], color="#7c3aed", lw=1.2)
        self.ax_spec.axhline(cfg.heart_snr_db, color="#9ca3af", ls="--", lw=1)
        (self.peak_mark,) = self.ax_spec.plot([], [], "o", color=RED, ms=5)
        self.breath_marks = [self.ax_spec.axvline(np.nan, color=GREEN, ls=":", lw=1) for _ in range(8)]
        if ref_bpm:
            self.ax_spec.axvline(ref_bpm, color="#0891b2", lw=1.5, alpha=0.6)
        self.ax_spec.set_xlim(42, 180)
        self.ax_spec.set_xlabel("bpm", fontsize=8)
        _style(self.ax_spec, "heart spectrum (dB)   - - lock threshold   : breathing harmonics")

        self.anim = FuncAnimation(self.fig, self._frame, interval=interval_ms, cache_frame_data=False)

    def _frame(self, _):
        if STOP.is_set():               # SIGINT / SIGTERM: close the window -> normal shutdown path
            plt.close(self.fig)
            return
        x = self.src.read(timeout=0.05)
        for s in self.pipe.feed(x):
            self.last = s
            if self.on_status:
                self.on_status(s)
            h = s.heart
            if not self.trend or s.t - self.trend[-1][0] >= 0.99:
                raw = h.bpm if h is not None and s.state != EMPTY else None   # no dots for an empty room
                self.trend.append((s.t, s.heart_bpm, raw))
        if self.src.finished and not len(x):
            self.anim.event_source.stop()
        s = self.last
        if s is None:
            return
        self.trend = [r for r in self.trend if s.t - r[0] <= 120]
        cfg = self.pipe.cfg
        bnd = self.pipe.th.boundary_m

        # --- cards ---
        rng = f"within ~{bnd:g} m" if bnd else "in range"
        far = f"motion beyond ~{bnd:g} m ignored" if (s.far_motion and bnd) else ""
        if s.state == MOVING:
            self.c_status.set("MOVING", far or f"person moving {rng}", ORANGE)
        elif s.heart_bpm is not None:
            self.c_status.set("STILL", "breathing + heart rate locked", RED)
        elif s.breath_bpm is not None:
            self.c_status.set("STILL", "breathing locked, heart measuring", GREEN)
        elif s.state == STILL:
            self.c_status.set("STILL", "acquiring - sit still 0.3-0.5 m away", YELLOW)
        else:
            self.c_status.set("NO PERSON", far or f"nobody {rng}", GREY)

        b = s.breath
        if s.breath_bpm is not None:
            avg = s.breath_last_bpm if s.breath_last_bpm is not None else s.breath_bpm
            self.c_breath.set(f"{avg:.0f} /min", f"live  -  SNR {b.snr_db:.0f} dB" if b else "live", GREEN)
        elif s.breath_last_bpm is not None:
            self.c_breath.set(f"{s.breath_last_bpm:.0f} /min", f"last reading, {_ago(s.breath_last_age_s)}",
                              DKGREEN)
        elif s.breath_wait:
            self.c_breath.set("--", s.breath_wait, GREY)
        elif s.state != EMPTY:
            self.c_breath.set("...", "measuring", YELLOW)
        else:
            self.c_breath.set("--", "", GREY)

        h = s.heart
        if s.heart_bpm is not None:
            avg = s.heart_last_bpm if s.heart_last_bpm is not None else s.heart_bpm
            self.c_heart.set(f"{avg:.0f} bpm", f"average of {max(s.heart_last_span_s, 1):.0f} s locked", RED)
        elif s.heart_last_bpm is not None:
            self.c_heart.set(f"{s.heart_last_bpm:.0f} bpm", f"last reading, {_ago(s.heart_last_age_s)}", DKRED)
        elif s.heart_wait.startswith("hold still"):
            self.c_heart.set("--", s.heart_wait.replace(" - nothing may move in range", ""), GREY)
        elif s.state == EMPTY:
            self.c_heart.set("--", "", GREY)
        elif h is not None:
            self.c_heart.set("...", f"measuring - confidence {h.snr_db:.0f}/{cfg.heart_snr_db:.0f} dB", YELLOW)
        else:
            self.c_heart.set("...", "measuring", YELLOW)

        # --- spectrogram ---
        n_spec = int(SPEC_SECONDS * self.pipe.fs)
        xc = self.pipe.hp.last(n_spec)
        filled = len(xc) / n_spec
        if len(xc) < n_spec:
            xc = np.concatenate([np.zeros(n_spec - len(xc), complex), xc])
        _, _, P = doppler_spectrogram(xc, self.pipe.fs)
        valid = P[:, int(P.shape[1] * (1 - filled)):] if filled < 1 else P
        P = np.maximum(P, valid.min()) if valid.size else P
        lo = np.percentile(valid if valid.size else P, 30)
        self.im.set_data(P)
        self.im.set_clim(lo, max(lo + 25, P.max()))

        # --- waveforms ---
        if b is not None:
            tb = (np.arange(len(b.wave)) - len(b.wave)) / cfg.breath_fs
            self.br_line.set_data(tb, b.wave)
            m = np.abs(b.wave).max() * 1.2 + 1e-12
            self.ax_br.set_ylim(-m, m)
            self.br_txt.set_text(f"estimate {b.bpm:.1f} /min   SNR {b.snr_db:.0f} dB   ACF {b.acf:.2f}"
                                 + ("" if self.pipe.detector.breath_ok(b) else "   (weak)"))
        else:
            self.br_line.set_data([], [])
            self.br_txt.set_text("")
        if h is not None:
            tw = (np.arange(len(h.wave)) - len(h.wave)) / cfg.breath_fs
            self.wave_line.set_data(tw, h.wave)
            m = np.abs(h.wave).max() * 1.2 + 1e-12
            self.ax_wave.set_ylim(-m, m)
            self.spec_line.set_data(60 * h.freqs, h.score)
            self.peak_mark.set_data([h.bpm], [h.snr_db])
            self.ax_spec.set_ylim(min(-5, h.score.min()), max(h.snr_db + 4, cfg.heart_snr_db + 4))
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

    def show(self):
        plt.show()


TRACK_COLORS = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2", "#ca8a04", "#db2777"]


class MapView:
    """Top-down 2D map of people (run --map): heatmap, tracks with IDs, trails and breathing."""

    def __init__(self, source: Source, pipe, on_status=None, interval_ms: int = 200):
        from .mapping import MOVING_T
        self._moving = MOVING_T
        self.src, self.pipe, self.on_status = source, pipe, on_status
        self.last = None
        cfg = pipe.cfg
        g = pipe.grid
        n_art = max(cfg.max_targets * 2, 8)

        self.fig = plt.figure(figsize=(14, 8.5), facecolor="white")
        self.fig.canvas.manager.set_window_title("bladeRF 2D people radar")
        outer = self.fig.add_gridspec(1, 2, width_ratios=[1.45, 1], wspace=0.12,
                                      left=0.05, right=0.98, top=0.95, bottom=0.07)
        right = outer[1].subgridspec(3, 1, height_ratios=[0.75, 1.5, 1.6], hspace=0.45)
        cards = right[0].subgridspec(1, 2, wspace=0.05)
        self.c_people = _Card(self.fig, cards[0], "PEOPLE")
        self.c_status = _Card(self.fig, cards[1], "NEAREST / STATUS")

        # --- map ---
        ax = self.ax_map = self.fig.add_subplot(outer[0])
        ext = [g.xs[0] - cfg.map_res / 2, g.xs[-1] + cfg.map_res / 2,
               g.ys[0] - cfg.map_res / 2, g.ys[-1] + cfg.map_res / 2]
        self.im = ax.imshow(np.zeros(g.shape), origin="lower", extent=ext, cmap="Greys", vmin=0, vmax=1,
                            interpolation="bilinear", alpha=0.85)
        th = np.linspace(0, np.pi, 181)
        for r in range(1, int(np.hypot(max(map(abs, cfg.map_x)), cfg.map_y[1])) + 1):
            ax.plot(r * np.cos(th), r * np.sin(th), color="#9ca3af", lw=0.6, ls=":")
            if r < cfg.map_y[1]:
                ax.text(0.05, r + 0.05, f"{r} m", fontsize=7, color="#6b7280")
        for az in (-60, -30, 0, 30, 60):
            a = np.radians(az)
            R = 20
            ax.plot([0, R * np.sin(a)], [0, R * np.cos(a)], color="#d1d5db", lw=0.6)
        ax.plot(*np.array(cfg.tx_pos).T, "^", color="#dc2626", ms=7, label="TX")
        ax.plot(*np.array(cfg.rx_pos).T, "v", color="#2563eb", ms=7, label="RX")
        ax.set_xlim(cfg.map_x[0], cfg.map_x[1])
        ax.set_ylim(-0.25, cfg.map_y[1])
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)  - array axis -", fontsize=8)
        ax.set_ylabel("y (m)  - distance in front of the antennas -", fontsize=8)
        _style(ax, "people map  (shade: radar evidence; dots: tracked people)")
        ax.legend(loc="lower right", fontsize=7, frameon=False, ncol=2)
        self.dots = [ax.plot([], [], "o", ms=14, mec="white", mew=2)[0] for _ in range(n_art)]
        self.trails = [ax.plot([], [], "-", lw=2, alpha=0.5)[0] for _ in range(n_art)]
        self.labels = [ax.text(0, 0, "", fontsize=9, fontweight="bold", va="bottom",
                               bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.8))
                       for _ in range(n_art)]
        (self.det_pts,) = ax.plot([], [], "x", color="#f59e0b", ms=6, mew=1.5)

        # --- spectrogram (reference channel) ---
        self.ax_dop = self.fig.add_subplot(right[1])
        n_spec = int(SPEC_SECONDS * pipe.fs)
        f, _, P = doppler_spectrogram(np.zeros(n_spec, complex) + 1e-6, pipe.fs)
        self.spec = self.ax_dop.imshow(P, aspect="auto", origin="lower", cmap="magma",
                                       extent=[-SPEC_SECONDS, 0, f[0], f[-1]], interpolation="nearest")
        self.ax_dop.set_ylim(-60, 60)
        self.ax_dop.grid(False)
        _style(self.ax_dop, f"Doppler (all people, ref. channel; {cfg.wavelength / 2 * 100:.1f} cm/s per Hz)")

        # --- track list ---
        self.ax_list = self.fig.add_subplot(right[2])
        self.ax_list.axis("off")
        self.ax_list.set_title("tracked people", fontsize=10, loc="left", color="#374151")
        self.list_txt = self.ax_list.text(0.0, 1.0, "", va="top", fontsize=10, family="monospace",
                                          transform=self.ax_list.transAxes)
        self.anim = FuncAnimation(self.fig, self._frame, interval=interval_ms, cache_frame_data=False)

    def _frame(self, _):
        if STOP.is_set():
            plt.close(self.fig)
            return
        x = self.src.read(timeout=0.05)
        for s in self.pipe.feed(x):
            self.last = s
            if self.on_status:
                self.on_status(s)
        if self.src.finished and not x.shape[-1]:
            self.anim.event_source.stop()
        s = self.last
        if s is None:
            return
        h = s.heat
        self.im.set_data((h / max(h.max(), 0.3)) ** 2)

        tracks = sorted(s.tracks, key=lambda tr: tr.id)
        for i in range(len(self.dots)):
            if i < len(tracks):
                tr = tracks[i]
                col = TRACK_COLORS[tr.id % len(TRACK_COLORS)]
                self.dots[i].set_data([tr.x], [tr.y])
                self.dots[i].set_color(col)
                self.dots[i].set_markeredgecolor("white")
                self.dots[i].set_marker("o" if tr.kind == self._moving else "s")
                tx = [p[0] for p in tr.trail] + [tr.x]
                ty = [p[1] for p in tr.trail] + [tr.y]
                self.trails[i].set_data(tx, ty)
                self.trails[i].set_color(col)
                if tr.kind == self._moving:
                    lab = f"#{tr.id} moving"
                else:
                    lab = f"#{tr.id} still" + (f" {tr.bpm:.0f}/min" if tr.bpm is not None else "")
                self.labels[i].set_text(lab)
                self.labels[i].set_position((tr.x + 0.2, tr.y + 0.15))
                self.labels[i].set_color(col)
            else:
                self.dots[i].set_data([], [])
                self.trails[i].set_data([], [])
                self.labels[i].set_text("")
        self.det_pts.set_data([d.x for d in s.detections], [d.y for d in s.detections])

        # cards
        n = len(tracks)
        self.c_people.set(str(n), "moving: %d   still: %d" % (
            sum(tr.kind == self._moving for tr in tracks), sum(tr.kind != self._moving for tr in tracks)),
            GREEN if n else GREY)
        if tracks:
            near = min(tracks, key=lambda tr: np.hypot(tr.x, tr.y))
            r = np.hypot(near.x, near.y)
            self.c_status.set(f"{r:.1f} m", f"#{near.id} at {np.degrees(np.arctan2(near.x, near.y)):+.0f} deg, "
                              f"{near.kind}", ORANGE if near.kind == self._moving else GREEN)
        else:
            st = s.status.state if s.status is not None else EMPTY
            self.c_status.set("--", "nobody tracked" + (f" (ref. channel: {st})" if st != EMPTY else ""),
                              GREY)

        # spectrogram of the reference channel
        n_spec = int(SPEC_SECONDS * self.pipe.fs)
        xc = self.pipe.legacy.hp.last(n_spec)
        filled = len(xc) / n_spec
        if len(xc) < n_spec:
            xc = np.concatenate([np.zeros(n_spec - len(xc), complex), xc])
        _, _, P = doppler_spectrogram(xc, self.pipe.fs)
        valid = P[:, int(P.shape[1] * (1 - filled)):] if filled < 1 else P
        P = np.maximum(P, valid.min()) if valid.size else P
        lo = np.percentile(valid if valid.size else P, 30)
        self.spec.set_data(P)
        self.spec.set_clim(lo, max(lo + 25, P.max()))

        # list
        lines = []
        for tr in tracks:
            r, az = np.hypot(tr.x, tr.y), np.degrees(np.arctan2(tr.x, tr.y))
            what = (f"moving {tr.speed:3.1f} m/s" if tr.kind == self._moving else
                    "still  " + (f"{tr.bpm:4.1f} breaths/min" + ("" if tr.bpm_live else " (last)")
                                 if tr.bpm is not None else "breathing: measuring"))
            lines.append(f"#{tr.id:<3d} {r:4.1f} m {az:+4.0f} deg  {what}")
        if not lines:
            lines = ["nobody tracked"]
        lines.append("")
        lines.append(f"Doppler bins used: moving {s.n_fast_bins}, still sources {s.n_slow_bins}")
        self.list_txt.set_text("\n".join(lines))

    def show(self):
        plt.show()
