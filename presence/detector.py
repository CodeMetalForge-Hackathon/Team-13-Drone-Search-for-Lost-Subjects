"""Presence state machine and the slow-time processing pipeline."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .config import Config, Thresholds
from .heart import HeartResult, HeartTracker, SpectrumAverager, estimate_heart
from .dsp import (BreathResult, ClutterFilter, PhaseJumpCorrector, block_decimate,
                  estimate_breathing, motion_levels, still_level)

EMPTY, MOVING, STILL = "EMPTY", "MOVING", "STILL"


class Ring:
    """Fixed-size complex ring buffer."""

    def __init__(self, size: int):
        self.buf = np.zeros(size, np.complex128)
        self.size = size
        self.count = 0  # total samples ever written

    def push(self, x: np.ndarray) -> None:
        x = x[-self.size:]
        i = self.count % self.size
        first = min(len(x), self.size - i)
        self.buf[i:i + first] = x[:first]
        self.buf[:len(x) - first] = x[first:]
        self.count += len(x)

    def last(self, n: int) -> np.ndarray:
        n = min(n, self.count, self.size)
        end = self.count % self.size
        idx = (np.arange(end - n, end)) % self.size
        return self.buf[idx]


class AutoFloor:
    """Learns the background motion level while running (no empty-room calibration needed).

    People move intermittently, so a low percentile of the motion metric over ~10 min is the
    background (receiver noise + fans, AC, curtains ...). Constant background movers raise it and
    are thereby ignored; a person is still detected because they stand out above it.
    """

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.vals: deque = deque(maxlen=int(cfg.floor_window_s))   # one sample per second
        self._next_t = -np.inf
        self._t = None
        self.floor: float | None = None   # rate-limited floor estimate (dB)

    def update(self, t: float, ratio_db: float, base: float) -> None:
        if t < self._next_t:
            return
        self._next_t = t + 1.0
        self.vals.append(ratio_db)
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        if len(self.vals) < self.cfg.floor_warmup_s:
            return
        target = float(np.percentile(self.vals, self.cfg.floor_percentile))
        if self.floor is None:
            # start no higher than "base threshold" so warm-up behaves like the fixed threshold
            self.floor = min(target, base - self.cfg.floor_margin_db)
        elif target < self.floor:
            self.floor = target
        else:
            self.floor = min(target, self.floor + self.cfg.floor_rise_db_per_min * dt / 60.0)

    def threshold(self, base: float) -> float:
        if self.floor is None:
            return base
        c = self.cfg
        return float(min(max(base, self.floor + c.floor_margin_db), base + c.floor_max_rise_db))


class Detector:
    def __init__(self, cfg: Config, th: Thresholds):
        self.cfg, self.th = cfg, th
        self.state = EMPTY
        self.confirmed = False          # STILL backed by a breathing lock (vs. grace period)
        self.last_t: float | None = None
        self.motion_run = 0.0
        self.breath_run = 0.0
        self._bpm_lo, self._bpm_hi = np.inf, -np.inf
        self.last_motion_t = -np.inf    # last time the motion metric crossed threshold
        self.last_present_t = -np.inf   # last time presence was positively confirmed
        self.near_run = 0.0

    @property
    def proximity(self) -> bool:
        return self.th.still_abs_db is not None

    def breath_ok(self, br: BreathResult | None) -> bool:
        return br is not None and br.snr_db >= self.th.breath_snr_db and br.acf >= self.th.breath_acf

    def is_motion(self, motion_db: float, motion_abs: float | None, thr: float | None = None) -> bool:
        if motion_db <= (self.th.motion_db if thr is None else thr):
            return False
        # soft range boundary: too weak in absolute terms -> someone/something farther away
        return self.th.motion_abs_db is None or motion_abs is None or motion_abs >= self.th.motion_abs_db

    def step(self, t: float, motion_db: float, br: BreathResult | None,
             motion_abs: float | None = None, thr: float | None = None,
             still_abs: float | None = None) -> str:
        c = self.cfg
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t

        if self.is_motion(motion_db, motion_abs, thr):
            self.motion_run += dt
            if self.motion_run > 0:      # two consecutive hits: ignore single noise spikes
                self.last_motion_t = t
        else:
            self.motion_run = 0.0
        ok = self.breath_ok(br)
        if ok:
            self.breath_run += dt
            self._bpm_lo = min(self._bpm_lo, br.bpm)
            self._bpm_hi = max(self._bpm_hi, br.bpm)
            if self._bpm_hi - self._bpm_lo > c.breath_rate_span_bpm:   # rate jumped: restart
                self.breath_run, self._bpm_lo, self._bpm_hi = 0.0, br.bpm, br.bpm
        else:
            self.breath_run, self._bpm_lo, self._bpm_hi = 0.0, np.inf, -np.inf

        moving = self.motion_run >= c.motion_confirm_s or (
            self.state == MOVING and t - self.last_motion_t < c.motion_hold_s)

        if self.proximity:
            # presence = near micro-motion energy (breathing / sway) or near motion; breathing lock
            # only adds the rate ("confirmed"). Leaving is detected within proximity_absent_s.
            near = still_abs is not None and still_abs >= self.th.still_abs_db
            self.near_run = self.near_run + dt if near else 0.0
            if moving:
                self.state, self.confirmed = MOVING, True
                self.last_present_t = t
            elif self.near_run >= c.near_confirm_s:
                # (breathing lock alone never keeps presence: its 30 s window still holds the past)
                self.state, self.confirmed = STILL, ok
                self.last_present_t = t
            elif self.state != EMPTY and t - self.last_present_t < c.proximity_absent_s:
                self.confirmed = self.confirmed and ok
                if self.state == MOVING:
                    self.state = STILL
            else:
                self.state, self.confirmed = EMPTY, False
            return self.state

        if moving:
            self.state, self.confirmed = MOVING, True
            self.last_present_t = t
        elif ok and (self.state != EMPTY or self.breath_run >= c.breath_confirm_s):
            self.state, self.confirmed = STILL, True
            self.last_present_t = t
        elif self.state != EMPTY and t - self.last_present_t < c.absent_timeout_s:
            self.state, self.confirmed = STILL, False   # stopped moving, waiting for breathing lock
        else:
            self.state, self.confirmed = EMPTY, False
        return self.state


@dataclass
class Status:
    t: float
    state: str
    confirmed: bool
    motion_db: float
    breath: BreathResult | None
    breath_win_s: float
    motion_abs: float = -200.0     # dBFS
    motion_thr: float = 0.0        # effective ratio threshold (base or auto floor)
    far_motion: bool = False       # motion present but weaker than the range boundary (ignored)
    still_abs: float = -200.0      # near micro-motion energy (dBFS)
    heart: HeartResult | None = None
    heart_bpm: float | None = None   # locked heart rate
    heart_wait: str = ""             # why no heart estimate right now
    heart_last_bpm: float | None = None   # moving-window average of the most recent lock (held)
    heart_last_age_s: float = 0.0         # seconds since that lock ended / was last updated
    heart_last_span_s: float = 0.0        # how many seconds of locked readings the average covers
    proximity: bool = False
    jumps: int = 0
    extra: dict = field(default_factory=dict)


class Pipeline:
    """Consumes slow-time IQ chunks, produces a Status every `update_s` of data."""

    def __init__(self, cfg: Config, th: Thresholds, gate_breathing_on_motion: bool = True,
                 auto_floor: bool | None = None):
        self.cfg, self.th = cfg, th
        self.fs = cfg.slow_fs
        size = int(cfg.history_s * self.fs)
        self.raw = Ring(size)       # clutter included (breathing path)
        self.hp = Ring(size)        # clutter removed (Doppler / motion path)
        self.clutter = ClutterFilter(cfg.clutter_tau_s, self.fs)
        self.jump = PhaseJumpCorrector(cfg.jump_fix_rad) if cfg.jump_fix_rad > 0 else None
        self.detector = Detector(cfg, th)
        self.gate = gate_breathing_on_motion
        self.step_n = int(round(cfg.update_s * self.fs))
        self.noise_psd: float | None = None
        self.heart_tracker = HeartTracker(cfg.heart_snr_db, cfg.heart_lock_s, cfg.heart_span_bpm)
        self.heart_avg = SpectrumAverager(cfg.heart_avg_alpha)
        self._heart_next_t = 0.0
        self._last_any_motion_t = -np.inf
        self._heart: HeartResult | None = None
        self._heart_bpm: float | None = None
        self._heart_wait = ""
        self._heart_locks: deque = deque()      # (t, bpm) of locked readings, moving window
        self._heart_last: float | None = None
        self._heart_last_t = -np.inf
        self._pending = 0
        self.floor = AutoFloor(cfg) if (cfg.auto_floor if auto_floor is None else auto_floor) else None
        self.history: deque = deque(maxlen=int(cfg.history_s / cfg.update_s))  # (t, ratio, abs, thr)

    @property
    def t(self) -> float:
        return self.raw.count / self.fs

    def _ingest(self, x: np.ndarray) -> None:
        if self.jump is not None:
            x = self.jump.process(x)
        self.raw.push(x)
        self.hp.push(self.clutter.process(x))

    def feed(self, x: np.ndarray) -> list[Status]:
        """Ingest samples; returns one Status per elapsed update interval."""
        out = []
        while len(x):
            take = min(len(x), self.step_n - self._pending)
            self._ingest(x[:take])
            x = x[take:]
            self._pending += take
            if self._pending >= self.step_n:
                self._pending = 0
                s = self.update()
                if s is not None:
                    out.append(s)
        return out

    def _expire_last_heart(self, t: float) -> None:
        if self._heart_last is not None and t - self._heart_last_t > self.cfg.heart_last_hold_s:
            self._heart_last, self._heart_locks = None, deque()

    def _update_heart(self, t: float, near_motion: bool, br) -> None:
        """Heart estimate every heart_every_s, only over a window with no motion inside the range
        boundary (calibrate --boundary). Without a boundary, any detected motion pauses it: a CW
        radar cannot separate a nearby bystander's slow movements from a heartbeat."""
        c = self.cfg
        if near_motion:
            self._last_any_motion_t = t
        if t < self._heart_next_t:
            return
        self._heart_next_t = t + c.heart_every_s
        quiet = t - self._last_any_motion_t - c.motion_win_s
        if quiet < c.heart_win_s:
            self._heart = None
            self._heart_wait = (f"hold still - nothing may move in range ({max(0.0, quiet):.0f}/"
                                f"{c.heart_win_s:.0f} s)")
            self._heart_bpm = self.heart_tracker.update(t, None)
            self.heart_avg.reset()
            self._expire_last_heart(t)
            return
        x20 = block_decimate(self.raw.last(int(c.heart_win_s * self.fs)), self.fs // c.breath_fs)
        bh = br.bpm / 60 if br is not None and self.detector.breath_ok(br) else None
        self._heart = estimate_heart(x20, c.breath_fs, c.wavelength, bh, avg=self.heart_avg)
        self._heart_bpm = self.heart_tracker.update(t, self._heart)
        self._heart_wait = "" if self._heart_bpm else "measuring..."
        self._expire_last_heart(t)
        if self._heart_bpm is not None:
            if self._heart_locks and t - self._heart_locks[-1][0] > c.heart_last_avg_s:
                self._heart_locks.clear()           # new session: don't mix with an old reading
            self._heart_locks.append((t, self._heart_bpm))
            while t - self._heart_locks[0][0] > c.heart_last_avg_s:
                self._heart_locks.popleft()
            self._heart_last = float(np.mean([b for _, b in self._heart_locks]))
            self._heart_last_t = t

    def update(self) -> Status | None:
        c, t = self.cfg, self.t
        if t < c.motion_win_s:
            return None
        motion_db, motion_abs, n0 = motion_levels(self.hp.last(int(c.motion_win_s * self.fs)), self.fs,
                                                  c.motion_band, c.motion_noise_band, return_noise=True)
        # smoothed white-noise density (robust to single bursts: slow EMA, clipped jumps)
        if self.noise_psd is None:
            self.noise_psd = n0
        else:
            self.noise_psd += 0.05 * (min(n0, 4 * self.noise_psd) - self.noise_psd)
        thr = self.th.motion_db
        if self.floor is not None:
            self.floor.update(t, motion_db, thr)
            thr = self.floor.threshold(thr)
        if self.gate:
            quiet = t - self.detector.last_motion_t - c.motion_win_s
        else:
            quiet = np.inf
        win = min(c.breath_win_s, quiet, t)
        br = None
        if win >= c.breath_min_win_s:
            x = self.raw.last(int(win * self.fs))
            x20 = block_decimate(x, self.fs // c.breath_fs)
            br = estimate_breathing(x20, c.breath_fs, c.wavelength, c.breath_band, c.breath_noise_band)
        still_abs = still_level(self.raw.last(int(c.still_win_s * self.fs)), self.fs, c.still_band,
                                self.fs // c.breath_fs, noise_psd=self.noise_psd)
        state = self.detector.step(t, motion_db, br, motion_abs, thr, still_abs)
        if c.heart:
            self._update_heart(t, self.detector.is_motion(motion_db, motion_abs, thr), br)
        self.history.append((t, motion_db, motion_abs, thr, still_abs))
        far_motion = (self.th.motion_abs_db is not None and motion_db > thr
                      and motion_abs < self.th.motion_abs_db)
        return Status(t=t, state=state, confirmed=self.detector.confirmed, motion_db=motion_db,
                      breath=br, breath_win_s=win if br is not None else 0.0,
                      motion_abs=motion_abs, motion_thr=thr, far_motion=far_motion,
                      still_abs=still_abs, proximity=self.detector.proximity,
                      heart=self._heart, heart_bpm=self._heart_bpm, heart_wait=self._heart_wait,
                      heart_last_bpm=self._heart_last,
                      heart_last_age_s=t - self._heart_last_t if self._heart_last is not None else 0.0,
                      heart_last_span_s=(self._heart_locks[-1][0] - self._heart_locks[0][0]
                                         if self._heart_locks else 0.0),
                      jumps=self.jump.jumps if self.jump else 0)


def boundary_level(statuses: list[Status], min_frames: int = 10) -> float | None:
    """Absolute motion level of someone moving at the boundary distance.

    Uses only updates where motion was clearly detected (ratio above threshold) and returns their
    median level, so a person exactly at the boundary is detected about half the time and anyone
    closer reliably (echo power grows ~R^-4 as they approach).
    """
    lv = [s.motion_abs for s in statuses if s.motion_db > s.motion_thr]
    if len(lv) < min_frames:
        return None
    return float(np.median(lv))


def format_status(s: Status) -> str:
    label = s.state if s.state != STILL or s.confirmed else ("PRESENT" if s.proximity else "STILL?")
    txt = (f"t={s.t:7.1f}s  {label:7s} motion={s.motion_db:5.1f}/{s.motion_thr:4.1f} dB "
           f"level={s.motion_abs:6.1f} dBFS{' (far, ignored)' if s.far_motion else ''} near={s.still_abs:6.1f} dBFS")
    if s.breath is not None:
        b = s.breath
        txt += f"  breath={b.bpm:5.1f} bpm snr={b.snr_db:5.1f} dB acf={b.acf:4.2f} ({b.method})"
    else:
        txt += "  breath=   --"
    if s.heart is not None or s.heart_wait:
        txt += (f"  heart={s.heart_bpm:5.1f} bpm (locked)" if s.heart_bpm is not None else
                f"  heart~{s.heart.bpm:5.1f} ({s.heart.snr_db:4.1f} dB)" if s.heart is not None else "  heart: wait")
        if s.heart_bpm is None and s.heart_last_bpm is not None:
            txt += f"  last={s.heart_last_bpm:.0f} bpm ({s.heart_last_age_s:.0f}s ago)"
    if s.jumps:
        txt += f"  phase-jumps={s.jumps}"
    return txt
