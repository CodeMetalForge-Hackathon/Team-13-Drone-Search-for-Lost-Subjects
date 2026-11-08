"""2D people map from the bladeRF MIMO multi-tone CW radar.

Virtual channels: 2 RX x 2 TX x K tones (Config.channels()). A scatterer at p appears in channel c
with phase -2 pi f_c (|p - tx| + |p - rx|) / c0:
  * the RX/TX positions (4-element virtual array) give the angle,
  * the tone frequencies (phase slope across ~6 MHz) give the range.

Every channel is referenced to its own static TX->RX leakage phase (a slow running mean). The
leakage passes through exactly the same TX and RX chains as the echoes, so per-session stream
offsets, cable / filter phases and LO drift cancel; only the geometry remains, which is modelled
exactly (near field, bistatic). `calibrate --array` can refine the residual per-channel phases.

Noise is not white across channels on real hardware: LO phase noise on the huge leakage is common
to all channels. Its spatial covariance is measured in the motion-free 70-95 Hz Doppler band and
both data and steering vectors are whitened with it before localization.

People are separated by *Doppler*: each frequency bin of the slow-time spectrum is assumed to be
dominated by one scatterer and is localized on its own (fast map: walking / gesturing, 2-60 Hz over
1 s; slow map: breathing / sway, 0.15-2 Hz over 10 s). The per-bin maps are summed, peaks become
detections, and a Kalman / Hungarian tracker keeps stable person IDs.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage, signal
from scipy.optimize import linear_sum_assignment

from .config import SPEED_OF_LIGHT, Config, Thresholds
from .detector import LockedReading, Pipeline, Ring, Status
from .dsp import BreathResult, ClutterFilter, PhaseJumpCorrector, block_decimate, estimate_breathing

MOVING_T, STILL_T = "moving", "still"


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------
class SteeringGrid:
    """Leakage-referenced steering vectors on the (x, y) map grid: A[g, c], rows of unit norm."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        r = cfg.map_res
        self.xs = np.arange(cfg.map_x[0], cfg.map_x[1] + r / 2, r)
        self.ys = np.arange(cfg.map_y[0], cfg.map_y[1] + r / 2, r)
        gx, gy = np.meshgrid(self.xs, self.ys)          # [ny, nx]
        self.shape = gx.shape
        self.gx, self.gy = gx.ravel(), gy.ravel()
        chans = cfg.channels()
        self.k = np.array([2 * np.pi * f / SPEED_OF_LIGHT for *_, f in chans])
        self.rx = np.array(cfg.rx_pos, float)[[c[0] for c in chans]]
        self.tx = np.array(cfg.tx_pos, float)[[c[1] for c in chans]]
        self.leak = np.hypot(*(self.tx - self.rx).T)
        self.A = self.steer(self.gx, self.gy).astype(np.complex64)
        self.W = np.eye(len(self.k))
        self.AHw = np.ascontiguousarray(self.A.conj())

    def set_whitening(self, W: np.ndarray) -> None:
        """Use whitened steering vectors W a (unit norm) from now on; data must be whitened with W."""
        self.W = W
        Aw = self.A @ W.T.astype(np.complex64)
        Aw /= np.linalg.norm(Aw, axis=1, keepdims=True)
        self.AHw = np.ascontiguousarray(Aw.conj())

    def steer_w(self, x, y) -> np.ndarray:
        a = self.W @ self.steer(x, y)
        return a / np.linalg.norm(a)

    def steer(self, x, y) -> np.ndarray:
        """[len(x), C] (or [C] for scalars) unit-norm steering vectors."""
        scalar = np.ndim(x) == 0
        x, y = np.atleast_1d(x).astype(float), np.atleast_1d(y).astype(float)
        dt = np.hypot(x[:, None] - self.tx[None, :, 0], y[:, None] - self.tx[None, :, 1])
        dr = np.hypot(x[:, None] - self.rx[None, :, 0], y[:, None] - self.rx[None, :, 1])
        a = np.exp(-1j * self.k[None, :] * (dt + dr - self.leak[None, :])) / np.sqrt(len(self.k))
        return a[0] if scalar else a

    def coherence(self, Y: np.ndarray) -> np.ndarray:
        """|a^H y|^2 / |y|^2 (whitened a and y) for every grid point and every column of Y[C, nb]
        -> [G, nb] in [0, 1]."""
        Yn = Y / (np.linalg.norm(Y, axis=0, keepdims=True) + 1e-30)
        return np.abs(self.AHw @ Yn.astype(np.complex64)) ** 2


def bin_map(grid: SteeringGrid, Y: np.ndarray, snr: np.ndarray, min_coh: float = 0.9) -> np.ndarray:
    """Sum of per-bin localization maps, each peaking at 1 at its bin's best position.

    Per bin, the coherence B is sharpened to 1 / (1 - B + 1/snr): a high-SNR bin localizes sharply,
    a weak bin only contributes a broad hint. Bins are weighted by log SNR.
    """
    if Y.shape[1] == 0:
        return np.zeros(grid.shape)
    B = grid.coherence(Y)
    # a bin holding one scatterer matches some steering vector almost perfectly (coherence near
    # snr/(snr+1)); bins shared by two people match nothing well - skip them, they map to ghosts
    expect = np.maximum(snr - 1.0, 0.0) / snr
    single = B.max(axis=0) >= min_coh * expect
    if not single.any():
        return np.zeros(grid.shape)
    B, snr, Y = B[:, single], snr[single], Y[:, single]
    eps = 1.0 / np.clip(snr * Y.shape[0], 1.0, 1e4)
    S = 1.0 / (1.0 - np.minimum(B, 1.0) + eps[None, :])
    S /= S.max(axis=0, keepdims=True)
    w = np.log10(np.maximum(snr, 1.0001))
    m = (S @ w) / (w.sum() + 1e-12)
    return m.reshape(grid.shape)


def _detrend(X: np.ndarray) -> np.ndarray:
    n = X.shape[-1]
    tc = np.arange(n) - (n - 1) / 2
    X = X - X.mean(axis=-1, keepdims=True)
    return X - (X @ tc / (tc @ tc))[..., None] * tc


def hann_fft(X: np.ndarray, fs: float):
    n = X.shape[-1]
    return np.fft.fft(_detrend(X) * np.hanning(n), axis=-1), np.fft.fftfreq(n, 1.0 / fs)


def spectrum_bins(X: np.ndarray, fs: float, band, max_bins: int, det_db: float,
                  noise_bin: float | None = None, noise_band=None):
    """Hann FFT of [C, n]; returns (Y_sel[C, nb], snr[nb], freqs[nb], noise per bin)."""
    return select_bins(*hann_fft(X, fs), band, max_bins, det_db, noise_bin, noise_band)


def select_bins(Y: np.ndarray, f: np.ndarray, band, max_bins: int, det_db: float,
                noise_bin: float | None = None, noise_band=None):
    P = np.sum(np.abs(Y) ** 2, axis=0)
    af = np.abs(f)
    if noise_bin is None:
        nb = (af >= noise_band[0]) & (af <= noise_band[1])
        noise_bin = float(np.mean(P[nb])) if nb.any() else float(np.median(P))
    inb = np.flatnonzero((af >= band[0]) & (af <= band[1]))
    snr = P[inb] / (noise_bin + 1e-30)
    keep = inb[snr > 10 ** (det_db / 10)]
    if len(keep) > max_bins:
        keep = keep[np.argsort(P[keep])[-max_bins:]]
    return Y[:, keep], P[keep] / (noise_bin + 1e-30), f[keep], noise_bin


@dataclass
class Detection:
    x: float
    y: float
    kind: str           # MOVING_T / STILL_T
    strength: float


def find_peaks(m: np.ndarray, grid: SteeringGrid, cfg: Config, kind: str,
               rel: float | None = None, max_n: int | None = None) -> list[Detection]:
    if not np.any(m > 0):
        return []
    rel = cfg.map_peak_rel if rel is None else rel
    size = max(3, int(round(cfg.map_min_sep_m / cfg.map_res)) | 1)
    loc = (m == ndimage.maximum_filter(m, size=size, mode="nearest")) & (m >= rel * m.max())
    # a maximum on the map border is someone (or an artefact) outside the map: not a detection
    loc[0, :] = loc[-1, :] = loc[:, 0] = loc[:, -1] = False
    iy, ix = np.nonzero(loc)
    out: list[Detection] = []
    for i in np.argsort(m[iy, ix])[::-1]:
        d = Detection(float(grid.xs[ix[i]]), float(grid.ys[iy[i]]), kind, float(m[iy[i], ix[i]]))
        # range is the weak dimension: a weaker maximum on (almost) the same bearing and not far
        # away in range is the stronger one's range sidelobe, not a second person
        az, r = np.arctan2(d.x, d.y), np.hypot(d.x, d.y)
        if any(abs(np.degrees(az - np.arctan2(o.x, o.y))) < cfg.map_same_bearing_deg
               and abs(r - np.hypot(o.x, o.y)) < cfg.map_same_bearing_m for o in out):
            continue
        out.append(d)
        if len(out) >= (max_n or cfg.max_targets):
            break
    return out


# ---------------------------------------------------------------------------
# Tracking
# ---------------------------------------------------------------------------
@dataclass
class Track:
    id: int
    x: np.ndarray                   # [px, py, vx, vy]
    P: np.ndarray
    t_last: float
    t_born: float
    hits: int = 1
    kind: str = MOVING_T
    last_moving_t: float = -np.inf
    still_since: float = 0.0
    trail: list = field(default_factory=list)
    breath: BreathResult | None = None
    reading: LockedReading | None = None
    confirmed: bool = False
    born_still: bool = False

    @property
    def pos(self) -> tuple[float, float]:
        return float(self.x[0]), float(self.x[1])

    @property
    def speed(self) -> float:
        return float(np.hypot(self.x[2], self.x[3]))

    @property
    def bpm(self) -> float | None:
        if self.reading is None:
            return None
        return self.reading.live if self.reading.live is not None else self.reading.last

    def snapshot(self) -> "TrackView":
        return TrackView(id=self.id, x=float(self.x[0]), y=float(self.x[1]), vx=float(self.x[2]),
                         vy=float(self.x[3]), kind=self.kind, bpm=self.bpm,
                         bpm_live=self.reading is not None and self.reading.live is not None,
                         breath=self.breath, trail=[p for _, p in self.trail], age_s=self.t_last - self.t_born)


@dataclass
class TrackView:
    """Immutable copy of a confirmed track for display / logging."""
    id: int
    x: float
    y: float
    vx: float
    vy: float
    kind: str
    bpm: float | None
    bpm_live: bool
    breath: BreathResult | None
    trail: list
    age_s: float

    @property
    def pos(self) -> tuple[float, float]:
        return self.x, self.y

    @property
    def speed(self) -> float:
        return float(np.hypot(self.vx, self.vy))


class MultiTracker:
    """Constant-velocity Kalman tracks, Hungarian assignment with a distance gate."""

    def __init__(self, cfg: Config, q: float = 3.0, r_m: float = 0.35):
        self.cfg, self.q, self.R = cfg, q, np.eye(2) * r_m ** 2
        self.tracks: list[Track] = []
        self._next_id = 1
        self.t = 0.0

    def _predict(self, tr: Track, t: float) -> None:
        dt = t - tr.t_last if t > tr.t_last else 0.0
        if dt <= 0:
            return
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        tr.x = F @ tr.x
        if tr.kind == STILL_T:
            tr.x[2:] *= 0.5            # still people do not drift away
        tr.P = F @ tr.P @ F.T + self.q * G @ G.T

    def update(self, t: float, dets: list[Detection]) -> list[Track]:
        c = self.cfg
        self.t = t
        used = set()
        if self.tracks and dets:
            pred = [tr.x[:2] + (tr.x[2:] * min(t - tr.t_last, 1.0) if tr.kind == MOVING_T else 0.0)
                    for tr in self.tracks]
            D = np.array([[np.hypot(p[0] - d.x, p[1] - d.y) for d in dets] for p in pred])
            # the gate grows while a track coasts (a walker keeps going while undetected)
            gate = [c.track_gate_still_m if tr.kind == STILL_T else
                    c.track_gate_m + min(1.0 * (t - tr.t_last), 2.0) for tr in self.tracks]
            rows, cols = linear_sum_assignment(D)
            for i, j in zip(rows, cols):
                if D[i, j] > gate[i]:
                    continue
                self._correct(self.tracks[i], dets[j], t)
                used.add(j)
        for j, d in enumerate(dets):
            if j not in used and not any(np.hypot(tr.x[0] - d.x, tr.x[1] - d.y) < c.map_min_sep_m
                                         for tr in self.tracks):
                self.tracks.append(Track(self._next_id, np.array([d.x, d.y, 0.0, 0.0]),
                                         np.diag([0.3, 0.3, 1.0, 1.0]), t, t, kind=d.kind,
                                         last_moving_t=t if d.kind == MOVING_T else -np.inf,
                                         still_since=t, born_still=d.kind == STILL_T))
                self._next_id += 1
        keep = []
        for tr in self.tracks:
            age = t - tr.t_last
            if tr.confirmed and age > (c.track_drop_still_s if tr.kind == STILL_T else c.track_drop_s):
                continue
            if not tr.confirmed and age > (2.5 if tr.born_still else 1.5):
                continue
            keep.append(tr)
        # merge tracks that converged onto the same person (keep the older one)
        keep.sort(key=lambda tr: tr.t_born)
        merged = []
        for tr in keep:
            if any(np.hypot(tr.x[0] - o.x[0], tr.x[1] - o.x[1]) < c.map_min_sep_m * 0.75 for o in merged):
                continue
            merged.append(tr)
        self.tracks = merged
        # tracks are kept (coasting) for ID continuity, but only shown while being refreshed
        return [tr for tr in self.tracks if tr.confirmed and t - tr.t_last <= (
            c.track_show_still_s if tr.kind == STILL_T else c.track_show_s)]

    def _correct(self, tr: Track, d: Detection, t: float) -> None:
        self._predict(tr, t)
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1
        z = np.array([d.x, d.y])
        S = H @ tr.P @ H.T + self.R
        K = tr.P @ H.T @ np.linalg.inv(S)
        tr.x = tr.x + K @ (z - H @ tr.x)
        tr.P = (np.eye(4) - K @ H) @ tr.P
        tr.t_last = t
        tr.hits += 1
        # tracks born from still detections need longer evidence (the slow map comes once a second
        # and walkers leave transient leftovers in it)
        need = self.cfg.track_confirm_still if tr.born_still else self.cfg.track_confirm
        if tr.hits >= need:
            tr.confirmed = True
        if d.kind == MOVING_T:
            tr.last_moving_t = t
        # turning around takes a walker through zero Doppler for a second or two: only call a
        # track still after `still_after_s` without any motion detection
        kind = MOVING_T if t - tr.last_moving_t < self.cfg.still_after_s else STILL_T
        if kind == STILL_T and tr.kind != STILL_T:
            tr.still_since = t
            tr.x[2:] = 0.0
        tr.kind = kind
        tr.trail.append((t, tr.pos))
        tr.trail = [p for p in tr.trail if t - p[0] <= max(8.0, self.cfg.map_slow_win_s)]


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------
@dataclass
class MapStatus:
    t: float
    status: Status | None           # single-channel presence / breathing / heart (reference channel)
    heat: np.ndarray                # display map (EMA of fast + slow), [ny, nx], 0..1
    fast_map: np.ndarray
    slow_map: np.ndarray
    detections: list
    tracks: list                    # confirmed tracks (TrackView snapshots)
    n_fast_bins: int = 0
    n_slow_bins: int = 0


def whitener(Rn: np.ndarray, loading: float = 0.05) -> np.ndarray:
    """W with W Rn W^H ~ (tr Rn / C) I: white noise keeps its level, correlated noise is suppressed."""
    C = Rn.shape[0]
    R = Rn / (np.trace(Rn).real / C) + loading * np.eye(C)
    w, V = np.linalg.eigh(R)
    return (V * (1.0 / np.sqrt(np.maximum(w, 1e-6)))) @ V.conj().T * np.sqrt(1 + loading)


def array_cal_vector(th: Thresholds, n: int) -> np.ndarray:
    if th.array_cal is None or len(th.array_cal) != n:
        return np.ones(n, np.complex128)
    return np.array([complex(a, b) for a, b in th.array_cal])


class MapPipeline:
    """Consumes [channels, n] slow-time chunks; one MapStatus per `update_s`."""

    def __init__(self, cfg: Config, th: Thresholds, use_cal: bool = True):
        cfg.validate()
        self.cfg, self.th = cfg, th
        self.fs = cfg.slow_fs
        self.C = cfg.n_channels
        self.legacy = Pipeline(cfg, th)
        self.grid = SteeringGrid(cfg)
        self.cal = array_cal_vector(th, self.C) if use_cal else np.ones(self.C, np.complex128)
        size = int(max(cfg.breath_win_s, cfg.map_slow_win_s, 20.0) * self.fs) + self.fs
        self.raw = Ring(size, self.C)       # leakage-referenced, calibrated
        self.hp = Ring(size, self.C)        # + static clutter removed
        self.clutter = ClutterFilter(cfg.clutter_tau_s, self.fs)
        self.jump = PhaseJumpCorrector(cfg.jump_fix_rad) if cfg.jump_fix_rad > 0 else None
        self.ref: np.ndarray | None = None
        self.tracker = MultiTracker(cfg)
        self.step_n = int(round(cfg.update_s * self.fs))
        self._pending = 0
        self._noise_fast: float | None = None
        self._Rn: np.ndarray | None = None          # noise spatial covariance (70-95 Hz band)
        self._w_next_t = 0.0
        self._slow_next_t = 0.0
        self._slow_map = np.zeros(self.grid.shape)
        self._slow_dets: list[Detection] = []
        self._n_slow = 0
        self._slow_new = False
        self._breath_next_t = 0.0
        self.heat = np.zeros(self.grid.shape)
        self._last_status: Status | None = None
        dec = self.fs // cfg.breath_fs
        self._dec = dec

    @property
    def t(self) -> float:
        return self.raw.count / self.fs

    # --- ingest ----------------------------------------------------------------
    def _ingest(self, x: np.ndarray) -> None:
        if self.jump is not None:
            x = self.jump.process(x)
        m = x.mean(axis=1)
        if self.ref is None:
            self.ref = m
        else:
            a = 1.0 - (1.0 - 1.0 / (self.cfg.leak_tau_s * self.fs)) ** x.shape[1]
            self.ref = self.ref + a * (m - self.ref)
        rot = np.conj(self.ref) / (np.abs(self.ref) + 1e-30) * self.cal
        xn = x * rot[:, None]
        self.raw.push(xn)
        self.hp.push(self.clutter.process(xn))

    def feed(self, x: np.ndarray) -> list[MapStatus]:
        if x.ndim != 2 or x.shape[0] != self.C:
            raise ValueError(f"map needs [{self.C}, n] multichannel data, got shape {x.shape}")
        out = []
        while x.shape[1]:
            take = min(x.shape[1], self.step_n - self._pending)
            chunk = x[:, :take]
            x = x[:, take:]
            for s in self.legacy.feed(chunk[0]):
                self._last_status = s
            self._ingest(chunk)
            self._pending += take
            if self._pending >= self.step_n:
                self._pending = 0
                s = self.update()
                if s is not None:
                    out.append(s)
        return out

    # --- per update ----------------------------------------------------------------
    @property
    def W(self) -> np.ndarray:
        return self.grid.W

    def _fast(self):
        c = self.cfg
        Y, f = hann_fft(self.hp.last(int(c.map_fast_win_s * self.fs)), self.fs)
        af = np.abs(f)
        nb = (af >= c.motion_noise_band[0]) & (af <= c.motion_noise_band[1])
        Yn = Y[:, nb]
        R = Yn @ Yn.conj().T / max(Yn.shape[1], 1)
        self._Rn = R if self._Rn is None else self._Rn + 0.05 * (R - self._Rn)
        if self.t >= self._w_next_t and self.t > 3.0:
            self._w_next_t = self.t + 1.0
            self.grid.set_whitening(whitener(self._Rn))
        Yw = self.W @ Y
        Ysel, snr, _, noise = select_bins(Yw, f, c.motion_band, c.map_max_bins, c.map_det_db,
                                          noise_band=c.motion_noise_band)
        # smoothed white-noise level per bin, used to scale the slow map's noise
        self._noise_fast = noise if self._noise_fast is None else (
            self._noise_fast + 0.05 * (min(noise, 4 * self._noise_fast) - self._noise_fast))
        if Ysel.shape[1] < 2:                    # a single bin over threshold is likely noise
            return np.zeros(self.grid.shape), [], 0
        m = bin_map(self.grid, Ysel, snr)
        return m, find_peaks(m, self.grid, c, MOVING_T), Ysel.shape[1]

    def _mover_positions(self, fdets: list[Detection]) -> list[tuple[float, float]]:
        """Where people moved during the slow window: current motion detections plus each recently
        moving track's trail, thinned to ~0.7 m spacing (at most 4 points per track)."""
        c, t = self.cfg, self.t
        pts = [(d.x, d.y) for d in fdets]
        for tr in self.tracker.tracks:
            if t - tr.last_moving_t >= c.map_slow_win_s:
                continue
            kept: list[tuple[float, float]] = []
            for tp, p in reversed(tr.trail):
                if tp < tr.last_moving_t - c.map_slow_win_s or tp > tr.last_moving_t + 0.5:
                    continue
                # someone who has stopped must stay detectable where they now stand
                if t - tr.last_moving_t > 1.5 and np.hypot(p[0] - tr.x[0], p[1] - tr.x[1]) < 1.2:
                    continue
                if all(np.hypot(p[0] - q[0], p[1] - q[1]) > 0.7 for q in kept):
                    kept.append(p)
                if len(kept) >= 4:
                    break
            pts += kept
        return pts

    def _slow(self, movers: list[tuple[float, float]] = ()):
        """Still people: MUSIC on the spatial covariance of 0.15-2 Hz micro-motion (breathing, sway).

        Unlike the per-bin method this separates several still people even when they breathe at
        the same rate, as long as their motions are not perfectly in step (independent sway).
        The number of people is the number of covariance eigenvalues clearly above the noise.
        Moving people also leave energy in this band; their steering vectors (along their recent
        path) are nulled first, so MUSIC only searches what the movers do not explain.
        """
        c = self.cfg
        X = self.raw.last(int(c.map_slow_win_s * self.fs))
        fs2 = c.breath_fs
        # remove the static part (leakage ~2000 counts) *before* decimating: the FIR decimator pads
        # with zeros, and a DC step at the edges would swamp the 0.15-2 Hz band
        X2 = self.W @ signal.decimate(_detrend(X), self._dec, ftype="fir", axis=-1, zero_phase=True)
        sos = signal.butter(2, c.map_slow_band, btype="bandpass", fs=fs2, output="sos")
        Z = signal.sosfiltfilt(sos, _detrend(X2), axis=-1)
        k = Z.shape[-1] // 8
        Z = Z[:, k:Z.shape[-1] - k]
        if Z.shape[-1] < 4 * self.C:
            return np.zeros(self.grid.shape), [], 0
        Pn = np.eye(self.C, dtype=np.complex128)        # projector away from the movers
        if len(movers):
            B = self.W @ self.grid.steer(*np.array(movers).T).T
            U, sv, _ = np.linalg.svd(B, full_matrices=False)
            Q = U[:, sv > 1e-2 * sv[0]][:, :self.C // 2]
            Pn = Pn - Q @ Q.conj().T
            Z = Pn @ Z
        R = Z @ Z.conj().T / Z.shape[-1]
        w, V = np.linalg.eigh(R)                    # ascending
        # white noise per channel and sample, from the fast path (no human motion at 70-95 Hz):
        # density N0 = P_bin / (fs * sum(w^2)) per channel, times the complex band width
        nf = int(c.map_fast_win_s * self.fs)
        n0 = self._noise_fast / self.C / (self.fs * np.sum(np.hanning(nf) ** 2))
        sigma2 = n0 * 2 * (c.map_slow_band[1] - c.map_slow_band[0])
        thr = sigma2 * 10 ** ((c.map_det_db + 4) / 10)
        n_src = int(min(np.sum(w > thr), c.max_targets, self.C - 2))
        if n_src == 0:
            return np.zeros(self.grid.shape), [], 0
        En = V[:, :self.C - n_src].astype(np.complex64)      # noise subspace
        # projected steering a_p = Pn a (a whitened, unit norm): |En^H a_p|^2 / |a_p|^2
        q = np.sum(np.abs(self.grid.AHw @ (Pn.astype(np.complex64) @ En)) ** 2, axis=1)
        na = np.sum(np.abs(self.grid.AHw @ Pn.astype(np.complex64)) ** 2, axis=1)
        # blind where the movers' nulls remove most of the steering vector (on and near their path)
        P = np.where(na > 0.5, na / np.maximum(q, 1e-6), 0.0)
        m = P.reshape(self.grid.shape)
        if m.max() <= 0:
            return np.zeros(self.grid.shape), [], 0
        dets = find_peaks(m, self.grid, c, STILL_T, rel=0.0, max_n=2 * n_src + 2)
        # a real source's steering vector lies (almost) inside the signal subspace; MUSIC maxima
        # in between sources (or from moving people's leftovers) do not
        Es = V[:, self.C - n_src:]

        def in_signal_space(d):
            a = Pn @ self.grid.steer_w(d.x, d.y)
            return np.sum(np.abs(Es.conj().T @ a) ** 2) / np.vdot(a, a).real >= c.map_subspace_min
        dets = [d for d in dets if in_signal_space(d)][:n_src]
        # display: dB-compressed, 0..1
        disp = np.clip(1 + 10 * np.log10(np.maximum(m, 1e-12) / m.max()) / 15, 0, 1)
        return disp, dets, n_src

    def _breathing(self, t: float, tracks: list[Track]) -> None:
        c = self.cfg
        th = self.th
        for tr in tracks:
            if tr.reading is None:
                tr.reading = LockedReading(c.breath_lock_s, c.breath_lock_span_bpm,
                                           c.heart_last_avg_s, c.heart_last_hold_s)
            win = min(c.breath_win_s, t - tr.still_since) if tr.kind == STILL_T else 0.0
            br = None
            if win >= c.breath_min_win_s:
                a = self.grid.steer(*tr.pos)
                s = a.conj() @ self.raw.last(int(win * self.fs))
                br = estimate_breathing(block_decimate(s, self._dec), c.breath_fs, c.wavelength,
                                        c.breath_band, c.breath_noise_band)
            tr.breath = br
            ok = br is not None and br.snr_db >= th.breath_snr_db and br.acf >= th.breath_acf
            tr.reading.update(t, br.bpm if br is not None else None, ok)

    def update(self) -> MapStatus | None:
        c, t = self.cfg, self.t
        if t < c.map_fast_win_s:
            return None
        fast, fdets, nfb = self._fast()
        if t >= self._slow_next_t and t >= c.map_slow_win_s * 0.5:
            self._slow_next_t = t + c.map_slow_every_s
            self._slow_map, self._slow_dets, self._n_slow = self._slow(self._mover_positions(fdets))
            self._slow_new = True
        # a walker also leaves energy in the slow band all along its recent path: keep slow
        # detections away from where anyone moved during the slow window
        movers = [(d.x, d.y) for d in fdets]
        for tr in self.tracker.tracks:
            if t - tr.last_moving_t < c.map_slow_win_s:
                movers += [p for tp, p in tr.trail if tp >= tr.last_moving_t - c.map_slow_win_s]
        # ...unless it sits on a track's current position (a walker who just stopped keeps its ID)
        here = [tr.pos for tr in self.tracker.tracks if t - tr.last_moving_t > c.still_after_s]
        sdets = [d for d in self._slow_dets
                 if any(np.hypot(d.x - hx, d.y - hy) < c.track_gate_m for hx, hy in here)
                 or all(np.hypot(d.x - mx, d.y - my) > 1.5 for mx, my in movers)] if self._slow_new else []
        self._slow_new = False
        tracks = self.tracker.update(t, fdets + sdets)
        if t >= self._breath_next_t:
            self._breath_next_t = t + c.track_breath_every_s
            self._breathing(t, tracks)
        combined = np.maximum(fast, self._slow_map * 0.7)
        self.heat += 0.35 * (combined - self.heat)
        return MapStatus(t=t, status=self._last_status, heat=self.heat.copy(), fast_map=fast,
                         slow_map=self._slow_map, detections=fdets + sdets,
                         tracks=[tr.snapshot() for tr in tracks],
                         n_fast_bins=nfb, n_slow_bins=self._n_slow)


# ---------------------------------------------------------------------------
# Array calibration
# ---------------------------------------------------------------------------
def calibrate_array(raw: np.ndarray, cfg: Config, x0: float, y0: float,
                    W: np.ndarray | None = None) -> tuple[np.ndarray, float]:
    """Per-channel phase correction from a person swaying / breathing at a known spot (x0, y0).

    raw: leakage-referenced, *uncalibrated* slow time [C, n] (MapPipeline(use_cal=False).raw).
    W: the pipeline's noise whitener (common-mode leakage phase noise must not win the eigenvector).
    The dominant eigenvector of the slow-band covariance is that person's array response v; the
    correction makes it match the model: cal = exp(j (angle(a) - angle(v))).
    Returns (cal[C], quality = fraction of slow-band power in the dominant eigenvector).
    """
    dec = cfg.slow_fs // cfg.breath_fs
    W = np.eye(raw.shape[0]) if W is None else W
    X2 = W @ signal.decimate(_detrend(raw), dec, ftype="fir", axis=-1, zero_phase=True)
    sos = signal.butter(2, cfg.map_slow_band, btype="bandpass", fs=cfg.breath_fs, output="sos")
    Z = signal.sosfiltfilt(sos, _detrend(X2), axis=-1)
    k = Z.shape[-1] // 8
    Z = Z[:, k:-k]
    R = Z @ Z.conj().T / Z.shape[-1]
    w, V = np.linalg.eigh(R)
    v = np.linalg.solve(W, V[:, -1])                # back to the unwhitened channel domain
    a = SteeringGrid(cfg).steer(x0, y0)
    cal = np.exp(1j * (np.angle(a) - np.angle(v)))
    cal *= np.exp(-1j * np.angle(cal[0]))
    return cal, float(w[-1] / (w.sum() + 1e-30))


def format_map_status(s: MapStatus) -> str:
    parts = [f"t={s.t:7.1f}s  people={len(s.tracks)}"]
    for tr in sorted(s.tracks, key=lambda tr: tr.id):
        x, y = tr.pos
        r, az = np.hypot(x, y), np.degrees(np.arctan2(x, y))
        txt = f"#{tr.id} ({x:+5.1f},{y:4.1f}) m r={r:3.1f} m az={az:+4.0f}deg {tr.kind}"
        if tr.kind == MOVING_T:
            txt += f" {tr.speed:3.1f} m/s"
        elif tr.bpm is not None:
            txt += f" {tr.bpm:4.1f} bpm"
        parts.append(txt)
    return "  |  ".join(parts)
