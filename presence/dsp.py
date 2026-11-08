"""Signal processing for multi-tone CW Doppler radar (pure numpy/scipy, no hardware)."""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from scipy import signal


# ---------------------------------------------------------------------------
# Fast time -> slow time
# ---------------------------------------------------------------------------
class ToneDemodulator:
    """Mixes the received tone to DC and integrate-and-dumps to the slow-time rate.

    Keeps a running sample counter so the mixing reference stays phase-continuous
    across buffers of any length (leftover samples are carried to the next call).
    """

    def __init__(self, tone_freq: float, sample_rate: int, dec: int):
        self.f = tone_freq / sample_rate      # cycles per sample
        self.dec = dec
        self.n = 0                            # samples consumed so far (incl. leftover)
        self._left = np.zeros(0, np.complex128)
        self._cache: dict[tuple[int, int], np.ndarray] = {}

    def _ref(self, n0: int, length: int) -> np.ndarray:
        cyc0 = (n0 * self.f) % 1.0
        key = (round(cyc0 * 1e9), length)
        ref = self._cache.get(key)
        if ref is None:
            ref = np.exp(-2j * np.pi * (cyc0 + self.f * np.arange(length)))
            if len(self._cache) > 64:
                self._cache.clear()
            self._cache[key] = ref
        return ref

    def process(self, buf: np.ndarray) -> np.ndarray:
        buf = np.asarray(buf, np.complex128)
        n0 = self.n - len(self._left)         # absolute index of first sample we hold
        data = np.concatenate([self._left, buf]) if len(self._left) else buf
        self.n += len(buf)
        usable = len(data) // self.dec * self.dec
        self._left = data[usable:].copy()
        if usable == 0:
            return np.zeros(0, np.complex128)
        mixed = data[:usable] * self._ref(n0, usable)
        return mixed.reshape(-1, self.dec).mean(axis=1)


class MultiToneDemodulator:
    """Integrate-and-dump demodulation of many tones on several RX streams at once.

    All tones lie on a grid with period `period` samples (tone_grid = fs / period) and `dec` is a
    multiple of that period, so the mixing reference is identical in every output block. Each block
    is first folded modulo the period (sum of dec/period rows), then one small matrix product
    against a (period x n_tones) reference gives every tone: ~10x cheaper than mixing each tone.

    process(buf[n_rx, n]) -> slow time [n_rx * n_tones, n_blocks], rx-major (rx0 tones, rx1 tones).
    Leftover samples are carried to the next call, so blocks stay aligned to the stream start.
    """

    def __init__(self, tones, sample_rate: int, dec: int, period: int, n_rx: int = 2):
        if dec % period:
            raise ValueError("dec must be a multiple of the tone period")
        f = np.asarray(tones, float)
        if np.any(np.abs(f * period / sample_rate - np.round(f * period / sample_rate)) > 1e-9):
            raise ValueError("every tone must complete whole cycles in one period")
        m = np.arange(period)
        self.ref = (np.exp(-2j * np.pi * np.outer(m, f) / sample_rate) / dec).astype(np.complex64)
        self.dec, self.period, self.n_rx, self.n_tones = dec, period, n_rx, len(f)
        self._left = np.zeros((n_rx, 0), np.complex64)

    def process(self, buf: np.ndarray) -> np.ndarray:
        buf = np.asarray(buf, np.complex64).reshape(self.n_rx, -1)
        data = np.concatenate([self._left, buf], axis=1) if self._left.shape[1] else buf
        usable = data.shape[1] // self.dec * self.dec
        self._left = data[:, usable:].copy()
        nb = usable // self.dec
        if nb == 0:
            return np.zeros((self.n_rx * self.n_tones, 0), np.complex128)
        folded = data[:, :usable].reshape(self.n_rx, nb, self.dec // self.period, self.period).sum(axis=2)
        out = folded @ self.ref                     # (n_rx, nb, n_tones)
        return out.transpose(0, 2, 1).reshape(self.n_rx * self.n_tones, nb).astype(np.complex128)


def block_decimate(x: np.ndarray, factor: int) -> np.ndarray:
    """Block-average along the last axis (keeps the most recent samples)."""
    n = x.shape[-1] // factor * factor
    return x[..., x.shape[-1] - n:].reshape(*x.shape[:-1], -1, factor).mean(axis=-1)


# ---------------------------------------------------------------------------
# Clutter handling
# ---------------------------------------------------------------------------
class ClutterFilter:
    """Stateful one-pole running mean; output = input - static clutter estimate.

    Works on 1D streams or on [channels, n] (independent state per channel, along the last axis).
    """

    def __init__(self, tau_s: float, fs: float):
        self.a = min(1.0, 1.0 / (tau_s * fs))
        self._zi = None

    def process(self, x: np.ndarray) -> np.ndarray:
        if x.shape[-1] == 0:
            return x
        b, a = [self.a], [1.0, -(1.0 - self.a)]
        if self._zi is None:
            # steady state for a constant input equal to the first sample
            self._zi = ((1.0 - self.a) * x[..., :1]).astype(np.complex128)
        mean, self._zi = signal.lfilter(b, a, x, axis=-1, zi=self._zi)
        return x - mean

    def reset(self) -> None:
        self._zi = None


class PhaseJumpCorrector:
    """Undo sudden rotations of the whole (clutter-dominated) slow-time vector.

    If the radio drops samples (or TX underruns), each received tone shifts phase relative to our
    reference, which rotates everything, clutter included. The static clutter dominates, so its
    phase should never move much between two chunks 0.1 s apart. 1D or [channels, n]: each
    channel (tone) is corrected on its own, since the jump is 2*pi*f*dt, different per tone.
    """

    def __init__(self, threshold_rad: float):
        self.thr = threshold_rad
        self.rot = None
        self.prev = None
        self.jumps = 0

    def process(self, chunk: np.ndarray) -> np.ndarray:
        if chunk.shape[-1] == 0:
            return chunk
        if self.rot is None:
            self.rot = np.ones(chunk.shape[:-1], np.complex128)
        out = chunk * self.rot[..., None]
        m = out.mean(axis=-1)
        if self.prev is not None:
            ok = (np.abs(m) > 0) & (np.abs(self.prev) > 0)
            d = np.where(ok, np.angle(m * np.conj(self.prev)), 0.0)
            jump = np.abs(d) > self.thr
            if np.any(jump):
                corr = np.where(jump, np.exp(-1j * d), 1.0)
                self.rot = self.rot * corr
                out = out * corr[..., None]
                m = m * corr
                self.jumps += 1
        self.prev = m
        return out


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------
def motion_levels(xc: np.ndarray, fs: float, band=(2.0, 60.0), noise_band=(70.0, 95.0),
                  full_scale: float = 2048.0, return_noise: bool = False):
    """Motion in one window of clutter-removed slow time.

    Returns (ratio_db, abs_dbfs):
      ratio_db - Doppler power density in the human-motion band relative to a motion-free band.
                 Self-normalising (tracks the live noise floor), used for "is anything moving".
      abs_dbfs - absolute power of the motion component (noise floor subtracted), dB re ADC full
                 scale. Falls ~R^-4 with distance, used as a soft range boundary.
    """
    n = len(xc)
    if n < 16:
        return 0.0, -200.0
    # linear detrend removes residual clutter drift that would leak into the low Doppler bins
    t = np.arange(n)
    xr = xc - (np.polyval(np.polyfit(t, xc.real, 1), t) + 1j * np.polyval(np.polyfit(t, xc.imag, 1), t))
    w = np.hanning(n)
    P = np.abs(np.fft.fft(xr * w)) ** 2 / (n * np.sum(w ** 2))   # Parseval: sum(P) = mean power
    af = np.abs(np.fft.fftfreq(n, 1.0 / fs))
    mb = (af >= band[0]) & (af <= band[1])
    pm = P[mb].mean()
    pn = P[(af >= noise_band[0]) & (af <= noise_band[1])].mean()
    ratio_db = float(10 * np.log10((pm + 1e-30) / (pn + 1e-30)))
    excess = max(pm - pn, 0.0) * mb.sum()
    abs_dbfs = float(10 * np.log10(excess / full_scale ** 2 + 1e-20))
    if return_noise:
        return ratio_db, abs_dbfs, float(pn * n / fs)   # noise density, counts^2 per Hz
    return ratio_db, abs_dbfs


@lru_cache(maxsize=16)
def _bandpass(band: tuple, fs: float, order: int = 2):
    return signal.butter(order, band, btype="bandpass", fs=fs, output="sos")


def still_level(x: np.ndarray, fs: float, band=(0.15, 2.0), dec: int = 10,
                full_scale: float = 2048.0, noise_psd: float = 0.0) -> float:
    """Absolute power of near-DC micro-motion (breathing, posture sway, fidgets), dBFS.

    Operates on raw (clutter-included) slow time; the static clutter and its slow drift are
    removed by a linear detrend + band-pass. Like the motion level this falls ~R^-4 with distance,
    so it acts as a proximity-sensitive presence signal for people who are not walking.
    `noise_psd` (white noise density, counts^2/Hz) is subtracted so an empty scene reads very low.
    """
    y = x[len(x) % dec:].reshape(-1, dec).mean(axis=1)
    n = len(y)
    fs2 = fs / dec
    if n < 4 * fs2:
        return -200.0
    t = np.arange(n)
    y = y - (np.polyval(np.polyfit(t, y.real, 1), t) + 1j * np.polyval(np.polyfit(t, y.imag, 1), t))
    z = signal.sosfiltfilt(_bandpass(tuple(band), fs2), y)
    k = n // 6                     # drop filter edge transients
    p = np.mean(np.abs(z[k:n - k]) ** 2)
    p = max(p - noise_psd * 2 * (band[1] - band[0]), 0.0)     # complex: +/- band
    return float(10 * np.log10(p / full_scale ** 2 + 1e-20))


def motion_metric(xc: np.ndarray, fs: float, band=(2.0, 60.0), noise_band=(70.0, 95.0)) -> float:
    """Doppler power in the human-motion band relative to a motion-free band (dB)."""
    return motion_levels(xc, fs, band, noise_band)[0]


def doppler_spectrogram(xc: np.ndarray, fs: float, nperseg: int = 128, overlap: float = 0.75):
    """Returns (freqs [fftshifted], times, power_dB[freq, time])."""
    noverlap = int(nperseg * overlap)
    f, t, Z = signal.stft(xc, fs, window="hann", nperseg=nperseg, noverlap=noverlap,
                          return_onesided=False, boundary=None, padded=False)
    f = np.fft.fftshift(f)
    P = 20 * np.log10(np.abs(np.fft.fftshift(Z, axes=0)) + 1e-12)
    return f, t, P


# ---------------------------------------------------------------------------
# Breathing
# ---------------------------------------------------------------------------
def fit_circle(z: np.ndarray):
    """Algebraic (Kasa) circle fit in the IQ plane -> (center, radius, rms residual)."""
    x, y = z.real, z.imag
    A = np.column_stack([x, y, np.ones_like(x)])
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2, sol[1] / 2
    r2 = sol[2] + cx * cx + cy * cy
    if not np.isfinite(r2) or r2 <= 0:
        return None
    r = np.sqrt(r2)
    resid = np.sqrt(np.mean((np.abs(z - (cx + 1j * cy)) - r) ** 2))
    return cx + 1j * cy, r, resid


@dataclass
class BreathResult:
    bpm: float
    snr_db: float
    acf: float
    wave: np.ndarray        # band-passed breathing waveform (mm if method == "arc")
    method: str             # "arc" (circle-fit phase) or "pca" (principal-axis projection)


def _pca_projection(z: np.ndarray) -> np.ndarray:
    z = z - z.mean()
    M = np.vstack([z.real, z.imag])
    _, vecs = np.linalg.eigh(M @ M.T)
    return vecs[:, -1] @ M


def estimate_breathing(x: np.ndarray, fs: float, wavelength: float,
                       band=(0.1, 0.6), noise_band=(0.8, 3.0)) -> BreathResult | None:
    """Estimate breathing from raw (clutter-included) slow-time IQ at `fs` (~20 Hz)."""
    n = len(x)
    if n < fs * 8:
        return None
    # remove slow clutter/LO drift (below ~0.6 x lowest breathing rate), keep the mean so the
    # IQ arc geometry (circle centre) is preserved for the phase demodulation below
    sos_hp = signal.butter(2, band[0] * 0.6, btype="highpass", fs=fs, output="sos")
    x = x.mean() + signal.sosfiltfilt(sos_hp, x - x.mean())

    s, method = None, "pca"
    cf = fit_circle(x)
    if cf is not None:
        center, r, resid = cf
        phase = np.unwrap(np.angle(x - center))
        if np.ptp(phase) > 0.5 and resid < 0.25 * r:
            s = phase * wavelength / (4 * np.pi) * 1e3  # mm
            method = "arc"
    if s is None:
        s = _pca_projection(x)
    s = signal.detrend(s)

    nfft = max(4096, 1 << int(np.ceil(np.log2(8 * n))))
    S = np.abs(np.fft.rfft(s * np.hanning(n), nfft)) ** 2
    f = np.fft.rfftfreq(nfft, 1.0 / fs)
    inb = np.flatnonzero((f >= band[0]) & (f <= band[1]))
    k = inb[np.argmax(S[inb])]
    # parabolic interpolation of the peak
    if 0 < k < len(S) - 1:
        a, b, c = np.log(S[k - 1:k + 2] + 1e-30)
        den = a - 2 * b + c
        delta = 0.5 * (a - c) / den if den != 0 else 0.0
        if not np.isfinite(delta):
            delta = 0.0
        delta = float(np.clip(delta, -0.5, 0.5))
    else:
        delta = 0.0
    f_peak = f[k] + delta * (f[1] - f[0])

    # Spectral concentration: peak vs. everything else from band[0]/2 up to noise_band[1]
    # (excluding the peak and its 2nd harmonic). Red drift noise spreads its power over the
    # low bins and scores low; a breathing line stands out.
    half = 0.04
    near = lambda fc: np.abs(f - fc) <= half
    p_peak = S[near(f_peak)].mean()
    ref = (f >= band[0] / 2) & (f <= noise_band[1]) & ~near(f_peak) & ~near(2 * f_peak)
    p_noise = S[ref].mean()
    snr_db = float(10 * np.log10((p_peak + 1e-30) / (p_noise + 1e-30)))
    # a maximum sitting on the lower band edge (not a true local peak) is drift, not breathing
    if S[k - 1] >= S[k]:
        snr_db = min(snr_db, 0.0)

    sos = signal.butter(2, band, btype="bandpass", fs=fs, output="sos")
    wave = signal.sosfiltfilt(sos, s)
    lag = int(round(fs / f_peak))
    if lag < n // 2:
        e = np.mean(wave ** 2)
        acf = float(np.mean(wave[:-lag] * wave[lag:]) / e) if e > 0 else 0.0
    else:
        acf = 0.0
    return BreathResult(bpm=float(60 * f_peak), snr_db=snr_db, acf=acf, wave=wave, method=method)
