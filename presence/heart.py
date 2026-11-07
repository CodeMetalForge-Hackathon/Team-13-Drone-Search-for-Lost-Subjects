"""Heart-rate estimation from CW-Doppler slow time.

Heartbeat chest displacement is ~0.1-0.5 mm (0.01-0.05 rad at 2.45 GHz), 20-40x smaller than
breathing, and its band (0.8-2 Hz) overlaps the breathing harmonics. So:
  1. demodulate displacement (circle-fit phase or PCA), as for breathing
  2. subtract the breathing waveform (least-squares fit of breathing harmonics k*f_b) and high-pass
  3. harmonic-sum spectrum S(f) = P(f) + 0.5 P(2f) over 48-120 bpm (radar heart signals often carry
     much of their energy in the 2nd harmonic), with bins on breathing harmonics suppressed
  4. confidence = peak vs median of the spectrum (self-referenced: no empty-room calibration)
A tracker then requires a stable rate over several seconds before reporting a lock.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy import signal

from .dsp import _bandpass, _pca_projection, fit_circle


@dataclass
class HeartResult:
    bpm: float
    snr_db: float
    wave: np.ndarray        # heart-band waveform (breathing removed)
    freqs: np.ndarray       # Hz, for `score`
    score: np.ndarray       # harmonic-sum spectrum (dB, 0 = median)
    breath_hz: float | None
    method: str


def displacement(x: np.ndarray, fs: float, wavelength: float, hp_hz: float = 0.08):
    """Raw slow-time IQ -> displacement-like real signal (mm if 'arc')."""
    sos = signal.butter(2, hp_hz, btype="highpass", fs=fs, output="sos")
    x = x.mean() + signal.sosfiltfilt(sos, x - x.mean())
    cf = fit_circle(x)
    if cf is not None:
        center, r, resid = cf
        phase = np.unwrap(np.angle(x - center))
        if np.ptp(phase) > 0.5 and resid < 0.25 * r:
            return signal.detrend(phase * wavelength / (4 * np.pi) * 1e3), "arc"
    return signal.detrend(_pca_projection(x)), "pca"


def remove_harmonics(s: np.ndarray, fs: float, f0: float, kmax: int = 12, fmax: float = 4.0) -> np.ndarray:
    """Least-squares removal of sinusoids at k*f0 (breathing and its harmonics)."""
    t = np.arange(len(s)) / fs
    cols = []
    for k in range(1, kmax + 1):
        if k * f0 > fmax:
            break
        w = 2 * np.pi * k * f0 * t
        cols += [np.cos(w), np.sin(w)]
    if not cols:
        return s
    A = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(A, s, rcond=None)
    return s - A @ coef


def amplitude_signal(x: np.ndarray, fs: float, hp_hz: float = 0.08) -> np.ndarray:
    """|x|: immune to common-mode LO phase drift (a rotation of everything, leakage included).
    Keeps only the radial component of the target modulation (~3 dB loss on average)."""
    a = np.abs(x)
    return signal.detrend(signal.sosfiltfilt(signal.butter(2, hp_hz, btype="highpass", fs=fs, output="sos"),
                                             a - a.mean()))


def estimate_heart(x: np.ndarray, fs: float, wavelength: float, breath_hz: float | None = None,
                   band=(0.8, 2.0), ref_band=(0.7, 3.0), methods=("amp",),
                   avg: SpectrumAverager | None = None) -> HeartResult | None:
    """x: raw slow-time IQ at fs (~20 Hz), ~12-20 s of a still subject.

    Default is amplitude demodulation: it is immune to LO phase drift, which otherwise creates
    sharp false peaks in the heart band (measured in simulation: phase demod or "pick the cleaner
    of both" are much worse). methods=("phase",) is available for experiments.
    """
    n = len(x)
    if n < fs * 8:
        return None
    best = None
    for m in methods:
        if m == "amp":
            s, name = amplitude_signal(x, fs), "amp"
        else:
            s, name = displacement(x, fs, wavelength)
        r = _heart_from_signal(s, fs, breath_hz, band, ref_band, name, avg if len(methods) == 1 else None)
        if best is None or r.snr_db > best.snr_db:
            best = r
    return best


def _heart_from_signal(s, fs, breath_hz, band, ref_band, method, avg: "SpectrumAverager | None" = None
                       ) -> HeartResult:
    n = len(s)
    if breath_hz:
        s = remove_harmonics(s, fs, breath_hz)
    s = signal.sosfiltfilt(_bandpass((0.7, min(4.5, fs / 2 * 0.9)), fs, 4), s)

    nfft = max(8192, 1 << int(np.ceil(np.log2(16 * n))))
    P = np.abs(np.fft.rfft(s * np.hanning(n), nfft)) ** 2
    f = np.fft.rfftfreq(nfft, 1.0 / fs)
    rb = (f >= ref_band[0]) & (f <= ref_band[1])
    P = P / (np.median(P[rb]) + 1e-30)          # normalise so averaging isn't dominated by loud windows
    score = P + 0.5 * np.interp(2 * f, f, P, right=0.0)
    if breath_hz:   # suppress residual breathing harmonics
        for k in range(1, 20):
            score[np.abs(f - k * breath_hz) < 0.04] *= 0.1
    if avg is not None:
        score = avg.update(score)
    med = np.median(score[rb]) + 1e-30
    inb = np.flatnonzero((f >= band[0]) & (f <= band[1]))
    k = inb[np.argmax(score[inb])]
    f_peak = f[k]
    snr = float(10 * np.log10(score[k] / med))
    # a maximum pinned to the band edge (not a true local peak) is drift / breathing leakage
    if k in (inb[0], inb[-1]) or score[k - 1] >= score[k] or score[k + 1] >= score[k]:
        snr = min(snr, 0.0)
    wave = signal.sosfiltfilt(_bandpass((0.8, 3.0), fs, 2), s)
    return HeartResult(bpm=60 * f_peak, snr_db=snr, wave=wave, freqs=f[rb],
                       score=10 * np.log10(score[rb] / med), breath_hz=breath_hz, method=method)


class SpectrumAverager:
    """Exponential average of successive heart spectra: a real heart peak stays put from second to
    second while noise / residual peaks wander, so averaging raises the true peak's contrast."""

    def __init__(self, alpha: float = 0.2):
        self.alpha, self.S = alpha, None

    def reset(self) -> None:
        self.S = None

    def update(self, score: np.ndarray) -> np.ndarray:
        if self.S is None or self.S.shape != score.shape:
            self.S = score.copy()
        else:
            self.S += self.alpha * (score - self.S)
        return self.S


class HeartTracker:
    """Reports a heart rate only after `lock_s` of confident, mutually consistent estimates."""

    def __init__(self, snr_db: float = 12.0, lock_s: float = 6.0, span_bpm: float = 8.0, hold_s: float = 4.0):
        self.snr_db, self.lock_s, self.span, self.hold_s = snr_db, lock_s, span_bpm, hold_s
        self.hist: deque = deque()          # (t, bpm, ok)
        self.locked_bpm: float | None = None
        self.last_lock_t = -np.inf

    def reset(self) -> None:
        self.hist.clear()

    def update(self, t: float, hr: HeartResult | None) -> float | None:
        if hr is None:
            self.hist.clear()
        else:
            self.hist.append((t, hr.bpm, hr.snr_db >= self.snr_db))
        while self.hist and t - self.hist[0][0] > self.lock_s:
            self.hist.popleft()
        if (self.hist and t - self.hist[0][0] >= self.lock_s - 1.01 and all(ok for *_, ok in self.hist)):
            b = np.array([h[1] for h in self.hist])
            if np.ptp(b) <= self.span:
                self.locked_bpm = float(np.median(b))
                self.last_lock_t = t
                return self.locked_bpm
        if t - self.last_lock_t <= self.hold_s:       # brief dropouts keep the last reading
            return self.locked_bpm
        self.locked_bpm = None
        return None
