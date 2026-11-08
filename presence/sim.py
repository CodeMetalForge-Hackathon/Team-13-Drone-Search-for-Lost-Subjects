"""Synthetic CW-Doppler scenes for testing and demo without hardware."""
from __future__ import annotations

import numpy as np

from .config import SPEED_OF_LIGHT

DEFAULT_SCENARIO = [
    ("empty", 20.0),
    ("walk", 15.0),
    ("still", 45.0),
    ("walk", 8.0),
    ("empty", 30.0),
]


def simulate_slow_time(segments, fs: float = 200.0, fc: float = 2.45e9, *,
                       clutter: complex = 2000 * np.exp(0.6j), noise: float = 1.0,
                       walk_amp: float = 6.0, breath_amp: float = 3.0, breath_bpm: float = 15.0,
                       breath_mm: float = 5.0, drift_rad_per_s: float = 2e-4,
                       fan_amp: float = 0.0, heart_mm: float = 0.0, heart_bpm: float = 70.0,
                       fidgets=(), seed: int = 0) -> np.ndarray:
    """Slow-time complex baseband for a list of (kind, seconds[, amp_scale]) segments,
    kind in empty/walk/still. amp_scale scales the person's echo (distance: ~ (r0/r)^2).

    Amplitudes are in the same units as `noise` (std of complex noise per slow sample).
    fan_amp adds an always-on vibrating reflector (fan / AC louvre) producing Doppler lines.
    """
    rng = np.random.default_rng(seed)
    lam = SPEED_OF_LIGHT / fc
    k = 4 * np.pi / lam
    parts = []
    r = 3.0  # target range (m), kept continuous across segments
    t0 = 0.0
    for seg in segments:
        kind, dur = seg[0], seg[1]
        scale = seg[2] if len(seg) > 2 else 1.0
        n = int(dur * fs)
        t = t0 + np.arange(n) / fs
        if kind == "empty":
            tgt = np.zeros(n, np.complex128)
        elif kind == "walk":
            # back-and-forth walk, 0.4..1.2 m/s, plus limb micro-Doppler
            v = 0.8 * np.sin(2 * np.pi * t / 6.0) + 0.4 * np.sign(np.sin(2 * np.pi * t / 6.0))
            rr = r + np.cumsum(v) / fs
            limb = 0.05 * np.sin(2 * np.pi * 1.8 * t)
            amp = walk_amp * (1 + 0.3 * np.sin(2 * np.pi * 0.7 * t))
            tgt = amp * np.exp(1j * k * rr) + 0.3 * walk_amp * np.exp(1j * k * (rr + limb))
            r = rr[-1]
        elif kind == "still":
            fb = breath_bpm / 60.0 * (1 + 0.03 * np.sin(2 * np.pi * t / 40))  # slight rate wander
            phase_b = 2 * np.pi * np.cumsum(fb) / fs
            sway = np.convolve(rng.normal(0, 2e-4, n), np.ones(int(fs)) / fs, "same")
            rr = r + breath_mm / 2e3 * np.sin(phase_b) + sway
            if heart_mm:
                # pulse-shaped heartbeat with ~4% rate variability; heart_mm = peak-to-peak
                fh = heart_bpm / 60.0 * (1 + 0.04 * np.sin(2 * np.pi * t / 23 + 1.0))
                ph = 2 * np.pi * np.cumsum(fh) / fs
                pulse = np.sin(ph) + 0.6 * np.sin(2 * ph + 0.7) + 0.3 * np.sin(3 * ph + 1.3)
                rr = rr + heart_mm / 1e3 * pulse / np.ptp(pulse)
            tgt = breath_amp * np.exp(1j * k * rr)
        else:
            raise ValueError(kind)
        parts.append(scale * tgt)
        t0 += dur
    tgt = np.concatenate(parts)
    n = len(tgt)
    drift = np.cumsum(rng.normal(0, drift_rad_per_s / np.sqrt(fs), n))
    noise_v = noise / np.sqrt(2) * (rng.normal(size=n) + 1j * rng.normal(size=n))
    out = clutter * np.exp(1j * drift) + tgt + noise_v
    # fidgets: (start_s, dur_s, amp) - a small limb echo swinging ~10 cm with a smooth envelope,
    # added on top of the (continuously breathing) body
    tt_all = np.arange(n) / fs
    for t0f, durf, ampf in fidgets:
        m = (tt_all >= t0f) & (tt_all < t0f + durf)
        u = (tt_all[m] - t0f) / durf
        env = np.sin(np.pi * u) ** 2
        out[m] += ampf * env * np.exp(1j * k * (r + 0.05 * np.sin(2 * np.pi * u)))
    if fan_amp:
        tt = np.arange(n) / fs
        out += fan_amp * np.exp(1j * (2.5 * np.sin(2 * np.pi * 12.0 * tt) + 0.3 * rng.normal(size=n).cumsum() / fs))
    return out


def segment_times(segments):
    """[(kind, start, end)] for the scenario."""
    out, t = [], 0.0
    for seg in segments:
        kind, dur = seg[0], seg[1]
        out.append((kind, t, t + dur))
        t += dur
    return out


def simulate_mix(near_segments, far_segments, fs: float = 200.0, fc: float = 2.45e9,
                 seed: int = 0, **kw) -> np.ndarray:
    """Near target (with clutter + noise) plus independent far people (echo only)."""
    near = simulate_slow_time(near_segments, fs, fc, seed=seed, **kw)
    far = simulate_slow_time(far_segments, fs, fc, clutter=0, noise=0, drift_rad_per_s=0,
                             seed=seed + 1000)
    n = min(len(near), len(far))
    return near[:n] + far[:n]
