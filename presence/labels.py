"""Display helpers shared by the matplotlib views and the cv2 OSD: status text / colours, live spectrogram."""
from __future__ import annotations

import numpy as np

from .detector import EMPTY, MOVING, STILL, Pipeline, Status
from .dsp import doppler_spectrogram

GREY, ORANGE, YELLOW, GREEN, DKGREEN, RED, DKRED = ("#6b7280", "#ea580c", "#ca8a04", "#16a34a",
                                                     "#166534", "#dc2626", "#991b1b")
STATE_COLORS = {EMPTY: GREY, MOVING: RED, STILL: GREEN, "STILL?": YELLOW, "PRESENT": "#15803d"}


def ago(sec: float) -> str:
    sec = int(sec)
    return f"{sec} s ago" if sec < 60 else f"{sec // 60} min {sec % 60:02d} s ago"


def banner(s: Status) -> tuple[str, str]:
    """Presence-mode state banner -> (text, hex colour)."""
    label = s.state if s.state != STILL or s.confirmed else ("PRESENT" if s.proximity else "STILL?")
    text = {EMPTY: "NO ONE DETECTED", MOVING: "PERSON MOVING",
            STILL: "PERSON STILL (breathing)", "STILL?": "PERSON STILL? (acquiring breathing)",
            "PRESENT": "PERSON PRESENT (still)"}[label]
    if label == STILL and s.breath is not None:
        text += f"  {s.breath.bpm:.0f} bpm"
    if s.far_motion:
        text += "   (far motion ignored)"
    return text, STATE_COLORS[label]


def heart_cards(s: Status, heart_snr_db: float, boundary_m: float | None):
    """Heart-mode cards: [(heading, value, caption, hex colour)] for STATUS, BREATHING, HEART RATE."""
    rng = f"within ~{boundary_m:g} m" if boundary_m else "in range"
    far = f"motion beyond ~{boundary_m:g} m ignored" if (s.far_motion and boundary_m) else ""
    if s.state == MOVING:
        st = ("MOVING", far or f"person moving {rng}", ORANGE)
    elif s.heart_bpm is not None:
        st = ("STILL", "breathing + heart rate locked", RED)
    elif s.breath_bpm is not None:
        st = ("STILL", "breathing locked, heart measuring", GREEN)
    elif s.state == STILL:
        st = ("STILL", "acquiring - sit still 0.3-0.5 m away", YELLOW)
    else:
        st = ("NO PERSON", far or f"nobody {rng}", GREY)

    b = s.breath
    if s.breath_bpm is not None:
        avg = s.breath_last_bpm if s.breath_last_bpm is not None else s.breath_bpm
        br = (f"{avg:.0f} /min", f"live  -  SNR {b.snr_db:.0f} dB" if b else "live", GREEN)
    elif s.breath_last_bpm is not None:
        br = (f"{s.breath_last_bpm:.0f} /min", f"last reading, {ago(s.breath_last_age_s)}", DKGREEN)
    elif s.breath_wait:
        br = ("--", s.breath_wait, GREY)
    elif s.state != EMPTY:
        br = ("...", "measuring", YELLOW)
    else:
        br = ("--", "", GREY)

    h = s.heart
    if s.heart_bpm is not None:
        avg = s.heart_last_bpm if s.heart_last_bpm is not None else s.heart_bpm
        hr = (f"{avg:.0f} bpm", f"average of {max(s.heart_last_span_s, 1):.0f} s locked", RED)
    elif s.heart_last_bpm is not None:
        hr = (f"{s.heart_last_bpm:.0f} bpm", f"last reading, {ago(s.heart_last_age_s)}", DKRED)
    elif s.heart_wait.startswith("hold still"):
        hr = ("--", s.heart_wait.replace(" - nothing may move in range", ""), GREY)
    elif s.state == EMPTY:
        hr = ("--", "", GREY)
    elif h is not None:
        hr = ("...", f"measuring - confidence {h.snr_db:.0f}/{heart_snr_db:.0f} dB", YELLOW)
    else:
        hr = ("...", "measuring", YELLOW)
    return [("STATUS", *st), ("BREATHING", *br), ("HEART RATE", *hr)]


def live_spectrogram(pipe: Pipeline, seconds: float):
    """Doppler spectrogram of the last `seconds` of clutter-removed IQ -> (freqs, P_dB, lo, hi).

    Zero-pads at start-up; the colour range (lo, hi) only uses the columns that hold data."""
    n = int(seconds * pipe.fs)
    xc = pipe.hp.last(n)
    filled = len(xc) / n
    if len(xc) < n:
        xc = np.concatenate([np.zeros(n - len(xc), complex), xc])
    f, _, P = doppler_spectrogram(xc, pipe.fs)
    valid = P[:, int(P.shape[1] * (1 - filled)):] if filled < 1 else P
    P = np.maximum(P, valid.min()) if valid.size else P
    lo = np.percentile(valid if valid.size else P, 30)
    return f, P, lo, max(lo + 25, P.max())
