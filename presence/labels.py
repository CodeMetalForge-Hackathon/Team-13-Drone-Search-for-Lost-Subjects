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


def _short_wait(w: str) -> str:
    return w.replace(" - nothing may move in range", "")


def heart_cards(s: Status, heart_snr_db: float, boundary_m: float | None):
    """Heart-mode cards: [(heading, value, caption, hex colour)] for STATUS, BREATHING, HEART RATE."""
    rng = f"within ~{boundary_m:g} m" if boundary_m else "in range"
    far = f"motion beyond ~{boundary_m:g} m ignored" if (s.far_motion and boundary_m) else ""
    if s.state == MOVING:
        st = ("MOVING", far or f"person moving {rng}", ORANGE)
    elif s.state == EMPTY:
        st = ("NO PERSON", far or f"nobody {rng}", GREY)
    elif not s.confirmed:
        st = ("STILL?", f"no breathing yet - no person in {s.absent_in_s:.0f} s", YELLOW)
    elif s.heart_bpm is not None:
        st = ("STILL", "breathing + heart rate locked", RED)
    else:
        st = ("STILL", "breathing locked, heart measuring", GREEN)

    # BREATHING / HEART RATE show the current estimate - the same number as the live charts.
    # The held "last reading" is shown only while there is no current estimate.
    # nobody present: the estimators are only fitting background noise - don't show their numbers
    b = s.breath if s.state != EMPTY else None
    if b is not None:
        if s.breath_bpm is not None:
            br = (f"{b.bpm:.0f} /min", f"live  -  SNR {b.snr_db:.0f} dB", GREEN)
        else:
            br = (f"{b.bpm:.0f} /min", f"unconfirmed  -  SNR {b.snr_db:.0f} dB", YELLOW)
    elif s.breath_last_bpm is not None:
        cap = f"last reading, {ago(s.breath_last_age_s)}"
        br = (f"{s.breath_last_bpm:.0f} /min", f"{cap}  -  {s.breath_wait}" if s.breath_wait else cap, DKGREEN)
    elif s.breath_wait:
        br = ("--", s.breath_wait, GREY)
    elif s.state != EMPTY:
        br = ("...", "measuring", YELLOW)
    else:
        br = ("--", "", GREY)

    # like breathing: live estimate (locked / unconfirmed), else the hold-still countdown
    h = s.heart if s.state != EMPTY else None
    hwait = _short_wait(s.heart_wait) if s.heart_wait.startswith("hold still") else ""
    if h is not None:
        if s.heart_bpm is not None:
            hr = (f"{h.bpm:.0f} bpm", f"live  -  confidence {h.snr_db:.0f} dB", RED)
        else:
            hr = (f"{h.bpm:.0f} bpm", f"unconfirmed  -  confidence {h.snr_db:.0f} dB", YELLOW)
    elif hwait:                                  # (no held "last reading" for heart rate)
        hr = ("--", hwait, GREY)
    elif s.state != EMPTY:
        hr = ("...", "measuring", YELLOW)
    else:
        hr = ("--", "", GREY)
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
