"""OpenCV on-screen display: live camera frame with the radar graphs overlaid (run --osd)."""
from __future__ import annotations

import os
import threading
import time

import cv2
import numpy as np

from .detector import EMPTY, Pipeline, Status
from .labels import DKGREEN, DKRED, GREEN, RED, banner, heart_cards, live_spectrogram
from .radio import Source

STOP = threading.Event()        # set from a signal handler; checked once per displayed frame
SPEC_SECONDS = 20.0
WINDOW = "Pluto radar OSD"
FONT = cv2.FONT_HERSHEY_SIMPLEX
MAX_FPS = 30
PANEL_DIM = 0.35                # camera brightness kept behind a panel (0 = black, 1 = untouched)

# The opencv-python wheel bundles Qt without a Wayland plugin or fonts: use XWayland and system
# fonts. cv2's import points QT_QPA_FONTDIR at its empty font dir; Qt reads it at the first window.
if os.environ.get("WAYLAND_DISPLAY") and os.environ.get("DISPLAY"):
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
if os.path.isdir("/usr/share/fonts/truetype/dejavu"):
    os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype/dejavu"


def bgr(hex_color: str) -> tuple[int, int, int]:
    h = hex_color.lstrip("#")
    return int(h[4:6], 16), int(h[2:4], 16), int(h[0:2], 16)


WHITE, GRAY, BLUE, VIOLET, CYAN, ORANGE_ = (255, 255, 255), (170, 170, 170), bgr("#3b82f6"), \
    bgr("#a78bfa"), bgr("#22d3ee"), bgr("#f97316")


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
class Camera:
    """Grabs frames on a background thread and keeps only the newest one.

    spec: "auto" (Pi camera, else /dev/video0, else none), "picam", "none",
          a V4L2 index ("0") or a device path / URL / video file."""

    def __init__(self, spec: str = "auto", size: tuple[int, int] = (1280, 720), rotate: int = 0):
        if rotate not in (0, 90, 180, 270):
            raise ValueError("rotate must be 0, 90, 180 or 270")
        self.size, self.desc, self.rotate = size, "no camera", rotate
        self._sw_rotate = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                           270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(rotate)
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._grab = None
        self._close = lambda: None
        if spec == "none":
            return
        if spec in ("auto", "picam") and self._open_picam():
            pass
        elif spec == "picam":
            print("[camera] no Pi camera found")
        elif not self._open_cv(0 if spec == "auto" else (int(spec) if spec.isdigit() else spec)):
            print(f"[camera] could not open {spec!r}" if spec != "auto" else "[camera] no camera found")
        if self._grab is not None:
            if rotate:
                self.desc += f" rot{rotate}"
            print(f"[camera] {self.desc}")
            threading.Thread(target=self._run, daemon=True, name="camera").start()

    def _open_picam(self) -> bool:
        try:
            from picamera2 import Picamera2
            if not Picamera2.global_camera_info():
                return False
            cam = Picamera2()
            # picamera2 "RGB888" is BGR byte order in memory, i.e. what OpenCV expects
            kw = {}
            if self.rotate == 180:                  # done by the ISP: free
                from libcamera import Transform
                kw["transform"] = Transform(hflip=True, vflip=True)
                self._sw_rotate = None
            cam.configure(cam.create_video_configuration(main={"size": self.size, "format": "RGB888"}, **kw))
            cam.start()
            if "AfMode" in cam.camera_controls:     # Camera Module 3: keep refocusing
                from libcamera import controls
                cam.set_controls({"AfMode": controls.AfModeEnum.Continuous})
        except Exception as e:                      # noqa: BLE001 - any failure -> try the next option
            print(f"[camera] Pi camera unavailable: {e}")
            return False
        self._grab = lambda: cam.capture_array("main")
        self._close = lambda: (cam.stop(), cam.close())
        self.desc = f"Pi camera {self.size[0]}x{self.size[1]}"
        return True

    def _open_cv(self, src) -> bool:
        cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if isinstance(src, int) else cv2.VideoCapture(src)
        if not cap.isOpened():
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        is_file = isinstance(src, str) and not src.startswith(("/dev/", "rtsp:", "http"))

        def grab():
            ok, f = cap.read()
            if not ok and is_file:                  # loop a video file
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, f = cap.read()
            return f if ok else None
        self._grab, self._close = grab, cap.release
        self.desc = f"camera {src}"
        return True

    def _run(self):
        while not self._stop.is_set():
            try:
                f = self._grab()
            except Exception as e:                  # noqa: BLE001
                print(f"[camera] read failed: {e}")
                f = None
            if f is None:
                time.sleep(0.05)
                continue
            if f.ndim == 3 and f.shape[2] == 4:
                f = cv2.cvtColor(f, cv2.COLOR_BGRA2BGR)
            if self._sw_rotate is not None:
                f = cv2.rotate(f, self._sw_rotate)
            with self._lock:
                self._frame = f

    def read(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    def stop(self):
        self._stop.set()
        try:
            self._close()
        except Exception:                           # noqa: BLE001
            pass


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------
def text(img, s, org, scale, color=WHITE, thick=1, anchor="left"):
    """Anti-aliased text with a 1 px drop shadow so it stays readable over video."""
    (w, h), _ = cv2.getTextSize(s, FONT, scale, thick)
    x, y = org
    if anchor == "center":
        x -= w // 2
    elif anchor == "right":
        x -= w
    cv2.putText(img, s, (x + 1, y + 1), FONT, scale, (0, 0, 0), thick, cv2.LINE_AA)
    cv2.putText(img, s, (x, y), FONT, scale, color, thick, cv2.LINE_AA)
    return w, h


def dim(img, rect, keep=PANEL_DIM):
    """Darken a rectangle of the camera frame in place (panel background)."""
    x, y, w, h = rect
    roi = img[y:y + h, x:x + w]
    cv2.convertScaleAbs(roi, dst=roi, alpha=keep)


def plot(img, rect, series, xlim, ylim, title="", hlines=(), info="", info_color=GRAY, msg="", s=1.0):
    """Line plot into rect = (x, y, w, h). series: [(xs, ys, color, thickness)]; NaNs break lines,
    thickness < 0 draws dots of that radius. msg is centred in the plot (e.g. why it is empty)."""
    x0, y0, w, h = rect
    dim(img, rect)
    pad_t = int(22 * s)
    if title:
        text(img, title, (x0 + 6, y0 + int(16 * s)), 0.45 * s, GRAY)
    if info:
        text(img, info, (x0 + w - 6, y0 + int(16 * s)), 0.45 * s, info_color, anchor="right")
    px, py, pw, ph = x0 + 6, y0 + pad_t, w - 12, h - pad_t - 6
    (xa, xb), (ya, yb) = xlim, ylim
    if not (np.isfinite(ya) and np.isfinite(yb)) or yb <= ya:
        ya, yb = -1.0, 1.0
    cv2.line(img, (px, py + ph), (px + pw, py + ph), (90, 90, 90), 1)
    if msg:
        text(img, msg, (px + pw // 2, py + ph // 2), 0.5 * s, GRAY, anchor="center")

    def to_px(xs, ys):
        u = px + (np.asarray(xs, float) - xa) / (xb - xa) * pw
        v = py + ph - (np.clip(np.asarray(ys, float), ya, yb) - ya) / (yb - ya) * ph
        return u, v

    for yv, color in hlines:
        if yv is not None and ya <= yv <= yb:
            _, v = to_px([0], [yv])
            for k in range(px, px + pw, 10):        # dashed
                cv2.line(img, (k, int(v[0])), (min(k + 5, px + pw), int(v[0])), color, 1, cv2.LINE_AA)
    for xs, ys, color, thick in series:
        if len(xs) < 2:
            continue
        u, v = to_px(xs, ys)
        ok = np.isfinite(u) & np.isfinite(v) & (u >= px) & (u <= px + pw)
        if thick < 0:
            for a, b in np.round(np.stack([u[ok], v[ok]], 1)).astype(int):
                cv2.circle(img, (a, b), -thick, color, -1, cv2.LINE_AA)
            continue
        # split into runs of valid points
        idx = np.flatnonzero(np.diff(np.concatenate([[0], ok.astype(np.int8), [0]])))
        pts = np.stack([u, v], 1)
        for a, b in zip(idx[::2], idx[1::2]):
            if b - a >= 2:
                cv2.polylines(img, [np.round(pts[a:b]).astype(np.int32)], False, color, thick, cv2.LINE_AA)
            elif b - a == 1:
                cv2.circle(img, tuple(np.round(pts[a]).astype(int)), max(2, thick), color, -1, cv2.LINE_AA)


def card(img, rect, heading, value, caption, color, s=1.0, alpha=0.8, small=False):
    """Coloured, slightly transparent card: heading, big value, caption (small: held reading)."""
    x, y, w, h = rect
    roi = img[y:y + h, x:x + w]
    cv2.addWeighted(roi, 1 - alpha, np.full_like(roi, color), alpha, 0, dst=roi)
    cx = x + w // 2
    text(img, heading, (cx, y + int(0.24 * h)), 0.5 * s, (235, 235, 235), anchor="center")
    vs = (0.75 if small else 1.2) * s
    while vs > 0.5 and cv2.getTextSize(value, FONT, vs, 2)[0][0] > w - 12:
        vs -= 0.1
    text(img, value, (cx, y + int(0.62 * h)), vs, WHITE, 2, anchor="center")
    cs = 0.45 * s
    while cs > 0.3 and cv2.getTextSize(caption, FONT, cs, 1)[0][0] > w - 12:
        cs -= 0.05
    text(img, caption, (cx, y + int(0.88 * h)), cs, (235, 235, 235), anchor="center")


# ---------------------------------------------------------------------------
# View
# ---------------------------------------------------------------------------
class OsdView:
    """Camera frame with radar status and graphs drawn on top, shown with cv2.imshow.

    Keys: q / Esc quit, f toggle fullscreen, h hide / show graphs,
          [ / ] move the range cutoff 1 dB closer / farther, s save it to the calibration file."""

    def __init__(self, source: Source, pipe: Pipeline, camera: Camera, on_status=None,
                 heart: bool = False, ref_bpm: float | None = None, size=(1280, 720),
                 fullscreen: bool = False, save_thresholds=None, streams=(), window: bool = True,
                 reconnect=None, on_samples=None, max_backlog_s: float = 0.5):
        """reconnect: callable returning a new (unstarted) live Source; enables auto-reconnect and
        'latest data' mode (a backlog > max_backlog_s is skipped). on_samples(x): e.g. recording."""
        self.src, self.pipe, self.cam, self.on_status = source, pipe, camera, on_status
        self.reconnect, self.on_samples = reconnect, on_samples
        self.max_backlog = int(max_backlog_s * pipe.fs) if reconnect is not None else None
        self.lock = threading.Lock()            # guards pipe / last / trend between radar and display
        self._radar_stop = threading.Event()
        self.radar_lost = False
        self.skipped_s = 0.0
        self._last_status_wall = time.monotonic()
        self.heart, self.ref, self.size = heart, ref_bpm, size
        self.fullscreen, self.show_graphs = fullscreen, True
        self.save_thresholds = save_thresholds                 # callable(th) -> path, for the [s] key
        self.streams, self.window = list(streams), window      # outputs with push(frame): browser view, MP4
        th = pipe.th
        # range cutoff reference: distance scales ~ 10^(-dB/40) (echo power falls ~R^4)
        self._bnd_ref = (th.motion_abs_db, th.boundary_m) if th.motion_abs_db is not None else None
        self._note, self._note_t = "", 0.0
        self.last: Status | None = None
        self.trend: list[tuple[float, float, float]] = []      # (t, locked bpm, raw bpm), NaN = none
        self._spec: np.ndarray | None = None                   # colour-mapped spectrogram (BGR)
        self._spec_key = None
        self._fps, self._t_prev = 0.0, time.monotonic()

    # --- radar (own thread: display / encoding can never delay it) ---
    def _radar_loop(self):
        last_try = 0.0
        while not self._radar_stop.is_set():
            if self.src.finished:
                if self.reconnect is None:          # replay / sim ended: keep showing the last state
                    time.sleep(0.1)
                    continue
                if not self.radar_lost:
                    self.radar_lost = True
                    self._flash("radar lost - reconnecting...")
                    try:
                        self.src.stop()
                    except Exception:               # noqa: BLE001
                        pass
                if time.monotonic() - last_try >= 3.0:
                    last_try = time.monotonic()
                    try:
                        src = self.reconnect()
                        src.start()
                        self.src, self.radar_lost = src, False
                        self._flash("radar reconnected")
                    except Exception as e:          # noqa: BLE001 - Pluto still gone: retry later
                        print(f"[osd] reconnect failed: {e}", flush=True)
                time.sleep(0.2)
                continue
            x = self.src.read(timeout=0.1)
            if not len(x):
                continue
            if self.max_backlog is not None and len(x) > self.max_backlog:
                keep = self.max_backlog // 2        # fell behind: jump to the newest samples
                self.skipped_s += (len(x) - keep) / self.pipe.fs
                print(f"[osd] radar backlog {len(x) / self.pipe.fs:.2f} s - skipped to latest", flush=True)
                x = x[-keep:]
            if self.on_samples:
                self.on_samples(x)
            with self.lock:
                sts = self.pipe.feed(x)
                for st in sts:
                    self.last = st
                    if self.heart and (not self.trend or st.t - self.trend[-1][0] >= 0.99):
                        h = st.heart
                        raw = h.bpm if h is not None and st.state != EMPTY else np.nan
                        self.trend.append((st.t, np.nan if st.heart_bpm is None else st.heart_bpm, raw))
                if sts:
                    self.trend = [r for r in self.trend if self.last.t - r[0] <= 120]
                    self._last_status_wall = time.monotonic()
            if self.on_status:
                for st in sts:
                    self.on_status(st)

    def radar_age(self) -> float:
        """Seconds since the radar last produced a status (wall clock)."""
        return time.monotonic() - self._last_status_wall

    def _counted(self, times: np.ndarray) -> np.ndarray:
        """For radar times, True where the detector counted motion (above threshold, inside range)."""
        h = np.array(self.pipe.history)
        if not len(h):
            return np.zeros(len(times), bool)
        cut = self.pipe.th.motion_abs_db
        ok = (h[:, 1] > h[:, 3]) & (h[:, 2] >= cut if cut is not None else True)
        i = np.clip(np.searchsorted(h[:, 0], times), 0, len(h) - 1)
        return ok[i] & (times >= h[0, 0] - 0.2)

    def _spectrogram(self, w: int, h: int) -> np.ndarray:
        """Colour-mapped Doppler spectrogram (+-60 Hz), recomputed only on new radar data.
        Columns where the detector did NOT count motion (too weak, or beyond the range cutoff)
        are drawn in dim grey, so the chart shows what the status actually reacts to."""
        key = (self.last.t if self.last else None, w, h)
        if key != self._spec_key:
            f, P, lo, hi = live_spectrogram(self.pipe, SPEC_SECONDS)
            P = P[np.abs(f) <= 60]
            u8 = np.clip((P - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
            img = cv2.applyColorMap(np.flipud(u8), cv2.COLORMAP_MAGMA)
            ncol = img.shape[1]
            hop, half = 32 / self.pipe.fs, 64 / self.pipe.fs          # STFT: nperseg 128, 75 % overlap
            col_t = self.pipe.t - SPEC_SECONDS + half + np.arange(ncol) * hop
            counted = self._counted(col_t)
            grey = cv2.cvtColor(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
            grey = (grey * 0.55).astype(np.uint8)
            img = np.where(counted[None, :, None], img, grey)
            self._spec = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
            self._spec_key = key
        return self._spec

    # --- layout ---
    def _draw(self, img):
        W, H = img.shape[1], img.shape[0]
        s = H / 720
        m = int(12 * s)
        st = self.last
        if st is None:
            if self.radar_lost or self.src.finished:
                msg, sub = "RADAR NOT CONNECTED", "plug in the Pluto - retrying every 3 s"
                (tw, th_), _ = cv2.getTextSize(msg, FONT, 1.0 * s, 2)
                cv2.rectangle(img, (W // 2 - tw // 2 - 16, H // 2 - th_ - 16), (W // 2 + tw // 2 + 16, H // 2 + 12),
                              bgr(RED), -1)
                text(img, msg, (W // 2, H // 2), 1.0 * s, WHITE, 2, anchor="center")
                text(img, sub, (W // 2, H // 2 + int(40 * s)), 0.6 * s, WHITE, 1, anchor="center")
            else:
                text(img, "waiting for radar data...", (W // 2, H // 2), 1.0 * s, WHITE, 2, anchor="center")
            return
        cfg = self.pipe.cfg

        foot = int(18 * s)                                   # status line at the very bottom

        # --- bottom: summary cards (heart mode) or state banner ---
        if self.heart:
            ch = int(0.15 * H)
            bar_top = H - foot - ch
            cw = (W - 4 * m) // 3
            for k, (head, value, cap, color) in enumerate(
                    heart_cards(st, cfg.heart_snr_db, self.pipe.th.boundary_m)):
                card(img, (m + k * (cw + m), bar_top, cw, ch), head, value, cap, bgr(color), s,
                     small=color in (DKGREEN, DKRED))           # held "last reading": not live
        else:
            msg, color = banner(st)
            sc = 0.9 * s
            (tw, th), _ = cv2.getTextSize(msg, FONT, sc, 2)
            bw, bh = tw + 2 * int(20 * s), th + 2 * int(14 * s)
            bar_top = H - foot - bh
            x = (W - bw) // 2
            roi = img[bar_top:bar_top + bh, x:x + bw]
            cv2.addWeighted(roi, 0.2, np.full_like(roi, bgr(color)), 0.8, 0, dst=roi)
            text(img, msg, (W // 2, bar_top + bh // 2 + th // 2), sc, WHITE, 2, anchor="center")

        # radar health warning and key-press notes sit just above the bottom bar
        y = bar_top - m
        age = self.radar_age()
        if self.radar_lost or age > 1.5:
            msg = "RADAR DISCONNECTED - reconnecting..." if self.radar_lost else f"RADAR STALLED ({age:.0f} s)"
            (tw, th_), _ = cv2.getTextSize(msg, FONT, 0.8 * s, 2)
            y -= th_ + 20
            cv2.rectangle(img, (W // 2 - tw // 2 - 12, y), (W // 2 + tw // 2 + 12, y + th_ + 20), bgr(RED), -1)
            text(img, msg, (W // 2, y + th_ + 10), 0.8 * s, WHITE, 2, anchor="center")
            y -= m
        if self._note and time.monotonic() - self._note_t < 3.0:
            text(img, self._note, (W // 2, y - int(6 * s)), 0.7 * s, ORANGE_, 2, anchor="center")

        # footer
        text(img, f"{self._fps:4.1f} fps   radar t={st.t:6.1f} s   {self.cam.desc}"
             f"   {self._cutoff_text()}   [q] quit  [f] fullscreen  [h] graphs  [ ] range  [s] save",
             (W - m, H - int(5 * s)), 0.4 * s, (220, 220, 220), anchor="right")

        if not self.show_graphs:
            return

        # --- top: graphs ---
        gh = int(0.30 * H)
        gy = m
        sw = int(0.38 * W)

        # Doppler spectrogram
        x0, y0 = m, gy
        dim(img, (x0, y0, sw, gh))
        pad_t = int(22 * s)
        text(img, f"Doppler, last {SPEC_SECONDS:.0f} s (+-60 Hz)   colour = counted, grey = ignored",
             (x0 + 6, y0 + int(16 * s)), 0.42 * s, GRAY)
        sp = self._spectrogram(sw - 12, gh - pad_t - 6)
        img[y0 + pad_t:y0 + pad_t + sp.shape[0], x0 + 6:x0 + 6 + sp.shape[1]] = sp
        cy = y0 + pad_t + sp.shape[0] // 2                   # 0 Hz line
        cv2.line(img, (x0 + 6, cy), (x0 + 6 + sp.shape[1], cy), (110, 110, 110), 1)

        rx = x0 + sw + m
        rw = W - rx - m
        hh = (gh - m) // 2
        if self.heart:
            colw = (rw - m) // 2
            self._breathing(img, (rx, gy, colw, hh), s)
            self._heartbeat(img, (rx, gy + hh + m, colw, hh), s)
            self._hr_trend(img, (rx + colw + m, gy, rw - colw - m, hh), s)
            self._motion(img, (rx + colw + m, gy + hh + m, rw - colw - m, hh), s)
        else:
            self._motion(img, (rx, gy, rw, hh), s)
            self._breathing(img, (rx, gy + hh + m, rw, hh), s)

    # --- range cutoff ---
    def cutoff_m(self) -> float | None:
        """Approximate distance of the current range cutoff (None if never calibrated in metres)."""
        cur = self.pipe.th.motion_abs_db
        if cur is None or self._bnd_ref is None or not self._bnd_ref[1]:
            return None
        ref_db, ref_m = self._bnd_ref
        return ref_m * 10 ** ((ref_db - cur) / 40)

    def _cutoff_text(self) -> str:
        cur = self.pipe.th.motion_abs_db
        if cur is None:
            return "no range cutoff"
        d = self.cutoff_m()
        return f"cutoff {cur:.1f} dBFS" + (f" (~{d:.1f} m)" if d else "")

    def _nudge_cutoff(self, db: float):
        """Raise (db > 0, shorter range) or lower the motion-level range cutoff."""
        th = self.pipe.th
        if th.motion_abs_db is None:            # no boundary yet: start just below the current level
            lvl = self.last.motion_abs if self.last is not None and self.last.motion_abs > -150 else -70.0
            th.motion_abs_db = lvl - 3.0
        th.motion_abs_db += db
        self._flash(self._cutoff_text())

    def _save_cutoff(self):
        if self.save_thresholds is None:
            return self._flash("saving not available here")
        th = self.pipe.th
        th.boundary_m = self.cutoff_m() or th.boundary_m
        cfg = self.pipe.cfg
        th.rx_gain_db, th.tx_atten_db = cfg.rx_gain_db, cfg.tx_atten_db   # cutoff is valid at these gains
        self._flash(f"saved {self._cutoff_text()} to {os.path.basename(self.save_thresholds(th))}")

    def _flash(self, msg: str):
        print(f"[osd] {msg}", flush=True)
        self._note, self._note_t = msg, time.monotonic()

    def _motion(self, img, rect, s):
        """Motion level (dBFS, only while moving) against the range cutoff: below the line = ignored."""
        h = np.array(self.pipe.history)
        title = ("motion (grey = ignored)" if self.heart else
                 "motion level (dBFS)  - - range cutoff:  [ closer  ] farther  s save   grey = ignored")
        cut = self.pipe.th.motion_abs_db
        if not len(h):
            return plot(img, rect, [], (-60, 0), (0, 1), title, s=s)
        tt = h[:, 0] - self.last.t
        above = h[:, 1] > h[:, 3]
        inside = above & (h[:, 2] >= cut if cut is not None else True)
        lv = np.where(above, h[:, 2], np.nan)                    # everything above the motion threshold
        lv_in = np.where(inside, h[:, 2], np.nan)                # ...of which the status counted these
        vals = np.concatenate([lv[np.isfinite(lv)], [cut] if cut is not None else []])
        lo, hi = (vals.min() - 6, vals.max() + 6) if len(vals) else (-90, -40)
        st = self.last
        moving_in = st.motion_db > st.motion_thr and not st.far_motion
        d = self.cutoff_m()
        short = ("no cutoff" if cut is None else f"cutoff ~{d:.1f} m" if d else f"cutoff {cut:.0f} dBFS")
        info = (short if self.heart else self._cutoff_text()) + (
            f"  now {st.motion_abs:.0f}" if st.motion_db > st.motion_thr else "")
        plot(img, rect, [(tt, lv, (110, 110, 110), 1), (tt, lv_in, BLUE, 2)], (-60, 0), (lo, hi), title,
             hlines=[(cut, ORANGE_)] if cut is not None else (), info=info,
             info_color=bgr(RED) if moving_in else (ORANGE_ if st.far_motion else GRAY), s=s)

    def _breathing(self, img, rect, s):
        b = self.last.breath
        cfg = self.pipe.cfg
        if self.last.state == EMPTY:            # estimator only sees background noise
            return plot(img, rect, [], (-cfg.breath_win_s, 0), (-1, 1), "breathing", msg="no person", s=s)
        if b is None:
            wait = self.last.breath_wait or ("waiting for stillness..." if self.last.state != EMPTY else "")
            return plot(img, rect, [], (-cfg.breath_win_s, 0), (-1, 1), "breathing", msg=wait, s=s)
        tb = (np.arange(len(b.wave)) - len(b.wave)) / cfg.breath_fs
        mx = np.abs(b.wave).max() * 1.2 + 1e-12
        ok = self.pipe.detector.breath_ok(b)
        plot(img, rect, [(tb, b.wave, bgr(GREEN), 2)], (-cfg.breath_win_s, 0), (-mx, mx), "breathing",
             info=f"{b.bpm:.1f} /min  SNR {b.snr_db:.0f} dB  " + ("LOCK" if ok else "no lock"),
             info_color=bgr(GREEN) if ok else GRAY, s=s)

    def _heartbeat(self, img, rect, s):
        h = self.last.heart
        cfg = self.pipe.cfg
        if self.last.state == EMPTY:
            return plot(img, rect, [], (-cfg.heart_win_s, 0), (-1, 1), "heartbeat", msg="no person", s=s)
        if h is None:
            return plot(img, rect, [], (-cfg.heart_win_s, 0), (-1, 1), "heartbeat",
                        msg=self.last.heart_wait.replace(" - nothing may move in range", ""), s=s)
        tw = (np.arange(len(h.wave)) - len(h.wave)) / cfg.breath_fs
        mx = np.abs(h.wave).max() * 1.2 + 1e-12
        plot(img, rect, [(tw, h.wave, bgr(RED), 2)], (-cfg.heart_win_s, 0), (-mx, mx),
             "heartbeat (breathing removed)", info=f"{h.bpm:.0f} bpm  conf {h.snr_db:.0f} dB",
             info_color=bgr(RED) if h.snr_db >= cfg.heart_snr_db else GRAY, s=s)

    def _hr_trend(self, img, rect, s):
        tr = np.array(self.trend, float).reshape(-1, 3)
        tt = tr[:, 0] - self.last.t if len(tr) else []
        series = []
        if len(tr):
            series = [(tt, tr[:, 2], VIOLET, -2), (tt, tr[:, 1], bgr(RED), 3)]
        plot(img, rect, series, (-120, 0), (45, 125), "heart rate, last 2 min (bpm)",
             hlines=[(self.ref, CYAN)] if self.ref else (),
             info="ref " + f"{self.ref:.0f}" if self.ref else "", info_color=CYAN, s=s)
        x, y, w, h = rect
        for v in (60, 80, 100, 120):                 # y-axis labels
            yy = y + int(22 * s) + int((h - int(22 * s) - 6) * (1 - (v - 45) / 80))
            text(img, str(v), (x + w - 6, yy + 4), 0.35 * s, (140, 140, 140), anchor="right")

    def _placeholder(self):
        W, H = self.size
        img = np.full((H, W, 3), 28, np.uint8)
        text(img, "no camera", (W // 2, int(H * 0.45)), 1.0 * H / 720, (90, 90, 90), 2, anchor="center")
        return img

    @staticmethod
    def _window_open() -> bool:
        try:
            return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
        except cv2.error:
            return False

    # --- main loop ---
    def show(self):
        if self.window:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, *self.size)
            if self.fullscreen:
                cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        worker = threading.Thread(target=self._radar_loop, daemon=True, name="radar")
        worker.start()
        try:
            while not STOP.is_set():
                t_loop = time.monotonic()
                frame = self.cam.read()
                if frame is None:
                    img = self._placeholder()
                elif (frame.shape[1], frame.shape[0]) != self.size:
                    img = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
                else:
                    img = frame.copy()
                with self.lock:
                    self._draw(img)
                for out in self.streams:
                    out.push(img)

                wait_ms = max(1, int(1000 / MAX_FPS - (time.monotonic() - t_loop) * 1000))
                if self.window:
                    cv2.imshow(WINDOW, img)
                    key = cv2.waitKey(wait_ms) & 0xFF
                else:
                    time.sleep(wait_ms / 1000)
                    key = 255
                now = time.monotonic()
                self._fps = 0.9 * self._fps + 0.1 / max(now - self._t_prev, 1e-3)
                self._t_prev = now
                if key in (ord("q"), 27):
                    break
                if key == ord("f"):
                    self.fullscreen = not self.fullscreen
                    cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN,
                                          cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL)
                elif key == ord("h"):
                    self.show_graphs = not self.show_graphs
                elif key == ord("["):
                    self._nudge_cutoff(+1.0)
                elif key == ord("]"):
                    self._nudge_cutoff(-1.0)
                elif key == ord("s"):
                    self._save_cutoff()
                if self.window and not self._window_open():
                    break
        finally:
            self._radar_stop.set()
            worker.join(timeout=2)
            self.src.stop()
            self.cam.stop()
            if self.window:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
