"""Record the OSD (camera + radar overlay) to an MP4 file (run --osd --mp4 [FILE])."""
from __future__ import annotations

import shutil
import subprocess
import threading
import time
from datetime import datetime

import numpy as np


def default_name() -> str:
    return datetime.now().strftime("osd_%Y%m%d_%H%M%S.mp4")


class Mp4Recorder:
    """Encodes the newest pushed frame at a fixed rate with ffmpeg / libx264.

    Fixed-rate sampling keeps the video in real time even though the OSD loop's frame rate
    varies. Fragmented MP4 keeps the file playable if the program is killed mid-recording."""

    def __init__(self, path: str, size: tuple[int, int], fps: float = 30.0, crf: int = 23):
        if shutil.which("ffmpeg") is None:
            raise RuntimeError("ffmpeg not found (sudo apt install ffmpeg)")
        self.path, self.size, self.fps = path, size, fps
        self.frames = 0
        self._frame: np.ndarray | None = None
        self._stop = threading.Event()
        w, h = size
        self.proc = subprocess.Popen(
            ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
             "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", f"{fps:g}", "-i", "-",
             "-c:v", "libx264", "-preset", "ultrafast", "-crf", str(crf), "-pix_fmt", "yuv420p",
             "-g", str(max(1, int(round(2 * fps)))),
             "-movflags", "+frag_keyframe+empty_moov+default_base_moof", path],
            stdin=subprocess.PIPE)
        self._thread = threading.Thread(target=self._writer, daemon=True, name="mp4-writer")
        self._thread.start()

    def push(self, frame: np.ndarray):
        self._frame = frame                 # newest frame wins (never blocks the OSD)

    def _writer(self):
        period = 1.0 / self.fps
        nxt = time.monotonic()
        w, h = self.size
        while not self._stop.is_set():
            f = self._frame
            if f is not None and f.shape[:2] == (h, w):
                try:
                    self.proc.stdin.write(np.ascontiguousarray(f).tobytes())
                    self.frames += 1
                except (BrokenPipeError, ValueError, OSError):
                    print(f"[mp4] encoder stopped - recording to {self.path} ended", flush=True)
                    return
            nxt += period
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()      # encoder fell behind: skip rather than drift

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)
        try:
            self.proc.stdin.close()         # lets ffmpeg flush and finish the file
        except OSError:
            pass
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        print(f"[mp4] saved {self.frames / self.fps:.1f} s of video to {self.path}", flush=True)
