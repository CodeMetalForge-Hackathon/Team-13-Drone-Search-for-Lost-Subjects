"""Low-latency browser view of the OSD: MJPEG over HTTP (run --osd --web).

Open http://<pi>:8081/ on any phone or laptop. Browsers show each JPEG as soon as it arrives
(no jitter buffer), so the delay is about one frame plus the network (~5-15 Mbit/s at 720p).
"""
from __future__ import annotations

import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


def local_ip() -> str:
    """Best guess of this machine's LAN address (for printing the stream URL)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


PAGE = b"""<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pluto radar</title><style>html,body{margin:0;height:100%;background:#000}
img{width:100%;height:100%;object-fit:contain}</style></head>
<body><img src="/stream" alt="radar OSD"></body></html>"""


class MjpegServer:
    def __init__(self, port: int = 8081, fps: float = 30, quality: int = 75):
        self.port, self.period, self.quality = port, 1.0 / fps, quality
        self._frame: np.ndarray | None = None
        self._jpeg: bytes | None = None
        self._seq = 0
        self._new = threading.Condition()
        self._clients = 0
        self._stop = threading.Event()
        srv = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(PAGE)))
                    self.end_headers()
                    self.wfile.write(PAGE)
                elif self.path.startswith("/stream"):
                    srv._stream(self)
                else:
                    self.send_error(404)

        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True, name="mjpeg-http").start()
        threading.Thread(target=self._encoder, daemon=True, name="mjpeg-enc").start()
        self.url = f"http://{local_ip()}:{port}/"

    def push(self, frame: np.ndarray):
        self._frame = frame                  # newest frame wins; encoded only while someone watches

    def _encoder(self):
        last = None
        while not self._stop.is_set():
            t = time.monotonic()
            f = self._frame
            if self._clients and f is not None and f is not last:
                ok, buf = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if ok:
                    with self._new:
                        self._jpeg, self._seq = buf.tobytes(), self._seq + 1
                        self._new.notify_all()
                last = f
            time.sleep(max(0.002, self.period - (time.monotonic() - t)))

    def _stream(self, h: BaseHTTPRequestHandler):
        h.send_response(200)
        h.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        h.send_header("Cache-Control", "no-cache, no-store")
        h.send_header("Pragma", "no-cache")
        h.end_headers()
        with self._new:
            self._clients += 1
        seen = -1
        try:
            while not self._stop.is_set():
                with self._new:
                    self._new.wait_for(lambda: self._seq != seen or self._stop.is_set(), timeout=1.0)
                    jpg, seen = self._jpeg, self._seq
                if jpg is None:
                    continue
                h.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                              + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                h.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with self._new:
                self._clients -= 1

    def stop(self):
        self._stop.set()
        with self._new:
            self._new.notify_all()
        self.httpd.shutdown()
        self.httpd.server_close()
