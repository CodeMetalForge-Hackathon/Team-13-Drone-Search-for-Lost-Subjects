"""Minimal RTSP server streaming the OSD frames as H.264 (run --osd --rtsp).

ffmpeg (libx264, ultrafast / zerolatency) encodes the frames; this module splits its Annex-B
output into NAL units, packetizes them as RTP (RFC 6184: single NAL / FU-A) and serves them to
any number of RTSP clients over TCP (interleaved) or UDP. Watch with e.g.

    ffplay -rtsp_transport tcp rtsp://<pi>:8080/osd
    vlc rtsp://<pi>:8080/osd
"""
from __future__ import annotations

import base64
import queue
import random
import re
import shutil
import socket
import struct
import subprocess
import threading
import time

import numpy as np

MTU_PAYLOAD = 1400
CLOCK = 90_000


def local_ip() -> str:
    """Best guess of this machine's LAN address (for printing the stream URL)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.255.255.255", 1))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


# ---------------------------------------------------------------------------
# Encoder: frames -> ffmpeg -> access units [(nal bytes, ...)]
# ---------------------------------------------------------------------------
class H264Encoder:
    def __init__(self, size: tuple[int, int], fps: float, bitrate_kbps: int, on_access_unit):
        if shutil.which("ffmpeg") is None:
            raise RuntimeError("ffmpeg not found (sudo apt install ffmpeg)")
        self.size, self.fps, self.on_au = size, fps, on_access_unit
        self.sps: bytes | None = None
        self.pps: bytes | None = None
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        w, h = size
        gop = max(1, int(round(fps)))       # keyframe every second: new clients start quickly
        self.proc = subprocess.Popen(
            ["ffmpeg", "-hide_banner", "-loglevel", "error",
             "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}", "-r", f"{fps:g}", "-i", "-",
             "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency", "-profile:v", "baseline",
             "-pix_fmt", "yuv420p", "-b:v", f"{bitrate_kbps}k", "-maxrate", f"{bitrate_kbps}k",
             "-bufsize", f"{bitrate_kbps // 2}k", "-g", str(gop), "-bf", "0",
             "-x264-params", "aud=1:repeat-headers=1:sliced-threads=0",
             "-f", "h264", "-"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        threading.Thread(target=self._writer, daemon=True, name="rtsp-enc-in").start()
        threading.Thread(target=self._reader, daemon=True, name="rtsp-enc-out").start()

    def push(self, frame: np.ndarray):
        """Offer the newest frame (never blocks; the writer samples at a fixed rate)."""
        with self._lock:
            self._frame = frame

    def _writer(self):
        period = 1.0 / self.fps
        nxt = time.monotonic()
        w, h = self.size
        blank = np.zeros((h, w, 3), np.uint8)
        while not self._stop.is_set():
            with self._lock:
                f = self._frame
            f = blank if f is None or f.shape[:2] != (h, w) else f
            try:
                self.proc.stdin.write(np.ascontiguousarray(f).tobytes())
            except (BrokenPipeError, ValueError, OSError):
                return
            nxt += period
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()      # encoder fell behind: don't try to catch up

    def _reader(self):
        buf = b""
        au: list[bytes] = []
        out = self.proc.stdout
        while True:
            chunk = out.read(65536)
            if not chunk:
                return
            buf += chunk
            # split on start codes; keep the (possibly incomplete) last NAL in the buffer
            parts = re.split(b"\x00\x00\x00\x01|\x00\x00\x01", buf)
            buf = b"\x00\x00\x00\x01" + parts[-1] if len(parts) > 1 else buf
            for nal in parts[1:-1] if len(parts) > 1 else []:
                if not nal:
                    continue
                t = nal[0] & 0x1F
                if t == 9:                  # access unit delimiter: previous frame is complete
                    if au:
                        self.on_au(au)
                    au = []
                    continue
                if t == 7:
                    self.sps = nal
                elif t == 8:
                    self.pps = nal
                au.append(nal)

    def stop(self):
        self._stop.set()
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()


# ---------------------------------------------------------------------------
# RTP / RTSP
# ---------------------------------------------------------------------------
def rtp_packets(au: list[bytes], seq: int, ts: int, ssrc: int):
    """Packetize one access unit -> (list of RTP packets, next seq). Marker on the last packet."""
    pkts = []

    def hdr(marker):
        nonlocal seq
        h = struct.pack("!BBHII", 0x80, (0x80 if marker else 0) | 96, seq & 0xFFFF, ts & 0xFFFFFFFF, ssrc)
        seq += 1
        return h

    for i, nal in enumerate(au):
        last_nal = i == len(au) - 1
        if len(nal) <= MTU_PAYLOAD:
            pkts.append(hdr(last_nal) + nal)
            continue
        nri_type, body = nal[0], nal[1:]
        fu_ind = (nri_type & 0xE0) | 28
        for off in range(0, len(body), MTU_PAYLOAD):
            piece = body[off:off + MTU_PAYLOAD]
            start, end = off == 0, off + MTU_PAYLOAD >= len(body)
            fu_hdr = (0x80 if start else 0) | (0x40 if end else 0) | (nri_type & 0x1F)
            pkts.append(hdr(last_nal and end) + bytes([fu_ind, fu_hdr]) + piece)
    return pkts, seq


class _Client:
    def __init__(self, conn: socket.socket, addr):
        self.conn, self.addr = conn, addr
        self.session = f"{random.getrandbits(32):08X}"
        self.ssrc = random.getrandbits(32)
        self.seq = random.getrandbits(16)
        self.tcp_channel: int | None = None
        self.udp: tuple[socket.socket, tuple] | None = None
        self.playing = False
        self.need_key = True
        self.q: queue.Queue = queue.Queue(maxsize=60)
        self.send_lock = threading.Lock()
        self.alive = True


class RtspServer:
    """Serves the encoder's stream at rtsp://<host>:<port>/<path> (any path is accepted)."""

    def __init__(self, size=(1280, 720), fps: float = 15, port: int = 8080, path: str = "osd",
                 bitrate_kbps: int = 2500):
        self.port, self.path = port, path
        self.clients: list[_Client] = []
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.listen(8)
        self.enc = H264Encoder(size, fps, bitrate_kbps, self._on_access_unit)
        threading.Thread(target=self._accept, daemon=True, name="rtsp-accept").start()
        self.url = f"rtsp://{local_ip()}:{port}/{path}"

    def push(self, frame: np.ndarray):
        self.enc.push(frame)

    def stop(self):
        self.enc.stop()
        try:
            self.sock.close()
        except OSError:
            pass
        with self._lock:
            for c in self.clients:
                c.alive = False
                try:
                    c.conn.close()
                except OSError:
                    pass

    # --- media fan-out ---
    def _on_access_unit(self, au: list[bytes]):
        ts = int((time.monotonic() - self._t0) * CLOCK)
        key = any((n[0] & 0x1F) == 5 for n in au)
        with self._lock:
            clients = [c for c in self.clients if c.playing and c.alive]
        for c in clients:
            if c.need_key and not key:
                continue                    # start (and resync) on a keyframe
            c.need_key = False
            try:
                c.q.put_nowait((au, ts))
            except queue.Full:              # slow client: drop until the next keyframe
                c.need_key = True

    def _sender(self, c: _Client):
        while c.alive:
            try:
                au, ts = c.q.get(timeout=0.5)
            except queue.Empty:
                continue
            pkts, c.seq = rtp_packets(au, c.seq, ts, c.ssrc)
            try:
                if c.tcp_channel is not None:
                    with c.send_lock:
                        c.conn.sendall(b"".join(b"$" + struct.pack("!BH", c.tcp_channel, len(p)) + p
                                                for p in pkts))
                elif c.udp is not None:
                    s, dst = c.udp
                    for p in pkts:
                        s.sendto(p, dst)
            except OSError:
                c.alive = False

    # --- RTSP control ---
    def _accept(self):
        while True:
            try:
                conn, addr = self.sock.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            c = _Client(conn, addr)
            with self._lock:
                self.clients.append(c)
            threading.Thread(target=self._serve, args=(c,), daemon=True, name="rtsp-client").start()

    def _sdp(self) -> str:
        fmtp = "packetization-mode=1"
        if self.enc.sps and self.enc.pps:
            sps, pps = self.enc.sps, self.enc.pps
            fmtp += (f";profile-level-id={sps[1:4].hex().upper()};sprop-parameter-sets="
                     f"{base64.b64encode(sps).decode()},{base64.b64encode(pps).decode()}")
        return ("v=0\r\n"
                f"o=- 0 0 IN IP4 {local_ip()}\r\n"
                "s=Pluto radar OSD\r\n"
                "c=IN IP4 0.0.0.0\r\n"
                "t=0 0\r\n"
                "a=control:*\r\n"
                "m=video 0 RTP/AVP 96\r\n"
                f"a=rtpmap:96 H264/{CLOCK}\r\n"
                f"a=fmtp:96 {fmtp}\r\n"
                "a=control:track0\r\n")

    def _serve(self, c: _Client):
        f = c.conn.makefile("rb")
        try:
            while c.alive:
                line = f.readline()
                if not line:
                    break
                if line.startswith(b"$"):    # interleaved RTCP from the client: skip it
                    rest = f.read(3)
                    if len(rest) < 3:
                        break
                    f.read(struct.unpack("!H", rest[1:3])[0])
                    continue
                req = line.decode("latin-1").strip()
                if not req:
                    continue
                headers = {}
                while True:
                    h = f.readline().decode("latin-1").strip()
                    if not h:
                        break
                    k, _, v = h.partition(":")
                    headers[k.strip().lower()] = v.strip()
                if int(headers.get("content-length", 0) or 0):
                    f.read(int(headers["content-length"]))
                self._handle(c, req, headers)
        except (OSError, ValueError):
            pass
        finally:
            c.alive = False
            if c.udp:
                c.udp[0].close()
            try:
                c.conn.close()
            except OSError:
                pass
            with self._lock:
                if c in self.clients:
                    self.clients.remove(c)

    def _reply(self, c: _Client, cseq: str, status="200 OK", extra: dict | None = None, body: str = ""):
        lines = [f"RTSP/1.0 {status}", f"CSeq: {cseq}", "Server: pluto-radar"]
        for k, v in (extra or {}).items():
            lines.append(f"{k}: {v}")
        if body:
            lines += ["Content-Type: application/sdp", f"Content-Length: {len(body.encode())}"]
        msg = ("\r\n".join(lines) + "\r\n\r\n" + body).encode()
        with c.send_lock:
            c.conn.sendall(msg)

    def _handle(self, c: _Client, req: str, h: dict):
        method, url = (req.split(" ") + ["", ""])[:2]
        cseq = h.get("cseq", "0")
        sess = {"Session": f"{c.session};timeout=60"}
        if method == "OPTIONS":
            self._reply(c, cseq, extra={"Public": "OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN, GET_PARAMETER"})
        elif method == "DESCRIBE":
            for _ in range(30):              # wait (<= 3 s) for the encoder's first SPS/PPS
                if self.enc.sps and self.enc.pps:
                    break
                time.sleep(0.1)
            base = url if url.endswith("/") else url + "/"
            self._reply(c, cseq, extra={"Content-Base": base}, body=self._sdp())
        elif method == "SETUP":
            tr = h.get("transport", "")
            if "TCP" in tr.upper():
                m = re.search(r"interleaved=(\d+)", tr)
                c.tcp_channel = int(m.group(1)) if m else 0
                ch = c.tcp_channel
                self._reply(c, cseq, extra={**sess, "Transport": f"RTP/AVP/TCP;unicast;interleaved={ch}-{ch + 1}"})
            else:
                m = re.search(r"client_port=(\d+)(?:-(\d+))?", tr)
                if not m:
                    return self._reply(c, cseq, "461 Unsupported Transport")
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind(("0.0.0.0", 0))
                sp = s.getsockname()[1]
                c.udp = (s, (c.addr[0], int(m.group(1))))
                self._reply(c, cseq, extra={**sess, "Transport": f"RTP/AVP;unicast;client_port={m.group(1)}-"
                                                                 f"{m.group(2) or int(m.group(1)) + 1};"
                                                                 f"server_port={sp}-{sp + 1}"})
        elif method == "PLAY":
            self._reply(c, cseq, extra={**sess, "Range": "npt=0.000-"})
            if not c.playing:
                c.playing = True
                threading.Thread(target=self._sender, args=(c,), daemon=True, name="rtsp-send").start()
        elif method == "TEARDOWN":
            self._reply(c, cseq, extra=sess)
            c.alive = False
        elif method in ("GET_PARAMETER", "SET_PARAMETER"):
            self._reply(c, cseq, extra=sess)
        else:
            self._reply(c, cseq, "405 Method Not Allowed")
