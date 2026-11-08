#!/usr/bin/env python3
"""Live version of offline/preview.py, for the Pi's own monitor (Pi OS Lite, no desktop).

Draws straight into the console framebuffer (/dev/fb0) instead of using picamera2's
DRM preview, so it doesn't depend on DRM plane commits. Layout on screen:

  +----------------------------------------------+
  | t, frame, fps, exposure, drops | IMU status  |   text bar (5x per second)
  +----------------------------------------------+
  |                                              |
  |   camera, exactly as recorder.py records it  |   grayscale, rotated like the recorder
  |                                              |
  +----------------------------------------------+
  | gyro x/y/z, last 4 s ............... | now   |   strip (5x per second)
  +----------------------------------------------+

Shake the drone: the image should jump at the same moment the gyro spikes at the right edge.

Nothing is saved. The camera and UART can only be used by one program, so stop
recorder.py first. Works from the local console or over SSH (it draws on the HDMI monitor).
  python3 live_view.py                 # same camera settings as recorder.py
  python3 live_view.py --no-imu        # camera only
  python3 live_view.py --scale 2       # 2x bigger on screen (more CPU)
Ctrl+C to quit.
"""
import argparse
import binascii
import collections
import os
import struct
import sys
import threading
import time

import numpy as np
from PIL import Image, ImageDraw

PKT = struct.Struct("<2sHI6fH")  # same 34-byte packet as firmware/app_imu_uart
SYNC = b"\xA5\x5A"
DEG2RAD = np.pi / 180.0
HIST_S = 4.0
BAR_H = 36
STRIP_H = 110
COLORS = [(255, 90, 90), (90, 220, 90), (90, 170, 255)]  # gyro x, y, z (RGB)


def now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


# ---------------------------------------------------------------- IMU
class ImuStream:
    """Parses the Crazyflie UART stream and keeps the last few seconds of gyro on the Pi clock.

    CF time is mapped to Pi time with a running lower envelope of (arrival - t_cf),
    the live equivalent of the clock fit in offline/convert.py.
    """

    def __init__(self, hist_s=HIST_S):
        self.buf = bytearray()
        self.lock = threading.Lock()
        self.samples = collections.deque()          # (t_pi_s, gx, gy, gz) rad/s
        self.hist_s = hist_s
        self.wrap = 0
        self.last_tus = None
        self.last_seq = None
        self.offset_mins = collections.deque()      # (second, min(arr - t_cf)) per second
        self.ok = self.crc_err = self.lost = 0
        self.rate_hz = 0.0
        self._rate_t, self._rate_n = time.monotonic(), 0

    def _offset_ns(self, arr_ns, tcf_ns):
        d, sec = arr_ns - tcf_ns, arr_ns // 1_000_000_000
        if self.offset_mins and self.offset_mins[-1][0] == sec:
            if d < self.offset_mins[-1][1]:
                self.offset_mins[-1] = (sec, d)
        else:
            self.offset_mins.append((sec, d))
        while self.offset_mins and self.offset_mins[0][0] < sec - 5:
            self.offset_mins.popleft()
        return min(m for _, m in self.offset_mins)

    def feed(self, data: bytes, arr_ns: int):
        self.buf += data
        out = []
        while True:
            p = self.buf.find(SYNC)
            if p < 0:
                del self.buf[:-1]
                break
            if p > 0:
                del self.buf[:p]
            if len(self.buf) < PKT.size:
                break
            body = bytes(self.buf[2:32])
            if binascii.crc_hqx(body, 0xFFFF) != struct.unpack_from("<H", self.buf, 32)[0]:
                self.crc_err += 1
                del self.buf[:1]
                continue
            _, seq, tus, gx, gy, gz, _, _, _, _ = PKT.unpack_from(self.buf)
            del self.buf[:PKT.size]
            if self.last_tus is not None and tus < self.last_tus - (1 << 31):
                self.wrap += 1 << 32
            self.last_tus = tus
            if self.last_seq is not None:
                self.lost += (seq - self.last_seq - 1) & 0xFFFF
            self.last_seq = seq
            tcf_ns = (tus + self.wrap) * 1000
            t_pi = (tcf_ns + self._offset_ns(arr_ns, tcf_ns)) * 1e-9
            out.append((t_pi, gx * DEG2RAD, gy * DEG2RAD, gz * DEG2RAD))
        if out:
            with self.lock:
                self.samples.extend(out)
                while self.samples and self.samples[0][0] < out[-1][0] - self.hist_s - 1:
                    self.samples.popleft()
            self.ok += len(out)
            self._rate_n += len(out)
        t = time.monotonic()
        if t - self._rate_t >= 1.0:
            self.rate_hz, self._rate_t, self._rate_n = self._rate_n / (t - self._rate_t), t, 0

    def window(self, t_end):
        with self.lock:
            a = np.array(self.samples) if self.samples else np.zeros((0, 4))
        return a[(a[:, 0] >= t_end - self.hist_s) & (a[:, 0] <= t_end)] if len(a) else a


def serial_reader(port, baud, imu, stop):
    import serial
    ser = serial.Serial(port, baud, timeout=0.01)
    ser.reset_input_buffer()
    while not stop.is_set():
        data = ser.read(max(1, ser.in_waiting))
        if data:
            imu.feed(data, now_ns())
    ser.close()


# ---------------------------------------------------------------- framebuffer
class Framebuffer:
    """Minimal /dev/fb0 writer for 16 bpp (RGB565) and 32 bpp (XRGB8888) consoles."""

    def __init__(self, dev="/dev/fb0", sysfs="/sys/class/graphics/fb0", geometry=None):
        if geometry is None:
            w, h = (int(v) for v in open(f"{sysfs}/virtual_size").read().strip().split(","))
            bpp = int(open(f"{sysfs}/bits_per_pixel").read())
            stride = int(open(f"{sysfs}/stride").read())
        else:
            w, h, bpp, stride = geometry
        if bpp not in (16, 32):
            raise SystemExit(f"Unsupported framebuffer depth {bpp} bpp")
        self.w, self.h, self.bpp = w, h, bpp
        self.mem = np.memmap(dev, np.uint8, "r+", shape=(h, stride))

    def blit(self, rgb, x0, y0):
        """Write an HxWx3 RGB uint8 image with its top-left corner at (x0, y0), clipped."""
        h = min(rgb.shape[0], self.h - y0)
        w = min(rgb.shape[1], self.w - x0)
        if h <= 0 or w <= 0:
            return
        rgb = rgb[:h, :w]
        if self.bpp == 32:  # little-endian XRGB8888 -> bytes B, G, R, X
            px = np.empty((h, w, 4), np.uint8)
            px[..., 0], px[..., 1], px[..., 2], px[..., 3] = rgb[..., 2], rgb[..., 1], rgb[..., 0], 255
            self.mem[y0:y0 + h, x0 * 4:(x0 + w) * 4] = px.reshape(h, w * 4)
        else:               # RGB565
            r, g, b = (rgb[..., i].astype(np.uint16) for i in range(3))
            v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
            self.mem[y0:y0 + h, x0 * 2:(x0 + w) * 2] = v.view(np.uint8).reshape(h, w * 2)


# ---------------------------------------------------------------- drawing
def cpu_temp():
    try:
        return int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
    except OSError:
        return float("nan")


def draw_bar(w, cam, imu):
    img = Image.new("RGB", (w, BAR_H), (0, 0, 0))
    d = ImageDraw.Draw(img)
    d.text((6, 3), f"t={cam['t_rel']:7.1f}s  frame {cam['n']}  {cam['fps']:4.1f} fps  "
                   f"exp {cam['exp']}us  gain {cam['gain']:.1f}  CPU {cpu_temp():.0f}C",
           fill=(255, 255, 0))
    if imu is not None:
        d.text((6, 19), f"IMU {imu.rate_hz:4.0f} Hz  crc_err {imu.crc_err}  lost {imu.lost}"
                        + ("   NO IMU DATA" if imu.ok == 0 else ""),
               fill=(255, 255, 0) if imu.ok else (255, 60, 60))
    if cam["dropped"]:
        d.text((w - 110, 19), f"{cam['dropped']} DROPPED", fill=(255, 60, 60))
    return np.asarray(img)


def draw_strip(w, t_now, imu, gyro_range):
    img = Image.new("RGB", (w, STRIP_H), (25, 25, 25))
    d = ImageDraw.Draw(img)
    mid = STRIP_H // 2
    d.line((0, mid, w, mid), fill=(80, 80, 80))
    win = imu.window(t_now)
    if len(win) > 1:
        x = (win[:, 0] - (t_now - HIST_S)) / HIST_S * (w - 1)
        for c in range(3):
            y = np.clip(mid - win[:, 1 + c] / gyro_range * (mid - 4), 2, STRIP_H - 2)
            d.line(list(zip(x.tolist(), y.tolist())), fill=COLORS[c], width=1)
    d.line((w - 2, 0, w - 2, STRIP_H), fill=(255, 255, 255), width=2)
    for c, name in enumerate("xyz"):
        d.text((6 + 60 * c, 3), f"gyro {name}", fill=COLORS[c])
    d.text((w - 190, 3), f"+-{gyro_range:.0f} rad/s, last {HIST_S:.0f} s", fill=(220, 220, 220))
    return np.asarray(img)


def upscale(a, s):
    return a if s == 1 else np.repeat(np.repeat(a, s, axis=0), s, axis=1)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--exposure-us", type=int, default=3000)
    ap.add_argument("--gain", type=float, default=4.0)
    ap.add_argument("--lens-position", type=float, default=0.5)
    ap.add_argument("--sensor-size", default="2304x1296")
    ap.add_argument("--rotate", type=int, choices=(0, 180), default=180)
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--no-imu", action="store_true")
    ap.add_argument("--gyro-range", type=float, default=5.0, help="strip half-height in rad/s")
    ap.add_argument("--scale", type=int, choices=(1, 2), default=1, help="on-screen size multiplier")
    ap.add_argument("--display-fps", type=float, default=15.0,
                    help="screen refresh rate; the camera still runs at --fps and drops are counted")
    ap.add_argument("--panel-hz", type=float, default=5.0, help="text bar / gyro strip redraw rate")
    args = ap.parse_args()

    from libcamera import Transform
    from picamera2 import Picamera2
    try:
        from libcamera import controls as lc_controls
    except ImportError:
        lc_controls = None

    fb = Framebuffer()
    w, h, s = args.width, args.height, args.scale
    total_h = BAR_H + h + (0 if args.no_imu else STRIP_H)
    x0 = max(0, (fb.w - w * s) // 2)
    y0 = max(0, (fb.h - total_h * s) // 2)
    print(f"Framebuffer {fb.w}x{fb.h} @ {fb.bpp} bpp -> view {w * s}x{total_h * s} at ({x0},{y0})")

    stop = threading.Event()
    imu = None
    if not args.no_imu:
        imu = ImuStream()
        threading.Thread(target=serial_reader, args=(args.port, args.baud, imu, stop), daemon=True).start()

    # camera: identical settings to recorder.py so the view matches what gets recorded
    sw, sh = (int(v) for v in args.sensor_size.split("x"))
    frame_us = int(round(1e6 / args.fps))
    cam = Picamera2()
    ctrl = {"AeEnable": False, "AwbEnable": False, "ExposureTime": args.exposure_us,
            "AnalogueGain": args.gain, "FrameDurationLimits": (frame_us, frame_us)}
    if "AfMode" in cam.camera_controls and lc_controls is not None:
        ctrl["AfMode"] = lc_controls.AfModeEnum.Manual
        ctrl["LensPosition"] = args.lens_position
    cfg = cam.create_video_configuration(
        main={"size": (w, h), "format": "YUV420"}, raw={"size": (sw, sh)},
        buffer_count=6, controls=ctrl,
        transform=Transform(hflip=args.rotate == 180, vflip=args.rotate == 180))
    cam.configure(cfg)
    cam.start()

    sys.stdout.write("\033[2J\033[?25l")  # clear console, hide cursor
    sys.stdout.flush()
    state = {"n": 0, "t_rel": 0.0, "exp": 0, "gain": 0.0, "dropped": 0, "fps": 0.0}
    dts = collections.deque(maxlen=30)
    period = frame_us * 1e-6
    t0 = last = None
    next_show = next_panel = 0.0
    try:
        while True:
            req = cam.capture_request()
            try:
                md = req.get_metadata()
                t = md.get("SensorTimestamp", 0) * 1e-9
                tick = time.monotonic()
                show = tick >= next_show
                gray = req.make_array("main")[:h, :w].copy() if show else None
            finally:
                req.release()
            if t0 is None:
                t0 = t
            if last is not None:
                dts.append(t - last)
                state["dropped"] += max(0, round((t - last) / period) - 1)
            last = t
            state.update(n=state["n"] + 1, t_rel=t - t0, exp=md.get("ExposureTime", 0),
                         gain=md.get("AnalogueGain", 0.0), fps=1.0 / np.mean(dts) if dts else 0.0)

            if tick >= next_panel:
                next_panel = tick + 1.0 / args.panel_hz
                fb.blit(upscale(draw_bar(w, state, imu), s), x0, y0)
                if imu is not None:
                    fb.blit(upscale(draw_strip(w, t, imu, args.gyro_range), s), x0, y0 + (BAR_H + h) * s)
            if show:
                next_show = tick + 1.0 / args.display_fps
                fb.blit(upscale(np.repeat(gray[..., None], 3, axis=2), s), x0, y0 + BAR_H * s)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        cam.stop()
        sys.stdout.write("\033[?25h\033[2J\033[H")  # restore cursor, clear
        sys.stdout.flush()


if __name__ == "__main__":
    main()
