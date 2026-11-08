#!/usr/bin/env python3
"""Pluto SDR 2.4 GHz CW Doppler human presence detector.

    python main.py run                     live radar + plots
    python main.py run --headless          live radar, status lines only
    python main.py run --sim               simulated scene (no hardware)
    python main.py calibrate               empty-room calibration -> calib.json
    python main.py record out.npz -s 120   save slow-time IQ for offline tuning
    python main.py replay out.npz          replay a recording through the detector
"""
from __future__ import annotations

import argparse
import faulthandler
import os
import sys
import time
from pathlib import Path

import numpy as np

from presence.config import Config, Thresholds
from presence.detector import Pipeline, boundary_level, format_status
from presence.radio import ArraySource, PlutoSource, load_recording, save_recording
from presence.sim import DEFAULT_SCENARIO, simulate_mix, simulate_slow_time


def build_config(a) -> Config:
    cfg = Config()
    cfg.uri = a.uri
    cfg.center_freq = a.freq
    cfg.rx_gain_db = a.rx_gain
    cfg.tx_atten_db = a.tx_atten
    if a.breath_win:
        cfg.breath_win_s = a.breath_win
    cfg.auto_floor = not getattr(a, "no_auto_floor", False)
    cfg.heart = bool(getattr(a, "heart", False))
    if cfg.heart:
        cfg.breath_min_win_s = 10.0     # breathing reading appears ~8 s before heart rate
    cfg.validate()
    return cfg


def build_source(a, cfg: Config, scenario=DEFAULT_SCENARIO):
    if getattr(a, "file", None):
        iq, fs = load_recording(a.file)
        if int(fs) != cfg.slow_fs:
            sys.exit(f"recording is at {fs} Hz, expected {cfg.slow_fs} Hz")
        return ArraySource(iq, fs, realtime=not a.fast)
    if a.sim:
        iq = simulate_slow_time(scenario, fs=cfg.slow_fs, fc=cfg.center_freq, seed=a.seed)
        return ArraySource(iq, cfg.slow_fs, realtime=not a.fast, loop=not a.fast)
    return PlutoSource(cfg)


class Recorder:
    def __init__(self, path: str | None):
        self.path, self.chunks = path, []

    def add(self, x):
        if self.path is not None and len(x):
            self.chunks.append(x.astype(np.complex64))

    def save(self, cfg: Config):
        if self.path and self.chunks:
            iq = np.concatenate(self.chunks)
            save_recording(self.path, iq, cfg.slow_fs, cfg)
            print(f"saved {len(iq) / cfg.slow_fs:.1f} s to {self.path}")


def cmd_run(a):
    cfg = build_config(a)
    th = Thresholds.load(a.calib)
    print(f"thresholds: motion {th.motion_db:.1f} dB, breathing SNR {th.breath_snr_db:.1f} dB, "
          f"ACF {th.breath_acf:.2f}" + ("" if Path(a.calib).exists() else "  (defaults)"))
    print("background: " + ("auto floor (learns over ~10 min; first 15 s uses the base threshold)"
                            if cfg.auto_floor else "fixed threshold"))
    if th.motion_abs_db is not None:
        print(f"range boundary: motion weaker than {th.motion_abs_db:.1f} dBFS is ignored"
              + (f" (~{th.boundary_m:g} m)" if th.boundary_m else ""))
        if th.gain_mismatch(cfg):
            print(f"WARNING: boundary was measured at rx-gain {th.rx_gain_db:g} / tx-atten {th.tx_atten_db:g}; "
                  f"now {cfg.rx_gain_db:g} / {cfg.tx_atten_db:g}. Shifting it by the gain difference.")
            th.motion_abs_db += (cfg.rx_gain_db - th.rx_gain_db) + (cfg.tx_atten_db - th.tx_atten_db)
    src = build_source(a, cfg)
    pipe = Pipeline(cfg, th)
    rec = Recorder(a.record)
    src.start()
    last_print = [-1e9]

    def on_status(s):
        if s.t - last_print[0] >= a.print_every:
            last_print[0] = s.t
            print(format_status(s), flush=True)

    try:
        if a.headless:
            while not src.finished:
                x = src.read()
                rec.add(x)
                for s in pipe.feed(x):
                    on_status(s)
        else:
            import matplotlib
            if a.backend:
                matplotlib.use(a.backend)
            import signal

            from presence import viz
            from presence.viz import HeartView, LiveView
            # Tk swallows KeyboardInterrupt inside callbacks; route SIGINT/SIGTERM through a flag
            # so the window closes and the recording is saved.
            for sig in (signal.SIGINT, signal.SIGTERM):
                signal.signal(sig, lambda *_: viz.STOP.set())
            orig_read = src.read

            def read_and_record(timeout=0.5):
                x = orig_read(timeout)
                rec.add(x)
                return x
            src.read = read_and_record
            view = (HeartView(src, pipe, on_status=on_status, ref_bpm=a.ref_bpm) if cfg.heart
                    else LiveView(src, pipe, on_status=on_status))
            view.show()
            if getattr(view.anim, "event_source", None) is not None:
                view.anim.event_source.stop()   # window closed: no more timer callbacks
    except KeyboardInterrupt:
        pass
    finally:
        src.stop()
        rec.save(cfg)


def cmd_record(a):
    cfg = build_config(a)
    src = build_source(a, cfg)
    rec = Recorder(a.out)
    src.start()
    t0 = time.monotonic()
    n = 0
    try:
        while n < a.seconds * cfg.slow_fs and not src.finished:
            x = src.read()
            rec.add(x)
            prev, n = n, n + len(x)
            if n // cfg.slow_fs != prev // cfg.slow_fs:   # once per second
                print(f"\rrecording {n / cfg.slow_fs:5.0f} / {a.seconds:.0f} s", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        src.stop()
        rec.save(cfg)
    if a.sim or getattr(a, "file", None):
        return
    print(f"wall time {time.monotonic() - t0:.1f} s")


def _collect(src, pipe, seconds, every=25):
    stats = []
    src.start()
    try:
        while pipe.t < seconds and not src.finished:
            for s in pipe.feed(src.read()):
                stats.append(s)
                if len(stats) % every == 0:
                    print(format_status(s), flush=True)
    finally:
        src.stop()
    return stats


def _countdown(sec):
    for k in range(int(sec), 0, -1):
        print(f"\rstarting in {k:3d} s ", end="", flush=True)
        time.sleep(1)
    if sec:
        print()


def cmd_calibrate(a):
    cfg = build_config(a)
    th = Thresholds.load(a.calib)          # merge into an existing file
    if a.proximity is not None:
        a.sim_scale = a.sim_scale or 1.0
        return calibrate_proximity(a, cfg, th)
    if a.boundary is not None:
        return calibrate_boundary(a, cfg, th)
    seconds = a.seconds or 90.0
    src = build_source(a, cfg, scenario=[("empty", seconds + 5)])
    pipe = Pipeline(cfg, Thresholds(motion_db=np.inf), gate_breathing_on_motion=False, auto_floor=False)
    print(f"Empty-area calibration ({seconds:.0f} s). Optional: the auto floor handles the motion "
          "threshold without it. Nobody should be within range.")
    _countdown(a.delay)
    stats = _collect(src, pipe, seconds)
    m = np.array([s.motion_db for s in stats])
    br = [s.breath for s in stats if s.breath is not None]
    if len(m) < 10 or len(br) < 10:
        sys.exit("not enough data for calibration - run longer (>= 60 s)")
    snr = np.array([b.snr_db for b in br])
    acf = np.array([b.acf for b in br])
    th.motion_db = float(max(4.0, np.percentile(m, 99.5) + a.margin))
    th.breath_snr_db = float(max(8.0, np.percentile(snr, 99) + a.margin))
    th.breath_acf = float(np.clip(np.percentile(acf, 95) + 0.15, 0.4, 0.85))
    print(f"empty-area motion: median {np.median(m):.1f} dB, p99.5 {np.percentile(m, 99.5):.1f} dB")
    print(f"empty-area breathing SNR: median {np.median(snr):.1f} dB, p99 {np.percentile(snr, 99):.1f} dB;"
          f" ACF p95 {np.percentile(acf, 95):.2f}")
    th.save(a.calib)
    print(f"saved {a.calib}: {th}")


# (label, prompt, settle seconds before collecting, collect seconds)
PROXIMITY_PHASES = [
    ("away", "STEP AWAY: go at least {far:g} m from the antennas (or out of the area). "
             "Other people can carry on as they will during the demo.", 10, 25),
    ("still", "COME TO {d:g} m and STAY STILL (sit or stand, breathe normally, face the antennas).", 14, 25),
    ("move", "STAY AT {d:g} m and MOVE: gesture, shift your weight, turn, wave.", 8, 15),
]


def _sim_proximity_source(cfg, a, total):
    t_away = PROXIMITY_PHASES[0][2] + PROXIMITY_PHASES[0][3]
    t_still = PROXIMITY_PHASES[1][2] + PROXIMITY_PHASES[1][3]
    near = [("empty", t_away), ("still", t_still, a.sim_scale), ("walk", total - t_away - t_still + 5, a.sim_scale)]
    far = [("walk", total + 5, 0.2)]          # bystanders farther away the whole time
    iq = simulate_mix(near, far, fs=cfg.slow_fs, fc=cfg.center_freq, seed=a.seed)
    return ArraySource(iq, cfg.slow_fs, realtime=not a.fast)


def calibrate_proximity(a, cfg, th):
    """Guided session: away / still at D / moving at D -> still-energy + motion-level thresholds."""
    d = a.proximity
    total = sum(p[2] + p[3] for p in PROXIMITY_PHASES)
    src = _sim_proximity_source(cfg, a, total) if a.sim else build_source(a, cfg)
    th.still_abs_db = th.motion_abs_db = None          # measure raw levels
    pipe = Pipeline(cfg, th, auto_floor=False)
    rec = Recorder(a.session)
    print(f"Proximity calibration for a {d:g} m presence zone, about {total:.0f} s. Follow the prompts.\n")
    schedule, t0 = [], 0.0
    for label, prompt, settle, dur in PROXIMITY_PHASES:
        schedule.append((label, prompt.format(d=d, far=max(3 * d, 3.0)), t0, t0 + settle, t0 + settle + dur))
        t0 += settle + dur
    stats = {label: [] for label, *_ in PROXIMITY_PHASES}
    announced, last_tick = set(), -1
    _countdown(a.delay)
    src.start()
    try:
        while pipe.t < total and not src.finished:
            x = src.read()
            rec.add(x)
            for s in pipe.feed(x):
                for label, prompt, ts, tc, te in schedule:
                    if ts <= s.t < te:
                        if label not in announced:
                            announced.add(label)
                            print(f"\n>>> {prompt}", flush=True)
                        if s.t >= tc:
                            stats[label].append(s)
                        if int(s.t) != last_tick:
                            last_tick = int(s.t)
                            what = "get ready" if s.t < tc else "measuring"
                            print(f"\r    [{label}] {what}: {te - s.t:4.0f} s left   motion {s.motion_abs:6.1f} dBFS"
                                  f"   near {s.still_abs:6.1f} dBFS ", end="", flush=True)
    finally:
        print()
        src.stop()
        rec.save(cfg)
    if any(len(v) < 20 for v in stats.values()):
        sys.exit("calibration interrupted - not enough data")

    def pct(v, q):
        return float(np.percentile(v, q))

    away_near = np.array([s.still_abs for s in stats["away"]])
    still_near = np.array([s.still_abs for s in stats["still"]])
    gap = pct(still_near, 10) - pct(away_near, 95)
    th.still_abs_db = (pct(still_near, 10) + pct(away_near, 95)) / 2 if gap > 0 else pct(away_near, 95) + 1.0

    away_mot = np.array([s.motion_abs for s in stats["away"] if s.motion_db > s.motion_thr])
    move_mot = np.array([s.motion_abs for s in stats["move"] if s.motion_db > s.motion_thr])
    if len(move_mot) < 5:
        sys.exit("hardly any motion seen in the MOVE phase - move more or come closer")
    hi_away = pct(away_mot, 95) if len(away_mot) >= 5 else -200.0
    m_gap = pct(move_mot, 25) - hi_away
    th.motion_abs_db = (pct(move_mot, 25) + hi_away) / 2 if hi_away > -200 and m_gap > 0 else (
        pct(move_mot, 25) - 3.0 if hi_away <= -200 else hi_away + 1.0)
    th.boundary_m = d
    th.rx_gain_db, th.tx_atten_db = cfg.rx_gain_db, cfg.tx_atten_db

    print("\nnear (still-energy) level, dBFS     p10     med     p95")
    print(f"  away (others around)          {pct(away_near, 10):7.1f} {np.median(away_near):7.1f} {pct(away_near, 95):7.1f}")
    print(f"  you still at {d:g} m              {pct(still_near, 10):7.1f} {np.median(still_near):7.1f} {pct(still_near, 95):7.1f}")
    print(f"  -> separation {gap:+.1f} dB, threshold {th.still_abs_db:.1f} dBFS")
    print("motion level while moving, dBFS")
    print(f"  away: {len(away_mot)} updates with motion" + (f", p95 {hi_away:.1f}" if len(away_mot) >= 5 else ""))
    print(f"  you moving at {d:g} m: p25 {pct(move_mot, 25):.1f}, median {np.median(move_mot):.1f}")
    print(f"  -> threshold {th.motion_abs_db:.1f} dBFS")
    if gap <= 0:
        print("\nWARNING: 'away' and 'still' overlap - presence of a still person can't be separated from "
              "the background. Come closer (smaller zone), or reduce what moves in the background.")
    elif gap < 3:
        print("\nNOTE: small margin (<3 dB); expect occasional flicker. A closer zone separates better.")
    th.save(a.calib)
    print(f"saved {a.calib} (session recording: {a.session})")


def calibrate_boundary(a, cfg, th):
    seconds = a.seconds or 30.0
    src = build_source(a, cfg, scenario=[("walk", seconds + 5, a.sim_scale or 0.25)])
    pipe = Pipeline(cfg, th, auto_floor=False)
    print(f"Boundary calibration: walk around / wave your arms at about {a.boundary:g} m from the "
          f"antennas for {seconds:.0f} s. Others should stay farther away than that or keep still.")
    _countdown(a.delay)
    stats = _collect(src, pipe, seconds, every=10)
    lvl = boundary_level(stats)
    if lvl is None:
        sys.exit("hardly any motion detected at the boundary - move more, come a bit closer, "
                 "or raise --tx-atten / --rx-gain")
    th.motion_abs_db = lvl - a.margin_boundary
    th.boundary_m = a.boundary
    th.rx_gain_db, th.tx_atten_db = cfg.rx_gain_db, cfg.tx_atten_db
    seen = np.array([s.motion_abs for s in stats if s.motion_db > s.motion_thr])
    print(f"motion level at {a.boundary:g} m: median {lvl:.1f} dBFS (p10 {np.percentile(seen, 10):.1f}, "
          f"p90 {np.percentile(seen, 90):.1f}), {len(seen)}/{len(stats)} updates with motion")
    print(f"boundary threshold {th.motion_abs_db:.1f} dBFS. Rough rule: someone 1.4x farther is 6 dB "
          "weaker, 2x farther 12 dB weaker.")
    th.save(a.calib)
    print(f"saved {a.calib}")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--uri", default="ip:192.168.2.1", help="Pluto URI (ip:192.168.2.1 or usb:x.y.z)")
    common.add_argument("--freq", type=float, default=2.45e9, help="LO frequency in Hz (default 2.45e9)")
    common.add_argument("--rx-gain", type=float, default=50.0, help="manual RX gain dB (0..73)")
    common.add_argument("--tx-atten", type=float, default=-10.0, help="TX gain dB (0 = max, -89 = min)")
    common.add_argument("--breath-win", type=float, default=None, help="breathing window seconds (default 30)")
    common.add_argument("--calib", default="calib.json", help="threshold file")
    common.add_argument("--sim", action="store_true", help="use simulated scene instead of the Pluto")
    common.add_argument("--fast", action="store_true", help="sim/replay: run faster than real time")
    common.add_argument("--seed", type=int, default=0)
    common.add_argument("--no-auto-floor", action="store_true",
                        help="use the fixed motion threshold instead of learning the background")
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", parents=[common], help="live detection")
    r.add_argument("--headless", action="store_true", help="no plots, print status lines")
    r.add_argument("--record", metavar="FILE.npz", help="also save slow-time IQ")
    r.add_argument("--print-every", type=float, default=1.0)
    r.add_argument("--heart", action="store_true", help="heart-rate mode (subject still, <=0.5 m)")
    r.add_argument("--ref-bpm", type=float, help="heart mode: reference pulse (watch / oximeter) to compare")
    r.add_argument("--backend", help="matplotlib backend override")
    r.set_defaults(func=cmd_run, file=None)

    rp = sub.add_parser("replay", parents=[common], help="replay a recording")
    rp.add_argument("file")
    rp.add_argument("--headless", action="store_true")
    rp.add_argument("--print-every", type=float, default=1.0)
    rp.add_argument("--heart", action="store_true", help="heart-rate mode (subject still, <=0.5 m)")
    rp.add_argument("--ref-bpm", type=float, help="heart mode: reference pulse (watch / oximeter) to compare")
    rp.add_argument("--backend")
    rp.set_defaults(func=cmd_run, record=None)

    c = sub.add_parser("calibrate", parents=[common], help="empty-room threshold calibration")
    c.add_argument("-s", "--seconds", type=float, default=None, help="default 90 (empty) / 30 (boundary)")
    c.add_argument("--margin", type=float, default=3.0, help="dB added above empty-area percentiles")
    c.add_argument("--proximity", type=float, metavar="METERS",
                   help="guided proximity calibration for a presence zone of this radius (recommended for demos)")
    c.add_argument("--session", default="calib_session.npz", help="proximity mode: save the session IQ here")
    c.add_argument("--boundary", type=float, metavar="METERS",
                   help="boundary mode: you move at this distance; weaker motion is ignored afterwards")
    c.add_argument("--margin-boundary", type=float, default=0.0,
                   help="dB to lower the boundary threshold by (positive = slightly larger range)")
    c.add_argument("--delay", type=float, default=0.0, help="countdown before starting (time to walk out)")
    c.add_argument("--sim-scale", type=float, default=None,
                   help="sim only: echo scale of the person at the boundary (default 0.25 / 1.0 proximity)")
    c.set_defaults(func=cmd_calibrate, file=None)

    rc = sub.add_parser("record", parents=[common], help="record slow-time IQ")
    rc.add_argument("out")
    rc.add_argument("-s", "--seconds", type=float, default=60.0)
    rc.set_defaults(func=cmd_record, file=None)

    a = p.parse_args(argv)
    a.func(a)


if __name__ == "__main__":
    faulthandler.enable()
    main()
    # Everything is saved and the radio released. Skip native (libiio / Tk) destructors at
    # interpreter teardown, which can segfault after closing the plot window.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
