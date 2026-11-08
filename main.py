#!/usr/bin/env python3
"""bladeRF 2.0 2.4 GHz multi-tone CW MIMO radar: presence, breathing, heart rate, 2D people map.

    python main.py run --map               live 2D map of people (direction + range) + plots
    python main.py run --map --headless    live map, track lines only
    python main.py run --map --sim         simulated multi-person scene (no hardware)
    python main.py run                     single-channel presence / breathing view
    python main.py run --heart             heart-rate view
    python main.py calibrate --array 2     optional: refine the antenna array (person at 2 m, 0 deg)
    python main.py record out.npz -s 120   save multichannel slow-time IQ for offline tuning
    python main.py replay out.npz --map    replay a recording through the map
"""
from __future__ import annotations

import argparse
import faulthandler
import os
import sys
import time
from pathlib import Path

import numpy as np

from presence.config import Config, Thresholds, parse_antennas
from presence.detector import Pipeline, boundary_level, format_status
from presence.mapping import MapPipeline, calibrate_array, format_map_status
from presence.radio import ArraySource, BladeRFSource, load_recording, save_recording
from presence.sim import (DEFAULT_SCENARIO, demo_people, person, sim_cal_err, simulate_mix, simulate_scene,
                          simulate_slow_time)


def build_config(a) -> Config:
    cfg = Config()
    cfg.device = a.device
    cfg.center_freq = a.freq
    cfg.rx_gain_db = a.rx_gain
    cfg.tx_gain_db = a.tx_gain
    if a.breath_win:
        cfg.breath_win_s = a.breath_win
    if getattr(a, "antennas", None):
        try:
            cfg.tx_pos, cfg.rx_pos = parse_antennas(a.antennas)
        except ValueError as e:
            sys.exit(str(e))
    cfg.auto_floor = not getattr(a, "no_auto_floor", False)
    cfg.heart = bool(getattr(a, "heart", False))
    if cfg.heart:
        cfg.breath_min_win_s = 10.0     # breathing reading appears ~8 s before heart rate
    cfg.validate()
    return cfg


def apply_saved_layout(a, cfg: Config, th: Thresholds) -> None:
    """Without --antennas, use the antenna layout stored by `calibrate --array` (if any)."""
    if not getattr(a, "antennas", None) and th.antennas is not None:
        cfg.tx_pos = tuple(map(tuple, th.antennas[:2]))
        cfg.rx_pos = tuple(map(tuple, th.antennas[2:]))
    elif th.antennas is not None and [list(p) for p in cfg.tx_pos + cfg.rx_pos] != th.antennas:
        print("WARNING: --antennas differs from the layout the array calibration was made with; "
              "ignoring that calibration (re-run calibrate --array)")
        th.array_cal = None
    cfg.validate()


def describe_layout(cfg: Config) -> str:
    names = ["TX1", "TX2", "RX1", "RX2"]
    pos = list(cfg.tx_pos) + list(cfg.rx_pos)
    return ", ".join(f"{n} {p[0] * 100:+.1f} cm" + (f" (y {p[1] * 100:+.1f})" if p[1] else "")
                     for n, p in sorted(zip(names, pos), key=lambda q: q[1][0]))


def build_source(a, cfg: Config, scenario=DEFAULT_SCENARIO, people=None, seconds: float = 120.0):
    """Recording, simulation (1D legacy `scenario`, or multichannel `people`), or the bladeRF."""
    if getattr(a, "file", None):
        iq, fs = load_recording(a.file, cfg)
        if int(fs) != cfg.slow_fs:
            sys.exit(f"recording is at {fs} Hz, expected {cfg.slow_fs} Hz")
        return ArraySource(iq, fs, realtime=not a.fast)
    if a.sim:
        if people is not None:
            iq = simulate_scene(people, cfg, seconds, seed=a.seed)
        else:
            iq = simulate_slow_time(scenario, fs=cfg.slow_fs, fc=cfg.center_freq, seed=a.seed)
        return ArraySource(iq, cfg.slow_fs, realtime=not a.fast, loop=not a.fast)
    return BladeRFSource(cfg)


class Recorder:
    def __init__(self, path: str | None):
        self.path, self.chunks = path, []

    def add(self, x):
        if self.path is not None and x.shape[-1]:
            self.chunks.append(x.astype(np.complex64))

    def save(self, cfg: Config):
        if self.path and self.chunks:
            iq = np.concatenate(self.chunks, axis=-1)
            save_recording(self.path, iq, cfg.slow_fs, cfg)
            ch = f", {iq.shape[0]} channels" if iq.ndim == 2 else ""
            print(f"saved {iq.shape[-1] / cfg.slow_fs:.1f} s{ch} to {self.path}")


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
            print(f"WARNING: boundary was measured at rx-gain {th.rx_gain_db:g} / tx-gain {th.tx_gain_db:g}; "
                  f"now {cfg.rx_gain_db:g} / {cfg.tx_gain_db:g}. Shifting it by the gain difference.")
            th.motion_abs_db += (cfg.rx_gain_db - th.rx_gain_db) + (cfg.tx_gain_db - th.tx_gain_db)
    use_map = bool(getattr(a, "map", False))
    if use_map:
        apply_saved_layout(a, cfg, th)
        print("antennas: " + describe_layout(cfg))
        print("array calibration: " + ("from " + a.calib if th.array_cal else
                                       "none (geometry + leakage reference only; optional: calibrate --array)"))
        src = build_source(a, cfg, people=demo_people(), seconds=120.0)
        if isinstance(src, ArraySource) and src.data.ndim != 2:
            sys.exit("--map needs a multichannel recording (made with this bladeRF version)")
        pipe = MapPipeline(cfg, th)
    else:
        src = build_source(a, cfg)
        pipe = Pipeline(cfg, th)
    rec = Recorder(a.record)
    src.start()
    last_print = [-1e9]

    def on_status(s):
        if s.t - last_print[0] >= a.print_every:
            last_print[0] = s.t
            print(format_map_status(s) if use_map else format_status(s), flush=True)

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
            from presence.viz import HeartView, LiveView, MapView
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
            if use_map:
                view = MapView(src, pipe, on_status=on_status)
            elif cfg.heart:
                view = HeartView(src, pipe, on_status=on_status, ref_bpm=a.ref_bpm)
            else:
                view = LiveView(src, pipe, on_status=on_status)
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
            prev, n = n, n + x.shape[-1]
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
    if a.array is not None:
        return calibrate_array_cmd(a, cfg, th)
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
    th.rx_gain_db, th.tx_gain_db = cfg.rx_gain_db, cfg.tx_gain_db

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
    th.rx_gain_db, th.tx_gain_db = cfg.rx_gain_db, cfg.tx_gain_db
    seen = np.array([s.motion_abs for s in stats if s.motion_db > s.motion_thr])
    print(f"motion level at {a.boundary:g} m: median {lvl:.1f} dBFS (p10 {np.percentile(seen, 10):.1f}, "
          f"p90 {np.percentile(seen, 90):.1f}), {len(seen)}/{len(stats)} updates with motion")
    print(f"boundary threshold {th.motion_abs_db:.1f} dBFS. Rough rule: someone 1.4x farther is 6 dB "
          "weaker, 2x farther 12 dB weaker.")
    th.save(a.calib)
    print(f"saved {a.calib}")


def calibrate_array_cmd(a, cfg, th):
    """Person stands still (breathing, gently swaying) straight in front of the array at D m."""
    d = a.array
    seconds = a.seconds or 30.0
    if a.sim:
        # unknown per-channel antenna phase errors (sim_cal_err in sim.py), same seed as `run --sim`
        iq = simulate_scene([person([(0, 0.0, d), (seconds + 10, 0.0, d)], bpm=15)], cfg, seconds + 10,
                            seed=a.seed, cal_err=sim_cal_err(cfg, a.seed))
        src = ArraySource(iq, cfg.slow_fs, realtime=not a.fast)
    else:
        src = build_source(a, cfg)
    apply_saved_layout(a, cfg, Thresholds(antennas=th.antennas))
    print("antennas: " + describe_layout(cfg))
    pipe = MapPipeline(cfg, Thresholds(), use_cal=False)
    rec = Recorder(None)
    print(f"Array calibration: stand STILL straight in front of the antennas (0 deg) at {d:g} m, breathe "
          f"normally, sway very slightly. Everyone else out of the area / still. {seconds:.0f} s.")
    _countdown(a.delay)
    src.start()
    last = -1
    try:
        while pipe.t < seconds + 5 and not src.finished:
            x = src.read()
            if isinstance(src, ArraySource) and x.ndim != 2:
                sys.exit("array calibration needs multichannel data")
            rec.add(x)
            pipe.feed(x)
            if int(pipe.t) != last:
                last = int(pipe.t)
                print(f"\r    measuring: {max(0.0, seconds + 5 - pipe.t):4.0f} s left ", end="", flush=True)
    finally:
        print()
        src.stop()
        rec.save(cfg)
    if pipe.t < seconds:
        sys.exit("calibration interrupted - not enough data")
    raw = pipe.raw.last(int(seconds * cfg.slow_fs))
    cal, quality = calibrate_array(raw, cfg, 0.0, d, W=pipe.W)
    print(f"dominant source holds {quality * 100:.0f}% of the micro-motion power"
          + ("" if quality > 0.5 else "  (LOW - someone else moving? stand closer / stiller and retry)"))
    if quality < 0.3:
        sys.exit("calibration rejected")
    th.array_cal = [[float(z.real), float(z.imag)] for z in cal]
    th.antennas = [list(p) for p in cfg.tx_pos + cfg.rx_pos]
    th.save(a.calib)
    print(f"saved array calibration ({len(cal)} channels) to {a.calib}")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--device", default="", help='bladeRF device string (default: first found, e.g. "*:serial=ab12")')
    common.add_argument("--freq", type=float, default=2.45e9, help="LO frequency in Hz (default 2.45e9)")
    common.add_argument("--antennas", metavar="LAYOUT",
                        help='antenna positions along the array axis in cm, e.g. "TX1=-4.5,TX2=-1.5,RX1=1.5,RX2=4.5" '
                             '(saved by calibrate --array; default: lambda/2 layout from config.py)')
    common.add_argument("--rx-gain", type=float, default=40.0, help="manual RX gain dB (both receivers)")
    common.add_argument("--tx-gain", type=float, default=30.0, help="TX gain dB (both transmitters, higher = more power)")
    common.add_argument("--breath-win", type=float, default=None, help="breathing window seconds (default 30)")
    common.add_argument("--calib", default="calib.json", help="threshold file")
    common.add_argument("--sim", action="store_true", help="use a simulated scene instead of the bladeRF")
    common.add_argument("--fast", action="store_true", help="sim/replay: run faster than real time")
    common.add_argument("--seed", type=int, default=0)
    common.add_argument("--no-auto-floor", action="store_true",
                        help="use the fixed motion threshold instead of learning the background")
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", parents=[common], help="live detection")
    r.add_argument("--headless", action="store_true", help="no plots, print status lines")
    r.add_argument("--record", metavar="FILE.npz", help="also save slow-time IQ")
    r.add_argument("--print-every", type=float, default=1.0)
    r.add_argument("--map", action="store_true", help="2D map of people (direction finding + range)")
    r.add_argument("--heart", action="store_true", help="heart-rate mode (subject still, <=0.5 m)")
    r.add_argument("--ref-bpm", type=float, help="heart mode: reference pulse (watch / oximeter) to compare")
    r.add_argument("--backend", help="matplotlib backend override")
    r.set_defaults(func=cmd_run, file=None)

    rp = sub.add_parser("replay", parents=[common], help="replay a recording")
    rp.add_argument("file")
    rp.add_argument("--headless", action="store_true")
    rp.add_argument("--print-every", type=float, default=1.0)
    rp.add_argument("--map", action="store_true", help="2D map of people (multichannel recordings)")
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
    c.add_argument("--array", type=float, metavar="METERS",
                   help="array calibration: one person stands still straight in front at this distance")
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
    # Everything is saved and the radio released. Skip native (libbladeRF / Tk) destructors at
    # interpreter teardown, which can segfault after closing the plot window.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
