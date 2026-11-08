import numpy as np
import pytest

from presence.config import Config, Thresholds
from presence.detector import EMPTY, MOVING, STILL, Pipeline, Ring
from presence.dsp import ClutterFilter, ToneDemodulator, estimate_breathing, motion_metric
from presence.sim import simulate_slow_time

CFG = Config()


def run(segments, **kw):
    x = simulate_slow_time(segments, fs=CFG.slow_fs, fc=CFG.center_freq, **kw)
    return Pipeline(CFG, Thresholds()).feed(x)


# --- fast-time demodulation ------------------------------------------------
def test_demod_phase_continuous_across_uneven_buffers():
    fs, f_tone, dec = CFG.sample_rate, CFG.tone_freq, CFG.dec
    n = fs * 2
    t = np.arange(n) / fs
    phi = 0.4 * np.sin(2 * np.pi * 0.3 * t) + 2 * np.pi * 7.0 * t     # breathing + 7 Hz Doppler
    rng = np.random.default_rng(1)
    rx = (100 * np.exp(1j * (2 * np.pi * f_tone * t + phi))
          + 500                                                          # DC / LO leakage
          + 50 * np.exp(-2j * np.pi * f_tone * t)                        # IQ image
          + rng.normal(0, 5, n) + 1j * rng.normal(0, 5, n))
    demod = ToneDemodulator(f_tone, fs, dec)
    out, pos = [], 0
    for size in rng.integers(1000, 90_000, 100):
        out.append(demod.process(rx[pos:pos + size]))
        pos += size
        if pos >= n:
            break
    y = np.concatenate(out)
    expect = (100 * np.exp(1j * phi)).reshape(-1, dec).mean(1)[:len(y)]
    assert len(y) >= len(expect) - 1
    err = np.abs(y - expect[:len(y)]) / 100
    assert err.max() < 0.02


def test_clutter_filter_streaming_matches_oneshot():
    x = simulate_slow_time([("walk", 10)], seed=3)
    a = ClutterFilter(2.0, 200).process(x)
    f = ClutterFilter(2.0, 200)
    b = np.concatenate([f.process(c) for c in np.array_split(x, 37)])
    assert np.allclose(a, b)


def test_ring_wraparound():
    r = Ring(10)
    r.push(np.arange(7))
    r.push(np.arange(7, 15))
    assert np.array_equal(r.last(10).real, np.arange(5, 15))
    assert np.array_equal(r.last(3).real, [12, 13, 14])


# --- motion -----------------------------------------------------------------
def test_motion_metric_noise_vs_walk():
    rng = np.random.default_rng(0)
    noise = rng.normal(size=200) + 1j * rng.normal(size=200)
    assert abs(motion_metric(noise, 200)) < 4
    t = np.arange(200) / 200
    walk = 10 * np.exp(2j * np.pi * 15 * t) + noise
    assert motion_metric(walk, 200) > 15


# --- breathing --------------------------------------------------------------
@pytest.mark.parametrize("bpm", [10, 15, 24])
def test_breathing_rate(bpm):
    x = simulate_slow_time([("still", 40)], breath_bpm=bpm, seed=bpm)
    x20 = x.reshape(-1, 10).mean(1)[-600:]
    br = estimate_breathing(x20, 20, CFG.wavelength, CFG.breath_band, CFG.breath_noise_band)
    assert br is not None
    assert abs(br.bpm - bpm) < 1.5
    assert br.snr_db > 10 and br.acf > 0.5


# --- end-to-end detector ----------------------------------------------------
def test_empty_room_stays_empty():
    for seed in range(3):
        states = {s.state for s in run([("empty", 90)], seed=seed)}
        assert states == {EMPTY}, seed


def test_walking_detected_quickly():
    st = run([("empty", 10), ("walk", 10)])
    moving = [s.t for s in st if s.state == MOVING]
    assert moving and moving[0] < 11.5
    assert all(s.state == MOVING for s in st if 12 <= s.t <= 20)


def test_still_person_breathing_after_walk():
    st = run([("walk", 10), ("still", 60)], breath_bpm=15)
    late = [s for s in st if s.t > 40]
    assert all(s.state == STILL for s in late)
    locked = [s for s in late if s.confirmed]
    assert len(locked) > 0.8 * len(late)
    assert abs(np.median([s.breath.bpm for s in locked]) - 15) < 1.5


def test_still_person_detected_without_prior_motion():
    st = run([("still", 60)], breath_bpm=18, seed=5)
    assert any(s.state == STILL and s.confirmed for s in st if s.t > 30)
    assert all(s.state != MOVING for s in st)


def test_person_leaves_returns_to_empty():
    st = run([("still", 40), ("walk", 5), ("empty", 50)])
    assert st[-1].state == EMPTY


def test_phase_jump_is_not_motion():
    x = simulate_slow_time([("empty", 30)], seed=2)
    x[3000:] *= np.exp(2.0j)          # e.g. dropped samples rotate everything
    st = Pipeline(CFG, Thresholds()).feed(x)
    assert all(s.state == EMPTY for s in st)
    assert st[-1].jumps == 1


def test_cli_sim_headless(capsys):
    import main
    main.main(["run", "--sim", "--fast", "--headless", "--print-every", "10", "--calib", "/nonexistent"])
    out = capsys.readouterr().out
    assert "MOVING" in out and "STILL" in out


# --- auto floor & range boundary -------------------------------------------
from presence.detector import boundary_level  # noqa: E402
from presence.dsp import motion_levels  # noqa: E402


def test_absolute_level_tracks_echo_strength():
    lv = []
    for scale in (1.0, 0.5, 0.25):
        x = simulate_slow_time([("walk", 20, scale)], seed=4)
        st = Pipeline(CFG, Thresholds(), auto_floor=False).feed(x)
        lv.append(np.median([s.motion_abs for s in st]))
    # halving the echo amplitude = -6 dB
    assert abs((lv[0] - lv[1]) - 6) < 1.5 and abs((lv[1] - lv[2]) - 6) < 1.5


def test_auto_floor_ignores_constant_fan_but_sees_person():
    segs = [("empty", 200), ("walk", 15)]
    x = simulate_slow_time(segs, fan_amp=2.0, seed=6)        # fan alone ~9 dB > fixed threshold
    fixed = Pipeline(CFG, Thresholds(), auto_floor=False).feed(x)
    assert sum(s.state == MOVING for s in fixed if 120 < s.t < 200) > 100   # fixed threshold: false alarms
    auto = Pipeline(CFG, Thresholds()).feed(x)
    assert all(s.state != MOVING for s in auto if 120 < s.t < 200)       # learned within ~2 min
    assert any(s.state == MOVING for s in auto if s.t > 201)


def test_auto_floor_does_not_learn_a_person_moving_for_minutes():
    st = Pipeline(CFG, Thresholds()).feed(simulate_slow_time([("walk", 240)], seed=1))
    late = [s for s in st if s.t > 180]
    assert sum(s.state == MOVING for s in late) > 0.8 * len(late)


def test_boundary_ignores_far_walker_detects_near():
    # boundary measured with a walker at echo scale 0.35 (~1.7x the near distance)
    cal = Pipeline(CFG, Thresholds(), auto_floor=False).feed(simulate_slow_time([("walk", 30, 0.35)], seed=7))
    th = Thresholds(motion_abs_db=boundary_level(cal))
    far = Pipeline(CFG, th).feed(simulate_slow_time([("empty", 10), ("walk", 30, 0.2)], seed=8))
    assert all(s.state == EMPTY for s in far)
    assert sum(s.far_motion for s in far) > 20   # seen as motion, but outside the boundary
    near = Pipeline(CFG, th).feed(simulate_slow_time([("empty", 10), ("walk", 30, 1.0)], seed=9))
    assert sum(s.state == MOVING for s in near if s.t > 12) > 0.9 * len([s for s in near if s.t > 12])


def test_near_breathing_detected_while_far_person_walks():
    cal = Pipeline(CFG, Thresholds(), auto_floor=False).feed(simulate_slow_time([("walk", 30, 0.35)], seed=7))
    th = Thresholds(motion_abs_db=boundary_level(cal))
    near = simulate_slow_time([("still", 70)], seed=10, breath_bpm=16)
    far = simulate_slow_time([("empty", 20), ("walk", 50, 0.12)], seed=11) - simulate_slow_time(
        [("empty", 70)], seed=11)  # far walker only (remove duplicate clutter/noise)
    st = Pipeline(CFG, th).feed(near + far)
    assert all(s.state != MOVING for s in st)
    late = [s for s in st if s.t > 50]
    assert sum(s.state == STILL and s.confirmed for s in late) > 0.7 * len(late)


def test_calibrate_boundary_cli(tmp_path, capsys):
    import main
    calib = tmp_path / "c.json"
    main.main(["calibrate", "--sim", "--fast", "--boundary", "4", "--calib", str(calib)])
    th = Thresholds.load(calib)
    assert th.motion_abs_db is not None and th.boundary_m == 4.0
    main.main(["run", "--sim", "--fast", "--headless", "--calib", str(calib), "--print-every", "20"])
    assert "range boundary" in capsys.readouterr().out


# --- proximity mode -----------------------------------------------------------
from presence.sim import simulate_mix  # noqa: E402


@pytest.fixture(scope="module")
def prox_th(tmp_path_factory):
    import main
    calib = tmp_path_factory.mktemp("p") / "prox.json"
    main.main(["calibrate", "--sim", "--fast", "--proximity", "1", "--calib", str(calib),
               "--session", str(calib.with_suffix(".npz"))])
    th = Thresholds.load(calib)
    assert th.still_abs_db is not None and th.motion_abs_db is not None
    return th


def test_proximity_away_vs_present_with_bystanders(prox_th):
    near = [("empty", 30), ("walk", 5), ("still", 40), ("walk", 8), ("empty", 30)]
    st = Pipeline(CFG, prox_th).feed(simulate_mix(near, [("walk", 120, 0.2)], seed=60))
    away = [s for s in st if 6 <= s.t < 30] + [s for s in st if 96 <= s.t < 113]
    still = [s for s in st if 38 <= s.t < 75]
    assert all(s.state == EMPTY for s in away)
    assert all(s.state != EMPTY for s in still)
    assert sum(s.confirmed for s in still if s.t > 55) > 0.8 * len([s for s in still if s.t > 55])
    left = next(s.t for s in st if s.t > 83 and s.state == EMPTY)
    assert left - 83 < 12


def test_proximity_ignores_far_still_person(prox_th):
    st = Pipeline(CFG, prox_th).feed(simulate_mix([("still", 60, 0.1)], [("walk", 60, 0.2)], seed=61))
    assert all(s.state == EMPTY for s in st)


def test_stale_breathing_does_not_hold_presence(prox_th):
    st = Pipeline(CFG, prox_th).feed(simulate_mix([("still", 45), ("empty", 30)], [("empty", 75)], seed=62))
    assert st[-1].state == EMPTY


# --- heart rate -----------------------------------------------------------------
def _heart_cfg():
    c = Config()
    c.heart = True
    return c


def test_heart_rate_locks_and_is_accurate():
    for hb, seed in [(62, 40), (78, 41)]:
        x = simulate_mix([("still", 90)], [("empty", 90)], seed=seed, heart_mm=0.4, heart_bpm=hb, noise=0.15)
        st = Pipeline(_heart_cfg(), Thresholds()).feed(x)
        locked = [s.heart_bpm for s in st if s.t > 40 and s.heart_bpm is not None]
        assert len(locked) > 0.3 * len([s for s in st if s.t > 40]), hb
        assert abs(np.median(locked) - hb) < 4, (hb, np.median(locked))


def test_heart_needs_quiet_scene():
    x = simulate_mix([("still", 90)], [("walk", 90, 0.2)], seed=42, heart_mm=0.4, noise=0.15)
    st = Pipeline(_heart_cfg(), Thresholds()).feed(x)
    assert all(s.heart_bpm is None for s in st)
    assert any("hold still" in s.heart_wait for s in st)


def test_heart_ignores_motion_beyond_boundary():
    # boundary from a walker at echo scale 0.1 (~2 m when the subject is ~0.5 m); bystanders at 0.04 (~3 m)
    cal = Pipeline(CFG, Thresholds(), auto_floor=False).feed(simulate_slow_time([("walk", 30, 0.1)], seed=7, noise=0.15))
    th = Thresholds(motion_abs_db=boundary_level(cal))
    x = simulate_mix([("still", 100)], [("walk", 100, 0.04)], seed=43, heart_mm=0.4, heart_bpm=70, noise=0.15)
    st = Pipeline(_heart_cfg(), th).feed(x)
    assert any(s.far_motion for s in st)
    locked = [s.heart_bpm for s in st if s.heart_bpm is not None]
    assert locked and abs(np.median(locked) - 70) < 4
    # without the boundary the same bystander pauses the heart reading entirely
    st2 = Pipeline(_heart_cfg(), Thresholds()).feed(x)
    assert sum(s.heart_bpm is not None for s in st2) < len(locked)


def test_heart_amplitude_demod_immune_to_lo_drift():
    from presence.dsp import block_decimate
    from presence.heart import estimate_heart
    x = simulate_slow_time([("still", 40)], noise=0.01, heart_mm=0.3, heart_bpm=72, seed=1,
                           drift_rad_per_s=1e-3)
    h = estimate_heart(block_decimate(x, 10)[-320:], 20, CFG.wavelength, 15 / 60)
    assert abs(h.bpm - 72) < 4 and h.method == "amp"


def test_heart_last_reading_held_after_walking_up():
    x = simulate_mix([("still", 80), ("walk", 20), ("still", 10)], [("empty", 110)], seed=44,
                     heart_mm=0.4, heart_bpm=66, noise=0.15)
    st = Pipeline(_heart_cfg(), Thresholds()).feed(x)
    walking = [s for s in st if 85 < s.t < 100]
    assert all(s.heart_bpm is None for s in walking)            # live measurement paused
    assert all(s.heart_last_bpm is not None for s in walking)   # ...but the last reading is held
    assert abs(walking[-1].heart_last_bpm - 66) < 4
    assert walking[-1].heart_last_age_s > 5


def test_breathing_reading_arrives_before_heart_and_is_held():
    cfg = _heart_cfg()
    cfg.breath_min_win_s = 10.0
    x = simulate_mix([("walk", 10), ("still", 60), ("walk", 15)], [("empty", 85)], seed=71,
                     breath_bpm=14, heart_mm=0.4, heart_bpm=68, noise=0.15)
    st = Pipeline(cfg, Thresholds()).feed(x)
    tb = next(s.t for s in st if s.breath_bpm is not None)
    th = next(s.t for s in st if s.heart_bpm is not None)
    assert tb < th and tb - 10 < 18
    assert abs(np.median([s.breath_bpm for s in st if s.breath_bpm is not None]) - 14) < 1
    walking = [s for s in st if 75 < s.t < 85]
    assert all(s.breath_bpm is None and s.breath_last_bpm is not None for s in walking)


def test_small_fidget_resets_heart_but_not_breathing():
    cfg = _heart_cfg()
    cfg.breath_min_win_s = 10.0
    x = simulate_mix([("still", 77)], [("empty", 77)], seed=72, breath_bpm=14, heart_mm=0.4,
                     heart_bpm=68, noise=0.15, fidgets=[(45, 1.0, 0.3)])   # brief hand movement
    st = Pipeline(cfg, Thresholds()).feed(x)
    after = [s for s in st if 48 < s.t < 60]
    assert any(s.heart_wait.startswith("hold still") for s in after)        # heart restarted
    assert all(s.breath_wait == "" for s in after)                           # breathing did not
    assert sum(s.breath_bpm is not None for s in after) > 0.8 * len(after)  # and stays locked
