"""bladeRF MIMO multi-tone radar: demodulation, localization, tracking, calibration (no hardware)."""
import numpy as np
import pytest

from presence.config import WIDE_LAYOUT, Config, Thresholds
from presence.detector import Ring
from presence.dsp import ClutterFilter, MultiToneDemodulator, PhaseJumpCorrector
from presence.mapping import MOVING_T, STILL_T, MapPipeline, calibrate_array
from presence.radio import ArraySource, load_recording, multitone_tx_buffer, save_recording
from presence.sim import _interp_path, person, simulate_scene

BOARD = Config()                                     # default: antennas on the bladeRF SMA ports
CFG = Config(**WIDE_LAYOUT)                          # algorithm tests: wide lambda/2 line


def run_map(people, seconds, th=None, seed=1, cfg=None, **kw):
    cfg = cfg or CFG
    x = simulate_scene(people, cfg, seconds, seed=seed, **kw)
    return MapPipeline(cfg, th or Thresholds()).feed(x)


def errors(statuses, people, t_min=6.0):
    """Distance of every shown track to the nearest true person, and the people count per update."""
    err, count = [], []
    for s in statuses:
        if s.t < t_min:
            continue
        truth = [_interp_path(p["path"], s.t) for p in people]
        count.append(len(s.tracks))
        err += [min(np.hypot(tr.x - x, tr.y - y) for x, y in truth) for tr in s.tracks]
    return np.array(err), np.array(count)


WALKER = person([(0, -2, 2), (8, -1.5, 5), (16, -2, 2), (24, -1.5, 5), (32, -2, 2)])


# --- radio / demodulation ------------------------------------------------------
def test_default_layout_is_board_sma_ports():
    BOARD.validate()
    # TX2, RX2, RX1, TX1 in one line, uniform 1.5 cm pitch
    xs = [BOARD.tx_pos[1][0], BOARD.rx_pos[1][0], BOARD.rx_pos[0][0], BOARD.tx_pos[0][0]]
    assert np.allclose(np.diff(xs), 0.015) and np.isclose(sum(xs), 0)


def test_config_defaults_valid_and_virtual_array():
    CFG.validate()
    assert CFG.n_channels == 16 and len(CFG.channels()) == 16
    assert CFG.dec % CFG.tone_period == 0
    assert CFG.unambiguous_range() > 20
    # TX spacing = 2 x RX spacing -> virtual element sums (tx + rx) are uniformly spaced
    v = sorted(t[0] + r[0] for t in CFG.tx_pos for r in CFG.rx_pos)
    assert np.allclose(np.diff(v), np.diff(v)[0])


@pytest.mark.parametrize("tones2", [
    (2.95e6, -0.90e6, 1.10e6, 3.10e6),     # image of TX1's -2.95 MHz
    (-2.95e6, -0.90e6, 1.10e6, 3.10e6),    # same tone as TX1
    (-2.90e6, -0.90e6, 1.10e6, 3.12e6),    # off the 50 kHz grid
    (-2.90e6, -0.90e6, 1.10e6, 4.50e6),    # outside the RF bandwidth
])
def test_config_rejects_bad_tones(tones2):
    c = Config()
    c.tones_tx2 = tones2
    with pytest.raises(ValueError):
        c.validate()


def test_multitone_demod_separates_tones_and_receivers():
    fs, dec, period = CFG.sample_rate, CFG.dec, CFG.tone_period
    n = dec * 12
    t = np.arange(n) / fs
    rng = np.random.default_rng(0)
    amp = rng.uniform(50, 200, (2, len(CFG.tones))) * np.exp(1j * rng.uniform(0, 6.3, (2, len(CFG.tones))))
    dop = rng.uniform(-40, 40, len(CFG.tones))           # Doppler per tone (Hz)
    rx = np.zeros((2, n), np.complex64)
    for r in range(2):
        for k, f in enumerate(CFG.tones):
            rx[r] += amp[r, k] * np.exp(2j * np.pi * (f + dop[k]) * t)
        rx[r] += 800 + 30 * np.exp(-2j * np.pi * CFG.tones[0] * t)        # LO leakage + an IQ image
        rx[r] += rng.normal(0, 3, n) + 1j * rng.normal(0, 3, n)
    demod = MultiToneDemodulator(CFG.tones, fs, dec, period, n_rx=2)
    out, pos = [], 0
    for size in rng.integers(1000, 3 * dec, 200):        # uneven buffers
        out.append(demod.process(rx[:, pos:pos + size]))
        pos += size
        if pos >= n:
            break
    y = np.concatenate(out, axis=1)
    assert y.shape == (16, 12)
    for r in range(2):
        for k in range(len(CFG.tones)):
            expect = (amp[r, k] * np.exp(2j * np.pi * dop[k] * t)).reshape(-1, dec).mean(1)
            got = y[r * len(CFG.tones) + k]
            assert np.max(np.abs(got - expect)) / abs(amp[r, k]) < 0.02, (r, k)


def test_tx_buffer_whole_periods_and_in_range():
    buf = multitone_tx_buffer(CFG)
    n = len(buf) // 4
    assert n % 1024 == 0 and n % CFG.tone_period == 0
    assert np.abs(buf).max() <= 2047
    iq = buf.astype(float).reshape(n, 2, 2)
    for ch, tones in enumerate((CFG.tones_tx1, CFG.tones_tx2)):
        z = iq[:, ch, 0] + 1j * iq[:, ch, 1]
        spec = np.abs(np.fft.fft(z))
        f = np.fft.fftfreq(n, 1 / CFG.sample_rate)
        top = np.sort(f[np.argsort(spec)[-len(tones):]])
        assert np.allclose(top, sorted(tones))


def test_2d_ring_clutter_and_jump_filters_match_1d():
    x = simulate_scene([WALKER], CFG, 5, seed=2)
    r = Ring(300, 16)
    r.push(x[:, :250])
    r.push(x[:, 250:700])
    assert np.allclose(r.last(300), x[:, 400:700])
    a = ClutterFilter(2.0, 200).process(x)
    one = ClutterFilter(2.0, 200).process(x[3])
    assert np.allclose(a[3], one)
    y = x.copy()
    y[5, 500:] *= np.exp(1.7j)                            # one tone jumps (dropped samples)
    j = PhaseJumpCorrector(1.0)
    fixed = np.concatenate([j.process(c) for c in np.array_split(y, 20, axis=1)], axis=1)
    assert j.jumps == 1
    assert abs(np.angle(fixed[5, 600:].mean() / x[5, 600:].mean())) < 0.2


def test_recording_roundtrip_multichannel_and_legacy(tmp_path):
    x = simulate_scene([WALKER], CFG, 3, seed=3)
    p = tmp_path / "m.npz"
    save_recording(str(p), x, 200, CFG)
    c = Config()
    iq, fs = load_recording(str(p), c)
    assert iq.shape == x.shape and fs == 200 and c.tones_tx2 == CFG.tones_tx2
    assert np.allclose(iq, x, atol=1e-2 * np.abs(x).max())
    save_recording(str(tmp_path / "l.npz"), x[0], 200)
    iq1, _ = load_recording(str(tmp_path / "l.npz"), Config())
    assert iq1.ndim == 1
    src = ArraySource(iq, 200, realtime=False)
    src.start()
    assert src.read().shape[0] == 16


# --- localization & tracking ---------------------------------------------------------
def test_single_walker_tracked_in_2d():
    st = run_map([WALKER], 32)
    err, count = errors(st, [WALKER])
    assert np.median(err) < 0.4 and np.percentile(err, 90) < 0.8
    assert np.mean(count == 1) > 0.9
    assert len({tr.id for s in st for tr in s.tracks}) == 1      # one stable ID
    assert all(tr.kind == MOVING_T for s in st for tr in s.tracks if s.t < 30)


def test_empty_room_has_no_tracks():
    st = run_map([], 40)
    assert all(len(s.tracks) == 0 for s in st)


def test_person_straight_ahead_is_seen():
    # broadside: every virtual element sees the same phase, nearly parallel to the leakage
    walk = person([(0, 0, 1.5), (8, 0, 5), (16, 0, 1.5), (24, 0, 5)])
    err, count = errors(run_map([walk], 24), [walk])
    assert np.median(err) < 0.4 and np.mean(count == 1) > 0.9


def test_walker_and_sitter_with_breathing():
    sitter = person([(0, 1.5, 3.0), (60, 1.5, 3.0)], bpm=14)
    walk = person([(0, -2, 2), (8, -2, 5), (16, -2, 2), (24, -2, 5), (32, -2, 2), (40, -2, 5), (48, -2, 2)])
    st = run_map([walk, sitter], 48)
    err, count = errors(st, [walk, sitter], t_min=15)
    assert np.median(err) < 0.4
    assert np.mean(count == 2) > 0.9
    late = st[-1].tracks
    still = [tr for tr in late if tr.kind == STILL_T]
    assert len(still) == 1 and np.hypot(still[0].x - 1.5, still[0].y - 3.0) < 0.6
    assert still[0].bpm is not None and abs(still[0].bpm - 14) < 1.0


def test_two_still_people_separated_with_own_breathing_rates():
    a = person([(0, -1.8, 2.5), (60, -1.8, 2.5)], bpm=12)
    b = person([(0, 1.8, 3.0), (60, 1.8, 3.0)], bpm=17)
    st = run_map([a, b], 50, seed=3)
    tracks = st[-1].tracks
    assert len(tracks) == 2
    for p, bpm in ((a, 12), (b, 17)):
        x, y = p["path"][0][1:]
        tr = min(tracks, key=lambda tr: np.hypot(tr.x - x, tr.y - y))
        assert np.hypot(tr.x - x, tr.y - y) < 0.6
        assert abs(np.degrees(np.arctan2(tr.x, tr.y) - np.arctan2(x, y))) < 8
        assert tr.bpm is not None and abs(tr.bpm - bpm) < 1.0


def test_two_independent_walkers():
    a = person([(0, -2, 2), (7, -2.5, 5), (14, -2, 2), (21, -2.5, 5), (28, -2, 2), (35, -2.5, 5)])
    b = person([(0, 3, 4), (4, 3, 4), (9, 1.5, 2), (13, 1, 5), (19, 2.5, 2.5), (24, 3, 5), (30, 1.5, 2),
                (35, 3, 4)], amp=5)
    st = run_map([a, b], 35)
    err, count = errors(st, [a, b])
    assert np.median(err) < 0.4
    # people sharing Doppler bins is the method's limit: the count is right most, not all, of the time
    assert np.mean(count == 2) > 0.85


def test_whitening_handles_leakage_phase_noise():
    sitter = person([(0, 0.0, 2.5), (60, 0.0, 2.5)])
    st = run_map([sitter], 40, leak_pn_rad=5e-3)
    err, count = errors(st, [sitter], t_min=12)
    assert np.mean(count == 1) > 0.9 and np.median(err) < 0.3


def test_array_calibration_recovers_antenna_phase_errors():
    err = np.exp(1j * np.random.default_rng(5).normal(0, 0.7, CFG.n_channels))
    walk = person([(0, -2, 2), (8, -1.5, 5), (16, -2, 2), (24, -1.5, 5)])
    _, count = errors(run_map([walk], 24, seed=6, cal_err=err), [walk])
    assert np.mean(count == 1) < 0.5                      # uncalibrated: badly broken
    mp = MapPipeline(CFG, Thresholds(), use_cal=False)
    mp.feed(simulate_scene([person([(0, 0, 2), (40, 0, 2)])], CFG, 35, seed=7, cal_err=err))
    cal, quality = calibrate_array(mp.raw.last(30 * CFG.slow_fs), CFG, 0.0, 2.0, W=mp.W)
    assert quality > 0.8
    resid = np.angle(cal * err * np.conj(cal[0] * err[0]))
    assert np.max(np.abs(resid)) < 0.15
    th = Thresholds(array_cal=[[z.real, z.imag] for z in cal])
    e, count = errors(run_map([walk], 24, th=th, seed=6, cal_err=err), [walk])
    assert np.mean(count == 1) > 0.9 and np.median(e) < 0.4


def test_array_cal_saved_and_loaded(tmp_path):
    th = Thresholds(array_cal=[[1.0, 0.0]] * 16)
    th.save(tmp_path / "c.json")
    assert Thresholds.load(tmp_path / "c.json").array_cal == [[1.0, 0.0]] * 16


# --- CLI ---------------------------------------------------------------------------
def test_cli_map_sim_headless(capsys):
    import main
    main.main(["run", "--map", "--sim", "--fast", "--headless", "--print-every", "10", "--calib", "/nonexistent"])
    out = capsys.readouterr().out
    assert "TX2 -2.2 cm, RX2 -0.8 cm, RX1 +0.8 cm, TX1 +2.2 cm" in out
    assert "people=2" in out and "moving" in out and "still" in out and "bpm" in out


def test_cli_calibrate_array_sim(tmp_path, capsys):
    import main
    calib = tmp_path / "c.json"
    main.main(["calibrate", "--sim", "--fast", "--array", "2", "--calib", str(calib)])
    th = Thresholds.load(calib)
    assert th.array_cal is not None and len(th.array_cal) == 16
    main.main(["run", "--map", "--sim", "--fast", "--headless", "--print-every", "30", "--calib", str(calib)])
    assert "array calibration: from" in capsys.readouterr().out


def test_parse_antennas_and_bad_input():
    from presence.config import parse_antennas
    tx, rx = parse_antennas("TX1=-4.5, RX1=-1.5, RX2=1.5, TX2=4.5:2")
    assert tx == ((-0.045, 0.0), (0.045, 0.02)) and rx == ((-0.015, 0.0), (0.015, 0.0))
    for bad in ("TX1=1,TX2=2,RX1=3", "TX3=1,TX2=2,RX1=3,RX2=4", "TX1=a,TX2=2,RX1=3,RX2=4"):
        with pytest.raises(ValueError):
            parse_antennas(bad)


def test_cli_antenna_layout_saved_with_array_calibration(tmp_path, capsys):
    import main
    calib = tmp_path / "c.json"
    layout = "TX1=-4.5,RX1=-1.5,RX2=1.5,TX2=4.5"
    main.main(["calibrate", "--sim", "--fast", "--array", "2", "--calib", str(calib), "--antennas", layout])
    th = Thresholds.load(calib)
    assert th.antennas == [[-0.045, 0.0], [0.045, 0.0], [-0.015, 0.0], [0.015, 0.0]]
    capsys.readouterr()
    main.main(["run", "--map", "--sim", "--fast", "--headless", "--print-every", "60", "--calib", str(calib)])
    out = capsys.readouterr().out
    assert "TX1 -4.5 cm, RX1 -1.5 cm, RX2 +1.5 cm, TX2 +4.5 cm" in out and "array calibration: from" in out


# --- the actual board layout (antennas on the SMA ports, 4.5 cm aperture) ------------------
def test_board_layout_single_walker_and_straight_ahead():
    for p in (WALKER, person([(0, 0, 1.5), (8, 0, 5), (16, 0, 1.5), (24, 0, 5)])):
        err, count = errors(run_map([p], 24, cfg=BOARD, seed=10), [p])
        assert np.median(err) < 0.4 and np.mean(count == 1) > 0.9


def test_board_layout_empty_room_and_two_sitters():
    assert all(len(s.tracks) == 0 for s in run_map([], 40, cfg=BOARD, seed=11))
    a = person([(0, -1.8, 2.5), (60, -1.8, 2.5)], bpm=12)
    b = person([(0, 1.8, 3.0), (60, 1.8, 3.0)], bpm=17)
    tracks = run_map([a, b], 50, cfg=BOARD, seed=12)[-1].tracks
    assert len(tracks) == 2
    assert sorted(round(tr.bpm) for tr in tracks) == [12, 17]


def test_board_layout_walker_and_sitter():
    # small aperture: the sitter is masked part of the time while the walker passes nearby
    sitter = person([(0, 1.5, 3.0), (60, 1.5, 3.0)], bpm=14)
    walk = person([(0, -2, 2), (8, -2, 5), (16, -2, 2), (24, -2, 5), (32, -2, 2), (40, -2, 5), (48, -2, 2)])
    st = run_map([walk, sitter], 48, cfg=BOARD, seed=11)
    err, count = errors(st, [walk, sitter], t_min=15)
    assert np.median(err) < 0.5 and np.mean(count == 2) > 0.5 and np.mean(count > 2) < 0.1
