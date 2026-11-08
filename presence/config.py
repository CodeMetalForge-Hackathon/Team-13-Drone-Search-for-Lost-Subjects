"""Configuration and detection thresholds."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Optional

SPEED_OF_LIGHT = 299_792_458.0


# Alternative layout with ~3x the aperture (SMA extension cables, all in one line, d = lambda/2):
#     TX1 ----2d---- TX2 --d-- RX1 --d-- RX2      TX spacing = 2 x RX spacing -> filled virtual array
#     use: --antennas "TX1=-21.42,TX2=-9.18,RX1=-3.06,RX2=3.06"
WIDE_LAYOUT = {"rx_pos": ((-0.0306, 0.0), (0.0306, 0.0)), "tx_pos": ((-0.2142, 0.0), (-0.0918, 0.0))}


@dataclass
class Config:
    # --- radio (bladeRF 2.0, 2x2 MIMO) ------------------------------------
    device: str = ""                     # libbladeRF device string ("" = first device found)
    center_freq: float = 2.45e9          # TX and RX LO (Hz)
    sample_rate: int = 8_000_000         # baseband rate per channel (S/s)
    rf_bandwidth: int = 8_000_000
    tx_gain_db: float = 30.0             # bladeRF TX gain (dB, higher = more power)
    rx_gain_db: float = 40.0             # manual RX gain (dB)
    rx_buffer_size: int = 400_000        # samples per channel per RX read (multiple of `dec`)
    num_buffers: int = 32                # libbladeRF sync-stream settings
    buffer_size: int = 32768             # samples per USB buffer (multiple of 1024)
    num_transfers: int = 16
    stream_timeout_ms: int = 3500
    # multi-tone CW: every TX sends K tones (baseband offsets, Hz). TX2 = TX1 + 50 kHz so the two
    # transmitters are separable at every RX (MIMO virtual array). The set is asymmetric so no tone
    # lands on another tone's IQ image, and it avoids DC (LO leakage). All tones lie on `tone_grid`.
    tones_tx1: tuple = (-2.95e6, -0.95e6, 1.05e6, 3.05e6)
    tones_tx2: tuple = (-2.90e6, -0.90e6, 1.10e6, 3.10e6)
    tone_grid: float = 50e3
    # antenna positions (x, y) in metres, array along x, looking towards +y (any layout works; the
    # steering model is exact). Default: antennas screwed straight onto the bladeRF 2.0 SMA ports,
    # one line at a uniform 1.5 cm pitch, seen from behind the board looking into the room:
    #     TX2 --1.5-- RX2 --1.5-- RX1 --1.5-- TX1          (x = -2.25, -0.75, +0.75, +2.25 cm)
    # A wider line (WIDE_LAYOUT, lambda/2 = 6.12 cm steps via SMA extension cables) maps better.
    rx_pos: tuple = ((0.0075, 0.0), (-0.0075, 0.0))      # RX1, RX2
    tx_pos: tuple = ((0.0225, 0.0), (-0.0225, 0.0))      # TX1, TX2

    # --- slow-time processing ---------------------------------------------
    slow_fs: int = 200                   # Doppler/slow-time sample rate after tone demod (Hz)
    breath_fs: int = 20                  # breathing path rate (Hz)
    history_s: float = 60.0
    clutter_tau_s: float = 2.0           # static-clutter running-mean time constant
    jump_fix_rad: float = 1.0            # correct phase jumps (dropped samples) larger than this; 0 disables

    motion_win_s: float = 1.0
    motion_band: tuple = (2.0, 60.0)        # |Doppler| band holding human motion (Hz)
    motion_noise_band: tuple = (70.0, 95.0)  # reference band (no human motion at 2.4 GHz)

    breath_band: tuple = (0.13, 0.6)        # 8..36 breaths / min (lower edge kept above drift)
    breath_noise_band: tuple = (0.8, 3.0)
    breath_win_s: float = 30.0
    breath_min_win_s: float = 15.0

    # --- automatic background floor (replaces empty-room calibration) -----
    auto_floor: bool = True
    floor_window_s: float = 600.0        # history used to learn the background
    floor_percentile: float = 20.0       # people move intermittently -> low percentile = background
    floor_margin_db: float = 4.0         # motion threshold = max(base, floor + margin)
    floor_max_rise_db: float = 20.0      # never raise the threshold more than this above base
    floor_warmup_s: float = 15.0         # use the base threshold until this much history exists
    floor_rise_db_per_min: float = 3.0   # floor may rise only this fast (falls instantly), so a person
                                         # moving for minutes is not learned as background

    # --- proximity mode (active when calib has still_abs_db) ----------------
    still_win_s: float = 8.0
    still_band: tuple = (0.15, 2.0)
    near_confirm_s: float = 1.5          # near still-energy this long -> present
    proximity_absent_s: float = 3.0      # no near energy / motion this long -> EMPTY

    # --- heart rate (run --heart) ------------------------------------------
    heart: bool = False
    heart_win_s: float = 16.0
    heart_every_s: float = 1.0
    heart_avg_alpha: float = 0.2         # spectral averaging across successive estimates
    heart_snr_db: float = 9.0            # per-estimate confidence for the lock tracker
    heart_lock_s: float = 6.0            # consistent estimates this long -> report bpm
    heart_span_bpm: float = 8.0
    heart_last_avg_s: float = 20.0       # "last reading" = mean of locked bpm over this moving window
    heart_last_hold_s: float = 300.0     # keep showing the last reading this long, then clear it
    breath_lock_s: float = 3.0           # breathing reading: confident + consistent this long -> shown
    breath_lock_span_bpm: float = 2.5
    breath_motion_margin_db: float = 6.0  # heart mode: breathing only restarts on motion this much
                                          # stronger than the heart gate (small fidgets don't reset it)

    # --- 2D map (run --map) ------------------------------------------------
    map_x: tuple = (-4.0, 4.0)           # map extent (m), array at the origin
    map_y: tuple = (0.3, 7.0)
    map_res: float = 0.1
    map_fast_win_s: float = 1.0          # moving people: Doppler bins of this window
    map_slow_win_s: float = 10.0         # still people: breathing / sway bins of this window
    map_slow_every_s: float = 1.0
    map_slow_band: tuple = (0.15, 2.0)
    map_slow_noise_band: tuple = (3.0, 9.0)
    map_det_db: float = 6.0              # a Doppler bin must be this far above the noise floor
    map_max_bins: int = 32               # strongest bins used per map update
    map_peak_rel: float = 0.3            # secondary peaks must reach this fraction of the strongest
    map_min_sep_m: float = 0.6
    map_min_range_m: float = 0.7         # no people reported closer than this to the antennas
    map_same_bearing_deg: float = 10.0   # weaker peak this close in bearing ...
    map_same_bearing_m: float = 2.0      # ... and range = range sidelobe of the stronger one
    map_subspace_min: float = 0.8
    map_null_energy: float = 0.999       # still search: null the movers' paths to this energy share,
    map_null_keep: float = 0.2           # ... shrinking the null until this share of the map stays visible
    map_null_min_rank: int = 2           # ... but never below this many directions        # still people: steering vector share inside the signal subspace
    max_targets: int = 4
    leak_tau_s: float = 30.0             # leakage-phase reference (per channel) time constant; long, so
                                         # people's echoes barely move it
    track_gate_m: float = 1.2
    track_gate_still_m: float = 0.8      # still people do not jump: tighter gate, no hijacking
    track_confirm: int = 4               # detections before a track is shown
    track_confirm_still: int = 6         # ... for a track that starts out still (~6 s)
    track_drop_s: float = 6.0            # moving track without detections this long -> gone
    track_drop_still_s: float = 10.0     # still people are seen only every second, and can be
                                         # masked for a while by a strong walker
    still_after_s: float = 3.0           # no motion detection this long -> track counts as still
    track_show_s: float = 1.5            # show a moving track only if detected within this time
    track_show_still_s: float = 4.0
    track_breath_every_s: float = 1.0

    # --- detector timing ---------------------------------------------------
    update_s: float = 0.2
    motion_confirm_s: float = 0.4        # motion above threshold this long -> MOVING
    motion_hold_s: float = 2.0           # stay MOVING this long after motion stops
    breath_confirm_s: float = 5.0        # stable breathing lock this long (from EMPTY) -> STILL
    breath_rate_span_bpm: float = 2.0    # max rate wander during that confirmation
    absent_timeout_s: float = 30.0       # no motion / no breathing this long -> EMPTY

    @property
    def dec(self) -> int:
        return self.sample_rate // self.slow_fs

    @property
    def wavelength(self) -> float:
        return SPEED_OF_LIGHT / self.center_freq

    @property
    def tones(self) -> tuple:
        """All tone offsets in demodulator order: TX1 tones then TX2 tones."""
        return tuple(self.tones_tx1) + tuple(self.tones_tx2)

    @property
    def n_tones(self) -> int:
        return len(self.tones_tx1)

    @property
    def n_channels(self) -> int:
        """Virtual channels = RX x TX x tones."""
        return len(self.rx_pos) * len(self.tones)

    @property
    def tone_period(self) -> int:
        """Samples after which every tone completes an integer number of cycles."""
        return int(round(self.sample_rate / self.tone_grid))

    def channels(self) -> list[tuple[int, int, int, float]]:
        """Virtual channel table, in data order: (rx, tx, tone index, RF frequency)."""
        out = []
        for r in range(len(self.rx_pos)):
            for tx, tones in enumerate((self.tones_tx1, self.tones_tx2)):
                for k, f in enumerate(tones):
                    out.append((r, tx, k, self.center_freq + f))
        return out

    def unambiguous_range(self) -> float:
        """Bistatic path / 2 at which the per-tone phases of one TX repeat."""
        g = 0
        for tones in (self.tones_tx1, self.tones_tx2):
            for f in tones:
                g = math.gcd(g, int(round(f - tones[0])))
        return SPEED_OF_LIGHT / (2 * g) if g else math.inf

    def validate(self) -> None:
        if self.sample_rate % self.slow_fs:
            raise ValueError("sample_rate must be a multiple of slow_fs")
        if self.slow_fs % self.breath_fs:
            raise ValueError("slow_fs must be a multiple of breath_fs")
        if self.rx_buffer_size % self.dec:
            raise ValueError(f"rx_buffer_size must be a multiple of {self.dec}")
        if self.motion_noise_band[1] >= self.slow_fs / 2:
            raise ValueError("motion_noise_band must be below slow_fs/2")
        if len(self.tones_tx1) != len(self.tones_tx2):
            raise ValueError("tones_tx1 and tones_tx2 need the same number of tones")
        if len(self.rx_pos) != 2 or len(self.tx_pos) != 2:
            raise ValueError("rx_pos and tx_pos need two (x, y) positions each")
        if self.sample_rate % self.tone_grid or self.dec % self.tone_period:
            raise ValueError("tone_grid must divide sample_rate, and its period must divide dec")
        guard = 2 * self.slow_fs
        tones = self.tones
        for i, f in enumerate(tones):
            if abs(f / self.tone_grid - round(f / self.tone_grid)) > 1e-9:
                raise ValueError(f"tone {f:g} Hz is not on the {self.tone_grid:g} Hz grid")
            if not abs(f) < self.rf_bandwidth / 2 * 0.95:
                raise ValueError(f"tone {f:g} Hz lies outside the RF bandwidth")
            if abs(f) < guard:
                raise ValueError(f"tone {f:g} Hz sits on DC (LO leakage)")
            for j, g in enumerate(tones):
                if i != j and abs(f - g) < guard:
                    raise ValueError(f"tones {f:g} and {g:g} Hz collide")
                if abs(f + g) < guard:
                    raise ValueError(f"tone {f:g} Hz sits on the IQ image of {g:g} Hz")
        if self.unambiguous_range() < math.hypot(max(map(abs, self.map_x)), self.map_y[1]):
            raise ValueError("tone spacing too coarse: range ambiguity inside the map")

    def tx_buffer_len(self, min_len: int = 32768) -> int:
        """Shortest length >= min_len holding whole tone periods and a multiple of 1024 samples."""
        period = self.tone_period * 1024 // math.gcd(self.tone_period, 1024)
        return period * math.ceil(min_len / period)


def parse_antennas(text: str) -> tuple[tuple, tuple]:
    """"TX1=-3,TX2=-1,RX1=1,RX2=3" (cm along the array axis; "x:y" for an offset towards the
    scene) -> (tx_pos, rx_pos) in metres."""
    pos = {}
    for part in text.replace(" ", "").split(","):
        name, _, val = part.partition("=")
        name = name.upper()
        if name not in ("TX1", "TX2", "RX1", "RX2") or not val:
            raise ValueError(f"bad antenna entry {part!r}; expected e.g. TX1=-3,TX2=-1,RX1=1,RX2=3 (cm)")
        x, _, y = val.partition(":")
        pos[name] = (float(x) / 100, float(y or 0) / 100)
    if len(pos) != 4:
        raise ValueError("give all four antennas: TX1, TX2, RX1, RX2")
    return (pos["TX1"], pos["TX2"]), (pos["RX1"], pos["RX2"])


@dataclass
class Thresholds:
    motion_db: float = 6.0       # motion-band / noise-band power ratio
    breath_snr_db: float = 10.0  # breathing peak vs 0.8-3 Hz floor
    breath_acf: float = 0.5      # autocorrelation at one breathing period
    # soft range boundary: motion weaker than this (dBFS) is treated as "outside" (None = off).
    # Only valid for the RX gain / TX attenuation it was measured with.
    motion_abs_db: Optional[float] = None
    still_abs_db: Optional[float] = None   # proximity mode: near micro-motion energy threshold (dBFS)
    boundary_m: Optional[float] = None
    rx_gain_db: Optional[float] = None
    tx_gain_db: Optional[float] = None
    array_cal: Optional[list] = None       # per virtual channel complex correction, [[re, im], ...]
    antennas: Optional[list] = None        # [[x, y] TX1, TX2, RX1, RX2] in metres (layout used for array_cal)

    @classmethod
    def load(cls, path: str | Path | None) -> "Thresholds":
        if path is None or not Path(path).exists():
            return cls()
        data = json.loads(Path(path).read_text())
        names = {f.name for f in fields(cls)}
        out = {}
        for k, v in data.items():
            if k not in names:
                continue
            if k in ("array_cal", "antennas"):
                out[k] = None if v is None else [[float(a), float(b)] for a, b in v]
            else:
                out[k] = None if v is None else float(v)
        return cls(**out)

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")

    def gain_mismatch(self, cfg: "Config") -> bool:
        return self.motion_abs_db is not None and (
            (self.rx_gain_db is not None and self.rx_gain_db != cfg.rx_gain_db)
            or (self.tx_gain_db is not None and self.tx_gain_db != cfg.tx_gain_db))
