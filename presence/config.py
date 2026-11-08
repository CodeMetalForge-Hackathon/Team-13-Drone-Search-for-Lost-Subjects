"""Configuration and detection thresholds."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Optional

SPEED_OF_LIGHT = 299_792_458.0


@dataclass
class Config:
    # --- radio -------------------------------------------------------------
    uri: str = "ip:192.168.2.1"
    center_freq: float = 2.45e9          # TX and RX LO (Hz)
    sample_rate: int = 1_000_000         # Pluto baseband rate (S/s)
    tone_freq: float = 50e3              # baseband tone offset (Hz), keeps us off DC / LO leakage
    rf_bandwidth: int = 1_000_000
    tx_atten_db: float = -10.0           # Pluto TX "gain": 0 = max power, -89 = min
    rx_gain_db: float = 50.0             # manual RX gain (0..73 dB)
    rx_buffer_size: int = 100_000        # samples per RX refill (must be a multiple of `dec`)
    kernel_buffers: int = 8

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
    floor_margin_db: float = 3.0         # motion threshold = max(base, floor + margin)
    floor_max_rise_db: float = 20.0      # never raise the threshold more than this above base
    floor_warmup_s: float = 15.0         # use the base threshold until this much history exists
    floor_rise_db_per_min: float = 3.0   # floor may rise only this fast (falls instantly), so a person
                                         # moving for minutes is not learned as background

    # --- proximity mode (active when calib has still_abs_db) ----------------
    still_win_s: float = 8.0
    still_band: tuple = (0.15, 2.0)
    near_confirm_s: float = 1.5          # near still-energy this long -> present
    still_hyst_db: float = 2.0           # hysteresis: once present, stay present down to this many dB
                                         # below the still threshold (e.g. leaning still against a wall)
    proximity_absent_s: float = 3.0      # no near energy / motion this long -> EMPTY

    # --- heart rate (run --heart) ------------------------------------------
    heart: bool = False
    heart_win_s: float = 16.0            # heart window grows from heart_min_win_s up to this
    heart_min_win_s: float = 8.0         # hold still this long before the first heart estimate
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
    # disturbance tolerance: slight / brief motion is absorbed instead of restarting a window.
    # A window restarts when motion is stronger than *_tol_db above its gate, or lasts longer than *_tol_s.
    heart_motion_tol_db: float = 3.0     # heart: motion up to thr + 3 dB ...
    heart_motion_tol_s: float = 0.6      # ... and shorter than 0.6 s is tolerated
    breath_motion_tol_db: float = 9.0    # breathing (heart mode): up to thr + margin + 9 dB ...
    breath_motion_tol_s: float = 1.0     # ... and shorter than 1 s is tolerated

    # --- detector timing ---------------------------------------------------
    update_s: float = 0.2
    motion_confirm_s: float = 0.2        # motion above threshold this long -> MOVING
    motion_hold_s: float = 1.5           # stay MOVING this long after motion stops
    breath_confirm_s: float = 5.0        # stable breathing lock this long (from EMPTY) -> STILL
    breath_rate_span_bpm: float = 2.0    # max rate wander during that confirmation
    absent_timeout_s: float = 20.0       # no motion / no breathing lock this long -> EMPTY
                                         # (>= breathing window + lock time, so a person who sits
                                         # down is confirmed before it expires)

    @property
    def dec(self) -> int:
        return self.sample_rate // self.slow_fs

    @property
    def wavelength(self) -> float:
        return SPEED_OF_LIGHT / self.center_freq

    def validate(self) -> None:
        if self.sample_rate % self.slow_fs:
            raise ValueError("sample_rate must be a multiple of slow_fs")
        if self.slow_fs % self.breath_fs:
            raise ValueError("slow_fs must be a multiple of breath_fs")
        if self.rx_buffer_size % self.dec:
            raise ValueError(f"rx_buffer_size must be a multiple of {self.dec}")
        if not 0 < self.tone_freq < self.rf_bandwidth / 2:
            raise ValueError("tone_freq must lie inside the RF bandwidth")
        if self.motion_noise_band[1] >= self.slow_fs / 2:
            raise ValueError("motion_noise_band must be below slow_fs/2")

    def tx_buffer_len(self, min_len: int = 4096) -> int:
        """Shortest length >= min_len holding an integer number of tone cycles."""
        period = self.sample_rate // math.gcd(self.sample_rate, int(self.tone_freq))
        return period * math.ceil(min_len / period)


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
    tx_atten_db: Optional[float] = None

    @classmethod
    def load(cls, path: str | Path | None) -> "Thresholds":
        if path is None or not Path(path).exists():
            return cls()
        data = json.loads(Path(path).read_text())
        names = {f.name for f in fields(cls)}
        return cls(**{k: (None if v is None else float(v)) for k, v in data.items() if k in names})

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")

    def gain_mismatch(self, cfg: "Config") -> bool:
        return self.motion_abs_db is not None and (
            (self.rx_gain_db is not None and self.rx_gain_db != cfg.rx_gain_db)
            or (self.tx_atten_db is not None and self.tx_atten_db != cfg.tx_atten_db))
