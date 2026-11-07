"""Slow-time sample sources: live Pluto CW radar, recorded file replay, and simulation.

Every source yields complex slow-time samples at `cfg.slow_fs` via `read()`.
"""
from __future__ import annotations

import queue
import threading
import time

import numpy as np

from .config import Config
from .dsp import ToneDemodulator

ADC_FULL_SCALE = 2048.0  # Pluto 12-bit ADC as returned by pyadi-iio


class Source:
    fs: float
    finished = False

    def start(self) -> None: ...
    def stop(self) -> None: ...

    def read(self, timeout: float = 0.5) -> np.ndarray:
        """All samples available now (blocks up to `timeout` for at least one chunk)."""
        raise NotImplementedError


class PlutoSource(Source):
    """ADALM-Pluto as a CW Doppler radar: cyclic TX tone, RX demodulated to slow time."""

    def __init__(self, cfg: Config, verbose: bool = True):
        cfg.validate()
        self.cfg, self.fs, self.verbose = cfg, cfg.slow_fs, verbose
        self.q: queue.Queue = queue.Queue(maxsize=200)
        self.dropped_chunks = 0
        self.peak_dbfs = -np.inf
        self._stop = threading.Event()
        self._thread = None
        self.sdr = None

    def start(self) -> None:
        try:
            import adi  # pyadi-iio
        except ImportError as e:
            raise SystemExit("pyadi-iio / libiio not installed - see README.md") from e
        c = self.cfg
        sdr = adi.Pluto(c.uri)
        sdr.sample_rate = int(c.sample_rate)
        sdr.rx_lo = int(c.center_freq)
        sdr.tx_lo = int(c.center_freq)
        sdr.rx_rf_bandwidth = int(c.rf_bandwidth)
        sdr.tx_rf_bandwidth = int(c.rf_bandwidth)
        sdr.gain_control_mode_chan0 = "manual"      # AGC would amplitude-modulate the Doppler signal
        sdr.rx_hardwaregain_chan0 = float(c.rx_gain_db)
        sdr.tx_hardwaregain_chan0 = float(c.tx_atten_db)
        sdr.rx_buffer_size = int(c.rx_buffer_size)
        try:
            sdr._rxadc.set_kernel_buffers_count(int(c.kernel_buffers))
        except Exception as e:  # older pyadi versions
            if self.verbose:
                print(f"[pluto] could not set kernel buffer count: {e}")

        n = c.tx_buffer_len()
        tone = 0.5 * 2**14 * np.exp(2j * np.pi * c.tone_freq * np.arange(n) / c.sample_rate)
        sdr.tx_cyclic_buffer = True
        sdr.tx(tone)
        self.sdr = sdr

        # flush a few buffers while the TX settles
        for _ in range(3):
            sdr.rx()
        self._check_levels(sdr.rx())

        self._demod = ToneDemodulator(c.tone_freq, c.sample_rate, c.dec)
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def _check_levels(self, buf: np.ndarray) -> None:
        peak = max(np.abs(buf.real).max(), np.abs(buf.imag).max())
        self.peak_dbfs = 20 * np.log10(peak / ADC_FULL_SCALE + 1e-12)
        spec = np.abs(np.fft.fft(buf[:65536] * np.hanning(min(len(buf), 65536))))
        f = np.fft.fftfreq(len(spec), 1 / self.cfg.sample_rate)
        k = np.argmin(np.abs(f - self.cfg.tone_freq))
        tone_db = 20 * np.log10(spec[k - 2:k + 3].max() + 1e-12)
        floor_db = 20 * np.log10(np.median(spec) + 1e-12)
        if self.verbose:
            print(f"[pluto] RX peak {self.peak_dbfs:.1f} dBFS, tone {tone_db - floor_db:.1f} dB above floor")
        if self.peak_dbfs > -1.5:
            print("[pluto] WARNING: RX is clipping - lower --rx-gain or lower TX power (--tx-atten more negative), "
                  "or move the antennas apart")
        elif tone_db - floor_db < 20:
            print("[pluto] WARNING: tone barely visible - check TX antenna / raise --tx-atten toward 0 / raise --rx-gain")

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                buf = self.sdr.rx()
            except Exception as e:
                print(f"[pluto] RX error: {e}")
                self.finished = True
                return
            slow = self._demod.process(buf)
            try:
                self.q.put_nowait(slow)
            except queue.Full:
                # dropping after demod keeps the mixing reference phase-continuous
                self.dropped_chunks += 1

    def read(self, timeout: float = 0.5) -> np.ndarray:
        chunks = []
        try:
            chunks.append(self.q.get(timeout=timeout))
            while True:
                chunks.append(self.q.get_nowait())
        except queue.Empty:
            pass
        return np.concatenate(chunks) if chunks else np.zeros(0, np.complex128)

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self._thread is not None and self._thread.is_alive():
            print("[pluto] WARNING: RX thread did not stop in time")
        if self.sdr is not None:
            for fn in (lambda: self.sdr.tx_destroy_buffer(),
                       lambda: setattr(self.sdr, "tx_hardwaregain_chan0", -89.0),   # stop radiating
                       lambda: self.sdr.rx_destroy_buffer()):
                try:
                    fn()
                except Exception:
                    pass
            # release libiio objects now, on the main thread, not during interpreter teardown
            self.sdr = None


class ArraySource(Source):
    """Plays back a precomputed slow-time array, paced in real time or as fast as possible."""

    def __init__(self, data: np.ndarray, fs: float, realtime: bool = True, loop: bool = False,
                 chunk_s: float = 0.1):
        self.data, self.fs, self.realtime, self.loop = data, fs, realtime, loop
        self.chunk = max(1, int(chunk_s * fs))
        self.pos = 0
        self._done = 0  # samples emitted so far
        self._t0 = None

    def start(self) -> None:
        self._t0 = time.monotonic()

    def read(self, timeout: float = 0.5) -> np.ndarray:
        if self.realtime:
            target = int((time.monotonic() - self._t0) * self.fs)
            if target <= self._emitted():
                time.sleep(min(timeout, self.chunk / self.fs))
                target = int((time.monotonic() - self._t0) * self.fs)
            n = max(0, target - self._emitted())
        else:
            n = self.chunk * 10
        out = []
        while n > 0 and not self.finished:
            take = min(n, len(self.data) - self.pos)
            out.append(self.data[self.pos:self.pos + take])
            self.pos += take
            self._done += take
            n -= take
            if self.pos >= len(self.data):
                if self.loop:
                    self.pos = 0
                else:
                    self.finished = True
        return np.concatenate(out) if out else np.zeros(0, np.complex128)

    def _emitted(self) -> int:
        return self._done


def load_recording(path: str) -> tuple[np.ndarray, float]:
    d = np.load(path)
    return d["iq"].astype(np.complex128), float(d["fs"])


def save_recording(path: str, iq: np.ndarray, fs: float, cfg: Config | None = None) -> None:
    meta = {} if cfg is None else {"center_freq": cfg.center_freq, "rx_gain_db": cfg.rx_gain_db,
                                    "tx_atten_db": cfg.tx_atten_db}
    np.savez_compressed(path, iq=iq.astype(np.complex64), fs=fs, **meta)
