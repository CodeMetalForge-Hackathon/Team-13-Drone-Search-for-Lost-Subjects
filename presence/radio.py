"""Slow-time sample sources: live bladeRF multi-tone MIMO radar, recorded file replay, simulation.

Every source yields complex slow-time samples at `cfg.slow_fs` via `read()`: shape [channels, n]
(virtual channels RX x TX x tone, see Config.channels()), or 1D for legacy single-channel data.
"""
from __future__ import annotations

import queue
import threading
import time

import numpy as np

from .config import Config
from .dsp import MultiToneDemodulator

ADC_FULL_SCALE = 2048.0  # bladeRF SC16 Q11: +-2048 = full scale (same scale as the old Pluto data)
TX_FULL_SCALE = 2047.0


class Source:
    fs: float
    finished = False

    def start(self) -> None: ...
    def stop(self) -> None: ...

    def read(self, timeout: float = 0.5) -> np.ndarray:
        """All samples available now (blocks up to `timeout` for at least one chunk)."""
        raise NotImplementedError


def multitone_tx_buffer(cfg: Config, n: int | None = None, backoff: float = 0.8) -> np.ndarray:
    """Interleaved SC16 Q11 samples for TX1/TX2 (I0 Q0 I1 Q1 ...), a whole number of tone periods.

    Each TX's peak is limited to `backoff` x full scale by splitting it evenly over its tones.
    """
    n = n or cfg.tx_buffer_len()
    t = np.arange(n) / cfg.sample_rate
    out = np.empty((n, 2, 2), np.int16)
    for ch, tones in enumerate((cfg.tones_tx1, cfg.tones_tx2)):
        amp = backoff * TX_FULL_SCALE / len(tones)
        # spread the tone start phases (Newman phases) so the composite peak is lower
        k = np.arange(len(tones))
        ph = np.pi * k * k / len(tones)
        z = amp * np.exp(1j * (2 * np.pi * np.outer(t, tones) + ph)).sum(axis=1)
        out[:, ch, 0] = np.round(z.real)
        out[:, ch, 1] = np.round(z.imag)
    return out.reshape(-1)


class BladeRFSource(Source):
    """bladeRF 2.0 as a 2x2 MIMO multi-tone CW radar.

    A TX thread streams a precomputed multi-tone buffer on both transmitters continuously (the
    bladeRF has no cyclic TX buffer); an RX thread reads both receivers and demodulates every tone
    to slow time, giving 2 RX x 2 TX x K tones virtual channels.
    """

    def __init__(self, cfg: Config, verbose: bool = True):
        cfg.validate()
        self.cfg, self.fs, self.verbose = cfg, cfg.slow_fs, verbose
        self.q: queue.Queue = queue.Queue(maxsize=200)
        self.dropped_chunks = 0
        self.peak_dbfs = -np.inf
        self._stop = threading.Event()
        self._rx_thread = self._tx_thread = None
        self.dev = None
        self._bf = None

    # --- setup ---------------------------------------------------------------
    def start(self) -> None:
        try:
            from bladerf import _bladerf as bf   # apt: python3-bladerf (libbladeRF bindings)
        except ImportError as e:
            raise SystemExit("libbladeRF Python bindings not found: sudo apt install bladerf "
                             "python3-bladerf (see README.md)") from e
        self._bf = bf
        c = self.cfg
        try:
            dev = bf.BladeRF(c.device) if c.device else bf.BladeRF()
        except Exception as e:
            raise SystemExit(f"could not open the bladeRF ({e}). Is it plugged into a USB 3 port, "
                             "and does `bladeRF-cli -p` see it?") from e
        self.dev = dev
        rx = [dev.Channel(bf.CHANNEL_RX(i)) for i in range(2)]
        tx = [dev.Channel(bf.CHANNEL_TX(i)) for i in range(2)]
        for ch in rx + tx:
            ch.frequency = int(c.center_freq)
            ch.sample_rate = int(c.sample_rate)
            ch.bandwidth = int(c.rf_bandwidth)
        for ch in rx:
            ch.gain_mode = bf.GainMode.Manual      # AGC would amplitude-modulate the Doppler signal
            ch.gain = int(c.rx_gain_db)
        for ch in tx:
            ch.gain = int(c.tx_gain_db)
        self._rx_ch, self._tx_ch = rx, tx
        for layout in (bf.ChannelLayout.RX_X2, bf.ChannelLayout.TX_X2):
            dev.sync_config(layout=layout, fmt=bf.Format.SC16_Q11, num_buffers=c.num_buffers,
                            buffer_size=c.buffer_size, num_transfers=c.num_transfers,
                            stream_timeout=c.stream_timeout_ms)
        for ch in tx + rx:
            ch.enable = True

        self._tx_buf = multitone_tx_buffer(c)
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()

        # RX: 2 channels x SC16 (4 bytes / sample)
        self._rx_n = int(c.rx_buffer_size)
        self._rx_bytes = bytearray(self._rx_n * 2 * 4)
        for _ in range(3):                        # flush while the TX settles
            self._read_raw()
        self._check_levels(self._read_raw())

        self._demod = MultiToneDemodulator(c.tones, c.sample_rate, c.dec, c.tone_period, n_rx=2)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _tx_loop(self) -> None:
        n = len(self._tx_buf) // 2          # samples across both channels / 2 int16 per sample
        while not self._stop.is_set():
            try:
                self.dev.sync_tx(self._tx_buf, n)
            except Exception as e:
                if not self._stop.is_set():
                    print(f"[bladerf] TX error: {e}")
                    self.finished = True
                return

    def _read_raw(self) -> np.ndarray:
        """One RX block -> complex64 [2, n] in ADC counts."""
        self.dev.sync_rx(self._rx_bytes, self._rx_n * 2)
        iq = np.frombuffer(self._rx_bytes, np.int16).astype(np.float32)
        return iq.view(np.complex64).reshape(-1, 2).T

    def _check_levels(self, buf: np.ndarray) -> None:
        c = self.cfg
        n = min(buf.shape[1], 262144)
        f = np.fft.fftfreq(n, 1 / c.sample_rate)
        lines = []
        worst_tone = np.inf
        for r in range(2):
            x = buf[r, :n]
            peak = max(np.abs(x.real).max(), np.abs(x.imag).max())
            pdb = 20 * np.log10(peak / ADC_FULL_SCALE + 1e-12)
            self.peak_dbfs = max(self.peak_dbfs, pdb)
            spec = np.abs(np.fft.fft(x * np.hanning(n)))
            floor = np.median(spec)
            above = []
            for tone in c.tones:
                k = int(np.argmin(np.abs(f - tone)))
                above.append(20 * np.log10(spec[k - 2:k + 3].max() / (floor + 1e-12) + 1e-12))
            worst_tone = min(worst_tone, min(above))
            lines.append(f"RX{r + 1} peak {pdb:5.1f} dBFS, tones {min(above):.0f}..{max(above):.0f} dB above floor")
        if self.verbose:
            for ln in lines:
                print("[bladerf] " + ln)
        if self.peak_dbfs > -1.5:
            print("[bladerf] WARNING: RX is clipping - lower --rx-gain or --tx-gain, or move TX and RX "
                  "antennas a bit further apart")
        elif worst_tone < 20:
            print("[bladerf] WARNING: some tones barely visible - check all 4 antennas / raise --tx-gain "
                  "/ raise --rx-gain")

    def _rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                buf = self._read_raw()
            except Exception as e:
                if not self._stop.is_set():
                    print(f"[bladerf] RX error: {e}")
                    self.finished = True
                return
            slow = self._demod.process(buf)
            try:
                self.q.put_nowait(slow)
            except queue.Full:
                # dropping after demod keeps the demodulator block-aligned
                self.dropped_chunks += 1

    def read(self, timeout: float = 0.5) -> np.ndarray:
        chunks = []
        try:
            chunks.append(self.q.get(timeout=timeout))
            while True:
                chunks.append(self.q.get_nowait())
        except queue.Empty:
            pass
        if not chunks:
            return np.zeros((self.cfg.n_channels, 0), np.complex128)
        return np.concatenate(chunks, axis=1)

    def stop(self) -> None:
        self._stop.set()
        for th in (self._rx_thread, self._tx_thread):
            if th is not None:
                th.join(timeout=self.cfg.stream_timeout_ms / 1000 + 1)
                if th.is_alive():
                    print("[bladerf] WARNING: stream thread did not stop in time")
        if self.dev is not None:
            for ch in getattr(self, "_tx_ch", []):
                for fn in (lambda: setattr(ch, "gain", 0), lambda: setattr(ch, "enable", False)):
                    try:
                        fn()                       # stop radiating first
                    except Exception:
                        pass
            for ch in getattr(self, "_rx_ch", []):
                try:
                    ch.enable = False
                except Exception:
                    pass
            try:
                self.dev.close()
            except Exception:
                pass
            self.dev = None


class ArraySource(Source):
    """Plays back a precomputed slow-time array (1D or [channels, n]), paced in real time or as fast
    as possible."""

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
        total = self.data.shape[-1]
        while n > 0 and not self.finished:
            take = min(n, total - self.pos)
            out.append(self.data[..., self.pos:self.pos + take])
            self.pos += take
            self._done += take
            n -= take
            if self.pos >= total:
                if self.loop:
                    self.pos = 0
                else:
                    self.finished = True
        if out:
            return np.concatenate(out, axis=-1)
        return np.zeros(self.data.shape[:-1] + (0,), np.complex128)

    def _emitted(self) -> int:
        return self._done


def load_recording(path: str, cfg: Config | None = None) -> tuple[np.ndarray, float]:
    """iq is 1D (legacy single channel) or [channels, n]. With `cfg`, the radio layout stored in a
    multichannel recording (tones, antenna positions, LO) is copied into it."""
    d = np.load(path)
    iq = d["iq"].astype(np.complex128)
    if cfg is not None and iq.ndim == 2:
        for k in ("tones_tx1", "tones_tx2", "rx_pos", "tx_pos"):
            if k in d:
                v = d[k]
                setattr(cfg, k, tuple(map(tuple, v.tolist())) if v.ndim == 2 else tuple(v.tolist()))
        if "center_freq" in d:
            cfg.center_freq = float(d["center_freq"])
        if iq.shape[0] != cfg.n_channels:
            raise ValueError(f"recording has {iq.shape[0]} channels, layout expects {cfg.n_channels}")
    return iq, float(d["fs"])


def save_recording(path: str, iq: np.ndarray, fs: float, cfg: Config | None = None) -> None:
    meta = {} if cfg is None else {
        "center_freq": cfg.center_freq, "rx_gain_db": cfg.rx_gain_db, "tx_gain_db": cfg.tx_gain_db,
        "tones_tx1": np.array(cfg.tones_tx1), "tones_tx2": np.array(cfg.tones_tx2),
        "rx_pos": np.array(cfg.rx_pos), "tx_pos": np.array(cfg.tx_pos)}
    np.savez_compressed(path, iq=iq.astype(np.complex64), fs=fs, **meta)
