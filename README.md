# bladeRF 2.4 GHz people radar: presence, breathing, heart rate, 2D map

This project turns a **bladeRF 2.0 micro (2 RX + 2 TX)** into a **multi-tone CW MIMO radar at 2.45 GHz**. It:

- **maps several people in 2D** (direction and distance) and tracks them with stable IDs, marking each as *moving* or *still*. Each still person gets their own breathing rate (`run --map`).
- detects presence (**MOVING / STILL / EMPTY**), breathing and heart rate on one reference channel, as the Pluto version did (`run`, `run --heart`).

The single-channel Pluto version is on the `radar` branch.

```
TX1: 4 tones (-2.95, -0.95, +1.05, +3.05 MHz)   TX2: same +50 kHz      @ LO 2.45 GHz, 8 MS/s
RX1, RX2 ─► fold every 40 000-sample block modulo the 50 kHz tone period ─► 8 tones per RX
        ─► 16 virtual channels (2 RX x 2 TX x 4 tones) at 200 Hz slow time
   ├─ reference channel (RX1·TX1·tone 0) ─► presence / breathing / heart pipeline (unchanged)
   └─ all 16 channels:
        referenced to each channel's own TX->RX leakage phase (cancels cable / filter / stream
        phases and LO drift), whitened against common-mode leakage phase noise
        ├─ moving people: Doppler bins 2-60 Hz over 1 s; each single-scatterer bin is localised
        │                 on an (x, y) grid with the exact near-field bistatic model
        └─ still people:  0.15-2 Hz micro-motion over 10 s, MUSIC (movers' paths nulled first)
        ─► peaks ─► Kalman / Hungarian tracker ─► per-person breathing (beamformed)
```

**How it gets angle and range.** A scatterer at p adds phase −2π f (|p−TX| + |p−RX|)/c to every virtual channel.
- *Angle* comes from the antenna positions: 2 RX × 2 TX act as a 4-element virtual array.
- *Range* comes from the phase slope across the tones, which span 6 MHz.
- *People are separated mostly by Doppler.* Each spectral bin is assumed to hold one person: walkers have different velocities, and still people have different breathing and sway.

## Setup (Raspberry Pi OS)

```bash
sudo apt install -y bladerf python3-bladerf libbladerf-dev     # libbladeRF + Python bindings
sudo apt install -y bladerf-fpga-hostedxa4 || bladeRF-cli -L /path/to/hostedxA4.rbf   # FPGA image (xA9: hostedxa9)
python3 -m venv --system-site-packages .venv                    # sees python3-bladerf
.venv/bin/pip install -r requirements.txt
bladeRF-cli -p                                                  # the board must be listed
bladeRF-cli -e info                                             # check: USB speed "SuperSpeed"
```

Use a **USB 3 port** (the blue one on the Pi 5). Two channels at 8 MS/s each are 64 MB/s, which USB 2 can't carry. Demodulation takes about 0.07 s of CPU per second of data on a Pi 5.

## Usage

```bash
.venv/bin/python main.py run --map --sim           # demo, no hardware: walker + sitter + visitor
.venv/bin/python main.py run --map                 # live 2D people map
.venv/bin/python main.py run --map --headless      # track lines over SSH
.venv/bin/python main.py run                       # single-channel presence / breathing view
.venv/bin/python main.py calibrate --array 2       # optional: refine the array (see below)
.venv/bin/python main.py run --map --record a.npz  # also save all 16 channels
.venv/bin/python main.py replay a.npz --map --fast # re-run offline
.venv/bin/python -m pytest -q tests                # synthetic tests (no hardware)
```

At startup the program prints each RX peak (dBFS) and how far the tones sit above the floor. Raise `--tx-gain` / `--rx-gain` until the peaks are around −10 dBFS. If it warns about clipping, back off.

## Antennas

The default layout (`config.py`) puts all four antennas in one line, d = λ/2 = 6.12 cm:

```
TX1 ----2d---- TX2 --d-- RX1 --d-- RX2          (x = -3.5d, -1.5d, -0.5d, +0.5d)
```

With TX spacing twice the RX spacing, the virtual array is filled: 4 elements, λ/2, no grating lobes.

**Antennas screwed directly onto the bladeRF's SMA ports** are allowed. Measure the centre-to-centre positions and pass them once:

```bash
.venv/bin/python main.py calibrate --array 2 --antennas "TX1=-3,RX1=-1,RX2=1,TX2=3"   # cm along the board edge
```

`calibrate --array` stores the layout in `calib.json`, and every later `run --map` uses it. With ports only 1–2 cm apart:
- **Angle resolution is about 3× worse** than the λ/2 layout, because the aperture is smaller.
- **TX→RX leakage is much stronger.** Watch the clipping warning and lower `--tx-gain`.
- **Mutual coupling distorts the array**, so `calibrate --array` matters.

Short SMA extension cables, or the λ/2 layout above, give clearly better maps. In simulation, which leaves out coupling, the port layout still localised one walker to about 0.2 m. But with a walker and a sitter, accuracy fell from 0.2 m to 0.3–0.9 m depending on port order.

Other placement notes:
- **Mount the array on a wall or the edge of a table, facing the room.** A line of omni antennas can't tell front from back, so the map only shows y > 0. People behind the array appear mirrored in front.
- Keep everything rigid. A swaying cable looks like breathing.

**Array calibration (optional, recommended for SMA-mounted antennas).** Stand still straight in front of the antennas at 2 m and breathe normally for 30 s. This measures the residual per-channel phase errors (antenna coupling, mismatched geometry) and saves them to `calib.json`. Without it, the exact geometry and the leakage reference already give a usable map.

## What to expect (and limits)

- **Range precision vs. resolution.** One isolated person is placed to about 0.2–0.4 m in simulation. But a 6 MHz span only *resolves* about 25 m, so two people are separated by Doppler, breathing rate, or angle (about 30° with the default array), not by range.
- **Two people walking** get the right count most of the time (85–100% of updates in simulation). When they share the same velocities for a while, their bins mix and one may drop out briefly. Bins that fit no single position are discarded rather than turned into ghosts.
- **Still people** need about 6–10 s to appear and about 15 s for a breathing rate. A weak still person (far away, behind a strong walker) may come and go.
- **At most 4 people** are tracked (`max_targets`), and in practice about 3 work well.
- All numbers above come from the simulator (`presence/sim.py`). Real rooms add multipath and coupling, so record a session (`--record`) and replay it to tune.

## Heart rate (`run --heart`)

```bash
.venv/bin/python main.py run --heart                 # subject sits still 0.3-0.5 m in front
.venv/bin/python main.py run --heart --ref-bpm 72    # overlay a watch / oximeter reading
```

**No empty-room calibration is needed.** The heart rate is found from the signal's own spectral structure:
1. Amplitude demodulation. It is immune to LO phase drift, which otherwise creates false heart peaks.
2. Subtract the breathing waveform (least-squares fit of the breathing harmonics), then high-pass.
3. A harmonic-sum spectrum over 48–120 bpm, averaged over time.
4. Confidence = the peak versus the median of that same spectrum.
5. A lock tracker requires 6 s of confident, consistent estimates before showing a rate.

What it *does* need:
- **A still subject, close:** 0.3–0.5 m, facing the antennas, not talking. Heartbeat chest motion is only 0.1–0.5 mm, 20–40× smaller than breathing.
- **Nothing else moving in range during the 16 s window.** A CW radar can't tell a bystander's slow movements from a heartbeat, so the estimate pauses ("hold still") whenever any motion is detected.
- **Low TX→RX leakage.** On this hardware the noise floor is set by LO phase noise on the direct leakage: 10 dB more TX raised the floor 7.4 dB. Separating the antennas more, or putting a metal sheet between them, improves heart SNR directly.

(Measured on the Pluto:) 5.8 GHz isn't better: the floor rises 7.6 dB, cancelling the 7.5 dB phase-sensitivity gain. Simulation results (0.3 mm heartbeat): locked about 40% of the time, about 8% wrong readings, and 2–5% false locks with no heartbeat present. Real-world numbers need a recording with a reference pulse.

## Demo with people around: proximity mode (recommended)

```bash
.venv/bin/python main.py calibrate --proximity 1     # guided, ~95 s, follow the prompts
.venv/bin/python main.py run
```

This makes the radar answer "is someone within ~1 m of the antennas?" and ignore bystanders farther away. It uses two absolute levels, both of which fall about R⁴ with distance, so 1 m vs 3 m is about 19 dB:
- **near energy:** power of 0.15–2 Hz micro-motion (breathing, posture sway, fidgets) over the last 8 s, with the receiver noise subtracted. This detects a person who is standing or sitting still, without needing a clean breathing lock.
- **motion level:** walking or gesturing (2–60 Hz Doppler).

The guided calibration records three phases and puts each threshold between them:
1. **away:** you step at least 3 m away while bystanders carry on normally.
2. **still:** you stand or sit still at the zone edge (e.g. 1 m).
3. **move:** you gesture at the zone edge.

It prints the separation in dB and warns if the phases overlap. If they do, use a smaller zone or calm the background. The session IQ is saved to `calib_session.npz` for offline analysis.

How it behaves:
- **PRESENT / STILL:** near energy is above its threshold. STILL with a bpm value is shown when breathing also locks.
- **MOVING:** near motion is detected.
- **EMPTY:** about 7–10 s after you leave, set by the 8 s energy window plus a 3 s hold.
- A breathing lock on its own never keeps presence, because its 30 s window still contains the past.

## No empty room needed: auto floor + range boundary

**Auto floor (on by default).** The motion threshold learns the background while running. It is the 20th percentile of the motion metric over the last 10 min, plus 4 dB. People move intermittently, while fans, AC and curtains move all the time, so constant movers get absorbed and people still stand out. The floor may rise only 3 dB/min but falls instantly. An always-on fan therefore stops triggering after about 1–2 min, and someone moving continuously for minutes is not learned away. Use `--no-auto-floor` to disable it.

**Range boundary (`calibrate --boundary M`).** CW radar can't measure distance, but echo power falls about R⁴. Walk around and wave your arms at M metres for 30 s; `--delay 10` gives you time to get there. Afterwards, motion weaker than the median level you produced there is labelled *far, ignored*. It does not trigger MOVING and does not block breathing detection of someone nearby. Other people only need to be farther away than M (or still) during this calibration.
- The edge is soft, about ±1 m. Big or energetic motion just outside can count, and tiny fidgets just inside may not. Someone 1.4× farther is about 6 dB weaker, 2× farther about 12 dB.
- The level depends on `--rx-gain` / `--tx-gain`. They are stored with the boundary, and the threshold is shifted automatically if you change them.
- The plot's lower-left panel shows the motion level (dBFS) against the boundary line.

Useful options: `--tx-gain` sets the TX gain in dB (default 30), `--rx-gain` sets the RX gain in dB (default 40), `--device` selects a bladeRF (e.g. `"*:serial=ab12"`), `--freq` sets the LO in Hz, and `--breath-win` sets the breathing window in seconds.

## Antennas and tuning, especially through walls or beyond 4 m

1. **Use directional antennas.** 2.4 GHz patch or yagi antennas give 8–14 dBi each, and every dB counts twice (TX and RX). The stock rubber ducks will do motion at short range, but breathing through a wall needs gain.
2. **Separate and point the antennas.** Mount TX and RX side by side, 30–50 cm apart, both aimed at the target area. Direct TX→RX leakage is the main thing that saturates the receiver.
3. **Set the levels.** At startup the program prints `RX1/RX2 peak … dBFS`. Raise `--tx-gain` and `--rx-gain` until the peaks sit around -10 to -6 dBFS. If it warns about clipping, back off.
4. **Hold everything still.** Mount the antennas and the bladeRF rigidly; a swaying cable looks like breathing. Fans, AC vents, curtains and pets in the beam produce Doppler too. Run `calibrate` with them in their normal state.
5. Drywall or wood costs about 3–6 dB, brick or concrete 10–20 dB, and metal blocks the signal completely. **Motion through a wall is reliable. Breathing through a wall beyond 4 m is marginal.** It needs the person still for about 15–30 s, good antennas and a calm scene. Watch the breathing panel's SNR and ACF to see how much margin you have.
6. **Check the Wi-Fi channel.** 2.45 GHz is Wi-Fi channel 8, and the tones span ±3.1 MHz around the LO. If the map is noisy in an empty room, try `--freq 2.475e9` (between Wi-Fi channels 11 and 13, still inside the ISM band).

## Legal

The radar transmits eight unmodulated carriers (two transmitters, four tones each) in the 2.4 GHz ISM band. Keep power and antenna gain within your local limits (US FCC Part 15.247/15.249, EU ETSI EN 300 440). The bladeRF at moderate `--tx-gain` with omni antennas is normally low power, but it is your responsibility. Only use it to detect people who know about it and agree.

## Files

| file | purpose |
|---|---|
| `presence/config.py` | all parameters: radio, tones, antenna layout, map / tracker; thresholds + array calibration (`calib.json`) |
| `presence/radio.py` | bladeRF 2x2 source (multi-tone TX thread, RX thread, demod), replay source, recordings |
| `presence/dsp.py` | multi-tone demod, clutter filter, phase-jump fix, motion metric, spectrogram, breathing estimator |
| `presence/detector.py` | ring buffers, auto floor, single-channel pipeline, EMPTY/MOVING/STILL state machine |
| `presence/mapping.py` | 2D map: steering model, whitening, per-bin localisation, MUSIC for still people, tracker, array calibration |
| `presence/viz.py` | live matplotlib views (map, presence, heart) |
| `presence/sim.py` | synthetic scenes: single-channel (legacy tests) and multichannel people on 2D paths |
| `main.py` | CLI |
