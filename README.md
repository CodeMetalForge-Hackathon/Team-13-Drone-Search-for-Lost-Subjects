# Pluto SDR 2.4 GHz human presence radar

This project turns an ADALM-Pluto into a **CW Doppler radar at 2.45 GHz** that detects:

- **MOVING**: someone walking or gesturing. Body motion shows up as Doppler, about 16 Hz per m/s.
- **STILL**: someone standing or sitting still, detected from **breathing**. Chest motion of about 5 mm produces about 0.5 rad of phase change at λ = 12.2 cm, and the code estimates the breathing rate in bpm.
- **EMPTY**: no one detected.

```
Pluto TX ── cyclic 50 kHz tone @ LO 2.45 GHz ──► room / wall ──► Pluto RX
RX 1 MS/s ─► mix tone to DC ─► integrate&dump ─► 200 Hz slow-time IQ
   ├─ clutter removal (2 s running mean) ─► Doppler spectrum ─► motion metric (2–60 Hz vs 70–95 Hz, dB)
   └─ 20 Hz, last ≤30 s motion-free ─► drift HPF ─► arc phase / PCA ─► breathing spectrum + ACF ─► bpm
state machine: EMPTY / MOVING / STILL (STILL? = stopped moving, breathing not locked yet)
```

## Setup (Raspberry Pi OS)

```bash
sudo apt install -y libiio0 libiio-utils          # libiio C library (needs sudo)
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt
iio_info -u ip:192.168.2.1 | head                 # check that the Pluto answers
```

## Usage

```bash
.venv/bin/python main.py run --sim                 # demo with a simulated scene, no hardware
.venv/bin/python main.py run                       # live plots (no calibration needed)
.venv/bin/python main.py run --headless            # status lines over SSH
.venv/bin/python main.py calibrate --boundary 4 --delay 10   # optional: ignore motion beyond ~4 m
.venv/bin/python main.py calibrate                 # optional: classic empty-area calibration
.venv/bin/python main.py run --record walk1.npz    # also save slow-time IQ
.venv/bin/python main.py replay walk1.npz --fast   # re-run detector offline (tune thresholds)
.venv/bin/python -m pytest -q tests                # synthetic tests
```

## Camera overlay (`run --osd`)

```bash
.venv/bin/python main.py run --osd                       # camera + radar graphs in one OpenCV window
.venv/bin/python main.py run --osd --heart --fullscreen  # heart cards + waveforms over the camera
.venv/bin/python main.py run --osd --sim --camera none   # no camera: graphs on a grey background
```

Shows the live camera frame with the radar drawn on top: the state banner (or the STATUS / BREATHING / HEART RATE cards with `--heart`), the Doppler spectrogram, motion vs auto threshold, and breathing waveform. Heart mode adds the heartbeat waveform and heart-rate history. Keys: `q`/Esc quit, `f` fullscreen, `h` hide/show the graphs.

**Range cutoff, live:** `[` / `]` move the range cutoff 1 dB closer / farther (12 dB halves the distance), and `s` saves it to `calib.json`. The right-hand panel shows your motion level against the cutoff line.

**Network stream:** `--rtsp` also serves the view as H.264 over RTSP on port 8080 (`--rtsp-port`), at `rtsp://<pi-ip>:8080/osd`. Open it with VLC (Media → Open Network Stream) or `ffplay -rtsp_transport tcp rtsp://<pi-ip>:8080/osd`. Add `--no-window` to stream without a local window, e.g. over SSH. It uses ffmpeg/libx264 (the Pi 5 has no hardware H.264 encoder); 720p at 15 fps takes about 10% CPU. `--rtsp-fps` and `--rtsp-kbps` tune the stream.

`--camera auto` (default) uses the Pi camera through picamera2 if one is connected, else `/dev/video0`, else no camera. You can also pass `picam`, a V4L2 index, a device path or a video file. `--cam-size 1280x720` sets the capture and window size. `--osd` also works with `replay`.

## Heart rate (`run --heart`)

```bash
.venv/bin/python main.py run --heart --tx-atten 0                 # subject sits still 0.3-0.5 m in front
.venv/bin/python main.py run --heart --tx-atten 0 --ref-bpm 72    # overlay a watch / oximeter reading
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

5.8 GHz was measured and isn't better here: the floor rises 7.6 dB, cancelling the 7.5 dB phase-sensitivity gain. Simulation results (0.3 mm heartbeat): locked about 40% of the time, about 8% wrong readings, and 2–5% false locks with no heartbeat present. Real-world numbers need a recording with a reference pulse.

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
- The level depends on `--rx-gain` / `--tx-atten`. They are stored with the boundary, and the threshold is shifted automatically if you change them.
- The plot's lower-left panel shows the motion level (dBFS) against the boundary line.

Useful options: `--tx-atten` sets TX gain in dB (0 = max ≈ +7 dBm, default -10), `--rx-gain` sets RX gain in dB (default 50), `--freq` sets the LO in Hz, and `--breath-win` sets the breathing window in seconds.

## Antennas and tuning, especially through walls or beyond 4 m

1. **Use directional antennas.** 2.4 GHz patch or yagi antennas give 8–14 dBi each, and every dB counts twice (TX and RX). The stock rubber ducks will do motion at short range, but breathing through a wall needs gain.
2. **Separate and point the antennas.** Mount TX and RX side by side, 30–50 cm apart, both aimed at the target area. Direct TX→RX leakage is the main thing that saturates the receiver.
3. **Set the levels.** At startup the program prints `RX peak … dBFS`. Raise TX (`--tx-atten -5`, then `0`) and RX gain until the peak sits around -10 to -6 dBFS. If it warns about clipping, back off.
4. **Hold everything still.** Mount the antennas and the Pluto rigidly; a swaying cable looks like breathing. Fans, AC vents, curtains and pets in the beam produce Doppler too. Run `calibrate` with them in their normal state.
5. Drywall or wood costs about 3–6 dB, brick or concrete 10–20 dB, and metal blocks the signal completely. **Motion through a wall is reliable. Breathing through a wall beyond 4 m is marginal.** It needs the person still for about 15–30 s, good antennas and a calm scene. Watch the breathing panel's SNR and ACF to see how much margin you have.
6. **Check the Wi-Fi channel.** 2.45 GHz is Wi-Fi channel 8. Strong Wi-Fi bursts mostly land outside our 1 MHz band, but if the motion metric is noisy in an empty room, try `--freq 2.48e9` (above Wi-Fi channel 11, still inside the ISM band).

## Legal

The radar transmits an unmodulated carrier in the 2.4 GHz ISM band. Keep power and antenna gain within your local limits (US FCC Part 15.247/15.249, EU ETSI EN 300 440). The Pluto's ~+7 dBm plus modest antennas is normally low, but it is your responsibility. Only use it to detect people who know about it and agree.

## Files

| file | purpose |
|---|---|
| `presence/config.py` | all parameters and thresholds (`calib.json`) |
| `presence/radio.py` | Pluto CW radar source (TX tone, RX thread, demod), replay source |
| `presence/dsp.py` | tone demod, clutter filter, phase-jump fix, motion metric, spectrogram, breathing estimator |
| `presence/detector.py` | ring buffers, auto floor, processing pipeline, EMPTY/MOVING/STILL state machine, boundary calibration |
| `presence/viz.py` | live matplotlib view |
| `presence/osd.py` | OpenCV camera view with the radar graphs overlaid (`--osd`) |
| `presence/labels.py` | status text / colours and live spectrogram shared by both views |
| `presence/sim.py` | synthetic scenes (incl. near person + far bystanders) for tests and `--sim` |
| `main.py` | CLI |
