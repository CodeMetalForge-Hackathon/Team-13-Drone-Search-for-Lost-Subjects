# Crazyflie + Pi Zero 2 W: record, then run VIO offline (MVP)

```
 Crazyflie 2.1+                         Pi Zero 2 W + Camera Module 3              Laptop (offline)
 ─────────────                          ─────────────────────────────              ────────────────
 app_imu_uart (500 Hz gyro+acc,  TX2 ─► recorder.py                    copy  ──►   convert.py  → EuRoC folder + ROS1 bag
 µs timestamp, CRC)              UART   ├ imu.bin    (raw UART + arrival time)     Kalibr      → camera + cam-IMU calibration
                                 1 Mbd  ├ frames.bin (raw 8-bit gray)              OpenVINS    → trajectory
                                        └ frames.csv (SensorTimestamp)
```

| Folder | What |
|---|---|
| `firmware/app_imu_uart/` | Out-of-tree Crazyflie app: streams IMU over UART2 |
| `pi/recorder.py` | Records camera + IMU stream on the Pi, no processing |
| `offline/convert.py` | Parses IMU, fits the CF→Pi clock, writes EuRoC + bag, sync check |
| `offline/demo_report.py`, `demo.sh` | Demo outputs: stats, IMU-only vs VIO plot, side-by-side video |
| `offline/make_synthetic.py` | Fake recording with known answers, to test the converter |
| `calib/` | Kalibr target/IMU templates, Kalibr → OpenVINS config helper, OpenVINS starter config |
| `docker/` | Docker install + Kalibr / OpenVINS images for WSL, `run.sh` wrapper |

---

## 1. Wiring

| Crazyflie expansion header | → | Pi Zero 2 W |
|---|---|---|
| TX2 (right header) | → | GPIO15 / RXD (physical pin 10) |
| GND | → | GND (physical pin 6) |
| VCOM (right header, unregulated battery) | → 1S→5V boost (≥1 A) → | 5V (physical pin 2) + GND |

* Only CF→Pi is needed; leave Pi TXD unconnected.
* The CF is a 3.0 V system; its TX drives the Pi's 3.3 V input fine.
* **Verify pin positions on the Bitcraze Breakout/Prototyping deck schematic before soldering.**
  A Prototyping deck makes a clean, removable mount.
* VCOM is fed directly from the battery: unplug the battery after flights or the Pi drains it.
* UART2 conflicts with the AI deck. Flow deck v2 and Lighthouse deck are fine.

## 2. Crazyflie firmware

Needs the ARM toolchain (`arm-none-eabi-gcc`); on Windows build inside WSL.

```bash
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
cp -r firmware/app_imu_uart crazyflie-firmware/examples/
cd crazyflie-firmware/examples/app_imu_uart
make CRAZYFLIE_BASE=../.. -j
make CRAZYFLIE_BASE=../.. cload      # flash over Crazyradio (CF in bootloader mode)
```

The cfclient console should print `IMUUART: Streaming IMU on UART2 at 1000000 baud, 500 Hz`.

## 3. Pi setup (Raspberry Pi OS Lite, 64-bit, Bookworm)

```bash
sudo raspi-config nonint do_serial_cons 1    # no login shell on serial
sudo raspi-config nonint do_serial_hw 0      # keep the UART hardware enabled
echo -e "enable_uart=1\ndtoverlay=disable-bt" | sudo tee -a /boot/firmware/config.txt
sudo systemctl disable hciuart
sudo apt install -y --no-install-recommends python3-picamera2 python3-serial
sudo reboot
```

`disable-bt` frees the good UART (PL011) for `/dev/serial0`; the mini-UART can't hold 1 Mbaud reliably.
Check the camera with `rpicam-hello --list-cameras`.

**IMU check** (seconds, before every session): `python3 imu_check.py` prints the live IMU rate, gyro and accel.
At rest `|acc|` should read about 1.00 g, and tilting the drone should move the gyro. `ALL ZEROS` means the
Crazyflie's sensors never calibrated (its stabilizer only runs after it has been still after power-on):
power-cycle it lying flat on the table and don't touch it for ~3 s. `NO DATA` means nothing arrives on the UART.
`recorder.py` runs the same check for 1 s and refuses to record unless it passes (`--skip-imu-check` overrides).

Bench test (CF powered, props off):

```bash
python3 recorder.py --duration 20
```

Copy `recordings/<stamp>/` to the laptop and convert it (step 4). `report.json` should show
`rate_hz ≈ 500`, `lost_packets ≈ 0`, `dropped_frames = 0`, and `drift_ppm` within a few tens of ppm.

Live check on the Pi's HDMI monitor (camera + status bar + gyro strip, nothing saved; stop
`recorder.py` first, needs `sudo apt install -y python3-pil`). It draws straight into the console
framebuffer `/dev/fb0`, not picamera2's DRM preview, so it also works when started over SSH:

```bash
python3 live_view.py               # same camera settings as recorder.py
```

Recorder knobs: `--rotate` (default 180 for an upside-down camera; 0 for upright),
`--exposure-us` (short = less blur, default 3000), `--gain`, `--fps`, `--width/--height`,
`--lens-position` (dioptres; Camera Module 3 autofocus is locked because focus changes the intrinsics,
so **use the same value for calibration and flights**).

## 4. Convert on the laptop

```bash
pip install -r offline/requirements.txt
python offline/convert.py recordings/20261107_120000 --out data/flight1 --bag --check-sync
```

Outputs `data/flight1/mav0/...` (EuRoC/ASL), `data/flight1/flight1.bag` (`/cam0/image_raw`, `/imu0`),
`report.json`, and `sync.png` (clock fit + flow-vs-gyro correlation).

`--check-sync` is a coarse (~1–2 ms) sanity check that needs strong rotations, so shake the drone at the
start of a recording. Kalibr gives the precise offset.

Watch a recording (MP4 with time/frame overlay and a gyro strip underneath; works on the raw
recording folder, no conversion needed):

```bash
python offline/preview.py recordings/20261107_120000        # -> recordings/20261107_120000/preview.mp4
```

Test without hardware:

```bash
python offline/make_synthetic.py /tmp/rec_synth
python offline/convert.py /tmp/rec_synth --out /tmp/out_synth --bag --check-sync
```

Expected: `drift_ppm ≈ 40`, `suggested_extra_cam_shift_ms ≈ +15`.

## 5. Docker images for Kalibr and OpenVINS (WSL, once)

Both tools run in ROS1 Noetic images built from `docker/` (sources are cloned to `~/vio_tools/`).

```bash
sudo bash docker/install_docker.sh     # Docker Engine in WSL; open a new shell afterwards
./docker/build_images.sh               # builds cf_vio/kalibr and cf_vio/openvins (~30-40 min, JOBS=4)
```

`./docker/run.sh <kalibr|openvins> <cmd...>` runs a command with this repo mounted at `/data`
(the working directory) and the WSLg display forwarded, so plots and RViz open on the Windows desktop.
With no command it gives a shell.

## 6. Calibration (Kalibr, once per camera mount / focus setting)

1. Print an AprilGrid and measure the tag size and set it in `calib/aprilgrid.yaml`.
2. Record 60–90 s with the recorder while moving the drone **by hand** in front of the grid
   (motors off). Excite all 3 rotation and 3 translation axes, and keep the grid sharp and filling the view.
   Use the same `--width/--height` and `--lens-position` as for flights.
3. Convert with `--bag` (e.g. `--out data/calib`), then run Kalibr. Results land next to the bag
   (`data/calib/calib-camchain.yaml`, then `calib-camchain-imucam.yaml`, plus PDF reports):

```bash
./docker/run.sh kalibr rosrun kalibr kalibr_calibrate_cameras --bag data/calib/calib.bag --bag-freq 10 \
    --target calib/aprilgrid.yaml --models pinhole-radtan --topics /cam0/image_raw --dont-show-report
./docker/run.sh kalibr rosrun kalibr kalibr_calibrate_imu_camera --bag data/calib/calib.bag \
    --cam data/calib/calib-camchain.yaml --imu calib/imu.yaml --target calib/aprilgrid.yaml --dont-show-report
```

Use `pinhole-equi` instead of `pinhole-radtan` with the Camera Module 3 *Wide*.
Good results: camera reprojection error well under 1 px, and a `timeshift_cam_imu` of a few ms to tens of ms.
`calib/imu.yaml` holds inflated BMI088 noise values; replace them with an Allan-variance fit if you have one.

## 7. Run OpenVINS

Turn the Kalibr result into an OpenVINS config folder (starts from `calib/openvins/estimator_config.yaml`,
a mono 640×360 starter config; `--meta` checks the calibration resolution matches the recording):

```bash
python calib/kalibr_to_openvins.py --camchain data/calib/calib-camchain-imucam.yaml --imu calib/imu.yaml \
    --out calib/openvins/crazyflie --meta recordings/<stamp>/meta.json
```

Run it on a converted flight, with RViz showing tracked features and the trajectory:

```bash
./docker/run.sh openvins bash -c 'roslaunch ov_msckf serial.launch \
    config_path:=/data/calib/openvins/crazyflie/estimator_config.yaml max_cameras:=1 use_stereo:=false \
    bag:=/data/data/flight1/flight1.bag dosave:=true path_est:=/data/data/flight1/traj_est.txt &
  sleep 5; rviz -d $(rospack find ov_msckf)/launch/display.rviz'
```

Close RViz to stop the container; `traj_est.txt` is complete once the bag has finished playing.

The camera-IMU time offset is passed via `timeshift_cam_imu` in the config, so convert with
`--cam-shift-ms 0`. For tools that don't model a time offset (e.g. ORB-SLAM3 on the EuRoC folder),
reconvert with `--cam-shift-ms <1000 × Kalibr timeshift_cam_imu>`.

## 8. Demo outputs in one command

After calibration (`calib/openvins/crazyflie/` exists), turn any recording into demo material:

```bash
./demo.sh <stamp>            # e.g. ./demo.sh 20261107_222749 ; add --rviz to watch OpenVINS live
```

It converts the recording, runs OpenVINS in Docker and writes into `data/<stamp>/`:

| File | What |
|---|---|
| `summary.json` | VIO path length, start-to-end gap, drift %, IMU-only end error |
| `traj_topdown.png` | VIO trajectory from above, start and end marked |
| `imu_vs_vio.png` | distance from the start over time: IMU-only dead reckoning vs VIO |
| `demo.mp4` | camera on the left, trajectory drawing itself on the right |
| `openvins.log` | OpenVINS output, if something goes wrong |

For a loop run (start and end on the same taped X) the start-to-end gap is the drift number for the demo.
The IMU-only baseline works even on a recording where the drone sits still on the desk.
`offline/demo_report.py data/<name> --video` reruns only the report.

### Flight protocol

1. Start `recorder.py` (over SSH), with the drone sitting still on the ground.
2. Keep it still for **≥3 s** (OpenVINS static initialisation), then take off.
3. Fly with some rotation and translation; avoid textureless floors and pure hover.
4. Land, keep still 2 s, then Ctrl+C.

---

## Known limitations of the MVP

* IMU samples are read through the log API at 500 Hz, so each sample is up to ~1 ms old when stamped.
  The mean is absorbed by the estimated time offset; the ±0.5 ms jitter is fine at 30 fps.
  Upgrade: patch `stabilizer.c` to export `sensorData.interruptTimestamp`.
* The Camera Module 3 has a rolling shutter. Short exposure and a soft mount help; OpenVINS doesn't
  model rolling shutter.
* Weight: Pi + camera + boost ≈ 15–17 g, at the limit of the 2.1+ payload. Expect short flights.
* Only CF→Pi traffic; there's no ground truth. Add Lighthouse/mocap logs if you need accuracy numbers.
