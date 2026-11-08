#!/usr/bin/env bash
# Validate the OpenVINS calibration against lidar-inertial odometry, frame by frame, in real time.
#
# For each recording: replays camera + IMU + lidar at real speed (live/streamer.py --replay) into the live
# container: OpenVINS (calibrated config) + lio_node (2D LIO, the reference) + live_node (web viewer).
# Watch it at http://<this machine>:8080 : raw camera, OpenVINS tracks, lidar map with the LIO and the aligned
# VIO path, and the VIO-vs-LIO position/heading error over time. Afterwards writes
# data/validate_<stamp>_<config>/{traj_est.txt, traj_lio.txt, vio_lio_summary.json, vio_lio_errors.csv, vio_lio.png}.
#
# Usage
#   live/validate.sh                         # the 2 latest recordings that have lidar.bin
#   live/validate.sh 20261108_061442 recordings/20261108_061640
#   CONFIG=calib/openvins/pi5 RATE=1.0 HOLD=20 HTTP_PORT=8080 live/validate.sh ...
#     HOLD: seconds the viewer stays up after each replay (interactive terminal: waits for Enter instead)
#     OV_OVERRIDES: estimator keys replaced on a temp copy of the config (calibration files untouched).
#       Default "init_imu_thresh=0.15": these walks start gently (accel std ramps 0.1 -> 0.4 m/s^2), so the
#       config's 0.5 only triggers once the camera is already moving and the static initialiser refuses.
#       Still noise on the rig is 0.02-0.03 m/s^2. Set OV_OVERRIDES="" to run the config exactly as is.
set -euo pipefail
cd "$(dirname "$0")/.."
config="${CONFIG:-calib/openvins/pi5}"
port="${HTTP_PORT:-8080}"
[ -f "$config/estimator_config.yaml" ] || { echo "error: $config/estimator_config.yaml not found" >&2; exit 1; }

recs=()
for r in "$@"; do
  [ -d "$r" ] || r="recordings/$r"
  [ -f "$r/lidar.bin" ] || { echo "error: $r has no lidar.bin" >&2; exit 1; }
  recs+=("${r%/}")
done
if [ ${#recs[@]} -eq 0 ]; then
  mapfile -t recs < <(for d in recordings/*/; do [ -s "$d/lidar.bin" ] && [ "$(stat -c %s "$d/frames.bin")" -gt 50000000 ] \
    && echo "${d%/}"; done | sort | tail -n 2)
fi
[ ${#recs[@]} -gt 0 ] || { echo "error: no recordings with lidar found" >&2; exit 1; }
py=python3
[ -x .venv/bin/python ] && py=.venv/bin/python

cleanup() { docker stop -t 3 cf_vio_validate >/dev/null 2>&1 || true; }
trap cleanup EXIT
ip=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "Viewer: http://${ip:-localhost}:$port/   (config: $config)"
echo "Recordings: ${recs[*]}"

for rec in "${recs[@]}"; do
  stamp=$(basename "$rec")
  out="data/validate_${stamp}_$(basename "$config")"
  mkdir -p "$out"
  rm -f "$out"/traj_est.txt "$out"/traj_lio.txt "$out"/lio_stats.csv
  echo
  echo "=== $stamp -> $out"
  docker rm -f cf_vio_validate >/dev/null 2>&1 || true
  docker run -d --rm --name cf_vio_validate --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -e CONFIG="$config" -e OUT="$out" -e HTTP_PORT="$port" -e TITLE="$stamp" -e OV_OVERRIDES="${OV_OVERRIDES-init_imu_thresh=0.15}" -e VERBOSITY="${VERBOSITY:-INFO}" \
    -v "$PWD":/data --entrypoint bash cf_vio/openvins /data/live/run_live.sh >/dev/null
  # replay in real time; the streamer waits for live_node to connect first
  $py live/streamer.py --replay "$rec" --rate "${RATE:-1.0}" 2>&1 | tee "$out/streamer.log" | grep --line-buffered -v "^\[streamer\] imu " || true
  docker logs cf_vio_validate > "$out/container.log" 2>&1 || true
  if [ -t 0 ]; then
    read -r -p "Replay of $stamp finished; viewer still up. Press Enter to continue... " _
  else
    echo "Replay of $stamp finished; viewer stays up for ${HOLD:-20} s"; sleep "${HOLD:-20}"
  fi
  cleanup
  sleep 1
  echo "--- report"
  docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e MPLCONFIGDIR=/tmp -v "$PWD":/data -w /data \
    --entrypoint python3 cf_vio/openvins offline/compare_vio_lio.py "$out" | tee "$out/report.txt" || true
done
echo
echo "Done. Per-run outputs: data/validate_<stamp>_<config>/vio_lio_summary.json, vio_lio.png, vio_lio_errors.csv"
