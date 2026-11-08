#!/usr/bin/env bash
# Live VIO in one command. Starts OpenVINS + the web viewer in Docker, then the sensor streamer on the host.
# Open http://<this machine>:8080 in any browser on the same network to watch.
#
# Usage
#   live/run.sh                                   # Pi 5: camera + Crazyflie UART (args go to streamer.py)
#   live/run.sh --port /dev/ttyAMA0               # pick the UART device
#   live/run.sh --replay recordings/<stamp>       # laptop test: play a recording back in real time
#   CONFIG=calib/openvins/crazyflie HTTP_PORT=8080 IMAGE=cf_vio/openvins live/run.sh ...
# Ctrl+C stops everything. Live sessions are recorded to recordings/<stamp>/ (reprocess with ./demo.sh).
set -euo pipefail
cd "$(dirname "$0")/.."
config="${CONFIG:-calib/openvins/crazyflie}"
[ -f "$config/estimator_config.yaml" ] || { echo "error: $config/estimator_config.yaml not found" >&2; exit 1; }
name="live_$(date +%Y%m%d_%H%M%S)"
py=python3
[ -x .venv/bin/python ] && py=.venv/bin/python   # laptop; on the Pi use system python (picamera2)

docker rm -f cf_vio_live >/dev/null 2>&1 || true
docker run -d --rm --name cf_vio_live --network host --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e CONFIG="$config" -e OUT="data/$name" -e HTTP_PORT="${HTTP_PORT:-8080}" -e VERBOSITY="${VERBOSITY:-WARNING}" \
  -v "$PWD":/data --entrypoint bash "${IMAGE:-cf_vio/openvins}" /data/live/run_live.sh >/dev/null
trap 'echo; echo "stopping OpenVINS container"; docker stop -t 3 cf_vio_live >/dev/null 2>&1 || true' EXIT
echo "OpenVINS starting in Docker (logs: docker logs -f cf_vio_live); trajectory -> data/$name/traj_est.txt"
echo "Viewer: http://$(hostname -I 2>/dev/null | awk '{print $1}'):${HTTP_PORT:-8080}/  (or http://localhost:${HTTP_PORT:-8080}/)"
$py live/streamer.py "$@"
