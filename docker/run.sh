#!/usr/bin/env bash
# Run a command in one of the images with ~/cf_vio mounted at /data and WSLg X11 forwarded.
#   ./run.sh kalibr   rosrun kalibr kalibr_calibrate_cameras --bag data/calib/calib.bag ...
#   ./run.sh openvins roslaunch ov_msckf serial.launch ...
#   ./run.sh kalibr                       # interactive shell
set -euo pipefail
img="cf_vio/${1:?usage: run.sh kalibr|openvins [cmd...]}"; shift
repo="$(cd "$(dirname "$0")/.." && pwd)"
tty=(); [ -t 0 ] && tty=(-it)
exec docker run --rm "${tty[@]}" --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$repo":/data -w /data "$img" "$@"
