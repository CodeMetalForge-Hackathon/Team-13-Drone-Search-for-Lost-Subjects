#!/usr/bin/env bash
# One command from a Pi recording to demo outputs:
#   convert -> OpenVINS (Docker) -> summary.json, traj_topdown.png, imu_vs_vio.png, demo.mp4
#
# Usage
#   ./demo.sh <stamp | recording folder> [--rviz] [--name NAME] [--no-video]
#     --rviz       show OpenVINS live in RViz (close RViz when done; the run still finishes)
#     --name NAME  output folder data/NAME (default: the recording stamp)
#   CONFIG=calib/openvins/crazyflie  OpenVINS config folder (from calib/kalibr_to_openvins.py)
set -euo pipefail
cd "$(dirname "$0")"

rec="" name="" rviz=0 video=1
while [ $# -gt 0 ]; do
  case "$1" in
    --rviz) rviz=1 ;;
    --no-video) video=0 ;;
    --name) name="$2"; shift ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) rec="$1" ;;
  esac
  shift
done
[ -n "$rec" ] || { sed -n '2,9p' "$0"; exit 1; }
[ -d "$rec" ] || rec="recordings/$rec"
[ -d "$rec" ] || { echo "error: recording not found: $rec" >&2; exit 1; }
rec="${rec%/}"
name="${name:-$(basename "$rec")}"
out="data/$name"
config="${CONFIG:-calib/openvins/crazyflie}"
py=.venv/bin/python

[ -x "$py" ] || { echo "error: no .venv; run: python3 -m venv .venv && .venv/bin/pip install -r offline/requirements.txt" >&2; exit 1; }
[ -f "$config/estimator_config.yaml" ] || {
  echo "error: $config/estimator_config.yaml not found; create it with calib/kalibr_to_openvins.py" >&2; exit 1; }

echo "== 1/3 convert $rec -> $out"
"$py" offline/convert.py "$rec" --out "$out" --bag --check-sync

echo "== 2/3 OpenVINS ($config)"
rm -f "$out/traj_est.txt"
launch="roslaunch ov_msckf serial.launch config_path:=/data/$config/estimator_config.yaml \
max_cameras:=1 use_stereo:=false bag:=/data/$out/$name.bag dosave:=true path_est:=/data/$out/traj_est.txt"
if [ "$rviz" = 1 ]; then
  # RViz in the foreground; after it is closed, wait for the bag to finish so the trajectory is complete
  ./docker/run.sh openvins bash -c "$launch > /data/$out/openvins.log 2>&1 & sleep 5; \
rviz -d \$(rospack find ov_msckf)/launch/display.rviz; wait" || true
else
  # the OpenVINS node aborts on exit (harmless ROS class_loader issue), so ignore the exit code
  ./docker/run.sh openvins bash -c "$launch" > "$out/openvins.log" 2>&1 || true
fi
rows=$(grep -vc '^#' "$out/traj_est.txt" 2>/dev/null || true)
echo "   trajectory poses: ${rows:-0}"

echo "== 3/3 report"
args=("$out")
[ "$video" = 1 ] && args+=(--video)
"$py" offline/demo_report.py "${args[@]}"
echo
echo "Outputs in $out:"; ls "$out" | grep -E "summary.json|.png$|demo.mp4" | sed "s/^/  /"
