#!/bin/bash
# Inside the cf_vio/openvins container: OpenVINS (ROS subscriber mode) + live_node (TCP bridge + web viewer).
# Started by live/run.sh; the repo is mounted at /data.
#   env: CONFIG (OpenVINS config folder, repo-relative), OUT (where to save traj_est.txt), STREAMER, HTTP_PORT
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
export ROS_LOG_DIR=/tmp/ros_log ROS_HOME=/tmp/ros
config="/data/${CONFIG:-calib/openvins/crazyflie}/estimator_config.yaml"
out="/data/${OUT:-data/live}"
mkdir -p "$out"

roslaunch ov_msckf subscribe.launch config_path:="$config" max_cameras:=1 use_stereo:=false \
  verbosity:="${VERBOSITY:-WARNING}" dosave:=true path_est:="$out/traj_est.txt" &
ov=$!
trap 'kill $ov 2>/dev/null; wait $ov 2>/dev/null' EXIT
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
python3 -u /data/live/live_node.py --host "${STREAMER:-127.0.0.1}" --port 5600 --http "${HTTP_PORT:-8080}"
