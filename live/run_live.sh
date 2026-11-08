#!/bin/bash
# Inside the cf_vio/openvins container: OpenVINS (ROS subscriber mode) + lio_node (2D lidar-inertial odometry)
# + live_node (TCP bridge + web viewer).
# Started by live/run.sh; the repo is mounted at /data.
#   env: CONFIG (OpenVINS config folder, repo-relative), OUT (where to save traj_est.txt / traj_lio.txt), STREAMER,
#        HTTP_PORT, TITLE (viewer header), LIO_YAW_SIGN (+1 = lidar right side up)
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
export ROS_LOG_DIR=/tmp/ros_log ROS_HOME=/tmp/ros
config="/data/${CONFIG:-calib/openvins/crazyflie}/estimator_config.yaml"
out="/data/${OUT:-data/live}"
mkdir -p "$out"
# OV_OVERRIDES="key=value key2=value2": run on a temporary copy of the config with those estimator keys
# replaced (the calibrated config folder is never modified). Used by validate.sh for init_imu_thresh.
if [ -n "${OV_OVERRIDES:-}" ]; then
  tmp=/tmp/ov_config
  rm -rf "$tmp" && cp -r "$(dirname "$config")" "$tmp"
  for kv in $OV_OVERRIDES; do
    k="${kv%%=*}" v="${kv#*=}"
    grep -q "^$k:" "$tmp/estimator_config.yaml" || { echo "OV_OVERRIDES: unknown key $k" >&2; exit 1; }
    sed -i "s|^$k:.*|$k: $v # overridden by OV_OVERRIDES|" "$tmp/estimator_config.yaml"
  done
  config="$tmp/estimator_config.yaml"
  echo "OpenVINS config overrides: $OV_OVERRIDES"
fi

roslaunch ov_msckf subscribe.launch config_path:="$config" max_cameras:=1 use_stereo:=false \
  verbosity:="${VERBOSITY:-WARNING}" dosave:=true path_est:="$out/traj_est.txt" &
ov=$!
trap 'kill $ov $lio 2>/dev/null; wait $ov $lio 2>/dev/null' EXIT
until rostopic list >/dev/null 2>&1; do sleep 0.5; done
python3 -u /data/live/lio_node.py --out "$out" --yaw-sign "${LIO_YAW_SIGN:-1}" &
lio=$!
python3 -u /data/live/live_node.py --host "${STREAMER:-127.0.0.1}" --port 5600 --http "${HTTP_PORT:-8080}" \
  --title "${TITLE:-}"
