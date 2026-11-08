#!/bin/bash
# Source the ROS + catkin workspace, then run the given command (default: bash).
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec "$@"
