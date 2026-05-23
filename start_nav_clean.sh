#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source /home/ucar/nav_clean_ws/devel/setup.bash

ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://${ROBOT_IP}:11311}"
export ROS_IP="${ROS_IP:-${ROBOT_IP}}"

exec roslaunch nav_clean navigation_runtime.launch
