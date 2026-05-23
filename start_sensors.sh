#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash

ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://${ROBOT_IP}:11311}"
export ROS_IP="${ROS_IP:-${ROBOT_IP}}"

exec roslaunch ucar_practice sensors.launch
