#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/ucar/nav_clean_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179

LINEAR_X="$1"
ANGULAR_Z="$2"
DURATION="${3:-0.8}"
RATE="${4:-20}"

MSG="{linear: {x: $LINEAR_X, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_Z}}"

timeout "$DURATION" rostopic pub -r "$RATE" /cmd_vel geometry_msgs/Twist "$MSG" >/dev/null 2>&1
