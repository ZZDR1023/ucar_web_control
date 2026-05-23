#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/ucar/nav_clean_ws/devel/setup.bash
ROBOT_IP="${ROBOT_IP:-$(hostname -I | awk '{print $1}')}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://${ROBOT_IP}:11311}"
export ROS_IP="${ROS_IP:-${ROBOT_IP}}"

LINEAR_X="$1"
ANGULAR_Z="$2"
if awk "BEGIN {exit !($LINEAR_X > 0.10 || $LINEAR_X < -0.10 || $ANGULAR_Z > 0.50 || $ANGULAR_Z < -0.50)}"; then
  echo "Refusing unsafe debug cmd_vel: linear must be within +/-0.10, angular within +/-0.50" >&2
  exit 2
fi
DURATION="${3:-0.8}"
RATE="${4:-20}"

MSG="{linear: {x: $LINEAR_X, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_Z}}"

timeout "$DURATION" rostopic pub -r "$RATE" /cmd_vel geometry_msgs/Twist "$MSG" >/dev/null 2>&1
