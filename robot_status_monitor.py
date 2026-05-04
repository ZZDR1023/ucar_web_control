#!/usr/bin/env python3
"""Monitor ROS car status from the local machine through SSH."""

import base64
import shlex
import sys
import textwrap

import pexpect

ROBOT_IP = "10.235.133.179"
ROBOT_USER = "ucar"
ROBOT_PASSWORD = "ucar"
ROS_MASTER_URI = f"http://{ROBOT_IP}:11311"

REMOTE_SCRIPT = r'''
import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu

state = {
    'cmd_linear': 0.0,
    'cmd_angular': 0.0,
    'odom_x': 0.0,
    'odom_y': 0.0,
    'odom_linear': 0.0,
    'odom_angular': 0.0,
    'imu_angular_z': 0.0,
    'battery_percentage': None,
    'battery_voltage': None,
}


def cmd_callback(msg):
    state['cmd_linear'] = msg.linear.x
    state['cmd_angular'] = msg.angular.z


def odom_callback(msg):
    state['odom_x'] = msg.pose.pose.position.x
    state['odom_y'] = msg.pose.pose.position.y
    state['odom_linear'] = msg.twist.twist.linear.x
    state['odom_angular'] = msg.twist.twist.angular.z


def imu_callback(msg):
    state['imu_angular_z'] = msg.angular_velocity.z


def battery_callback(msg):
    state['battery_voltage'] = msg.voltage
    state['battery_percentage'] = msg.percentage


def format_optional(value, suffix=''):
    if value is None or math.isnan(value):
        return 'unknown'
    return '%.2f%s' % (value, suffix)


def print_status():
    sys.stdout.write('\033[2J\033[H')
    print('Robot Status Monitor')
    print('Press Ctrl+C to exit')
    print('-' * 36)
    print('cmd_vel linear   : %.3f m/s' % state['cmd_linear'])
    print('cmd_vel angular  : %.3f rad/s' % state['cmd_angular'])
    print('odom linear      : %.3f m/s' % state['odom_linear'])
    print('odom angular     : %.3f rad/s' % state['odom_angular'])
    print('odom position    : x=%.3f, y=%.3f' % (state['odom_x'], state['odom_y']))
    print('imu angular z    : %.3f rad/s' % state['imu_angular_z'])
    print('battery voltage  : %s' % format_optional(state['battery_voltage'], ' V'))
    print('battery percent  : %s' % format_optional(state['battery_percentage'], ' %'))
    sys.stdout.flush()


def main():
    rospy.init_node('robot_status_monitor', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, cmd_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/imu', Imu, imu_callback)
    rospy.Subscriber('/battery_state', BatteryState, battery_callback)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        print_status()
        rate.sleep()


if __name__ == '__main__':
    main()
'''


def run_ssh(command, allocate_tty=False, interact=False):
    tty_flag = "-tt" if allocate_tty else "-T"
    ssh_command = (
        f"ssh {tty_flag} "
        "-o StrictHostKeyChecking=no "
        "-o UserKnownHostsFile=/dev/null "
        f"{ROBOT_USER}@{ROBOT_IP} {command}"
    )
    child = pexpect.spawn(ssh_command, encoding="utf-8", timeout=None)
    index = child.expect(["password:", pexpect.EOF, pexpect.TIMEOUT])
    if index == 0:
        child.sendline(ROBOT_PASSWORD)
    if interact:
        child.interact()
    else:
        child.expect(pexpect.EOF)
        print(child.before)


def main():
    encoded = base64.b64encode(REMOTE_SCRIPT.encode()).decode()
    install_script = f"echo {shlex.quote(encoded)} | base64 -d > /tmp/robot_status_monitor_remote.py"
    run_ssh(shlex.quote(install_script))

    remote_command = textwrap.dedent(f"""
        source /opt/ros/melodic/setup.bash
        source ~/ucar_ws/devel/setup.bash
        export ROS_MASTER_URI={ROS_MASTER_URI}
        export ROS_IP={ROBOT_IP}
        python /tmp/robot_status_monitor_remote.py
    """).strip().replace("\n", "; ")

    try:
        run_ssh(shlex.quote(remote_command), allocate_tty=True, interact=True)
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
