#!/usr/bin/env python3
"""Run a combined safe controller and status monitor on the ROS car through SSH."""

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
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu

LINEAR_GEARS = [0.10, 0.18, 0.28]
ANGULAR_GEARS = [0.5, 0.9, 1.3]
DEFAULT_GEAR = 1

control = {
    'linear': 0.0,
    'angular': 0.0,
    'gear': DEFAULT_GEAR,
}

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
    'last_key': 'none',
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


def build_twist():
    msg = Twist()
    msg.linear.x = control['linear']
    msg.angular.z = control['angular']
    return msg


def current_linear_speed():
    return LINEAR_GEARS[control['gear']]


def current_angular_speed():
    return ANGULAR_GEARS[control['gear']]


def set_gear(index):
    control['gear'] = max(0, min(index, len(LINEAR_GEARS) - 1))
    if control['linear'] > 0:
        control['linear'] = current_linear_speed()
    elif control['linear'] < 0:
        control['linear'] = -current_linear_speed()
    if control['angular'] > 0:
        control['angular'] = current_angular_speed()
    elif control['angular'] < 0:
        control['angular'] = -current_angular_speed()


def handle_key(key):
    if key == 'w':
        control['linear'] = current_linear_speed()
    elif key == 's':
        control['linear'] = -current_linear_speed()
    elif key == 'a':
        control['angular'] = current_angular_speed()
    elif key == 'd':
        control['angular'] = -current_angular_speed()
    elif key == 'z':
        control['linear'] = 0.0
    elif key == 'c':
        control['angular'] = 0.0
    elif key == 'x':
        control['linear'] = 0.0
        control['angular'] = 0.0
    elif key in ['1', '2', '3']:
        set_gear(int(key) - 1)


def publish_stop(pub):
    control['linear'] = 0.0
    control['angular'] = 0.0
    stop = Twist()
    rate = rospy.Rate(20)
    for _ in range(10):
        pub.publish(stop)
        rate.sleep()


def format_optional(value, suffix=''):
    if value is None or math.isnan(value):
        return 'unknown'
    return '%.2f%s' % (value, suffix)


def write_screen(lines):
    sys.stdout.write('\033[2J\033[H')
    sys.stdout.write('\r\n'.join(lines))
    sys.stdout.write('\r\n')
    sys.stdout.flush()


def print_status():
    width = 66
    border = '+' + '-' * (width - 2) + '+'
    gear_label = '%d  %.2f m/s  %.2f rad/s' % (
        control['gear'] + 1,
        current_linear_speed(),
        current_angular_speed(),
    )
    active_cmd = 'linear=%+.2f  angular=%+.2f' % (control['linear'], control['angular'])
    lines = [
        border,
        '| ROS Car Safe Control'.ljust(width - 1) + '|',
        border,
        '| Move: w/s set forward/back, a/d set left/right arc'.ljust(width - 1) + '|',
        '| Stop: z linear stop, c turn stop, x emergency stop, q quit'.ljust(width - 1) + '|',
        '| Gear: 1 slow, 2 normal, 3 fast'.ljust(width - 1) + '|',
        border,
        '| Gear'.ljust(20) + ': %-39s |' % gear_label,
        '| Active command'.ljust(20) + ': %-39s |' % active_cmd,
        '| Last key'.ljust(20) + ': %-39s |' % state['last_key'],
        border,
        '| Command linear'.ljust(20) + ': %8.3f m/s                         |' % state['cmd_linear'],
        '| Command angular'.ljust(20) + ': %8.3f rad/s                       |' % state['cmd_angular'],
        '| Odom linear'.ljust(20) + ': %8.3f m/s                         |' % state['odom_linear'],
        '| Odom angular'.ljust(20) + ': %8.3f rad/s                       |' % state['odom_angular'],
        '| Odom position'.ljust(20) + ': x=%7.3f   y=%7.3f              |' % (state['odom_x'], state['odom_y']),
        '| IMU angular z'.ljust(20) + ': %8.3f rad/s                       |' % state['imu_angular_z'],
        '| Battery voltage'.ljust(20) + ': %-39s |' % format_optional(state['battery_voltage'], ' V'),
        '| Battery percent'.ljust(20) + ': %-39s |' % format_optional(state['battery_percentage'], ' %'),
        border,
    ]
    write_screen(lines)


def read_latest_key():
    latest = None
    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return latest
        latest = sys.stdin.read(1)


def main():
    rospy.init_node('safe_control_with_status', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/cmd_vel', Twist, cmd_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/imu', Imu, imu_callback)
    rospy.Subscriber('/battery_state', BatteryState, battery_callback)
    rospy.sleep(0.5)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    rate = rospy.Rate(10)
    screen_tick = 0

    try:
        tty.setraw(fd)
        print_status()
        while not rospy.is_shutdown():
            key = read_latest_key()
            if key == 'q':
                state['last_key'] = 'q'
                break
            if key in ['w', 's', 'a', 'd', 'x', 'z', 'c', '1', '2', '3']:
                state['last_key'] = key
                handle_key(key)
                if key == 'x':
                    publish_stop(pub)
                else:
                    pub.publish(build_twist())
            elif key is not None:
                state['last_key'] = 'stop: unknown key'
                publish_stop(pub)
            else:
                pub.publish(build_twist())

            screen_tick += 1
            if screen_tick >= 10:
                print_status()
                screen_tick = 0
            rate.sleep()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        publish_stop(pub)
        sys.stdout.write('\r\nstopped and exited\r\n')
        sys.stdout.flush()


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
    install_script = f"echo {shlex.quote(encoded)} | base64 -d > /tmp/safe_control_with_status_remote.py"
    run_ssh(shlex.quote(install_script))

    remote_command = textwrap.dedent(f"""
        source /opt/ros/melodic/setup.bash
        source ~/ucar_ws/devel/setup.bash
        export ROS_MASTER_URI={ROS_MASTER_URI}
        export ROS_IP={ROBOT_IP}
        python /tmp/safe_control_with_status_remote.py
    """).strip().replace("\n", "; ")

    try:
        run_ssh(shlex.quote(remote_command), allocate_tty=True, interact=True)
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
