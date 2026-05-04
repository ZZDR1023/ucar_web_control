#!/usr/bin/env python3
"""
Safe keyboard controller for the ROS car.

Keys:
  w: forward
  s: backward
  a: turn left
  d: turn right
  x: stop
  q: quit
"""

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
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist

LINEAR_SPEED = 0.18
ANGULAR_SPEED = 0.9

HELP = """
Safe keyboard control is running.

w: forward
s: backward
a: turn left
d: turn right
x: stop
q: quit
"""


def read_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def build_twist(key):
    msg = Twist()
    if key == 'w':
        msg.linear.x = LINEAR_SPEED
    elif key == 's':
        msg.linear.x = -LINEAR_SPEED
    elif key == 'a':
        msg.angular.z = ANGULAR_SPEED
    elif key == 'd':
        msg.angular.z = -ANGULAR_SPEED
    return msg


def publish_stop(pub):
    stop = Twist()
    rate = rospy.Rate(20)
    for _ in range(10):
        pub.publish(stop)
        rate.sleep()


def main():
    rospy.init_node('safe_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(0.5)
    print(HELP)
    print('Waiting for key input...')

    try:
        while not rospy.is_shutdown():
            key = read_key()
            if key == 'q':
                break
            if key in ['w', 's', 'a', 'd', 'x']:
                msg = build_twist(key)
                pub.publish(msg)
                if key == 'x':
                    print('stop')
                else:
                    print('command:', key)
            else:
                publish_stop(pub)
    finally:
        publish_stop(pub)
        print('stopped and exited')


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
    install_script = f"echo {shlex.quote(encoded)} | base64 -d > /tmp/safe_keyboard_control_remote.py"
    run_ssh(shlex.quote(install_script))

    remote_command = textwrap.dedent(f"""
        source /opt/ros/melodic/setup.bash
        source ~/ucar_ws/devel/setup.bash
        export ROS_MASTER_URI={ROS_MASTER_URI}
        export ROS_IP={ROBOT_IP}
        python /tmp/safe_keyboard_control_remote.py
    """).strip().replace("\n", "; ")

    try:
        run_ssh(shlex.quote(remote_command), allocate_tty=True, interact=True)
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
