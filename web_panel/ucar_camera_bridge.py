#!/usr/bin/env python3
"""V4L2 camera publisher for person following."""

import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


def make_image(frame, frame_id):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.height = rgb.shape[0]
    msg.width = rgb.shape[1]
    msg.encoding = "rgb8"
    msg.is_bigendian = False
    msg.step = rgb.shape[1] * 3
    msg.data = np.ascontiguousarray(rgb).tobytes()
    return msg


def open_camera(device, width, height):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def main():
    rospy.init_node("ucar_camera_bridge", anonymous=False)
    device = rospy.get_param("~device", "/dev/video0")
    topic = rospy.get_param("~topic", "/ucar_camera/image_raw")
    width = int(rospy.get_param("~width", 640))
    height = int(rospy.get_param("~height", 480))
    rate_hz = float(rospy.get_param("~rate", 10.0))
    frame_id = rospy.get_param("~frame_id", "opencv")
    pub = rospy.Publisher(topic, Image, queue_size=1)
    cap = open_camera(device, width, height)
    if not cap.isOpened():
        raise RuntimeError("cannot open camera {}".format(device))
    rate = rospy.Rate(rate_hz)
    consecutive_failures = 0
    rospy.loginfo("ucar_camera_bridge publishing %s from %s", topic, device)
    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok or frame is None:
            consecutive_failures += 1
            rospy.logwarn("camera read failed: %s", consecutive_failures)
            if consecutive_failures >= 30:
                rospy.logerr(
                    "camera read failed 30 consecutive frames; exiting ucar_camera_bridge so /dev/video0 can be reopened"
                )
                raise RuntimeError("camera read failed 30 consecutive frames")
            time.sleep(0.05)
            continue
        consecutive_failures = 0
        pub.publish(make_image(frame, frame_id))
        rate.sleep()
    cap.release()


if __name__ == "__main__":
    main()
