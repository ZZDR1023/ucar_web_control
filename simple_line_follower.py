#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Minimal ROS1 HSV black-line follower.

This node is intentionally separate from the Web LineFollower. It follows only
one rule: look at the bottom ROI, find the black mask centroid, and publish a
bounded P-controller command.
"""

from __future__ import print_function

import math

import numpy as np


def clamp(value, low, high):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return low
    if not math.isfinite(value):
        return low
    return max(low, min(high, value))


def _component_boxes(mask):
    active = np.asarray(mask) > 0
    height, width = active.shape[:2]
    visited = np.zeros_like(active, dtype=bool)
    components = []
    for y, x in zip(*np.nonzero(active)):
        if visited[y, x]:
            continue
        stack = [(int(y), int(x))]
        visited[y, x] = True
        xs = []
        ys = []
        while stack:
            cy, cx = stack.pop()
            ys.append(cy)
            xs.append(cx)
            for ny in range(max(0, cy - 1), min(height, cy + 2)):
                for nx in range(max(0, cx - 1), min(width, cx + 2)):
                    if active[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((ny, nx))
        area = len(xs)
        if area:
            components.append({
                "area": area,
                "x": min(xs),
                "y": min(ys),
                "w": max(xs) - min(xs) + 1,
                "h": max(ys) - min(ys) + 1,
                "cx": float(sum(xs)) / float(area),
            })
    return components


def contour_boxes(mask):
    try:
        import cv2
    except Exception:
        return _component_boxes(mask)
    contours_info = cv2.findContours(np.asarray(mask).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
    components = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area <= 0.0:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        moments = cv2.moments(contour)
        if moments["m00"] > 0:
            cx = float(moments["m10"] / moments["m00"])
        else:
            cx = float(x) + float(w) / 2.0
        components.append({
            "area": area,
            "x": int(x),
            "y": int(y),
            "w": int(w),
            "h": int(h),
            "cx": cx,
        })
    return components


def _select_component(components, width, branch_choice):
    if not components:
        return None
    branch_choice = str(branch_choice or "left").lower()
    if branch_choice == "right":
        return max(components, key=lambda item: item["x"] + item["w"])
    if branch_choice == "largest":
        return max(components, key=lambda item: item["area"])
    if branch_choice == "center":
        center = width / 2.0
        return min(components, key=lambda item: abs(item["cx"] - center))
    return min(components, key=lambda item: item["x"])


def memory_command(found, command, last_command, lost_frames, lost_frame_limit):
    if found:
        return command[0], command[1], 0
    lost_frames = int(lost_frames) + 1
    if lost_frames <= int(lost_frame_limit):
        return last_command[0], last_command[1], lost_frames
    return 0.0, 0.0, lost_frames


def command_from_mask(mask, kp=0.005, linear_speed=0.06, max_angular=0.35, min_area=50.0, branch_choice="left", use_contours=False):
    """Return (linear_x, angular_z, found, error_px) from a binary mask."""
    mask_arr = np.asarray(mask)
    if mask_arr.ndim != 2 or mask_arr.size == 0:
        return 0.0, 0.0, False, 0.0

    width = mask_arr.shape[1]
    raw_components = contour_boxes(mask_arr) if use_contours else _component_boxes(mask_arr)
    components = [item for item in raw_components if item["area"] >= min_area]
    target = _select_component(components, width, branch_choice)
    if target is None:
        return 0.0, 0.0, False, 0.0

    cx = target["cx"]
    error = (width / 2.0) - cx
    angular = clamp(kp * error, -max_angular, max_angular)
    return float(linear_speed), angular, True, float(error)


class SimpleLineFollower(object):
    def __init__(self):
        import rospy
        from cv_bridge import CvBridge
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Image

        rospy.init_node("simple_line_follower", anonymous=True)
        self.rospy = rospy
        self.Twist = Twist
        self.bridge = CvBridge()

        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.video_device = rospy.get_param("~video_device", "")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.kp = clamp(rospy.get_param("~kp", 0.005), 0.0, 0.02)
        self.linear_speed = clamp(rospy.get_param("~linear_speed", 0.06), 0.0, 0.10)
        self.max_angular = clamp(rospy.get_param("~max_angular", 0.35), 0.0, 0.60)
        self.roi_top = clamp(rospy.get_param("~roi_top", 0.67), 0.0, 0.95)
        self.min_area = clamp(rospy.get_param("~min_area", 300.0), 1.0, 100000.0)
        self.branch_choice = str(rospy.get_param("~branch_choice", "left")).lower()
        if self.branch_choice not in ("left", "right", "largest", "center"):
            self.branch_choice = "left"
        self.lost_frame_limit = int(clamp(rospy.get_param("~lost_frame_limit", 10), 0, 60))
        self.show_debug = bool(rospy.get_param("~show_debug", False))
        self.lost_frames = 0
        self.last_command = (0.0, 0.0)

        self.lower_black = np.array([
            int(clamp(rospy.get_param("~h_min", 0), 0, 179)),
            int(clamp(rospy.get_param("~s_min", 0), 0, 255)),
            int(clamp(rospy.get_param("~v_min", 0), 0, 255)),
        ], dtype=np.uint8)
        self.upper_black = np.array([
            int(clamp(rospy.get_param("~h_max", 179), 0, 179)),
            int(clamp(rospy.get_param("~s_max", 255), 0, 255)),
            int(clamp(rospy.get_param("~v_max", 80), 0, 255)),
        ], dtype=np.uint8)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.image_sub = None
        if not self.video_device:
            self.image_sub = rospy.Subscriber(self.image_topic, Image, self.camera_callback, queue_size=1)
        rospy.on_shutdown(self.stop_robot)

        rospy.loginfo(
            "simple_line_follower: image_topic=%s video_device=%s cmd_topic=%s kp=%.4f linear=%.3f max_angular=%.3f hsv=[%s..%s]",
            self.image_topic if not self.video_device else "",
            self.video_device,
            self.cmd_topic,
            self.kp,
            self.linear_speed,
            self.max_angular,
            self.lower_black.tolist(),
            self.upper_black.tolist(),
        )

    def process_bgr(self, cv_image):
        import cv2

        height, width = cv_image.shape[:2]
        search_top = int(height * self.roi_top)
        roi = cv_image[search_top:height, 0:width]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)

        linear_x, angular_z, found, error = command_from_mask(
            mask,
            kp=self.kp,
            linear_speed=self.linear_speed,
            max_angular=self.max_angular,
            min_area=self.min_area,
            branch_choice=self.branch_choice,
            use_contours=True,
        )
        linear_x, angular_z, self.lost_frames = memory_command(
            found,
            (linear_x, angular_z),
            self.last_command,
            self.lost_frames,
            self.lost_frame_limit,
        )
        if found:
            self.last_command = (linear_x, angular_z)

        twist = self.Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

        if self.show_debug:
            moments = cv2.moments(mask)
            if found and moments["m00"] > 0:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
                cv2.circle(roi, (cx, cy), 8, (0, 0, 255), -1)
            cv2.imshow("Simple Mask (Black)", mask)
            cv2.imshow("Simple ROI", roi)
            cv2.waitKey(3)

        self.rospy.logdebug("simple_line_follower found=%s error=%.1f cmd=(%.3f, %.3f)", found, error, linear_x, angular_z)

    def camera_callback(self, data):
        from cv_bridge import CvBridgeError

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as exc:
            self.rospy.logerr("cv_bridge failed: %s", exc)
            return

        self.process_bgr(cv_image)

    def spin_video_device(self):
        import cv2

        cap = cv2.VideoCapture(self.video_device)
        if not cap.isOpened():
            self.rospy.logerr("failed to open video_device=%s", self.video_device)
            return
        rate = self.rospy.Rate(20)
        try:
            while not self.rospy.is_shutdown():
                ok, frame = cap.read()
                if ok:
                    self.process_bgr(frame)
                else:
                    self.stop_robot()
                    self.rospy.logwarn_throttle(2.0, "failed to read frame from %s", self.video_device)
                rate.sleep()
        finally:
            cap.release()

    def stop_robot(self):
        try:
            twist = self.Twist()
            self.cmd_pub.publish(twist)
        except Exception:
            pass
        if self.show_debug:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass

    def spin(self):
        if self.video_device:
            self.spin_video_device()
        else:
            self.rospy.spin()


if __name__ == "__main__":
    SimpleLineFollower().spin()
