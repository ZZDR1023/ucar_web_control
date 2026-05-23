#!/usr/bin/env python3
"""ROS1 person-follow adapter for ROSCAR1.

This node keeps the existing Web API and service names stable, but replaces the
underlying follow behavior with a lighter vision-based controller that is safer
and easier to tune on the real car.
"""

import json
import math
import os
import sys
import threading
import time
from collections import deque, namedtuple

import cv2
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

Detection = namedtuple("Detection", "x y w h score")
FollowCommand = namedtuple(
    "FollowCommand",
    "linear_x angular_z detected detail target_x target_y target_w target_h score",
)

STATUS_FILE = "/tmp/person_follow_adapter.json"


class PersonFollowController(object):
    def __init__(self):
        self.follow_enabled = False
        self.detect_enabled = True
        self.last_frame = None
        self.last_frame_stamp = 0.0
        self.last_scan_stamp = 0.0
        self.last_front_distance = None
        self.last_scan_ranges = []
        self.last_scan_angle_min = 0.0
        self.last_scan_angle_increment = 0.0
        self.last_scan_range_min = 0.0
        self.last_scan_range_max = 0.0
        self.last_person_scan_distance = None
        self.last_target_center_x = None
        self.last_turn_sign = 0
        self.lost_frames = 0
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.smoothed_center_error = 0.0
        self.target_center_history = deque()
        self.pending_path_center = None
        self.pending_path_frames = 0
        self.pending_turn_direction = 0
        self.turn_intent_candidate = 0
        self.turn_intent_count = 0
        self.last_turn_intent_error = 0.0
        self.turn_node_remaining_distance = 0.0
        self.turn_node_target_distance = 0.0
        self.turn_node_start_odom = None
        self.current_odom = None
        self.last_timer_time = time.time()
        self.stopped_person_frames = 0
        self.last_command = FollowCommand(0.0, 0.0, False, "未启动", -1, -1, 0, 0, 0.0)
        self.status = {
            "running": False,
            "detect_enabled": True,
            "follow_enabled": False,
            "detected": False,
            "lost_frames": 0,
            "detail": "未启动",
            "target": None,
            "front_distance": None,
            "command": {"linear_x": 0.0, "angular_z": 0.0},
            "stamp": 0.0,
        }

        self.cap = None
        self.pose_model = None
        self.pose_parse_objects = None
        self.pose_topology = None
        self.pose_mean = None
        self.pose_std = None
        self.pose_device = None
        self.pose_disabled = False
        self.yolo_net = None
        self.yolo_output_layers = []
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.36))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.90))
        self.angular_gain = float(rospy.get_param("~angular_gain", 0.85))
        self.center_deadband = float(rospy.get_param("~center_deadband", 0.16))
        self.target_height_ratio = float(rospy.get_param("~target_height_ratio", 0.62))
        self.close_height_ratio = float(rospy.get_param("~close_height_ratio", 0.78))
        self.min_height_ratio = float(rospy.get_param("~min_height_ratio", 0.16))
        self.linear_slowdown = float(rospy.get_param("~linear_slowdown", 0.65))
        self.search_angular_speed = float(rospy.get_param("~search_angular_speed", 0.18))
        self.search_frames = int(rospy.get_param("~search_frames", 12))
        self.stop_lost_frames = int(rospy.get_param("~stop_lost_frames", 20))
        self.lost_coast_frames = int(rospy.get_param("~lost_coast_frames", 6))
        self.lost_coast_max_linear = float(rospy.get_param("~lost_coast_max_linear", 0.12))
        self.low_light_threshold = float(rospy.get_param("~low_light_threshold", 75.0))
        self.low_light_alpha = float(rospy.get_param("~low_light_alpha", 1.35))
        self.low_light_beta = float(rospy.get_param("~low_light_beta", 22.0))
        self.turn_delay_height_ratio = float(rospy.get_param("~turn_delay_height_ratio", 0.30))
        self.far_turn_scale = float(rospy.get_param("~far_turn_scale", 0.18))
        self.turn_error_smoothing = float(rospy.get_param("~turn_error_smoothing", 0.70))
        self.path_follow_delay_frames = int(rospy.get_param("~path_follow_delay_frames", 8))
        self.path_follow_max_history = int(rospy.get_param("~path_follow_max_history", 18))
        self.path_follow_min_height_ratio = float(rospy.get_param("~path_follow_min_height_ratio", 0.50))
        self.path_node_stable_frames = int(rospy.get_param("~path_node_stable_frames", 3))
        self.path_node_update_error = float(rospy.get_param("~path_node_update_error", 0.12))
        self.center_lock_error = float(rospy.get_param("~center_lock_error", 0.12))
        self.turn_intent_error = float(rospy.get_param("~turn_intent_error", 0.22))
        self.turn_intent_delta = float(rospy.get_param("~turn_intent_delta", 0.05))
        self.turn_intent_frames = int(rospy.get_param("~turn_intent_frames", 2))
        self.min_turn_node_distance = float(rospy.get_param("~min_turn_node_distance", 0.08))
        self.max_turn_node_distance = float(rospy.get_param("~max_turn_node_distance", 0.30))
        self.blind_turn_linear = float(rospy.get_param("~blind_turn_linear", 0.08))
        self.stopped_person_height_ratio = float(rospy.get_param("~stopped_person_height_ratio", 0.62))
        self.stopped_person_frames_required = int(rospy.get_param("~stopped_person_frames_required", 1))
        self.obstacle_slow_margin = float(rospy.get_param("~obstacle_slow_margin", 0.05))
        self.obstacle_min_linear_scale = float(rospy.get_param("~obstacle_min_linear_scale", 0.25))
        self.publish_hz = float(rospy.get_param("~publish_hz", 8.0))
        self.smoothing = float(rospy.get_param("~smoothing", 0.45))
        self.front_stop_distance = float(rospy.get_param("~front_stop_distance", 0.35))
        self.front_sector_half_angle = math.radians(float(rospy.get_param("~front_sector_half_angle_deg", 18.0)))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 1.0))
        self.camera_horizontal_fov_deg = float(rospy.get_param("~camera_horizontal_fov_deg", 62.0))
        self.person_scan_window_deg = float(rospy.get_param("~person_scan_window_deg", 8.0))
        self.person_scan_distance_tolerance = float(rospy.get_param("~person_scan_distance_tolerance", 0.20))
        self.target_jump_reset_error = float(rospy.get_param("~target_jump_reset_error", 0.95))
        self.target_switch_area_ratio = float(rospy.get_param("~target_switch_area_ratio", 2.0))
        self.target_switch_center_error = float(rospy.get_param("~target_switch_center_error", 0.45))
        self.search_turn_hold = int(rospy.get_param("~search_turn_hold", 0))
        self.frame_width = int(rospy.get_param("~frame_width", 640))
        self.frame_height = int(rospy.get_param("~frame_height", 360))
        self.scale_factor = float(rospy.get_param("~scale_factor", 1.0))

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/person_follow/status", String, queue_size=1, latch=True)
        self.image_sub = None
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.on_scan, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.on_odom, queue_size=1)
        self.video_device = rospy.get_param("~video_device", "/dev/video0")
        self.pose_package_dir = rospy.get_param("~pose_package_dir", "/home/ucar/ucar_ws/src/person_follow")
        self.pose_weights = rospy.get_param("~pose_weights", os.path.join(self.pose_package_dir, "scripts/weights/person_detect.pth"))
        self.pose_config = rospy.get_param("~pose_config", os.path.join(self.pose_package_dir, "scripts/config/human_pose.json"))
        self.pose_input_size = int(rospy.get_param("~pose_input_size", 224))
        self.pose_height_rate = float(rospy.get_param("~pose_height_rate", 3.0))
        self.pose_width_rate = float(rospy.get_param("~pose_width_rate", 5.5))
        self.yolo_cfg = rospy.get_param("~yolo_cfg", "/home/ucar/web_panel/models/yolov3-tiny.cfg")
        self.yolo_weights = rospy.get_param("~yolo_weights", "/home/ucar/web_panel/models/yolov3-tiny.weights")
        self.yolo_input_size = int(rospy.get_param("~yolo_input_size", 416))
        self.yolo_confidence = float(rospy.get_param("~yolo_confidence", 0.35))
        self.yolo_nms_threshold = float(rospy.get_param("~yolo_nms_threshold", 0.40))
        self.detector_order = rospy.get_param("~detector_order", "pose,yolo,hog")
        self.cap = None
        self.load_pose_model()
        self.load_yolo()

        self.start_detect_srv = rospy.Service("/person_follow/start_person_detect", Trigger, self.start_detect)
        self.get_detect_info_srv = rospy.Service("/person_follow/get_detect_info", Trigger, self.get_detect_info)
        self.start_follow_srv = rospy.Service("/person_follow/start_person_follow", Trigger, self.start_follow)
        self.stop_follow_srv = rospy.Service("/person_follow/stop_person_follow", Trigger, self.stop_follow)

        self.timer = rospy.Timer(rospy.Duration(0.12), self.on_timer)
        rospy.on_shutdown(self.cleanup)

        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.90))
        self.angular_gain = float(rospy.get_param("~angular_gain", 0.85))
        self.center_deadband = float(rospy.get_param("~center_deadband", 0.16))
        self.target_height_ratio = float(rospy.get_param("~target_height_ratio", 0.62))
        self.close_height_ratio = float(rospy.get_param("~close_height_ratio", 0.78))
        self.min_height_ratio = float(rospy.get_param("~min_height_ratio", 0.16))
        self.linear_slowdown = float(rospy.get_param("~linear_slowdown", 0.65))
        self.search_angular_speed = float(rospy.get_param("~search_angular_speed", 0.18))
        self.search_frames = int(rospy.get_param("~search_frames", 12))
        self.stop_lost_frames = int(rospy.get_param("~stop_lost_frames", 20))
        self.lost_coast_frames = int(rospy.get_param("~lost_coast_frames", 6))
        self.lost_coast_max_linear = float(rospy.get_param("~lost_coast_max_linear", 0.12))
        self.low_light_threshold = float(rospy.get_param("~low_light_threshold", 75.0))
        self.low_light_alpha = float(rospy.get_param("~low_light_alpha", 1.35))
        self.low_light_beta = float(rospy.get_param("~low_light_beta", 22.0))
        self.turn_delay_height_ratio = float(rospy.get_param("~turn_delay_height_ratio", 0.30))
        self.far_turn_scale = float(rospy.get_param("~far_turn_scale", 0.18))
        self.turn_error_smoothing = float(rospy.get_param("~turn_error_smoothing", 0.70))
        self.path_follow_delay_frames = int(rospy.get_param("~path_follow_delay_frames", 8))
        self.path_follow_max_history = int(rospy.get_param("~path_follow_max_history", 18))
        self.path_follow_min_height_ratio = float(rospy.get_param("~path_follow_min_height_ratio", 0.50))
        self.path_node_stable_frames = int(rospy.get_param("~path_node_stable_frames", 3))
        self.path_node_update_error = float(rospy.get_param("~path_node_update_error", 0.12))
        self.center_lock_error = float(rospy.get_param("~center_lock_error", 0.12))
        self.turn_intent_error = float(rospy.get_param("~turn_intent_error", 0.22))
        self.turn_intent_delta = float(rospy.get_param("~turn_intent_delta", 0.05))
        self.turn_intent_frames = int(rospy.get_param("~turn_intent_frames", 2))
        self.min_turn_node_distance = float(rospy.get_param("~min_turn_node_distance", 0.08))
        self.max_turn_node_distance = float(rospy.get_param("~max_turn_node_distance", 0.30))
        self.blind_turn_linear = float(rospy.get_param("~blind_turn_linear", 0.08))
        self.stopped_person_height_ratio = float(rospy.get_param("~stopped_person_height_ratio", 0.62))
        self.stopped_person_frames_required = int(rospy.get_param("~stopped_person_frames_required", 1))
        self.obstacle_slow_margin = float(rospy.get_param("~obstacle_slow_margin", 0.05))
        self.obstacle_min_linear_scale = float(rospy.get_param("~obstacle_min_linear_scale", 0.25))
        self.publish_hz = float(rospy.get_param("~publish_hz", 8.0))
        self.smoothing = float(rospy.get_param("~smoothing", 0.45))
        self.front_stop_distance = float(rospy.get_param("~front_stop_distance", 0.35))
        self.front_sector_half_angle = math.radians(float(rospy.get_param("~front_sector_half_angle_deg", 18.0)))
        self.scan_timeout = float(rospy.get_param("~scan_timeout", 1.0))
        self.camera_horizontal_fov_deg = float(rospy.get_param("~camera_horizontal_fov_deg", 62.0))
        self.person_scan_window_deg = float(rospy.get_param("~person_scan_window_deg", 8.0))
        self.person_scan_distance_tolerance = float(rospy.get_param("~person_scan_distance_tolerance", 0.20))
        self.target_jump_reset_error = float(rospy.get_param("~target_jump_reset_error", 0.95))
        self.target_switch_area_ratio = float(rospy.get_param("~target_switch_area_ratio", 2.0))
        self.target_switch_center_error = float(rospy.get_param("~target_switch_center_error", 0.45))
        self.search_turn_hold = int(rospy.get_param("~search_turn_hold", 0))
        self.frame_width = int(rospy.get_param("~frame_width", 640))
        self.frame_height = int(rospy.get_param("~frame_height", 360))
        self.scale_factor = float(rospy.get_param("~scale_factor", 1.0))

        self.lost_coast_frames = max(1, self.lost_coast_frames)
        self.path_follow_delay_frames = max(0, self.path_follow_delay_frames)
        self.path_follow_max_history = max(self.path_follow_delay_frames + 1, self.path_follow_max_history)
        self.path_follow_min_height_ratio = max(0.0, min(1.0, self.path_follow_min_height_ratio))
        self.path_node_stable_frames = max(1, self.path_node_stable_frames)
        self.path_node_update_error = max(0.01, min(1.0, self.path_node_update_error))
        self.center_lock_error = max(0.01, min(1.0, self.center_lock_error))
        self.turn_intent_error = max(self.center_lock_error, min(1.0, self.turn_intent_error))
        self.turn_intent_delta = max(0.01, min(1.0, self.turn_intent_delta))
        self.turn_intent_frames = max(1, self.turn_intent_frames)
        self.min_turn_node_distance = max(0.0, self.min_turn_node_distance)
        self.max_turn_node_distance = max(self.min_turn_node_distance, self.max_turn_node_distance)
        self.blind_turn_linear = max(0.0, min(self.lost_coast_max_linear, self.blind_turn_linear))
        self.stopped_person_height_ratio = max(0.0, min(self.close_height_ratio, self.stopped_person_height_ratio))
        self.stopped_person_frames_required = max(1, self.stopped_person_frames_required)
        self.camera_horizontal_fov = math.radians(max(10.0, min(120.0, self.camera_horizontal_fov_deg)))
        self.person_scan_window = math.radians(max(1.0, min(30.0, self.person_scan_window_deg)))
        self.person_scan_distance_tolerance = max(0.05, min(2.0, self.person_scan_distance_tolerance))
        self.target_jump_reset_error = max(0.0, min(1.0, self.target_jump_reset_error))
        self.target_switch_area_ratio = max(1.0, self.target_switch_area_ratio)
        self.target_switch_center_error = max(0.0, min(1.0, self.target_switch_center_error))

        self.write_status("未启动")

    def reset_follow_memory(self):
        self.target_center_history.clear()
        self.pending_path_center = None
        self.pending_path_frames = 0
        self.pending_turn_direction = 0
        self.turn_intent_candidate = 0
        self.turn_intent_count = 0
        self.last_turn_intent_error = 0.0
        self.turn_node_remaining_distance = 0.0
        self.turn_node_target_distance = 0.0
        self.turn_node_start_odom = None
        self.stopped_person_frames = 0
        self.smoothed_center_error = 0.0
        self.last_target_center_x = None
        self.last_turn_sign = 0

    def start_detect(self, _req):
        self.detect_enabled = True
        self.write_status("检测已启动")
        return TriggerResponse(success=True, message=self.current_message())

    def get_detect_info(self, _req):
        payload = self.current_message()
        return TriggerResponse(success=True, message=payload)

    def start_follow(self, _req):
        self.follow_enabled = True
        self.lost_frames = 0
        self.reset_follow_memory()
        self.write_status("开始人体跟随")
        return TriggerResponse(success=True, message=self.current_message())

    def stop_follow(self, _req):
        self.follow_enabled = False
        self.publish_stop("人体跟随已停止")
        self.write_status("人体跟随已停止")
        return TriggerResponse(success=True, message=self.current_message())

    def current_message(self):
        data = dict(self.status)
        target = data.get("target")
        if isinstance(target, dict):
            data["target"] = dict(target)
        return json.dumps(data, ensure_ascii=False)

    def write_status(self, detail, detected=False, command=None, target=None):
        if command is None:
            command = self.last_command
        payload = {
            "running": bool(self.follow_enabled),
            "detect_enabled": bool(self.detect_enabled),
            "follow_enabled": bool(self.follow_enabled),
            "detected": bool(detected),
            "lost_frames": int(self.lost_frames),
            "detail": detail,
            "target": target,
            "front_distance": self.last_front_distance,
            "command": {"linear_x": float(command.linear_x), "angular_z": float(command.angular_z)},
            "stamp": time.time(),
        }
        self.status = payload
        try:
            with open(STATUS_FILE, "w") as f:
                json.dump(payload, f, ensure_ascii=False)
        except Exception:
            pass
        try:
            self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        except Exception:
            pass

    def publish_stop(self, detail):
        twist = Twist()
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.reset_follow_memory()
        self.cmd_pub.publish(twist)
        self.last_command = FollowCommand(0.0, 0.0, False, detail, -1, -1, 0, 0, 0.0)
        self.write_status(detail, detected=False, command=self.last_command)

    def pull_frame(self):
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                self.cap.set(cv2.CAP_PROP_FPS, 10)
        if self.cap is None or not self.cap.isOpened():
            self.write_status("等待相机画面")
            return None
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        self.last_frame = frame
        self.last_frame_stamp = time.time()
        return frame

    def on_odom(self, msg):
        try:
            position = msg.pose.pose.position
            self.current_odom = (float(position.x), float(position.y))
        except Exception:
            self.current_odom = None

    def on_scan(self, msg):
        min_distance = None
        if msg and msg.ranges:
            self.last_scan_ranges = list(msg.ranges)
            self.last_scan_angle_min = float(msg.angle_min)
            self.last_scan_angle_increment = float(msg.angle_increment)
            self.last_scan_range_min = float(msg.range_min)
            self.last_scan_range_max = float(msg.range_max)
            angle = msg.angle_min
            for r in msg.ranges:
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max and abs(angle) <= self.front_sector_half_angle:
                    if min_distance is None or r < min_distance:
                        min_distance = r
                angle += msg.angle_increment
        self.last_front_distance = min_distance
        self.last_scan_stamp = time.time()

    def _enhance_low_light(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_brightness = float(gray.mean()) if hasattr(gray, "mean") else self.low_light_threshold
        if mean_brightness >= self.low_light_threshold:
            return frame
        return cv2.convertScaleAbs(frame, alpha=self.low_light_alpha, beta=self.low_light_beta)

    def _resize_frame(self, frame):
        height, width = frame.shape[:2]
        target_width = self.frame_width
        if self.scale_factor and self.scale_factor != 1.0:
            target_width = max(160, int(width * self.scale_factor))
        if width == target_width:
            return frame
        target_height = max(120, int(height * (float(target_width) / float(width))))
        return cv2.resize(frame, (target_width, target_height))

    def load_pose_model(self):
        if self.pose_disabled or not (os.path.exists(self.pose_weights) and os.path.exists(self.pose_config)):
            return
        scripts_dir = os.path.join(self.pose_package_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)
        try:
            import torch
            import trt_pose.coco
            import torchvision.transforms as transforms
            from torch2trt import TRTModule
            from trt_pose.parse_objects import ParseObjects

            with open(self.pose_config, "r") as f:
                human_pose = json.load(f)
            self.pose_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.pose_topology = trt_pose.coco.coco_category_to_topology(human_pose)
            self.pose_parse_objects = ParseObjects(self.pose_topology)
            self.pose_model = TRTModule()
            self.pose_model.load_state_dict(torch.load(self.pose_weights))
            self.pose_model = self.pose_model.to(self.pose_device).eval()
            self.pose_mean = torch.Tensor([0.485, 0.456, 0.406]).to(self.pose_device)
            self.pose_std = torch.Tensor([0.229, 0.224, 0.225]).to(self.pose_device)
            self.pose_to_tensor = transforms.functional.to_tensor
        except Exception as exc:
            self.pose_model = None
            self.pose_parse_objects = None
            self.pose_disabled = True
            self.write_status("原厂人体模型加载失败，回退 OpenCV: {}".format(exc))

    def load_yolo(self):
        if not (os.path.exists(self.yolo_cfg) and os.path.exists(self.yolo_weights)):
            return
        try:
            self.yolo_net = cv2.dnn.readNetFromDarknet(self.yolo_cfg, self.yolo_weights)
            layer_names = self.yolo_net.getLayerNames()
            unconnected = self.yolo_net.getUnconnectedOutLayers()
            self.yolo_output_layers = []
            for layer in unconnected:
                index = int(layer[0]) if hasattr(layer, "__len__") else int(layer)
                self.yolo_output_layers.append(layer_names[index - 1])
        except Exception as exc:
            self.yolo_net = None
            self.yolo_output_layers = []
            self.write_status("YOLO 加载失败: {}".format(exc))

    def _preprocess_pose_frame(self, frame):
        import PIL.Image

        resized = cv2.resize(frame, (self.pose_input_size, self.pose_input_size))
        image = PIL.Image.fromarray(resized)
        tensor = self.pose_to_tensor(image).to(self.pose_device)
        tensor.sub_(self.pose_mean[:, None, None]).div_(self.pose_std[:, None, None])
        return resized, tensor[None, ...]

    def _pose_box_for_object(self, objects, peaks, object_index):
        obj = objects[0][object_index]
        if obj[11] == -1 or obj[12] == -1 or (obj[13] == -1 and obj[14] == -1):
            return None

        points = {}
        for part_index in (11, 12, 13, 14):
            peak_index = int(obj[part_index])
            if peak_index < 0:
                continue
            peak = peaks[0][part_index][peak_index]
            x = int(float(peak[1]) * self.pose_input_size)
            y = int(float(peak[0]) * self.pose_input_size)
            points[part_index] = (x, y)
        if 11 not in points or 12 not in points:
            return None
        hip_index = 13 if 13 in points else 14
        if hip_index not in points:
            return None

        shoulder_left = points[11]
        shoulder_right = points[12]
        hip = points[hip_index]
        body_height = abs(hip[1] - shoulder_left[1])
        shoulder_width = abs(shoulder_right[0] - shoulder_left[0])
        if body_height < 8 or shoulder_width < 4:
            return None
        x = int((shoulder_left[0] + shoulder_right[0]) / 2 - body_height / 2)
        y = int((shoulder_left[1] + hip[1]) / 2 - body_height / 2)
        w = int(body_height)
        h = int(body_height)
        return x, y, w, h

    def _scale_pose_box(self, box, frame_width, frame_height):
        x, y, w, h = box
        crop_y = int((frame_height - self.pose_input_size * self.pose_height_rate) / 2)
        crop_x = int((frame_width - self.pose_input_size * self.pose_width_rate) / 2)
        left = max(0, int(crop_x + x * self.pose_width_rate))
        top = max(0, int(crop_y + y * self.pose_height_rate))
        right = min(frame_width, int(crop_x + (x + w) * self.pose_width_rate + 0.5))
        bottom = min(frame_height, int(crop_y + (y + h) * self.pose_height_rate + 0.5))
        return left, top, max(0, right - left), max(0, bottom - top)

    def _detect_people_with_pose(self, frame):
        import torch

        _, tensor = self._preprocess_pose_frame(frame)
        with torch.no_grad():
            cmap, paf = self.pose_model(tensor)
        counts, objects, peaks = self.pose_parse_objects(cmap.detach().cpu(), paf.detach().cpu())
        object_count = int(counts[0]) if len(counts) else 0
        detections = []
        frame_height, frame_width = frame.shape[:2]
        for index in range(object_count):
            box = self._pose_box_for_object(objects, peaks, index)
            if box is None:
                continue
            x, y, w, h = self._scale_pose_box(box, frame_width, frame_height)
            if w < 20 or h < 40:
                continue
            detections.append(Detection(int(x), int(y), int(w), int(h), 1.0))
        return frame, detections

    def _detect_people_with_yolo(self, frame):
        resized = self._resize_frame(self._enhance_low_light(frame))
        height, width = resized.shape[:2]
        blob = cv2.dnn.blobFromImage(
            resized,
            1.0 / 255.0,
            (self.yolo_input_size, self.yolo_input_size),
            (0, 0, 0),
            swapRB=True,
            crop=False,
        )
        self.yolo_net.setInput(blob)
        outputs = self.yolo_net.forward(self.yolo_output_layers)
        boxes = []
        confidences = []
        for output in outputs:
            for row in output:
                scores = row[5:]
                class_id = int(scores.argmax()) if hasattr(scores, "argmax") else max(range(len(scores)), key=lambda idx: scores[idx])
                if class_id != 0:
                    continue
                confidence = float(scores[class_id]) * float(row[4])
                if confidence < self.yolo_confidence:
                    continue
                center_x = int(float(row[0]) * width)
                center_y = int(float(row[1]) * height)
                box_w = int(float(row[2]) * width)
                box_h = int(float(row[3]) * height)
                x = int(center_x - box_w / 2)
                y = int(center_y - box_h / 2)
                boxes.append([x, y, box_w, box_h])
                confidences.append(confidence)
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.yolo_confidence, self.yolo_nms_threshold)
        detections = []
        for item in indexes:
            idx = int(item[0]) if hasattr(item, "__len__") else int(item)
            x, y, w, h = boxes[idx]
            if w < 20 or h < 40:
                continue
            detections.append(Detection(int(x), int(y), int(w), int(h), float(confidences[idx])))
        return resized, detections

    def _detect_people_with_hog(self, frame):
        resized = self._resize_frame(self._enhance_low_light(frame))
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        rects, weights = self.hog.detectMultiScale(
            gray,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )
        detections = []
        for idx, (x, y, w, h) in enumerate(rects):
            score = float(weights[idx][0]) if idx < len(weights) else 0.0
            if score < 0.5:
                continue
            if w < 20 or h < 40:
                continue
            if h <= w:
                continue
            detections.append(Detection(int(x), int(y), int(w), int(h), score))
        return resized, detections

    def _detect_people(self, frame):
        fallback_frame = self._resize_frame(frame)
        for detector in [item.strip() for item in self.detector_order.split(",") if item.strip()]:
            if detector == "hog":
                fallback_frame, detections = self._detect_people_with_hog(frame)
                if detections:
                    return fallback_frame, detections
            elif detector == "pose" and self.pose_model is not None:
                try:
                    fallback_frame, detections = self._detect_people_with_pose(frame)
                    if detections:
                        return fallback_frame, detections
                except Exception as exc:
                    self.pose_model = None
                    self.pose_disabled = True
                    self.write_status("原厂人体模型检测失败，回退 OpenCV: {}".format(exc))
            elif detector == "yolo" and self.yolo_net is not None:
                try:
                    fallback_frame, detections = self._detect_people_with_yolo(frame)
                    if detections:
                        return fallback_frame, detections
                except Exception as exc:
                    self.yolo_net = None
                    self.write_status("YOLO 检测失败，回退 HOG: {}".format(exc))
        return fallback_frame, []

    def _select_detection(self, detections):
        if not detections:
            return None
        area_selected = max(detections, key=lambda det: det.w * det.h * (1.0 + det.score))
        if self.last_target_center_x is None:
            return area_selected

        def score(det):
            cx = det.x + det.w * 0.5
            area = det.w * det.h
            center_bias = abs(cx - self.last_target_center_x)
            return area * 0.9 + det.score * 400.0 - center_bias * 2.5

        selected = max(detections, key=score)
        selected_area = max(1.0, float(selected.w * selected.h))
        area_selected_area = float(area_selected.w * area_selected.h)
        selected_cx = selected.x + selected.w * 0.5
        selected_shift = abs(selected_cx - self.last_target_center_x) / 320.0
        if area_selected is not selected and area_selected_area >= selected_area * self.target_switch_area_ratio:
            selected = area_selected
            self.reset_follow_memory()
        elif len(detections) > 1 and selected_shift > self.target_switch_center_error:
            self.reset_follow_memory()
        return selected

    def _update_turn_intent(self, raw_center_error, height_ratio):
        direction = -1 if raw_center_error < 0.0 else 1
        delta = abs(raw_center_error) - abs(self.last_turn_intent_error)
        if abs(raw_center_error) <= self.center_lock_error:
            self.pending_turn_direction = 0
            self.turn_intent_candidate = 0
            self.turn_intent_count = 0
        elif abs(raw_center_error) >= self.turn_intent_error and delta >= self.turn_intent_delta and height_ratio < self.path_follow_min_height_ratio:
            if self.turn_intent_candidate == direction:
                self.turn_intent_count += 1
            else:
                self.turn_intent_candidate = direction
                self.turn_intent_count = 1
            if self.turn_intent_count >= self.turn_intent_frames:
                self.pending_turn_direction = direction
                self.turn_node_target_distance = self._estimate_turn_node_distance(height_ratio)
                self.turn_node_remaining_distance = self.turn_node_target_distance
                self.turn_node_start_odom = self.current_odom
        self.last_turn_intent_error = raw_center_error

    def _person_scan_distance(self, center_error):
        if not self.last_scan_ranges or not self.last_scan_angle_increment:
            return None
        target_angle = center_error * self.camera_horizontal_fov * 0.5
        distances = []
        for index, distance in enumerate(self.last_scan_ranges):
            angle = self.last_scan_angle_min + index * self.last_scan_angle_increment
            if abs(angle - target_angle) > self.person_scan_window * 0.5:
                continue
            if math.isfinite(distance) and self.last_scan_range_min <= distance <= self.last_scan_range_max:
                distances.append(float(distance))
        if not distances:
            return None
        return min(distances)

    def _estimate_turn_node_distance_from_height(self, height_ratio):
        if height_ratio >= self.path_follow_min_height_ratio:
            return self.min_turn_node_distance
        span = max(0.01, self.path_follow_min_height_ratio - self.min_height_ratio)
        far_scale = max(0.0, min(1.0, (self.path_follow_min_height_ratio - height_ratio) / span))
        return self.min_turn_node_distance + (self.max_turn_node_distance - self.min_turn_node_distance) * far_scale

    def _height_estimated_person_distance(self, height_ratio):
        turn_distance = self._estimate_turn_node_distance_from_height(height_ratio)
        return self.front_stop_distance + turn_distance

    def _scan_distance_matches_visual_height(self, scan_distance, height_ratio):
        expected = self._height_estimated_person_distance(height_ratio)
        return abs(scan_distance - expected) <= self.person_scan_distance_tolerance

    def _estimate_turn_node_distance(self, height_ratio):
        if self.last_person_scan_distance is None:
            return self._estimate_turn_node_distance_from_height(height_ratio)
        distance = self.last_person_scan_distance - self.front_stop_distance
        return max(self.min_turn_node_distance, min(self.max_turn_node_distance, distance))

    def _trail_center_error(self, cx, frame_width, height_ratio):
        current_error = (cx - frame_width * 0.5) / max(1.0, frame_width * 0.5)
        if self.target_center_history:
            previous_error = (self.target_center_history[-1] - frame_width * 0.5) / max(1.0, frame_width * 0.5)
            if abs(current_error - previous_error) > self.target_jump_reset_error:
                self.reset_follow_memory()
        if self.pending_path_center is None:
            self.pending_path_center = float(cx)
            self.pending_path_frames = 1
        else:
            pending_error = (self.pending_path_center - frame_width * 0.5) / max(1.0, frame_width * 0.5)
            if abs(current_error - pending_error) <= self.path_node_update_error:
                self.pending_path_center = (self.pending_path_center * self.pending_path_frames + float(cx)) / (self.pending_path_frames + 1)
                self.pending_path_frames += 1
            else:
                self.pending_path_center = float(cx)
                self.pending_path_frames = 1
        if self.pending_path_frames >= self.path_node_stable_frames:
            max_history = max(1, self.path_follow_max_history)
            center = float(self.pending_path_center)
            self.target_center_history.append(center)
            while len(self.target_center_history) > max_history:
                self.target_center_history.popleft()
        trail_cx = float(cx)
        if height_ratio < self.path_follow_min_height_ratio and len(self.target_center_history) > self.path_follow_delay_frames:
            trail_cx = self.target_center_history[-self.path_follow_delay_frames - 1]
        elif height_ratio < self.path_follow_min_height_ratio and self.target_center_history:
            trail_cx = self.target_center_history[0]
        error = (trail_cx - frame_width * 0.5) / max(1.0, frame_width * 0.5)
        return max(-1.0, min(1.0, error))

    def _compute_command(self, detection, frame_width, frame_height):
        cx = detection.x + detection.w * 0.5
        cy = detection.y + detection.h * 0.5
        raw_center_error = (cx - frame_width * 0.5) / max(1.0, frame_width * 0.5)
        raw_center_error = max(-1.0, min(1.0, raw_center_error))
        height_ratio = detection.h / max(1.0, float(frame_height))
        scan_distance = self._person_scan_distance(raw_center_error)
        if scan_distance is not None and self._scan_distance_matches_visual_height(scan_distance, height_ratio):
            self.last_person_scan_distance = scan_distance
        else:
            self.last_person_scan_distance = None
        self._update_turn_intent(raw_center_error, height_ratio)
        if abs(raw_center_error) <= self.center_lock_error:
            self.target_center_history.clear()
            self.pending_path_center = float(cx)
            self.pending_path_frames = self.path_node_stable_frames
            self.smoothed_center_error = 0.0
            self.prev_angular = 0.0
        center_error = self._trail_center_error(cx, frame_width, height_ratio)
        self.smoothed_center_error = (
            self.turn_error_smoothing * self.smoothed_center_error
            + (1.0 - self.turn_error_smoothing) * center_error
        )
        turn_error = self.smoothed_center_error

        angular = self.angular_gain * turn_error
        if height_ratio < self.turn_delay_height_ratio:
            turn_scale = self.far_turn_scale + (1.0 - self.far_turn_scale) * (height_ratio / max(0.01, self.turn_delay_height_ratio))
            angular *= max(self.far_turn_scale, min(1.0, turn_scale))
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
        if abs(turn_error) < self.center_deadband:
            angular = 0.0

        if height_ratio >= self.close_height_ratio:
            self.prev_linear = 0.0
            self.prev_angular = 0.0
            detail = "target too close cx={} cy={} h={:.3f}".format(int(cx), int(cy), height_ratio)
            return FollowCommand(0.0, 0.0, True, detail, int(cx), int(cy), int(detection.w), int(detection.h), float(detection.score))

        if abs(raw_center_error) <= self.center_lock_error and height_ratio >= self.stopped_person_height_ratio:
            self.stopped_person_frames += 1
        else:
            self.stopped_person_frames = 0
        if self.stopped_person_frames >= self.stopped_person_frames_required:
            self.prev_linear = 0.0
            self.prev_angular = 0.0
            detail = "target stopped cx={} cy={} h={:.3f}".format(int(cx), int(cy), height_ratio)
            return FollowCommand(0.0, 0.0, True, detail, int(cx), int(cy), int(detection.w), int(detection.h), float(detection.score))

        distance_scale = max(0.0, min(1.0, (self.target_height_ratio - height_ratio) / max(0.01, self.target_height_ratio)))
        angle_scale = max(0.20, 1.0 - abs(raw_center_error) * self.linear_slowdown)
        linear = self.max_linear_speed * distance_scale * angle_scale
        if height_ratio < self.min_height_ratio:
            linear = self.max_linear_speed * max(0.30, angle_scale)

        if self.last_front_distance is not None and self.last_front_distance < self.front_stop_distance:
            self.prev_linear = 0.0
            self.prev_angular = 0.0
            detail = "front obstacle stop cx={} cy={} d={:.2f}".format(int(cx), int(cy), self.last_front_distance)
            return FollowCommand(0.0, 0.0, True, detail, int(cx), int(cy), int(detection.w), int(detection.h), float(detection.score))
        if self.last_front_distance is not None:
            slow_start = self.front_stop_distance + self.obstacle_slow_margin
            if self.last_front_distance < slow_start:
                scale = (self.last_front_distance - self.front_stop_distance) / max(0.01, self.obstacle_slow_margin)
                scale = max(self.obstacle_min_linear_scale, min(1.0, scale))
                linear *= scale

        linear = self.smoothing * self.prev_linear + (1.0 - self.smoothing) * linear
        angular = self.smoothing * self.prev_angular + (1.0 - self.smoothing) * angular
        linear = max(0.0, min(self.max_linear_speed, linear))
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        if abs(raw_center_error) > 0.25 and linear > 0.0:
            linear = min(linear, self.max_linear_speed * 0.55)

        detail = "target cx={} cy={} err={:.3f} h={:.3f}".format(int(cx), int(cy), raw_center_error, height_ratio)
        return FollowCommand(linear, angular, True, detail, int(cx), int(cy), int(detection.w), int(detection.h), float(detection.score))

    def _scan_is_fresh(self):
        return self.last_scan_stamp > 0.0 and time.time() - self.last_scan_stamp <= self.scan_timeout

    def _compute_search_command(self):
        if self.pending_turn_direction != 0:
            angular = self.search_angular_speed * self.pending_turn_direction
        else:
            angular = self.search_angular_speed * (-1 if self.last_turn_sign < 0 else 1)
        if self.pending_turn_direction == 0 and self.last_turn_sign == 0:
            angular = self.search_angular_speed
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
        detail = "no person detection"
        return FollowCommand(0.0, angular, False, detail, -1, -1, 0, 0, 0.0)

    def _turn_node_remaining_by_odom(self):
        if self.turn_node_start_odom is None or self.current_odom is None:
            return self.turn_node_remaining_distance
        dx = self.current_odom[0] - self.turn_node_start_odom[0]
        dy = self.current_odom[1] - self.turn_node_start_odom[1]
        traveled = math.sqrt(dx * dx + dy * dy)
        return max(0.0, self.turn_node_target_distance - traveled)

    def _lost_target_linear_scale(self):
        if self.last_front_distance is None:
            return 1.0
        if self.last_front_distance < self.front_stop_distance:
            return 0.0
        slow_start = self.front_stop_distance + self.obstacle_slow_margin
        if self.last_front_distance < slow_start:
            scale = (self.last_front_distance - self.front_stop_distance) / max(0.01, self.obstacle_slow_margin)
            return max(self.obstacle_min_linear_scale, min(1.0, scale))
        return 1.0

    def on_timer(self, _event):
        frame = self.pull_frame()
        if frame is None:
            if self.follow_enabled:
                self.publish_stop("等待相机画面")
            else:
                self.write_status("等待相机画面")
            return

        if time.time() - self.last_frame_stamp > 1.2:
            if self.follow_enabled:
                self.publish_stop("相机画面超时")
            else:
                self.write_status("相机画面超时")
            return

        processed_frame, detections = self._detect_people(frame)
        detection = self._select_detection(detections)

        if detection is not None:
            if self.follow_enabled and not self._scan_is_fresh():
                self.publish_stop("激光数据超时，人体跟随已停止")
                return
            command = self._compute_command(detection, processed_frame.shape[1], processed_frame.shape[0])
            self.last_target_center_x = command.target_x
            self.last_turn_sign = 1 if command.angular_z > 0 else (-1 if command.angular_z < 0 else self.last_turn_sign)
            self.lost_frames = 0
            self.prev_linear = command.linear_x
            self.prev_angular = command.angular_z
            if self.follow_enabled:
                twist = Twist()
                twist.linear.x = command.linear_x
                twist.angular.z = command.angular_z
                self.cmd_pub.publish(twist)
            self.last_command = command
            target = {
                "x": command.target_x,
                "y": command.target_y,
                "w": command.target_w,
                "h": command.target_h,
                "score": command.score,
            }
            self.write_status(command.detail, detected=True, command=command, target=target)
            return

        now = time.time()
        elapsed = max(0.0, min(0.5, now - self.last_timer_time))
        self.last_timer_time = now
        self.lost_frames += 1
        if self.follow_enabled and self.lost_frames > self.stop_lost_frames:
            self.publish_stop("No person detection")
            return
        if self.follow_enabled and not self._scan_is_fresh():
            self.publish_stop("激光数据超时，人体跟随已停止")
            return
        if self.follow_enabled and self.last_front_distance is None:
            self.publish_stop("front scan unavailable while target lost")
            return
        if self.follow_enabled and self.last_front_distance < self.front_stop_distance:
            self.publish_stop("front obstacle stop while target lost")
            return
        if self.follow_enabled and self.pending_turn_direction != 0:
            self.turn_node_remaining_distance = self._turn_node_remaining_by_odom()
            if self.turn_node_remaining_distance > 0.0:
                linear = min(self.blind_turn_linear, self.lost_coast_max_linear) * self._lost_target_linear_scale()
                if self.turn_node_start_odom is None or self.current_odom is None:
                    self.turn_node_remaining_distance = max(0.0, self.turn_node_remaining_distance - linear * max(0.12, elapsed))
                detail = "前往转弯点" if self.turn_node_remaining_distance > 0.0 else "到达转弯点，等待重新识别"
            else:
                linear = 0.0
                detail = "到达转弯点，等待重新识别"
            if self.turn_node_remaining_distance <= 0.0:
                self.pending_turn_direction = 0
                self.turn_intent_candidate = 0
                self.turn_intent_count = 0
            command = FollowCommand(linear, 0.0, False, detail, -1, -1, 0, 0, 0.0)
            self.prev_linear = command.linear_x
            self.prev_angular = command.angular_z
            twist = Twist()
            twist.linear.x = command.linear_x
            twist.angular.z = command.angular_z
            self.cmd_pub.publish(twist)
            self.last_command = command
            self.write_status(command.detail, detected=False, command=command)
            return
        if self.follow_enabled and self.lost_frames <= self.lost_coast_frames and self.prev_linear > 0.0:
            coast_ratio = max(0.25, 1.0 - float(self.lost_frames) / float(self.lost_coast_frames + 1))
            linear = min(self.prev_linear * coast_ratio, self.lost_coast_max_linear) * self._lost_target_linear_scale()
            angular = 0.0
            command = FollowCommand(linear, angular, False, "短暂丢失目标，保持直线滑行", -1, -1, 0, 0, 0.0)
            self.prev_linear = linear
            self.prev_angular = 0.0
            twist = Twist()
            twist.linear.x = command.linear_x
            twist.angular.z = command.angular_z
            self.cmd_pub.publish(twist)
            self.last_command = command
            self.write_status(command.detail, detected=False, command=command)
            return

        if self.follow_enabled and self.lost_frames <= self.search_frames:
            command = self._compute_search_command()
            self.prev_linear = 0.0
            self.prev_angular = command.angular_z
            twist = Twist()
            twist.linear.x = command.linear_x
            twist.angular.z = command.angular_z
            self.cmd_pub.publish(twist)
            self.last_command = command
            self.write_status("正在搜索目标", detected=False, command=command)
            return

        if self.follow_enabled:
            self.publish_stop("No person detection")
        else:
            self.write_status("No person detection", detected=False, command=self.last_command)

    def cleanup(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def spin(self):
        rospy.spin()


def main():
    rospy.init_node("person_follow_adapter", anonymous=False)
    PersonFollowController().spin()


if __name__ == "__main__":
    main()
