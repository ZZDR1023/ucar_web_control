#!/usr/bin/env python3
"""Low-latency Flask web control panel for UCAR robot using rospy."""

import io
import json
import math
import os
import signal
import shlex
import subprocess
import threading
import time

from flask import Flask, Response, jsonify, request, render_template, send_file

import rospy
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import BatteryState, Image, LaserScan

try:
    from patrol import PatrolManager, safe_name
except ImportError:
    from web_panel.patrol import PatrolManager, safe_name

try:
    from line_follow import LineFollowConfig, LineFollower, decode_jpeg, encode_jpeg
except ImportError:
    from web_panel.line_follow import LineFollowConfig, LineFollower, decode_jpeg, encode_jpeg

try:
    from line_record import LineFollowRecorder
except ImportError:
    from web_panel.line_record import LineFollowRecorder

try:
    from voice_control import build_route_point_aliases, odom_target_for_step, parse_voice_command
except ImportError:
    from web_panel.voice_control import build_route_point_aliases, odom_target_for_step, parse_voice_command

app = Flask(__name__)

MANUAL_MODE = True
CURRENT_LINEAR_X = 0.0
CURRENT_ANGULAR_Z = 0.0
lock = threading.Lock()
latest_odom = {"x": 0.0, "y": 0.0, "yaw": 0.0, "linear": 0.0, "angular": 0.0, "stamp": 0.0}
latest_pose = {"seen": False, "stamp": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0}
latest_battery = {
    "seen": False,
    "stamp": 0.0,
    "voltage": None,
    "percentage": None,
    "display_percentage": None,
    "age": None,
    "stale": True,
    "low": False,
}
latest_goal = None
latest_move_base_status = {"code": None, "text": "no status"}
latest_scan = {"seen": False, "stamp": 0.0, "range_min": 0.0, "range_max": 0.0, "sample_count": 0}
latest_scan_points = []
latest_forward_obstacle = {
    "blocked": False,
    "caution": False,
    "stop_active": False,
    "min_distance": None,
    "box_min_distance": None,
    "stop_distance": 0.35,
    "clear_distance": 0.55,
    "half_angle_deg": 18.0,
    "safety_box": {"front": 0.50, "rear": 0.02, "half_width": 0.30, "stop_distance": 0.35},
}
latest_live_map = None
latest_goal_distance = None
latest_nav_state = {
    "move_base": False,
    "amcl": False,
    "map_server": False,
    "nodes": [],
}
pending_goal = None
nav_start_in_progress = False
last_goal_publish_time = 0.0
auto_reissue_count = 0
nav_tuning_applied = False
latest_nav_tuning = {"ok": False, "detail": "not applied"}
latest_person_follow = {"running": False, "pid": None, "detail": "未启动"}
person_follow_probe = {"stamp": 0.0, "pid": None}
person_follow_start_lock = threading.Lock()
person_follow_start_generation = 0
GOAL_REISSUE_DISTANCE = 0.12
GOAL_REISSUE_INTERVAL = 2.0
PERSON_FOLLOW_COMMAND = "python3 -u /home/ucar/web_panel/person_follow_adapter.py"
PERSON_FOLLOW_STATUS_FILE = "/tmp/person_follow_adapter.json"
PERSON_FOLLOW_USES_DIRECT_CAMERA = "person_follow_adapter.py" in PERSON_FOLLOW_COMMAND
UCAR_CAMERA_COMMAND = (
    "python3 -u /home/ucar/web_panel/ucar_camera_bridge.py "
    "_device:=/dev/video0 _topic:=/ucar_camera/image_raw "
    "_width:=1280 _height:=720 _rate:=10"
)
MAX_MANUAL_LINEAR_SPEED = 0.10
MAX_MANUAL_ANGULAR_SPEED = 0.50
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PREVIEW_CANDIDATES = [
    os.path.abspath(os.path.join(BASE_DIR, "..", "maps", "ucar_map_20260501_202713_sealed_preview.png")),
    "/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed_preview.png",
    "/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_preview.png",
    os.path.abspath(os.path.join(BASE_DIR, "..", "maps", "ucar_map_20260501_202713_preview.png")),
]
MAP_INFO = {
    "resolution": 0.05,
    "origin": [-10.435738, -11.509830, 0.0],
    "width": 453,
    "height": 430,
}
FRONT_STOP_DISTANCE = 0.35
FRONT_CLEAR_DISTANCE = 0.55
FRONT_STOP_HALF_ANGLE = math.radians(18)
SAFETY_BOX_FRONT = 0.50
SAFETY_BOX_REAR = 0.02
SAFETY_BOX_HALF_WIDTH = 0.30
SAFETY_BOX_STOP_DISTANCE = 0.35
BATTERY_STALE_SECONDS = 10.0
camera_lock = threading.Lock()
latest_camera_jpeg = None
latest_camera_stamp = 0.0
camera_clients = 0
line_follow_lock = threading.Lock()
line_follow_config = LineFollowConfig()
line_follower = LineFollower(line_follow_config)
latest_line_follow_result = line_follower.last_result
latest_line_follow_debug_jpeg = None
latest_line_follow_control = {"enabled": False, "active": False, "reason": "未启用"}
LINE_FOLLOW_INTERVAL = 0.12
PATROL_ROUTE_PATH = os.path.join(BASE_DIR, "patrol_route.json")
PATROL_ROUTES_PATH = os.path.join(BASE_DIR, "patrol_routes.json")
PATROL_RUNS_PATH = os.path.join(BASE_DIR, "patrol_runs.json")
PATROL_CAPTURE_DIR = os.path.join(BASE_DIR, "patrol_captures")
LINE_FOLLOW_RECORD_DIR = os.path.join(BASE_DIR, "line_follow_records")
PATROL_REACHED_DISTANCE = 0.15
PATROL_STATUS_REACHED_DISTANCE = 0.30
PATROL_POINT_WAIT_SECONDS = 3.0
patrol_manager = PatrolManager(PATROL_ROUTE_PATH, routes_path=PATROL_ROUTES_PATH, runs_path=PATROL_RUNS_PATH)
line_follow_recorder = LineFollowRecorder(LINE_FOLLOW_RECORD_DIR)
patrol_goal_publish_time = 0.0
voice_sequence_cancel = threading.Event()
voice_sequence_lock = threading.Lock()
voice_sequence_running = False

rospy.init_node("web_panel_server", anonymous=False, disable_signals=True)
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)


def odom_cb(msg):
    q = msg.pose.pose.orientation
    with lock:
        latest_odom["x"] = msg.pose.pose.position.x
        latest_odom["y"] = msg.pose.pose.position.y
        latest_odom["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        latest_odom["linear"] = msg.twist.twist.linear.x
        latest_odom["angular"] = msg.twist.twist.angular.z
        latest_odom["stamp"] = time.time()


rospy.Subscriber("/odom", Odometry, odom_cb)


def battery_cb(msg):
    global latest_battery
    percentage = msg.percentage if math.isfinite(msg.percentage) else None
    voltage = msg.voltage if math.isfinite(msg.voltage) and msg.voltage > 0.0 else None
    display_percentage = None
    if percentage is not None and percentage >= 0.0:
        # ROS standard is 0.0-1.0, but some UCAR images publish 0-100.
        display_percentage = percentage * 100.0 if percentage <= 1.0 else percentage
        display_percentage = max(0.0, min(100.0, display_percentage))
    latest_battery = {
        "seen": True,
        "stamp": time.time(),
        "voltage": voltage,
        "percentage": percentage,
        "display_percentage": display_percentage,
        "age": 0.0,
        "stale": False,
        "low": display_percentage is not None and display_percentage < 20.0,
    }


rospy.Subscriber("/battery_state", BatteryState, battery_cb)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def amcl_cb(msg):
    global last_goal_publish_time
    msg_to_publish = None
    with lock:
        latest_pose["seen"] = True
        latest_pose["stamp"] = time.time()
        latest_pose["x"] = msg.pose.pose.position.x
        latest_pose["y"] = msg.pose.pose.position.y
        latest_pose["yaw"] = quat_to_yaw(msg.pose.pose.orientation)
        if (not MANUAL_MODE) and pending_goal is not None and time.time() - last_goal_publish_time > 1.0:
            msg_to_publish = pending_goal
    if msg_to_publish is not None:
        for _ in range(3):
            msg_to_publish.header.stamp = rospy.Time.now()
            goal_pub.publish(msg_to_publish)
            last_goal_publish_time = time.time()
            time.sleep(0.05)


def move_base_status_cb(msg):
    global latest_move_base_status
    if msg.status_list:
        status = msg.status_list[-1]
        latest_move_base_status = {"code": status.status, "text": status.text}
    else:
        latest_move_base_status = {"code": None, "text": "no active move_base goal"}


def scan_cb(msg):
    global latest_scan, latest_scan_points, latest_forward_obstacle
    points = []
    front_min = None
    box_min = None
    was_blocked = latest_forward_obstacle.get("blocked", False)
    with lock:
        pose = dict(latest_pose)
    angle = msg.angle_min
    step = max(1, len(msg.ranges) // 240)
    for idx, distance in enumerate(msg.ranges):
        if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
            angle += msg.angle_increment
            continue
        if abs(angle) <= FRONT_STOP_HALF_ANGLE:
            front_min = distance if front_min is None else min(front_min, distance)
        local_x = math.cos(angle) * distance
        local_y = math.sin(angle) * distance
        if -SAFETY_BOX_REAR <= local_x <= SAFETY_BOX_FRONT and abs(local_y) <= SAFETY_BOX_HALF_WIDTH:
            box_min = distance if box_min is None else min(box_min, distance)
        if idx % step == 0 and distance <= 5.0:
            world_angle = pose["yaw"] + angle
            points.append({
                "x": pose["x"] + math.cos(world_angle) * distance,
                "y": pose["y"] + math.sin(world_angle) * distance,
            })
        angle += msg.angle_increment
    latest_scan_points = points
    front_blocked = front_min is not None and front_min < (FRONT_CLEAR_DISTANCE if was_blocked else FRONT_STOP_DISTANCE)
    box_blocked = box_min is not None and box_min < (FRONT_CLEAR_DISTANCE if was_blocked else SAFETY_BOX_STOP_DISTANCE)
    blocked = front_blocked or box_blocked
    latest_forward_obstacle = {
        "blocked": blocked,
        "caution": (front_min is not None and front_min < 0.8) or box_min is not None,
        "stop_active": blocked,
        "min_distance": front_min,
        "box_min_distance": box_min,
        "stop_distance": FRONT_STOP_DISTANCE,
        "clear_distance": FRONT_CLEAR_DISTANCE,
        "half_angle_deg": math.degrees(FRONT_STOP_HALF_ANGLE),
        "safety_box": {
            "front": SAFETY_BOX_FRONT,
            "rear": SAFETY_BOX_REAR,
            "half_width": SAFETY_BOX_HALF_WIDTH,
            "stop_distance": SAFETY_BOX_STOP_DISTANCE,
        },
    }
    latest_scan = {
        "seen": True,
        "stamp": time.time(),
        "range_min": msg.range_min,
        "range_max": msg.range_max,
        "sample_count": len(msg.ranges),
    }


def map_cb(msg):
    global latest_live_map, MAP_INFO
    data = list(msg.data)
    latest_live_map = {
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "origin": [
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            quat_to_yaw(msg.info.origin.orientation),
        ],
        "data": data,
        "stamp": time.time(),
    }
    MAP_INFO = {
        "resolution": msg.info.resolution,
        "origin": [
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            quat_to_yaw(msg.info.origin.orientation),
        ],
        "width": msg.info.width,
        "height": msg.info.height,
    }


rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_cb)
rospy.Subscriber("/move_base/status", GoalStatusArray, move_base_status_cb)
rospy.Subscriber("/scan", LaserScan, scan_cb)
rospy.Subscriber("/map", OccupancyGrid, map_cb)


def offline_camera_jpeg():
    from PIL import Image, ImageDraw
    img = Image.new("RGB", (640, 480), (0, 0, 0))
    d = ImageDraw.Draw(img)
    d.text((20, 20), "Camera offline", fill=(255, 255, 255))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    return buf.getvalue()


def camera_loop():
    global latest_camera_jpeg, latest_camera_stamp
    try:
        import cv2
    except Exception:
        latest_camera_jpeg = offline_camera_jpeg()
        latest_camera_stamp = time.time()
        return
    cap = None
    while not rospy.is_shutdown():
        with camera_lock:
            active = camera_clients > 0
        with line_follow_lock:
            line_follow_active = line_follow_config.enabled
        active = active or line_follow_active or line_follow_recorder.snapshot().get("active", False)
        if not active:
            if cap is not None:
                cap.release()
                cap = None
            time.sleep(0.2)
            continue
        try:
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture("/dev/ucar_video", cv2.CAP_V4L2)
                if not cap.isOpened():
                    cap = cv2.VideoCapture("/dev/ucar_video")
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc("M", "J", "P", "G"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            ret, frame = cap.read()
            if ret and frame is not None:
                ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    with camera_lock:
                        latest_camera_jpeg = jpeg.tobytes()
                        latest_camera_stamp = time.time()
            else:
                time.sleep(0.1)
        except Exception:
            if cap is not None:
                cap.release()
                cap = None
            time.sleep(0.5)
        time.sleep(0.08)


threading.Thread(target=camera_loop, daemon=True).start()


def release_web_camera_for_person_follow():
    global camera_clients
    with camera_lock:
        camera_clients = 0
    with line_follow_lock:
        line_follow_active = line_follow_config.enabled
    recorder_active = line_follow_recorder.snapshot().get("active", False)
    if line_follow_active or recorder_active:
        disable_line_follow("启动人体跟随，释放摄像头")
    if recorder_active:
        line_follow_recorder.stop()
    time.sleep(0.8)
    hard_release_video_device_for_person_follow()


def hard_release_video_device_for_person_follow():
    current_pid = os.getpid()
    command = (
        "for pid in $(fuser /dev/video0 /dev/ucar_video 2>/dev/null); do "
        "if [ \"$pid\" != \"{pid}\" ]; then kill -9 \"$pid\" || true; fi; "
        "done; "
        "pkill -9 -f '[u]car_camera.py|[u]car_camera_bridge.py|[u]car_camera' || true"
    ).format(pid=current_pid)
    run_shell(command, timeout=3)
    time.sleep(1.5)


def start_person_follow_camera_bridge():
    run_shell("pkill -f '[u]car_camera_bridge.py|[u]car_camera.py|[u]car_camera' || true", timeout=3)
    command = (
        ros_shell_prefix() + "; "
        "cd /home/ucar/web_panel; "
        "nohup python3 -u /home/ucar/web_panel/ucar_camera_bridge.py "
        "_device:=/dev/video0 _topic:=/ucar_camera/image_raw _width:=1280 _height:=720 _rate:=10 "
        ">/tmp/ucar_camera_bridge.log 2>&1 < /dev/null & true"
    )
    run_shell(command, timeout=5)


def stop_person_follow_camera_bridge():
    run_shell("pkill -f '[u]car_camera_bridge.py' || true", timeout=3)


def person_follow_camera_bridge_ready(max_age=1.5):
    return wait_for_camera_message(timeout=1.5)


def disable_line_follow(reason="已切换到其他控制"):
    global line_follow_config, latest_line_follow_control
    with line_follow_lock:
        line_follow_config = LineFollowConfig.from_dict({"enabled": False}, line_follow_config)
        line_follower.update_config(line_follow_config)
        latest_line_follow_control = {"enabled": False, "active": False, "reason": reason}


def line_follow_snapshot():
    with line_follow_lock:
        return {
            "config": line_follow_config.to_dict(),
            "result": latest_line_follow_result.to_dict(),
            "control": dict(latest_line_follow_control),
            "recording": line_follow_recorder.snapshot(),
        }


def line_follow_record_metadata(camera_stamp):
    with lock:
        cmd = {"linear_x": CURRENT_LINEAR_X, "angular_z": CURRENT_ANGULAR_Z}
        odom = dict(latest_odom)
        pose = dict(latest_pose)
    return {
        "camera_stamp": camera_stamp,
        "cmd": cmd,
        "odom": odom,
        "pose": pose,
        "line_follow": {
            "config": line_follow_config.to_dict(),
            "result": latest_line_follow_result.to_dict(),
            "control": dict(latest_line_follow_control),
        },
        "forward_obstacle": dict(latest_forward_obstacle),
    }


def process_line_follow_frame():
    global latest_line_follow_result, latest_line_follow_debug_jpeg, latest_line_follow_control
    with camera_lock:
        frame_jpeg = latest_camera_jpeg
        camera_stamp = latest_camera_stamp
    if frame_jpeg is None:
        with line_follow_lock:
            latest_line_follow_result = line_follower.process(None)
            latest_line_follow_control = {
                "enabled": line_follow_config.enabled,
                "active": False,
                "reason": "没有摄像头画面",
            }
        return line_follow_snapshot()
    try:
        frame = decode_jpeg(frame_jpeg)
    except Exception as exc:
        with line_follow_lock:
            latest_line_follow_control = {
                "enabled": line_follow_config.enabled,
                "active": False,
                "reason": "摄像头画面解码失败: {}".format(exc),
            }
        return line_follow_snapshot()
    with line_follow_lock:
        line_follower.update_config(line_follow_config)
        latest_line_follow_result = line_follower.process(frame)
        if line_follower.last_debug_frame is not None:
            latest_line_follow_debug_jpeg = encode_jpeg(line_follower.last_debug_frame, quality=78)
        latest_line_follow_control = {
            "enabled": line_follow_config.enabled,
            "active": line_follow_config.enabled and latest_line_follow_result.detected,
            "reason": latest_line_follow_result.message,
            "camera_stamp": camera_stamp,
        }
        debug_jpeg = latest_line_follow_debug_jpeg
        metadata = line_follow_record_metadata(camera_stamp)
    line_follow_recorder.record_sample(frame_jpeg, debug_jpeg, metadata)
    return line_follow_snapshot()


def line_follow_loop():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z, latest_line_follow_control
    while not rospy.is_shutdown():
        with line_follow_lock:
            enabled = line_follow_config.enabled
        if not enabled:
            time.sleep(0.2)
            continue
        if latest_forward_obstacle.get("blocked"):
            with lock:
                MANUAL_MODE = True
                CURRENT_LINEAR_X = 0.0
                CURRENT_ANGULAR_Z = 0.0
            cmd_pub.publish(Twist())
            disable_line_follow("前方障碍触发，巡线已停止")
            time.sleep(0.2)
            continue
        snapshot = process_line_follow_frame()
        result = snapshot["result"]
        with lock:
            MANUAL_MODE = True
            CURRENT_LINEAR_X = result["linear_x"]
            CURRENT_ANGULAR_Z = result["angular_z"]
        time.sleep(LINE_FOLLOW_INTERVAL)


threading.Thread(target=line_follow_loop, daemon=True).start()


def line_follow_record_loop():
    while not rospy.is_shutdown():
        if line_follow_recorder.snapshot().get("active", False):
            process_line_follow_frame()
            time.sleep(0.25)
        else:
            time.sleep(0.2)


threading.Thread(target=line_follow_record_loop, daemon=True).start()


def cmd_loop():
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        with lock:
            manual_mode = MANUAL_MODE
            msg.linear.x = CURRENT_LINEAR_X
            msg.angular.z = CURRENT_ANGULAR_Z
        if manual_mode and person_follow_process_running_cached() is None:
            try:
                cmd_pub.publish(msg)
            except rospy.ROSException:
                break
        rate.sleep()


threading.Thread(target=cmd_loop, daemon=True).start()


def ros_env_exports():
    ros_master_uri = os.environ.get("ROS_MASTER_URI", "")
    ros_ip = os.environ.get("ROS_IP", "")
    parts = []
    if ros_master_uri:
        parts.append("export ROS_MASTER_URI={}".format(shlex.quote(ros_master_uri)))
    if ros_ip:
        parts.append("export ROS_IP={}".format(shlex.quote(ros_ip)))
    return "; ".join(parts)


def ros_shell_prefix(workspace="/home/ucar/ucar_ws/devel/setup.bash"):
    parts = ["source /opt/ros/melodic/setup.bash"]
    if workspace:
        parts.append("source {}".format(shlex.quote(workspace)))
    exports = ros_env_exports()
    if exports:
        parts.append(exports)
    return "; ".join(parts)


def run_shell(command, timeout=8):
    process = None
    try:
        process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            start_new_session=True,
        )
        stdout, stderr = process.communicate(timeout=timeout)
        return {
            "stdout": stdout.strip(),
            "stderr": stderr.strip(),
            "returncode": process.returncode,
        }
    except subprocess.TimeoutExpired:
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except Exception:
                pass
        return {"stdout": "", "stderr": "timeout", "returncode": 124}
    except Exception as exc:
        return {"stdout": "", "stderr": str(exc), "returncode": 1}


def person_follow_process_running():
    result = run_shell(
        "pgrep -f '[p]erson_follow.py|[p]erson_follow_node|[p]erson_follow_adapter.py' | head -n 1",
        timeout=3,
    )
    if result["returncode"] == 0 and result["stdout"].strip():
        try:
            return int(result["stdout"].splitlines()[0].strip())
        except (TypeError, ValueError):
            return None
    return None


def person_follow_process_running_cached(max_age=2.0):
    now = time.time()
    with lock:
        if now - person_follow_probe.get("stamp", 0.0) < max_age:
            return person_follow_probe.get("pid")
    pid = person_follow_process_running()
    with lock:
        person_follow_probe["stamp"] = now
        person_follow_probe["pid"] = pid
    return pid


def ucar_camera_process_running():
    result = run_shell(
        "pgrep -f '[u]car_camera.py|[u]car_camera' | head -n 1",
        timeout=3,
    )
    return result["returncode"] == 0 and bool(result["stdout"].strip())


def start_ucar_camera_for_person_follow():
    run_shell("pkill -f '[u]car_camera.py|[u]car_camera' || true", timeout=3)
    start_person_follow_camera_bridge()


def wait_for_ros_name(pattern, timeout=8.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        result = run_shell(
            ros_shell_prefix() + "; "
            "rostopic list 2>/dev/null | grep -E '{}' || true; "
            "rosservice list 2>/dev/null | grep -E '{}' || true".format(pattern, pattern),
            timeout=4,
        )
        if result["stdout"].strip():
            return True
        time.sleep(0.4)
    return False


def wait_for_camera_message(timeout=8.0):
    try:
        rospy.wait_for_message("/ucar_camera/image_raw", Image, timeout=timeout)
        return True
    except rospy.ROSException:
        return False


def wait_for_person_follow_ready(timeout=60.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not person_follow_process_running():
            time.sleep(0.4)
            continue
        result = run_shell(
            ros_shell_prefix() + "; "
            "rosservice list 2>/dev/null | grep -q '^/person_follow/start_person_detect$' && "
            "rosservice list 2>/dev/null | grep -q '^/person_follow/start_person_follow$'",
            timeout=8,
        )
        if result["returncode"] == 0:
            return True
        time.sleep(1.0)
    return False


def wait_for_person_follow_stopped(timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not person_follow_process_running():
            return True
        time.sleep(0.2)
    return person_follow_process_running() is None


def start_person_follow_services(timeout=20.0, generation=None):
    deadline = time.time() + timeout
    details = []
    while time.time() < deadline:
        with lock:
            if generation is not None and generation != person_follow_start_generation:
                return False, "人体跟随启动已取消"
        detect = run_shell(
            ros_shell_prefix() + "; "
            "rosservice call /person_follow/start_person_detect '{}'",
            timeout=8,
        )
        follow = run_shell(
            ros_shell_prefix() + "; "
            "rosservice call /person_follow/start_person_follow '{}'",
            timeout=8,
        )
        details = []
        for label, result in (("detect", detect), ("follow", follow)):
            text = (result["stdout"] or result["stderr"] or "").strip()
            if text:
                details.append("{}: {}".format(label, text.replace("\n", " ")[:160]))
        adapter_status = person_follow_adapter_status()
        follow_enabled = bool(adapter_status and adapter_status.get("follow_enabled"))
        with lock:
            canceled = generation is not None and generation != person_follow_start_generation
        if canceled:
            return False, "人体跟随启动已取消"
        if follow["returncode"] == 0 and follow_enabled:
            return True, "; ".join(details)
        time.sleep(1.0)
    return False, "; ".join(details)


def person_follow_adapter_status():
    try:
        if not os.path.exists(PERSON_FOLLOW_STATUS_FILE):
            return None
        if time.time() - os.path.getmtime(PERSON_FOLLOW_STATUS_FILE) > 3.0:
            return None
        with open(PERSON_FOLLOW_STATUS_FILE, "r") as f:
            return json.load(f)
    except (IOError, OSError, ValueError):
        return None


def person_follow_snapshot():
    pid = person_follow_process_running_cached()
    adapter_status = person_follow_adapter_status()
    with lock:
        was_running = latest_person_follow.get("running")
        latest_person_follow["running"] = pid is not None
        latest_person_follow["pid"] = pid
        if adapter_status and pid is not None:
            latest_person_follow["detail"] = adapter_status.get("detail") or "运行中"
            latest_person_follow["detected"] = bool(adapter_status.get("detected"))
            latest_person_follow["target"] = adapter_status.get("target")
            latest_person_follow["command"] = adapter_status.get("command")
            latest_person_follow["lost_frames"] = adapter_status.get("lost_frames")
        elif pid is not None and latest_person_follow.get("detail") in ("未启动", "已停止"):
            latest_person_follow["detail"] = "运行中"
        if pid is None and was_running:
            latest_person_follow["detail"] = "已停止"
        return dict(latest_person_follow)


def stop_person_follow(reason="停止人体跟随", cancel_start=True, update_state=True):
    global latest_person_follow, person_follow_start_generation
    if cancel_start:
        with lock:
            person_follow_start_generation += 1
    cmd_pub.publish(Twist())
    run_shell("pkill -f '[p]erson_follow.py|[p]erson_follow_node|[p]erson_follow_adapter.py' || true", timeout=3)
    stop_person_follow_camera_bridge()
    if not update_state:
        with lock:
            person_follow_probe["stamp"] = time.time()
            person_follow_probe["pid"] = None
            return dict(latest_person_follow)
    with lock:
        latest_person_follow = {"running": False, "pid": None, "detail": reason}
        person_follow_probe["stamp"] = time.time()
        person_follow_probe["pid"] = None
        return dict(latest_person_follow)


def stop_person_follow_if_active(reason="停止人体跟随"):
    with lock:
        detail = latest_person_follow.get("detail") or ""
        active = (
            latest_person_follow.get("running")
            or latest_person_follow.get("pid") is not None
            or detail.startswith("启动中")
        )
        snapshot = dict(latest_person_follow)
    if not active:
        return snapshot
    return stop_person_follow(reason)


def person_follow_start_canceled(generation):
    with lock:
        return generation != person_follow_start_generation


def start_person_follow_worker(generation):
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z, latest_person_follow
    if not person_follow_start_lock.acquire(False):
        return
    try:
        disable_line_follow("启动人体跟随")
        patrol_manager.stop("person follow started")
        cancel_pub.publish(GoalID())
        with lock:
            MANUAL_MODE = True
            CURRENT_LINEAR_X = 0.0
            CURRENT_ANGULAR_Z = 0.0
        cmd_pub.publish(Twist())
        stop_person_follow("重启人体跟随", cancel_start=False, update_state=False)
        wait_for_person_follow_stopped()
        if person_follow_start_canceled(generation):
            return
        release_web_camera_for_person_follow()
        if person_follow_start_canceled(generation):
            return
        camera_ready = True
        if not PERSON_FOLLOW_USES_DIRECT_CAMERA:
            start_ucar_camera_for_person_follow()
            camera_ready = wait_for_camera_message(timeout=10.0)
            if not camera_ready:
                stop_person_follow_camera_bridge()
                with lock:
                    latest_person_follow = {
                        "running": False,
                        "pid": None,
                        "detail": "相机话题 /ucar_camera/image_raw 未就绪，查看 /tmp/ucar_camera_bridge.log",
                    }
                    person_follow_probe["stamp"] = time.time()
                    person_follow_probe["pid"] = None
                return
        if person_follow_start_canceled(generation):
            return
        command = (
            ros_shell_prefix() + "; "
            "nohup bash -lc 'exec {}' >/tmp/person_follow.log 2>&1 < /dev/null & true"
        ).format(PERSON_FOLLOW_COMMAND)
        result = run_shell(command, timeout=5)
        if person_follow_start_canceled(generation):
            stop_person_follow("人体跟随启动已取消", cancel_start=False)
            return
        ready = wait_for_person_follow_ready()
        canceled = person_follow_start_canceled(generation)
        service_ok = False
        service_detail = ""
        if ready and camera_ready and not canceled:
            service_ok, service_detail = start_person_follow_services(generation=generation)
        pid = person_follow_process_running()
        canceled = person_follow_start_canceled(generation)
        if canceled:
            stop_person_follow("人体跟随启动已取消", cancel_start=False)
            return
        if pid is not None and ready and camera_ready and service_ok:
            with lock:
                MANUAL_MODE = False
                CURRENT_LINEAR_X = 0.0
                CURRENT_ANGULAR_Z = 0.0
        with lock:
            person_follow_probe["stamp"] = time.time()
            person_follow_probe["pid"] = pid
            latest_person_follow = {
                "running": pid is not None,
                "pid": pid,
                "detail": (
                    "运行中，已请求检测/跟随"
                    if pid is not None and ready and camera_ready and service_ok
                    else (
                        "相机话题 /ucar_camera/image_raw 未就绪{}".format(
                            "，查看 /tmp/ucar_camera_bridge.log"
                        )
                        if not camera_ready
                        else (
                            "人体跟随服务未就绪"
                            if not ready
                            else (service_detail or result["stderr"] or result["stdout"] or "启动失败，查看 /tmp/person_follow.log")
                        )
                    )
                ),
            }
    finally:
        person_follow_start_lock.release()


def start_person_follow_async():
    global person_follow_start_generation
    with lock:
        if latest_person_follow.get("detail") == "启动中，等待原厂人体跟随节点就绪":
            return dict(latest_person_follow)
        person_follow_start_generation += 1
        generation = person_follow_start_generation
        latest_person_follow.update({"running": False, "pid": None, "detail": "启动中，等待原厂人体跟随节点就绪"})
        person_follow_probe["stamp"] = time.time()
        person_follow_probe["pid"] = None
        snapshot = dict(latest_person_follow)
    threading.Thread(target=start_person_follow_worker, args=(generation,), daemon=True).start()
    return snapshot


def kill_nav_processes():
    run_shell("pkill -9 -f move_base || true")
    run_shell("pkill -9 -f amcl || true")
    run_shell("pkill -9 -f map_server || true")
    run_shell("pkill -9 -f cartographer || true")


def nav_nodes_running():
    result = run_shell(
        ros_shell_prefix("/home/ucar/nav_clean_ws/devel/setup.bash") + " && "
        "rosnode list 2>/dev/null | grep -E '^/(move_base|amcl|map_server)$' || true",
        timeout=6,
    )
    listed = [line.strip() for line in result["stdout"].splitlines() if line.strip()]
    nodes = []
    for node in ["/move_base", "/amcl", "/map_server"]:
        if node not in listed:
            continue
        ping = run_shell(
            ros_shell_prefix(None) + " && "
            "timeout 2 rosnode ping -c 1 {} >/dev/null 2>&1".format(node),
            timeout=3,
        )
        if ping["returncode"] == 0:
            nodes.append(node)
    return {
        "move_base": "/move_base" in nodes,
        "amcl": "/amcl" in nodes,
        "map_server": "/map_server" in nodes,
        "nodes": nodes,
    }


def apply_nav_tuning():
    # Keep XY precision tighter, but relax final yaw to reduce in-place spinning.
    return run_shell(
        ros_shell_prefix("/home/ucar/nav_clean_ws/devel/setup.bash") + " && "
        "rosparam set /move_base/DWAPlannerROS/xy_goal_tolerance 0.08 && "
        "rosparam set /move_base/DWAPlannerROS/yaw_goal_tolerance 0.35 && "
        "rosparam set /move_base/DWAPlannerROS/latch_xy_goal_tolerance true && "
        "rosparam set /move_base/DWAPlannerROS/max_vel_x 0.18 && "
        "rosparam set /move_base/DWAPlannerROS/max_trans_vel 0.18 && "
        "rosparam set /move_base/DWAPlannerROS/min_vel_x 0.02 && "
        "rosparam set /move_base/DWAPlannerROS/max_rot_vel 0.8 && "
        "rosparam set /move_base/local_costmap/inflation_layer/inflation_radius 0.35 && "
        "rosparam set /move_base/local_costmap/inflation_layer/cost_scaling_factor 3.0 && "
        "rosparam set /move_base/global_costmap/inflation_layer/inflation_radius 0.35 && "
        "rosparam set /move_base/global_costmap/inflation_layer/cost_scaling_factor 3.0",
        timeout=12,
    )


def nav_state_loop():
    global latest_nav_state, nav_tuning_applied, latest_nav_tuning
    while not rospy.is_shutdown():
        latest_nav_state = nav_nodes_running()
        if (
            not nav_tuning_applied
            and latest_nav_state["move_base"]
            and latest_nav_state["amcl"]
            and latest_nav_state["map_server"]
        ):
            result = apply_nav_tuning()
            nav_tuning_applied = result["returncode"] == 0
            latest_nav_tuning = {
                "ok": nav_tuning_applied,
                "detail": result["stderr"] or result["stdout"] or "applied",
            }
        time.sleep(3)


threading.Thread(target=nav_state_loop, daemon=True).start()


def start_nav_processes():
    return run_shell(
        ros_shell_prefix("/home/ucar/nav_clean_ws/devel/setup.bash") + " && "
        "timeout 3 bash -lc 'yes | rosnode cleanup >/tmp/rosnode_cleanup.log 2>&1' || true; "
        "setsid -f bash -lc '" + ros_shell_prefix("/home/ucar/nav_clean_ws/devel/setup.bash") + "; "
        "exec roslaunch nav_clean navigation_runtime.launch' "
        ">/tmp/nav_restart.log 2>&1 < /dev/null; true"
    )


def start_nav_async():
    global nav_start_in_progress
    with lock:
        if nav_start_in_progress:
            return False
        nav_start_in_progress = True

    def worker():
        global nav_start_in_progress
        try:
            state = wait_for_nav_ready(timeout=1)
            if not (state["move_base"] and state["amcl"] and state["map_server"]):
                start_nav_processes()
                wait_for_nav_ready()
            publish_pending_goal()
        finally:
            with lock:
                nav_start_in_progress = False

    threading.Thread(target=worker, daemon=True).start()
    return True


def wait_for_nav_ready(timeout=12):
    deadline = time.time() + timeout
    state = nav_nodes_running()
    while time.time() < deadline:
        state = nav_nodes_running()
        if state["move_base"] and state["amcl"] and state["map_server"]:
            return state
        time.sleep(0.5)
    return state


def finite_float(value, default=0.0):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(parsed):
        return default
    return parsed


def make_goal_msg(x, y, yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def publish_goal_msg(msg, repeats=3, delay=0.05):
    global last_goal_publish_time
    for _ in range(repeats):
        msg.header.stamp = rospy.Time.now()
        goal_pub.publish(msg)
        last_goal_publish_time = time.time()
        if delay:
            time.sleep(delay)


def publish_pending_goal():
    with lock:
        msg = pending_goal
    if msg is not None:
        publish_goal_msg(msg)


def set_navigation_goal_from_point(point):
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z, pending_goal, latest_goal, patrol_goal_publish_time
    msg = make_goal_msg(point["x"], point["y"], point["yaw"])
    with lock:
        MANUAL_MODE = False
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
        pending_goal = msg
        latest_goal = {"x": point["x"], "y": point["y"], "yaw": point["yaw"]}
    nav_ready = latest_nav_state["move_base"] and latest_nav_state["amcl"] and latest_nav_state["map_server"]
    if nav_ready and amcl_pose_seen():
        publish_goal_msg(msg)
        patrol_goal_publish_time = time.time()
        return True
    start_nav_async()
    patrol_goal_publish_time = time.time()
    return False


def voice_locations():
    snapshot = patrol_manager.snapshot()
    locations = []

    def append_points(points, route_name="", include_generic=False):
        for index, point in enumerate(points):
            if not point.get("name"):
                continue
            aliases = []
            if route_name:
                aliases.extend(build_route_point_aliases(route_name, index, include_generic=include_generic))
            locations.append({
                "name": point.get("name", ""),
                "aliases": aliases,
                "x": point.get("x", 0.0),
                "y": point.get("y", 0.0),
                "yaw": point.get("yaw", 0.0),
            })

    append_points(snapshot.get("points", []), snapshot.get("current_route_name", ""), include_generic=True)
    for route in snapshot.get("routes", []):
        append_points(route.get("points", []), route.get("name", ""))
    return locations


def distance_to_patrol_point(point):
    with lock:
        pose = dict(latest_pose)
    return math.hypot(point["x"] - pose["x"], point["y"] - pose["y"])


def save_patrol_capture(point, index):
    with camera_lock:
        frame = latest_camera_jpeg
    if frame is None:
        return None, "no camera frame available"
    os.makedirs(PATROL_CAPTURE_DIR, exist_ok=True)
    filename = "{}_{:02d}_{}.jpg".format(
        time.strftime("%Y%m%d_%H%M%S"),
        index,
        safe_name(point["name"]),
    )
    path = os.path.join(PATROL_CAPTURE_DIR, filename)
    with open(path, "wb") as fh:
        fh.write(frame)
    return path, None


def cancel_navigation_and_stop():
    cancel_pub.publish(GoalID())
    stop_msg = Twist()
    for _ in range(3):
        cmd_pub.publish(stop_msg)
        time.sleep(0.03)


def set_manual_velocity(linear_x, angular_z):
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    with lock:
        MANUAL_MODE = True
        CURRENT_LINEAR_X = max(-0.12, min(0.12, float(linear_x)))
        CURRENT_ANGULAR_Z = max(-0.50, min(0.50, float(angular_z)))
    msg = Twist()
    msg.linear.x = CURRENT_LINEAR_X
    msg.angular.z = CURRENT_ANGULAR_Z
    cmd_pub.publish(msg)


def stop_voice_motion():
    set_manual_velocity(0.0, 0.0)
    cancel_navigation_and_stop()


def odom_snapshot():
    with lock:
        return dict(latest_odom)


def amcl_pose_seen(max_age=5.0):
    with lock:
        pose = dict(latest_pose)
    stamp = float(pose.get("stamp", 0.0) or 0.0)
    return bool(pose.get("seen")) and stamp > 0.0 and time.time() - stamp <= max_age


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def voice_motion_scales():
    move_default = float(rospy.get_param("/voice_move_odom_scale", 1.0))
    turn_default = float(rospy.get_param("/voice_turn_odom_scale", 1.0))
    return {
        "move": max(0.5, min(1.5, float(rospy.get_param("~voice_move_odom_scale", move_default)))),
        "turn": max(0.5, min(1.6, float(rospy.get_param("~voice_turn_odom_scale", turn_default)))),
    }


def odom_is_fresh(snapshot, max_age=1.0):
    stamp = float(snapshot.get("stamp", 0.0) or 0.0)
    return stamp > 0.0 and time.time() - stamp <= max_age


def calibrated_step_duration(step, linear_x, angular_z, scales):
    target = odom_target_for_step(step, move_scale=scales["move"], turn_scale=scales["turn"])
    if step.get("kind") == "move" and abs(linear_x) > 0.001:
        return max(0.1, target.get("distance_m", 0.0) / abs(linear_x))
    if step.get("kind") == "turn" and abs(angular_z) > 0.001:
        return max(0.1, target.get("angle_rad", 0.0) / abs(angular_z))
    return max(0.1, float(step.get("duration_s", 0.0) or 0.0))


def voice_step_reached(step, start_odom):
    current = odom_snapshot()
    scales = voice_motion_scales()
    target = odom_target_for_step(step, move_scale=scales["move"], turn_scale=scales["turn"])
    if step.get("kind") == "move" and step.get("distance_m"):
        distance = math.hypot(current.get("x", 0.0) - start_odom.get("x", 0.0), current.get("y", 0.0) - start_odom.get("y", 0.0))
        return distance >= target.get("distance_m", float(step.get("distance_m", 0.0)))
    if step.get("kind") == "turn" and step.get("angle_deg"):
        angle = abs(normalize_angle(current.get("yaw", 0.0) - start_odom.get("yaw", 0.0)))
        return angle >= target.get("angle_rad", math.radians(float(step.get("angle_deg", 0.0))))
    return False


def run_voice_sequence(command):
    global voice_sequence_running
    steps = command.get("steps", [])
    try:
        for step in steps:
            if voice_sequence_cancel.is_set():
                break
            linear_x = max(-0.12, min(0.12, float(step.get("linear_x", 0.0))))
            angular_z = max(-0.50, min(0.50, float(step.get("angular_z", 0.0))))
            if latest_forward_obstacle.get("blocked") and linear_x > 0.0:
                break
            start_odom = odom_snapshot()
            scales = voice_motion_scales()
            duration_s = calibrated_step_duration(step, linear_x, angular_z, scales)
            use_odom_feedback = odom_is_fresh(start_odom)
            set_manual_velocity(linear_x, angular_z)
            deadline = time.time() + (max(duration_s * 1.6, duration_s + 1.5) if use_odom_feedback else duration_s)
            while time.time() < deadline and not rospy.is_shutdown():
                if voice_sequence_cancel.is_set():
                    break
                if latest_forward_obstacle.get("blocked") and linear_x > 0.0:
                    voice_sequence_cancel.set()
                    break
                if use_odom_feedback and voice_step_reached(step, start_odom):
                    break
                time.sleep(0.05)
            set_manual_velocity(0.0, 0.0)
            time.sleep(0.15)
    finally:
        stop_voice_motion()
        with voice_sequence_lock:
            voice_sequence_running = False


def start_voice_sequence(command):
    global voice_sequence_running
    if latest_forward_obstacle.get("blocked") and any(step.get("linear_x", 0.0) > 0.0 for step in command.get("steps", [])):
        command["ok"] = False
        command["executed"] = False
        command["message"] = "前方障碍触发，拒绝语音动作序列"
        return command
    disable_line_follow("语音动作序列")
    cancel_pub.publish(GoalID())
    voice_sequence_cancel.set()
    time.sleep(0.1)
    with voice_sequence_lock:
        voice_sequence_running = True
        voice_sequence_cancel.clear()
    threading.Thread(target=run_voice_sequence, args=(command,), daemon=True).start()
    command["executed"] = True
    command["message"] = command.get("message") or "已启动语音动作序列"
    return command


def execute_voice_command(command):
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z, pending_goal, latest_goal, latest_nav_state
    action = command.get("action")
    if action == "stop":
        voice_sequence_cancel.set()
        disable_line_follow("语音停止")
        stop_person_follow("语音停止")
        with lock:
            MANUAL_MODE = True
            CURRENT_LINEAR_X = 0.0
            CURRENT_ANGULAR_Z = 0.0
        cancel_navigation_and_stop()
        command["executed"] = True
        return command
    if action == "sequence":
        command["calibration"] = voice_motion_scales()
        stop_person_follow("语音动作序列")
        return start_voice_sequence(command)
    if action == "cmd_vel":
        voice_sequence_cancel.set()
        if latest_forward_obstacle.get("blocked") and command.get("linear_x", 0.0) > 0.0:
            command["ok"] = False
            command["executed"] = False
            command["message"] = "前方障碍触发，拒绝语音前进"
            return command
        disable_line_follow("语音手动控制")
        stop_person_follow("语音手动控制")
        with lock:
            MANUAL_MODE = True
            CURRENT_LINEAR_X = max(-0.12, min(0.12, float(command.get("linear_x", 0.0))))
            CURRENT_ANGULAR_Z = max(-0.50, min(0.50, float(command.get("angular_z", 0.0))))
        cancel_pub.publish(GoalID())
        command["linear_x"] = CURRENT_LINEAR_X
        command["angular_z"] = CURRENT_ANGULAR_Z
        command["executed"] = True
        return command
    if action == "goal":
        disable_line_follow("语音导航目标")
        stop_person_follow("语音导航目标")
        x = finite_float(command.get("x", 0.0))
        y = finite_float(command.get("y", 0.0))
        yaw = finite_float(command.get("yaw", 0.0))
        msg = make_goal_msg(x, y, yaw)
        with lock:
            MANUAL_MODE = False
            CURRENT_LINEAR_X = 0.0
            CURRENT_ANGULAR_Z = 0.0
            pending_goal = msg
            latest_goal = {"x": x, "y": y, "yaw": yaw}
        nav_ready = latest_nav_state["move_base"] and latest_nav_state["amcl"] and latest_nav_state["map_server"]
        localized = amcl_pose_seen()
        started_nav = False
        if nav_ready and localized:
            publish_goal_msg(msg)
        else:
            started_nav = start_nav_async()
        target_name = command.get("target_name") or "目标点"
        if nav_ready and localized:
            command["message"] = "已发布导航目标：{}".format(target_name)
        elif nav_ready and not localized:
            command["message"] = "目标已排队，但尚未收到 AMCL 定位；请先在地图设置 AMCL 位姿：{}".format(target_name)
        elif started_nav:
            command["message"] = "导航未就绪，已保存目标并正在启动导航：{}".format(target_name)
        else:
            command["message"] = "导航正在启动或未就绪，目标已排队：{}".format(target_name)
        command.update({"x": x, "y": y, "yaw": yaw, "nav_ready": nav_ready, "localized": localized, "started_nav": started_nav, "executed": True})
        return command
    if action == "patrol_start":
        disable_line_follow("语音启动巡逻")
        stop_person_follow("语音启动巡逻")
        try:
            point = patrol_manager.start(0)
        except ValueError as exc:
            command.update({"ok": False, "executed": False, "message": str(exc)})
            return command
        command["nav_ready"] = set_navigation_goal_from_point(point)
        command["executed"] = True
        return command
    if action == "patrol_pause":
        disable_line_follow("语音暂停巡逻")
        stop_person_follow("语音暂停巡逻")
        patrol_manager.pause("voice paused patrol")
        with lock:
            MANUAL_MODE = True
            CURRENT_LINEAR_X = 0.0
            CURRENT_ANGULAR_Z = 0.0
        cancel_navigation_and_stop()
        command["executed"] = True
        return command
    if action == "patrol_resume":
        disable_line_follow("语音恢复巡逻")
        stop_person_follow("语音恢复巡逻")
        try:
            point = patrol_manager.resume()
        except ValueError as exc:
            command.update({"ok": False, "executed": False, "message": str(exc)})
            return command
        command["nav_ready"] = set_navigation_goal_from_point(point)
        command["executed"] = True
        return command
    if action == "patrol_stop":
        disable_line_follow("语音停止巡逻")
        stop_person_follow("语音停止巡逻")
        patrol_manager.stop("voice stopped patrol")
        with lock:
            MANUAL_MODE = True
            CURRENT_LINEAR_X = 0.0
            CURRENT_ANGULAR_Z = 0.0
        cancel_navigation_and_stop()
        command["executed"] = True
        return command
    if action == "person_follow_start":
        snapshot = start_person_follow_async()
        command["ok"] = True
        command["executed"] = True
        command["person_follow"] = snapshot
        command["message"] = snapshot.get("detail", "人体跟随启动中")
        return command
    if action == "person_follow_stop":
        snapshot = stop_person_follow("语音停止人体跟随")
        command["executed"] = True
        command["person_follow"] = snapshot
        command["message"] = "人体跟随已停止"
        return command
    command["executed"] = False
    return command


def goal_distance():
    with lock:
        goal = latest_goal
        pose = dict(latest_pose)
    if not goal or not pose.get("seen"):
        return None
    return math.hypot(goal["x"] - pose["x"], goal["y"] - pose["y"])


def goal_watchdog_loop():
    global latest_goal_distance, auto_reissue_count
    active_codes = set([0, 1, 6, 7])
    while not rospy.is_shutdown():
        latest_goal_distance = goal_distance()
        with lock:
            manual_mode = MANUAL_MODE
            msg = pending_goal
        status_code = latest_move_base_status.get("code")
        nav_ready = latest_nav_state["move_base"] and latest_nav_state["amcl"] and latest_nav_state["map_server"]
        should_reissue = (
            not manual_mode
            and msg is not None
            and nav_ready
            and latest_goal_distance is not None
            and latest_goal_distance > GOAL_REISSUE_DISTANCE
            and status_code not in active_codes
            and time.time() - last_goal_publish_time > GOAL_REISSUE_INTERVAL
        )
        if should_reissue:
            auto_reissue_count += 1
            publish_goal_msg(msg, repeats=2, delay=0.05)
        time.sleep(0.5)


threading.Thread(target=goal_watchdog_loop, daemon=True).start()


def safety_loop():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    last_stop_time = 0.0
    while not rospy.is_shutdown():
        blocked = latest_forward_obstacle.get("blocked")
        with lock:
            manual_mode = MANUAL_MODE
        if blocked and not manual_mode and time.time() - last_stop_time > 0.8:
            with lock:
                MANUAL_MODE = True
                CURRENT_LINEAR_X = 0.0
                CURRENT_ANGULAR_Z = 0.0
            cancel_pub.publish(GoalID())
            cmd_pub.publish(Twist())
            last_stop_time = time.time()
        time.sleep(0.1)


threading.Thread(target=safety_loop, daemon=True).start()


def patrol_loop():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    while not rospy.is_shutdown():
        snapshot = patrol_manager.snapshot()
        state = snapshot["state"]
        mode = state["mode"]
        point = state["current_point"]
        if mode == "running" and point:
            if latest_forward_obstacle.get("blocked"):
                patrol_manager.handle_blocked(True)
                with lock:
                    MANUAL_MODE = True
                    CURRENT_LINEAR_X = 0.0
                    CURRENT_ANGULAR_Z = 0.0
                cancel_navigation_and_stop()
                time.sleep(0.2)
                continue

            distance = distance_to_patrol_point(point)
            status_code = latest_move_base_status.get("code")
            status_reached = (
                status_code == 3
                and distance <= PATROL_STATUS_REACHED_DISTANCE
                and time.time() - patrol_goal_publish_time > 1.0
            )
            if distance <= PATROL_REACHED_DISTANCE or status_reached:
                with lock:
                    CURRENT_LINEAR_X = 0.0
                    CURRENT_ANGULAR_Z = 0.0
                cmd_pub.publish(Twist())
                index = state["current_index"] if state["current_index"] is not None else 0
                capture_path, capture_error = save_patrol_capture(point, index)
                next_point = patrol_manager.mark_current_reached(capture_path, capture_error)
                if next_point is None:
                    with lock:
                        MANUAL_MODE = True
                        CURRENT_LINEAR_X = 0.0
                        CURRENT_ANGULAR_Z = 0.0
                    cancel_navigation_and_stop()
                else:
                    time.sleep(PATROL_POINT_WAIT_SECONDS)
                    if patrol_manager.snapshot()["state"]["mode"] == "running":
                        set_navigation_goal_from_point(next_point)
        elif mode == "blocked" and not latest_forward_obstacle.get("blocked"):
            patrol_manager.handle_blocked(False)
        time.sleep(0.2)


threading.Thread(target=patrol_loop, daemon=True).start()


def stop_manual_command():
    global CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    with lock:
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0


def battery_status():
    battery = dict(latest_battery)
    if not battery.get("seen"):
        battery["age"] = None
        battery["stale"] = True
        return battery
    age = max(0.0, time.time() - battery.get("stamp", 0.0))
    battery["age"] = age
    battery["stale"] = age > BATTERY_STALE_SECONDS
    return battery


@app.after_request
def no_cache(resp):
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    with lock:
        odom = dict(latest_odom)
        pose = dict(latest_pose)
        current_cmd = {"linear_x": CURRENT_LINEAR_X, "angular_z": CURRENT_ANGULAR_Z}
    return jsonify({
        "odom": odom,
        "pose": pose,
        "battery": battery_status(),
        "trajectory": [],
        "manual_mode": MANUAL_MODE,
        "current_cmd": current_cmd,
        "goal": latest_goal,
        "goal_distance": latest_goal_distance,
        "auto_reissue_count": auto_reissue_count,
        "nav_tuning": latest_nav_tuning,
        "move_base_status": latest_move_base_status,
        "scan": latest_scan,
        "scan_points": latest_scan_points,
        "forward_obstacle": latest_forward_obstacle,
        "nav_state": latest_nav_state,
        "patrol": patrol_manager.snapshot(),
        "line_follow": line_follow_snapshot(),
        "person_follow": person_follow_snapshot(),
    })


@app.route("/api/map")
def api_map():
    for path in MAP_PREVIEW_CANDIDATES:
        if os.path.exists(path):
            with open(path, "rb") as fh:
                return Response(fh.read(), mimetype="image/png")
    return jsonify({"ok": False, "error": "map preview not found", "paths": MAP_PREVIEW_CANDIDATES}), 404


@app.route("/api/map_info")
def api_map_info():
    return jsonify(MAP_INFO)


@app.route("/api/live_map")
def api_live_map():
    if latest_live_map is None:
        return jsonify({"ok": False, "error": "no /map received yet"}), 404
    return jsonify({"ok": True, "map": latest_live_map})


@app.route("/api/camera")
def api_camera():
    with camera_lock:
        frame = latest_camera_jpeg
    return Response(frame or offline_camera_jpeg(), mimetype="image/jpeg")


@app.route("/api/camera_stream")
def api_camera_stream():
    global camera_clients
    with camera_lock:
        camera_clients += 1

    def frames():
        global camera_clients
        last_stamp = 0.0
        try:
            while True:
                with camera_lock:
                    frame = latest_camera_jpeg
                    stamp = latest_camera_stamp
                if frame is not None and stamp != last_stamp:
                    last_stamp = stamp
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Cache-Control: no-cache\r\n\r\n" + frame + b"\r\n"
                    )
                time.sleep(0.05)
        finally:
            with camera_lock:
                camera_clients = max(0, camera_clients - 1)

    return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/line_follow/status")
def api_line_follow_status():
    snapshot = process_line_follow_frame()
    snapshot["safety"] = {"forward_obstacle": latest_forward_obstacle}
    return jsonify({"ok": True, "line_follow": snapshot})


@app.route("/api/line_follow/debug.jpg")
def api_line_follow_debug():
    with line_follow_lock:
        frame = latest_line_follow_debug_jpeg
    if frame is None:
        process_line_follow_frame()
        with line_follow_lock:
            frame = latest_line_follow_debug_jpeg
    return Response(frame or offline_camera_jpeg(), mimetype="image/jpeg")


@app.route("/api/line_follow/config", methods=["POST"])
def api_line_follow_config():
    global line_follow_config, MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    data = request.json or {}
    next_config = LineFollowConfig.from_dict(data, line_follow_config)
    if next_config.enabled:
        patrol_mode = patrol_manager.snapshot()["state"]["mode"]
        if patrol_mode in ("running", "blocked"):
            return jsonify({
                "ok": False,
                "error": "巡逻正在运行，不能同时启用视觉巡线",
                "line_follow": line_follow_snapshot(),
            }), 409
        if latest_forward_obstacle.get("blocked"):
            next_config.enabled = False
            with line_follow_lock:
                line_follow_config = next_config
                line_follower.update_config(line_follow_config)
            return jsonify({
                "ok": False,
                "error": "前方障碍触发，不能启用视觉巡线",
                "line_follow": line_follow_snapshot(),
            }), 409
        cancel_pub.publish(GoalID())
        with lock:
            MANUAL_MODE = True
            CURRENT_LINEAR_X = 0.0
            CURRENT_ANGULAR_Z = 0.0
    with line_follow_lock:
        line_follow_config = next_config
        line_follower.update_config(line_follow_config)
        latest_line_follow_control["enabled"] = line_follow_config.enabled
        latest_line_follow_control["reason"] = "配置已更新"
    snapshot = process_line_follow_frame()
    return jsonify({"ok": True, "line_follow": snapshot})


@app.route("/api/line_follow/stop", methods=["POST"])
def api_line_follow_stop():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    disable_line_follow("用户停止视觉巡线")
    with lock:
        MANUAL_MODE = True
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    cmd_pub.publish(Twist())
    return jsonify({"ok": True, "line_follow": line_follow_snapshot()})


@app.route("/api/line_follow/record/status")
def api_line_follow_record_status():
    return jsonify({"ok": True, "recording": line_follow_recorder.snapshot(), "line_follow": line_follow_snapshot()})


@app.route("/api/line_follow/record/start", methods=["POST"])
def api_line_follow_record_start():
    data = request.json or {}
    disable_line_follow("开始人工巡线样本记录")
    session = line_follow_recorder.start(data.get("name") or "manual_line_follow", line_follow_config.to_dict())
    process_line_follow_frame()
    return jsonify({"ok": True, "recording": session, "line_follow": line_follow_snapshot()})


@app.route("/api/line_follow/record/stop", methods=["POST"])
def api_line_follow_record_stop():
    session = line_follow_recorder.stop()
    return jsonify({"ok": True, "recording": session, "line_follow": line_follow_snapshot()})


@app.route("/api/person_follow/status")
def api_person_follow_status():
    return jsonify({"ok": True, "person_follow": person_follow_snapshot()})


@app.route("/api/person_follow/start", methods=["POST"])
def api_person_follow_start():
    snapshot = start_person_follow_async()
    return jsonify({"ok": True, "person_follow": snapshot})


@app.route("/api/person_follow/stop", methods=["POST"])
def api_person_follow_stop():
    snapshot = stop_person_follow("用户停止人体跟随")
    return jsonify({"ok": True, "person_follow": snapshot})


@app.route("/api/cmd_vel", methods=["POST"])
def api_cmd_vel():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    disable_line_follow("手动方向控制接管")
    stop_person_follow_if_active("手动方向控制接管")
    data = request.json or {}
    lx = max(-MAX_MANUAL_LINEAR_SPEED, min(MAX_MANUAL_LINEAR_SPEED, float(data.get("linear_x", 0))))
    az = max(-MAX_MANUAL_ANGULAR_SPEED, min(MAX_MANUAL_ANGULAR_SPEED, float(data.get("angular_z", 0))))
    if lx > 0.0 and latest_forward_obstacle.get("blocked"):
        lx = 0.0
    with lock:
        MANUAL_MODE = True
        CURRENT_LINEAR_X = lx
        CURRENT_ANGULAR_Z = az
    cancel_pub.publish(GoalID())
    msg = Twist()
    msg.linear.x = lx
    msg.angular.z = az
    cmd_pub.publish(msg)
    return jsonify({"ok": True, "linear_x": lx, "angular_z": az, "manual_mode": MANUAL_MODE})


@app.route("/api/manual_stop", methods=["POST"])
def api_manual_stop():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    disable_line_follow("手动停止")
    stop_person_follow_if_active("手动停止")
    with lock:
        MANUAL_MODE = True
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    cancel_pub.publish(GoalID())
    cmd_pub.publish(Twist())
    return jsonify({"ok": True, "manual_mode": MANUAL_MODE})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    disable_line_follow("STOP 停止")
    stop_person_follow("STOP 停止")
    with lock:
        MANUAL_MODE = True
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    cancel_pub.publish(GoalID())
    stop_msg = Twist()
    for _ in range(5):
        cmd_pub.publish(stop_msg)
        time.sleep(0.05)
    return jsonify({"ok": True, "manual_mode": MANUAL_MODE})


@app.route("/api/voice_command", methods=["POST"])
def api_voice_command():
    data = request.json or {}
    text = data.get("text", "")
    command = parse_voice_command(text, locations=voice_locations())
    if command.get("ok"):
        command = execute_voice_command(command)
    status = 200 if command.get("ok") else 400
    return jsonify({"ok": bool(command.get("ok")), "voice": command, "patrol": patrol_manager.snapshot()}), status


@app.route("/api/goal", methods=["POST"])
def api_goal():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z, pending_goal, latest_goal, latest_nav_state
    disable_line_follow("发送导航目标")
    stop_person_follow("发送导航目标")
    data = request.json or {}
    x = finite_float(data.get("x", 0))
    y = finite_float(data.get("y", 0))
    yaw = finite_float(data.get("yaw", 0))
    msg = make_goal_msg(x, y, yaw)
    with lock:
        MANUAL_MODE = False
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
        pending_goal = msg
        latest_goal = {"x": x, "y": y, "yaw": yaw}
    nav_state = latest_nav_state
    nav_ready = nav_state["move_base"] and nav_state["amcl"] and nav_state["map_server"]
    localized = amcl_pose_seen()
    started_nav = False
    if nav_ready and localized:
        publish_goal_msg(msg)
    else:
        started_nav = start_nav_async()
    return jsonify({
        "ok": True,
        "x": x,
        "y": y,
        "yaw": yaw,
        "manual_mode": MANUAL_MODE,
        "nav_state": nav_state,
        "nav_ready": nav_ready,
        "localized": localized,
        "started_nav": started_nav,
    })


@app.route("/api/set_pose", methods=["POST"])
def api_set_pose():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z, pending_goal, latest_goal
    disable_line_follow("设置 AMCL 位姿")
    stop_person_follow("设置 AMCL 位姿")
    data = request.json or {}
    x = finite_float(data.get("x", 0))
    y = finite_float(data.get("y", 0))
    yaw = finite_float(data.get("yaw", 0))
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.pose.covariance = [
        0.25, 0, 0, 0, 0, 0,
        0, 0.25, 0, 0, 0, 0,
        0, 0, 0.0, 0, 0, 0,
        0, 0, 0, 0.0, 0, 0,
        0, 0, 0, 0, 0.0, 0,
        0, 0, 0, 0, 0, 0.068,
    ]
    with lock:
        keep_queued_goal = pending_goal is not None
        MANUAL_MODE = not keep_queued_goal
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    cancel_pub.publish(GoalID())
    cmd_pub.publish(Twist())
    for _ in range(3):
        msg.header.stamp = rospy.Time.now()
        pose_pub.publish(msg)
        time.sleep(0.05)
    return jsonify({"ok": True, "x": x, "y": y, "yaw": yaw, "manual_mode": MANUAL_MODE, "queued_goal": keep_queued_goal})


@app.route("/api/takeover", methods=["POST"])
def api_takeover():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    disable_line_follow("手动接管")
    stop_person_follow("手动接管")
    with lock:
        MANUAL_MODE = True
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    cancel_pub.publish(GoalID())
    cmd_pub.publish(Twist())
    return jsonify({"ok": True, "mode": "manual", "nav_state": latest_nav_state})


@app.route("/api/resume_nav", methods=["POST"])
def api_resume_nav():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z, latest_nav_state
    disable_line_follow("恢复导航")
    stop_person_follow("恢复导航")
    with lock:
        MANUAL_MODE = False
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    latest_nav_state = nav_nodes_running()
    started_nav = False
    if not (latest_nav_state["move_base"] and latest_nav_state["amcl"] and latest_nav_state["map_server"]):
        started_nav = start_nav_async()
    return jsonify({"ok": True, "mode": "nav", "started_nav": started_nav, "nav_state": latest_nav_state})


@app.route("/api/patrol")
def api_patrol():
    return jsonify({"ok": True, "patrol": patrol_manager.snapshot()})


@app.route("/api/patrol/routes")
def api_patrol_routes():
    return jsonify({"ok": True, "routes": patrol_manager.list_routes(), "current_route_name": patrol_manager.current_route_name})


@app.route("/api/patrol/runs")
def api_patrol_runs():
    return jsonify({"ok": True, "runs": patrol_manager.list_runs()})


@app.route("/api/patrol/runs/<run_id>")
def api_patrol_run(run_id):
    try:
        run = patrol_manager.get_run(run_id)
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 404
    return jsonify({"ok": True, "run": run})


@app.route("/api/patrol/captures/<filename>")
def api_patrol_capture(filename):
    safe_filename = os.path.basename(filename)
    path = os.path.join(PATROL_CAPTURE_DIR, safe_filename)
    if not os.path.exists(path):
        return jsonify({"ok": False, "error": "capture not found"}), 404
    return send_file(path, mimetype="image/jpeg")


@app.route("/api/patrol/points", methods=["POST"])
def api_patrol_add_point():
    data = request.json or {}
    try:
        point = patrol_manager.add_point(
            data.get("name", ""),
            data.get("x"),
            data.get("y"),
            data.get("yaw", 0.0),
        )
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc), "patrol": patrol_manager.snapshot()}), 400
    return jsonify({"ok": True, "point": point, "patrol": patrol_manager.snapshot()})


@app.route("/api/patrol/points/<int:index>", methods=["DELETE"])
def api_patrol_delete_point(index):
    try:
        deleted = patrol_manager.delete_point(index)
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc), "patrol": patrol_manager.snapshot()}), 400
    return jsonify({"ok": True, "deleted": deleted, "patrol": patrol_manager.snapshot()})


@app.route("/api/patrol/save", methods=["POST"])
def api_patrol_save():
    data = request.json or {}
    route_name = data.get("route_name")
    try:
        route = patrol_manager.save_named_route(route_name)
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc), "patrol": patrol_manager.snapshot()}), 400
    return jsonify({"ok": True, "route": route, "patrol": patrol_manager.snapshot()})


@app.route("/api/patrol/load", methods=["POST"])
def api_patrol_load():
    data = request.json or {}
    try:
        route = patrol_manager.load_named_route(data.get("route_name"))
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc), "patrol": patrol_manager.snapshot()}), 400
    return jsonify({"ok": True, "route": route, "patrol": patrol_manager.snapshot()})


@app.route("/api/patrol/start", methods=["POST"])
def api_patrol_start():
    data = request.json or {}
    disable_line_follow("启动巡逻")
    stop_person_follow("启动巡逻")
    try:
        if data.get("route_name"):
            patrol_manager.load_named_route(data.get("route_name"))
        point = patrol_manager.start(data.get("start_index", 0))
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc), "patrol": patrol_manager.snapshot()}), 400
    nav_ready = set_navigation_goal_from_point(point)
    return jsonify({"ok": True, "point": point, "nav_ready": nav_ready, "patrol": patrol_manager.snapshot()})


@app.route("/api/patrol/pause", methods=["POST"])
def api_patrol_pause():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    disable_line_follow("暂停巡逻")
    stop_person_follow("暂停巡逻")
    patrol_manager.pause("operator paused patrol")
    with lock:
        MANUAL_MODE = True
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    cancel_navigation_and_stop()
    return jsonify({"ok": True, "patrol": patrol_manager.snapshot()})


@app.route("/api/patrol/resume", methods=["POST"])
def api_patrol_resume():
    data = request.json or {}
    disable_line_follow("恢复巡逻")
    stop_person_follow("恢复巡逻")
    try:
        if data.get("route_name") and not patrol_manager.snapshot()["points"]:
            patrol_manager.load_named_route(data.get("route_name"))
        point = patrol_manager.resume()
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc), "patrol": patrol_manager.snapshot()}), 400
    nav_ready = set_navigation_goal_from_point(point)
    return jsonify({"ok": True, "point": point, "nav_ready": nav_ready, "patrol": patrol_manager.snapshot()})


@app.route("/api/patrol/stop", methods=["POST"])
def api_patrol_stop():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    disable_line_follow("停止巡逻")
    stop_person_follow("停止巡逻")
    patrol_manager.stop("operator stopped patrol")
    with lock:
        MANUAL_MODE = True
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    cancel_navigation_and_stop()
    return jsonify({"ok": True, "patrol": patrol_manager.snapshot()})


@app.route("/api/clear_trajectory", methods=["POST"])
def api_clear_trajectory():
    return jsonify({"ok": True})


if __name__ == "__main__":
    port = int(os.environ.get("WEB_PANEL_PORT", "8080"))
    probe = subprocess.run(["bash", "-lc", "ss -ltn | grep -q ':{} '".format(port)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if probe.returncode == 0:
        raise SystemExit("port {} already in use".format(port))
    app.run(host="0.0.0.0", port=port, threaded=True, use_reloader=False)
