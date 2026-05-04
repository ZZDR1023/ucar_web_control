#!/usr/bin/env python3
"""Low-latency Flask web control panel for UCAR robot using rospy."""

import io
import math
import os
import subprocess
import threading
import time

from flask import Flask, Response, jsonify, request, render_template

import rospy
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import BatteryState, LaserScan

app = Flask(__name__)

MANUAL_MODE = True
CURRENT_LINEAR_X = 0.0
CURRENT_ANGULAR_Z = 0.0
lock = threading.Lock()
latest_odom = {"x": 0.0, "y": 0.0, "linear": 0.0, "angular": 0.0}
latest_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
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
    "stop_active": False,
    "min_distance": None,
    "stop_distance": 0.40,
    "clear_distance": 0.65,
    "half_angle_deg": 18.0,
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
GOAL_REISSUE_DISTANCE = 0.12
GOAL_REISSUE_INTERVAL = 2.0
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
FRONT_STOP_DISTANCE = 0.40
FRONT_CLEAR_DISTANCE = 0.65
FRONT_STOP_HALF_ANGLE = math.radians(18)
BATTERY_STALE_SECONDS = 10.0

rospy.init_node("web_panel_server", anonymous=True, disable_signals=True)
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)


def odom_cb(msg):
    with lock:
        latest_odom["x"] = msg.pose.pose.position.x
        latest_odom["y"] = msg.pose.pose.position.y
        latest_odom["linear"] = msg.twist.twist.linear.x
        latest_odom["angular"] = msg.twist.twist.angular.z


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
    with lock:
        latest_pose["x"] = msg.pose.pose.position.x
        latest_pose["y"] = msg.pose.pose.position.y
        latest_pose["yaw"] = quat_to_yaw(msg.pose.pose.orientation)


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
        if idx % step == 0 and distance <= 5.0:
            world_angle = pose["yaw"] + angle
            points.append({
                "x": pose["x"] + math.cos(world_angle) * distance,
                "y": pose["y"] + math.sin(world_angle) * distance,
            })
        angle += msg.angle_increment
    latest_scan_points = points
    if was_blocked:
        blocked = front_min is not None and front_min < FRONT_CLEAR_DISTANCE
    else:
        blocked = front_min is not None and front_min < FRONT_STOP_DISTANCE
    latest_forward_obstacle = {
        "blocked": blocked,
        "stop_active": blocked,
        "min_distance": front_min,
        "stop_distance": FRONT_STOP_DISTANCE,
        "clear_distance": FRONT_CLEAR_DISTANCE,
        "half_angle_deg": math.degrees(FRONT_STOP_HALF_ANGLE),
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


def cmd_loop():
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        with lock:
            manual_mode = MANUAL_MODE
            msg.linear.x = CURRENT_LINEAR_X
            msg.angular.z = CURRENT_ANGULAR_Z
        if manual_mode:
            try:
                cmd_pub.publish(msg)
            except rospy.ROSException:
                break
        rate.sleep()


threading.Thread(target=cmd_loop, daemon=True).start()


def run_shell(command, timeout=8):
    try:
        result = subprocess.run(["bash", "-lc", command], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=timeout)
        return {
            "stdout": result.stdout.strip(),
            "stderr": result.stderr.strip(),
            "returncode": result.returncode,
        }
    except Exception as exc:
        return {"stdout": "", "stderr": str(exc), "returncode": 1}


def kill_nav_processes():
    run_shell("pkill -9 -f move_base || true")
    run_shell("pkill -9 -f amcl || true")
    run_shell("pkill -9 -f map_server || true")
    run_shell("pkill -9 -f cartographer || true")


def nav_nodes_running():
    result = run_shell("rosnode list 2>/dev/null | grep -E '^/(move_base|amcl|map_server)$' || true")
    nodes = [line.strip() for line in result["stdout"].splitlines() if line.strip()]
    return {
        "move_base": "/move_base" in nodes,
        "amcl": "/amcl" in nodes,
        "map_server": "/map_server" in nodes,
        "nodes": nodes,
    }


def apply_nav_tuning():
    # Keep XY precision tighter, but relax final yaw to reduce in-place spinning.
    return run_shell(
        "source /opt/ros/melodic/setup.bash && "
        "source /home/ucar/nav_clean_ws/devel/setup.bash && "
        "export ROS_MASTER_URI=http://10.90.122.179:11311 && "
        "export ROS_IP=10.90.122.179 && "
        "rosparam set /move_base/DWAPlannerROS/xy_goal_tolerance 0.08 && "
        "rosparam set /move_base/DWAPlannerROS/yaw_goal_tolerance 0.35 && "
        "rosparam set /move_base/DWAPlannerROS/latch_xy_goal_tolerance true",
        timeout=4,
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
        "source /opt/ros/melodic/setup.bash && "
        "source /home/ucar/nav_clean_ws/devel/setup.bash && "
        "export ROS_MASTER_URI=http://10.90.122.179:11311 && "
        "export ROS_IP=10.90.122.179 && "
        "nohup roslaunch nav_clean navigation_runtime.launch >/tmp/nav_restart.log 2>&1 &"
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


def goal_distance():
    with lock:
        goal = latest_goal
        pose = dict(latest_pose)
    if not goal:
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
    try:
        import cv2
        cap = cv2.VideoCapture("/dev/ucar_video")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret, frame = cap.read()
        cap.release()
        if ret and frame is not None:
            _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            return Response(jpeg.tobytes(), mimetype="image/jpeg")
    except Exception:
        pass
    from PIL import Image, ImageDraw
    img = Image.new("RGB", (640, 480), (0, 0, 0))
    d = ImageDraw.Draw(img)
    d.text((20, 20), "Camera offline", fill=(255, 255, 255))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    return Response(buf.getvalue(), mimetype="image/jpeg")


@app.route("/api/cmd_vel", methods=["POST"])
def api_cmd_vel():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    data = request.json or {}
    lx = max(-0.5, min(0.5, float(data.get("linear_x", 0))))
    az = max(-2.0, min(2.0, float(data.get("angular_z", 0))))
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


@app.route("/api/goal", methods=["POST"])
def api_goal():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z, pending_goal, latest_goal
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
    started_nav = False
    if nav_ready:
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
        "started_nav": started_nav,
    })


@app.route("/api/set_pose", methods=["POST"])
def api_set_pose():
    data = request.json or {}
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance = [
        0.25, 0, 0, 0, 0, 0,
        0, 0.25, 0, 0, 0, 0,
        0, 0, 0.0, 0, 0, 0,
        0, 0, 0, 0.0, 0, 0,
        0, 0, 0, 0, 0.0, 0,
        0, 0, 0, 0, 0, 0.068,
    ]
    pose_pub.publish(msg)
    return jsonify({"ok": True, "x": x, "y": y})


@app.route("/api/takeover", methods=["POST"])
def api_takeover():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    with lock:
        MANUAL_MODE = True
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    cancel_pub.publish(GoalID())
    cmd_pub.publish(Twist())
    return jsonify({"ok": True, "mode": "manual", "nav_state": latest_nav_state})


@app.route("/api/resume_nav", methods=["POST"])
def api_resume_nav():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    with lock:
        MANUAL_MODE = False
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    started_nav = False
    if not (latest_nav_state["move_base"] and latest_nav_state["amcl"] and latest_nav_state["map_server"]):
        started_nav = start_nav_async()
    return jsonify({"ok": True, "mode": "nav", "started_nav": started_nav, "nav_state": latest_nav_state})


@app.route("/api/clear_trajectory", methods=["POST"])
def api_clear_trajectory():
    return jsonify({"ok": True})


if __name__ == "__main__":
    port = int(os.environ.get("WEB_PANEL_PORT", "8080"))
    probe = subprocess.run(["bash", "-lc", "ss -ltn | grep -q ':{} '".format(port)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if probe.returncode == 0:
        raise SystemExit("port {} already in use".format(port))
    app.run(host="0.0.0.0", port=port, threaded=True, use_reloader=False)
