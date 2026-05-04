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
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry

app = Flask(__name__)

MANUAL_MODE = True
CURRENT_LINEAR_X = 0.0
CURRENT_ANGULAR_Z = 0.0
lock = threading.Lock()
latest_odom = {"x": 0.0, "y": 0.0, "linear": 0.0, "angular": 0.0}
latest_nav_state = {
    "move_base": False,
    "amcl": False,
    "map_server": False,
    "nodes": [],
}
pending_goal = None
nav_start_in_progress = False
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PREVIEW_CANDIDATES = [
    os.path.abspath(os.path.join(BASE_DIR, "..", "maps", "ucar_map_20260501_202713_sealed_preview.png")),
    "/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed_preview.png",
    "/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_preview.png",
    os.path.abspath(os.path.join(BASE_DIR, "..", "maps", "ucar_map_20260501_202713_preview.png")),
]

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


def cmd_loop():
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        with lock:
            manual_mode = MANUAL_MODE
            msg.linear.x = CURRENT_LINEAR_X
            msg.angular.z = CURRENT_ANGULAR_Z
        if manual_mode:
            cmd_pub.publish(msg)
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


def nav_state_loop():
    global latest_nav_state
    while not rospy.is_shutdown():
        latest_nav_state = nav_nodes_running()
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
    for _ in range(repeats):
        msg.header.stamp = rospy.Time.now()
        goal_pub.publish(msg)
        if delay:
            time.sleep(delay)


def publish_pending_goal():
    with lock:
        msg = pending_goal
    if msg is not None:
        publish_goal_msg(msg)


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
        current_cmd = {"linear_x": CURRENT_LINEAR_X, "angular_z": CURRENT_ANGULAR_Z}
    return jsonify({
        "odom": odom,
        "trajectory": [],
        "manual_mode": MANUAL_MODE,
        "current_cmd": current_cmd,
        "nav_state": latest_nav_state,
    })


@app.route("/api/map")
def api_map():
    for path in MAP_PREVIEW_CANDIDATES:
        if os.path.exists(path):
            with open(path, "rb") as fh:
                return Response(fh.read(), mimetype="image/png")
    return jsonify({"ok": False, "error": "map preview not found", "paths": MAP_PREVIEW_CANDIDATES}), 404


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
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z, pending_goal
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
    MANUAL_MODE = True
    with lock:
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    kill_nav_processes()
    time.sleep(1)
    return jsonify({"ok": True, "mode": "manual", "nav_state": latest_nav_state})


@app.route("/api/resume_nav", methods=["POST"])
def api_resume_nav():
    global MANUAL_MODE, CURRENT_LINEAR_X, CURRENT_ANGULAR_Z
    MANUAL_MODE = False
    with lock:
        CURRENT_LINEAR_X = 0.0
        CURRENT_ANGULAR_Z = 0.0
    kill_nav_processes()
    time.sleep(1)
    debug = start_nav_processes()
    time.sleep(3)
    return jsonify({"ok": True, "mode": "nav", "debug": debug, "nav_state": latest_nav_state})


@app.route("/api/clear_trajectory", methods=["POST"])
def api_clear_trajectory():
    return jsonify({"ok": True})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, threaded=True)
