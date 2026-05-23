import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SERVER_FILE = ROOT / "web_panel" / "server.py"
CAMERA_BRIDGE_FILE = ROOT / "web_panel" / "ucar_camera_bridge.py"


class WebPanelServerStaticTest(unittest.TestCase):
    def test_web_node_uses_stable_ros_name(self):
        text = SERVER_FILE.read_text(encoding="utf-8")

        self.assertIn('rospy.init_node("web_panel_server", anonymous=False', text)

    def test_nav_launch_is_detached_from_web_shell(self):
        text = SERVER_FILE.read_text(encoding="utf-8")

        self.assertIn("setsid -f bash -lc", text)
        self.assertIn("exec roslaunch nav_clean navigation_runtime.launch", text)

    def test_run_shell_kills_timed_out_process_groups(self):
        text = SERVER_FILE.read_text(encoding="utf-8")

        self.assertIn("start_new_session=True", text)
        self.assertIn("os.killpg", text)

    def test_goal_waits_for_amcl_pose_before_publish(self):
        text = SERVER_FILE.read_text(encoding="utf-8")

        self.assertIn("amcl_pose_seen()", text)
        self.assertIn("尚未收到 AMCL 定位", text)

    def test_set_pose_keeps_queued_goal(self):
        text = SERVER_FILE.read_text(encoding="utf-8")
        set_pose_block = text.split('def api_set_pose():', 1)[1].split('@app.route("/api/takeover"', 1)[0]

        self.assertIn("keep_queued_goal", set_pose_block)
        self.assertNotIn("pending_goal = None", set_pose_block)

    def test_person_follow_is_started_detached_and_stopped_by_safety_paths(self):
        text = SERVER_FILE.read_text(encoding="utf-8")

        self.assertIn('PERSON_FOLLOW_COMMAND = "python3 -u /home/ucar/web_panel/person_follow_adapter.py"', text)
        self.assertIn('@app.route("/api/person_follow/start"', text)
        self.assertIn('@app.route("/api/person_follow/stop"', text)
        start_block = text.split("def start_person_follow_worker", 1)[1].split("\ndef kill_nav_processes", 1)[0]
        route_block = text.split("def api_person_follow_start():", 1)[1].split("\n\n@app.route", 1)[0]
        self.assertIn("nohup bash -lc", start_block)
        self.assertIn("start_person_follow_async()", route_block)
        voice_block = text.split('if action == "person_follow_start":', 1)[1].split('if action == "person_follow_stop":', 1)[0]
        self.assertIn("start_person_follow_async()", voice_block)
        self.assertNotIn("start_person_follow()", voice_block)
        self.assertIn("person_follow_start_generation", text)
        self.assertIn('or detail.startswith("启动中")', text)
        self.assertIn("stop_person_follow(\"重启人体跟随\", cancel_start=False, update_state=False)", text)
        self.assertIn("threading.Thread(target=start_person_follow_worker, args=(generation,), daemon=True).start()", text)
        self.assertIn("MANUAL_MODE = False", start_block)
        self.assertIn("pid is not None and ready and camera_ready and service_ok", start_block)
        cmd_loop_block = text.split("def cmd_loop():", 1)[1].split("threading.Thread(target=cmd_loop", 1)[0]
        self.assertIn("person_follow_process_running_cached() is None", cmd_loop_block)
        self.assertNotIn("setsid -f", start_block)
        self.assertIn("UCAR_CAMERA_COMMAND", text)
        self.assertIn("_width:=1280 _height:=720", text)
        self.assertIn("ucar_camera_bridge.py", text)
        self.assertIn("/ucar_camera/image_raw", text)
        self.assertIn("PERSON_FOLLOW_USES_DIRECT_CAMERA", text)
        self.assertIn("release_web_camera_for_person_follow", text)
        self.assertIn("wait_for_camera_message", text)
        self.assertIn("/person_follow/start_person_detect", text)
        self.assertIn("/person_follow/start_person_follow", text)
        self.assertIn("wait_for_person_follow_ready", text)
        self.assertIn("wait_for_person_follow_stopped", text)
        self.assertIn("PERSON_FOLLOW_COMMAND", text)
        self.assertIn("stop_person_follow(", text)

        for marker in (
            'def api_goal():',
            'def api_patrol_start():',
        ):
            block = text.split(marker, 1)[1].split("\ndef ", 1)[0]
            self.assertIn("stop_person_follow", block, marker)

        stop_block = text.split('def api_stop():', 1)[1].split("\ndef ", 1)[0]
        self.assertIn("stop_person_follow", stop_block)
        cmd_block = text.split('def api_cmd_vel():', 1)[1].split("\ndef ", 1)[0]
        self.assertIn("stop_person_follow_if_active", cmd_block)
        self.assertNotIn('stop_person_follow("手动方向控制接管")', cmd_block)

    def test_person_follow_is_not_blocked_or_stopped_by_forward_obstacle(self):
        text = SERVER_FILE.read_text(encoding="utf-8")
        start_block = text.split("def start_person_follow_worker", 1)[1].split("\ndef kill_nav_processes", 1)[0]
        safety_block = text.split("def safety_loop():", 1)[1].split("\ndef patrol_loop", 1)[0]

        self.assertNotIn("latest_forward_obstacle.get(\"blocked\")", start_block)
        self.assertNotIn("拒绝启动人体跟随", start_block)
        self.assertNotIn("人体跟随已停止", safety_block)
        self.assertNotIn("stop_person_follow", safety_block)

    def test_manual_control_paths_do_not_force_person_follow_cleanup_when_inactive(self):
        text = SERVER_FILE.read_text(encoding="utf-8")

        self.assertIn("def stop_person_follow_if_active(", text)
        for marker, reason in (
            ('def api_cmd_vel():', "手动方向控制接管"),
            ('def api_manual_stop():', "手动停止"),
        ):
            block = text.split(marker, 1)[1].split("\ndef ", 1)[0]
            self.assertIn('stop_person_follow_if_active("{}")'.format(reason), block)
            self.assertNotIn('stop_person_follow("{}")'.format(reason), block)

    def test_person_follow_status_is_exposed(self):
        text = SERVER_FILE.read_text(encoding="utf-8")

        self.assertIn('"person_follow": person_follow_snapshot()', text)

    def test_person_follow_process_matching_does_not_match_shell_itself(self):
        text = SERVER_FILE.read_text(encoding="utf-8")

        self.assertIn("[p]erson_follow.py|[p]erson_follow_node|[p]erson_follow_adapter.py", text)
        self.assertNotIn("pgrep -f 'person_follow.py|person_follow_node'", text)
        self.assertNotIn("pkill -f 'person_follow.py|person_follow_node'", text)

    def test_person_follow_camera_bridge_uses_direct_nohup_python(self):
        text = SERVER_FILE.read_text(encoding="utf-8")
        block = text.split("def start_person_follow_camera_bridge():", 1)[1].split(
            "\ndef stop_person_follow_camera_bridge", 1
        )[0]

        self.assertIn("nohup python3 -u /home/ucar/web_panel/ucar_camera_bridge.py", block)
        self.assertNotIn("nohup bash -lc 'exec python3 /home/ucar/web_panel/ucar_camera_bridge.py", block)

    def test_person_follow_hard_releases_video_device_before_camera_bridge(self):
        text = SERVER_FILE.read_text(encoding="utf-8")
        release_block = text.split("def release_web_camera_for_person_follow():", 1)[1].split(
            "\ndef start_person_follow_camera_bridge", 1
        )[0]
        start_block = text.split("def start_person_follow_worker", 1)[1].split("\ndef kill_nav_processes", 1)[0]

        self.assertIn("hard_release_video_device_for_person_follow()", release_block)
        self.assertIn("line_follow_recorder.stop()", release_block)
        self.assertIn("fuser /dev/video0 /dev/ucar_video", text)
        self.assertIn("kill -9", text)
        self.assertIn("time.sleep(1.5)", text)
        self.assertLess(
            start_block.index("release_web_camera_for_person_follow()"),
            start_block.index("start_ucar_camera_for_person_follow()"),
        )
        self.assertIn("wait_for_camera_message(timeout=10.0)", start_block)
        self.assertIn("if not camera_ready:", start_block)
        self.assertLess(start_block.index("if not camera_ready:"), start_block.index("PERSON_FOLLOW_COMMAND"))
        self.assertNotIn('wait_for_ros_name("^/ucar_camera/image_raw$"', start_block)

    def test_person_follow_services_confirms_follow_enabled_before_release(self):
        text = SERVER_FILE.read_text(encoding="utf-8")
        ready_block = text.split("def wait_for_person_follow_ready(", 1)[1].split(
            "\ndef wait_for_person_follow_stopped", 1
        )[0]
        service_block = text.split("def start_person_follow_services", 1)[1].split(
            "\ndef person_follow_snapshot", 1
        )[0]

        self.assertIn("/person_follow/start_person_detect", ready_block)
        self.assertIn("/person_follow/start_person_follow", ready_block)
        self.assertIn("timeout=60.0", text)
        self.assertIn("timeout=8", ready_block)
        self.assertNotIn("nohup bash -lc", service_block)
        self.assertIn("/person_follow/start_person_follow '{}'", service_block)
        self.assertNotIn("/person_follow/get_detect_info", service_block)
        self.assertNotIn("target_id: 0", service_block)
        self.assertIn("def start_person_follow_services(timeout=20.0, generation=None):", text)
        self.assertIn("while time.time() < deadline:", service_block)
        self.assertIn("generation != person_follow_start_generation", service_block)
        self.assertIn("adapter_status = person_follow_adapter_status()", service_block)
        self.assertIn('adapter_status.get("follow_enabled")', service_block)
        self.assertIn('if follow["returncode"] == 0 and follow_enabled:', service_block)
        self.assertIn("time.sleep(1.0)", service_block)
        self.assertIn("PERSON_FOLLOW_STATUS_FILE", text)
        self.assertIn("person_follow_adapter_status", text)

    def test_person_follow_camera_ready_waits_for_full_image_message(self):
        text = SERVER_FILE.read_text(encoding="utf-8")
        block = text.split("def wait_for_camera_message(", 1)[1].split("\ndef wait_for_person_follow_ready", 1)[0]

        self.assertIn("rospy.wait_for_message", block)
        self.assertIn('"/ucar_camera/image_raw"', block)
        self.assertIn("Image", block)
        self.assertNotIn("/ucar_camera/image_raw/header", block)
        self.assertNotIn("rostopic echo", block)

    def test_camera_bridge_exits_after_30_consecutive_read_failures(self):
        text = CAMERA_BRIDGE_FILE.read_text(encoding="utf-8")

        self.assertIn("consecutive_failures >= 30", text)
        self.assertIn("rospy.logerr", text)
        self.assertIn("camera read failed 30 consecutive frames", text)
        self.assertIn("raise RuntimeError", text)

    def test_person_follow_snapshot_marks_dead_process_as_stopped(self):
        text = SERVER_FILE.read_text(encoding="utf-8")
        block = text.split("def person_follow_snapshot():", 1)[1].split("\ndef stop_person_follow", 1)[0]

        self.assertIn("person_follow_process_running_cached", block)
        self.assertIn("was_running = latest_person_follow.get(\"running\")", block)
        self.assertIn("if pid is None and was_running:", block)

    def test_person_follow_status_uses_cached_process_probe(self):
        text = SERVER_FILE.read_text(encoding="utf-8")

        self.assertIn("person_follow_probe = {", text)
        self.assertIn("def person_follow_process_running_cached(", text)
        status_block = text.split("def api_person_follow_status():", 1)[1].split("\n\n@app.route", 1)[0]
        self.assertIn("person_follow_snapshot()", status_block)


if __name__ == "__main__":
    unittest.main()
