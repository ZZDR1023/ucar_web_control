import importlib.util
import pathlib
import sys
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
ADAPTER_FILE = ROOT / "web_panel" / "person_follow_adapter.py"


class DummyPublisher:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class DummySubscriber:
    def __init__(self, *args, **kwargs):
        pass


class DummyService:
    def __init__(self, *args, **kwargs):
        pass


class DummyTimer:
    def __init__(self, *args, **kwargs):
        pass


class DummyDuration:
    def __init__(self, value):
        self.value = value


class DummyBridge:
    pass


class DummyHog:
    def setSVMDetector(self, _detector):
        pass

    def detectMultiScale(self, *args, **kwargs):
        return [], []


class DummyCapture:
    def __init__(self, *args, **kwargs):
        self._opened = False
        self.released = False

    def isOpened(self):
        return self._opened

    def set(self, *args, **kwargs):
        pass

    def read(self):
        return False, None

    def release(self):
        self.released = True
        self._opened = False


class DummyDnn:
    @staticmethod
    def readNetFromDarknet(*args, **kwargs):
        return types.SimpleNamespace(
            getLayerNames=lambda: [],
            getUnconnectedOutLayers=lambda: [],
            setInput=lambda _blob: None,
            forward=lambda _layers: [],
        )

    @staticmethod
    def blobFromImage(*args, **kwargs):
        return object()

    @staticmethod
    def NMSBoxes(boxes, confidences, score_threshold, nms_threshold):
        return list(range(len(boxes)))


class DummyTwist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0)
        self.angular = types.SimpleNamespace(z=0.0)


class PersonFollowAdapterTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.modules["cv2"] = types.SimpleNamespace(
            HOGDescriptor=lambda: DummyHog(),
            HOGDescriptor_getDefaultPeopleDetector=lambda: object(),
            VideoCapture=lambda *args, **kwargs: DummyCapture(),
            CAP_V4L2=0,
            CAP_PROP_FRAME_WIDTH=3,
            CAP_PROP_FRAME_HEIGHT=4,
            CAP_PROP_FPS=5,
            COLOR_BGR2GRAY=0,
            cvtColor=lambda frame, _code: frame,
            equalizeHist=lambda frame: frame,
            resize=lambda frame, _size: frame,
            convertScaleAbs=lambda frame, alpha=1.0, beta=0.0: types.SimpleNamespace(enhanced=True, source=frame, alpha=alpha, beta=beta),
            dnn=DummyDnn,
        )
        sys.modules["rospy"] = types.SimpleNamespace(
            Publisher=DummyPublisher,
            Subscriber=DummySubscriber,
            Service=DummyService,
            Timer=DummyTimer,
            Duration=DummyDuration,
            get_param=lambda _name, default=None: default,
            init_node=lambda *args, **kwargs: None,
            on_shutdown=lambda _callback: None,
            spin=lambda: None,
        )
        sys.modules["cv_bridge"] = types.SimpleNamespace(CvBridge=DummyBridge)
        sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
        sys.modules["geometry_msgs.msg"] = types.SimpleNamespace(Twist=DummyTwist)
        sys.modules["nav_msgs"] = types.ModuleType("nav_msgs")
        sys.modules["nav_msgs.msg"] = types.SimpleNamespace(Odometry=object)
        sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
        sys.modules["sensor_msgs.msg"] = types.SimpleNamespace(Image=object, LaserScan=object)
        sys.modules["std_msgs"] = types.ModuleType("std_msgs")
        sys.modules["std_msgs.msg"] = types.SimpleNamespace(String=lambda data="": types.SimpleNamespace(data=data))
        sys.modules["std_srvs"] = types.ModuleType("std_srvs")
        sys.modules["std_srvs.srv"] = types.SimpleNamespace(
            Trigger=object,
            TriggerResponse=lambda success=False, message="": types.SimpleNamespace(success=success, message=message),
        )
        spec = importlib.util.spec_from_file_location("person_follow_adapter", ADAPTER_FILE)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def make_controller(self):
        controller = self.module.PersonFollowController()
        controller.status_pub = DummyPublisher()
        controller.cmd_pub = DummyPublisher()
        return controller

    def test_centered_far_target_moves_forward_without_turning(self):
        controller = self.make_controller()
        detection = self.module.Detection(280, 120, 80, 120, 1.0)

        command = controller._compute_command(detection, 640, 360)

        self.assertTrue(command.detected)
        self.assertGreater(command.linear_x, 0.0)
        self.assertAlmostEqual(command.angular_z, 0.0, places=3)

    def test_left_target_uses_mirrored_turn_direction_and_slows_forward_motion(self):
        controller = self.make_controller()
        detection = self.module.Detection(80, 120, 80, 120, 1.0)

        command = controller._compute_command(detection, 640, 360)

        self.assertLess(command.angular_z, 0.0)
        self.assertGreaterEqual(command.linear_x, 0.0)
        self.assertLessEqual(command.linear_x, controller.max_linear_speed * 0.55)

    def test_slightly_off_center_target_does_not_turn(self):
        controller = self.make_controller()
        detection = self.module.Detection(235, 120, 80, 120, 1.0)

        command = controller._compute_command(detection, 640, 360)

        self.assertEqual(command.angular_z, 0.0)

    def test_mid_distance_target_moves_forward_after_retuning(self):
        controller = self.make_controller()
        detection = self.module.Detection(260, 60, 120, 200, 1.0)

        command = controller._compute_command(detection, 640, 360)

        self.assertGreater(command.linear_x, 0.0)

    def test_far_target_reduces_turning_until_robot_gets_closer(self):
        controller = self.make_controller()
        far_detection = self.module.Detection(500, 120, 60, 60, 1.0)
        near_detection = self.module.Detection(500, 80, 180, 180, 1.0)

        far_command = controller._compute_command(far_detection, 640, 360)
        controller.prev_angular = 0.0
        near_command = controller._compute_command(near_detection, 640, 360)

        self.assertLess(abs(far_command.angular_z), abs(near_command.angular_z))

    def test_low_light_frame_is_enhanced_for_detection(self):
        controller = self.make_controller()
        frame = types.SimpleNamespace(shape=(360, 640, 3), mean=lambda: 30.0)

        enhanced = controller._enhance_low_light(frame)

        self.assertTrue(enhanced.enhanced)
        self.assertEqual(enhanced.alpha, controller.low_light_alpha)
        self.assertEqual(enhanced.beta, controller.low_light_beta)

    def test_close_target_stops_forward_motion(self):
        controller = self.make_controller()
        controller.prev_linear = 0.08
        controller.prev_angular = 0.2
        detection = self.module.Detection(250, 20, 140, 290, 1.0)

        command = controller._compute_command(detection, 640, 360)

        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)
        self.assertEqual(controller.prev_linear, 0.0)
        self.assertEqual(controller.prev_angular, 0.0)

    def test_centered_stopped_person_holds_position_before_too_close(self):
        controller = self.make_controller()
        controller.prev_linear = 0.12
        detection = self.module.Detection(250, 55, 140, 230, 1.0)

        first = controller._compute_command(detection, 640, 360)
        second = controller._compute_command(detection, 640, 360)

        self.assertEqual(first.linear_x, 0.0)
        self.assertEqual(second.linear_x, 0.0)
        self.assertIn("target stopped", second.detail)

    def test_front_obstacle_forces_zero_motion(self):
        controller = self.make_controller()
        controller.prev_linear = 0.08
        controller.prev_angular = 0.2
        controller.last_front_distance = controller.front_stop_distance - 0.01
        detection = self.module.Detection(280, 120, 80, 120, 1.0)

        command = controller._compute_command(detection, 640, 360)

        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)
        self.assertEqual(controller.prev_linear, 0.0)
        self.assertEqual(controller.prev_angular, 0.0)

    def test_front_obstacle_margin_slows_without_stopping(self):
        controller = self.make_controller()
        controller.last_front_distance = controller.front_stop_distance + controller.obstacle_slow_margin * 0.5
        detection = self.module.Detection(280, 120, 80, 120, 1.0)

        slowed = controller._compute_command(detection, 640, 360)
        controller.last_front_distance = None
        controller.prev_linear = 0.0
        normal = controller._compute_command(detection, 640, 360)

        self.assertGreater(slowed.linear_x, 0.0)
        self.assertLess(slowed.linear_x, normal.linear_x)

    def test_person_scan_distance_uses_visual_direction_only(self):
        controller = self.make_controller()
        controller.last_scan_ranges = [2.0] * 181
        controller.last_scan_angle_min = -self.module.math.pi / 2
        controller.last_scan_angle_increment = self.module.math.pi / 180
        controller.last_scan_range_min = 0.05
        controller.last_scan_range_max = 6.0
        controller.last_scan_ranges[90] = 0.4
        controller.last_scan_ranges[105] = 1.2

        centered_distance = controller._person_scan_distance(0.0)
        right_distance = controller._person_scan_distance(0.5)

        self.assertAlmostEqual(centered_distance, 0.4)
        self.assertAlmostEqual(right_distance, 1.2)

    def test_turn_node_distance_prefers_scan_distance_when_available(self):
        controller = self.make_controller()
        controller.last_person_scan_distance = 0.5

        distance = controller._estimate_turn_node_distance(0.22)

        self.assertGreaterEqual(distance, controller.min_turn_node_distance)
        self.assertLessEqual(distance, controller.max_turn_node_distance)
        self.assertLess(distance, controller._estimate_turn_node_distance_from_height(0.22))

    def test_scan_distance_must_match_visual_height_estimate(self):
        controller = self.make_controller()
        controller.last_scan_ranges = [2.0] * 181
        controller.last_scan_angle_min = -self.module.math.pi / 2
        controller.last_scan_angle_increment = self.module.math.pi / 180
        controller.last_scan_range_min = 0.05
        controller.last_scan_range_max = 6.0
        controller.last_scan_ranges[90] = 0.2
        detection = self.module.Detection(280, 120, 80, 80, 1.0)

        controller._compute_command(detection, 640, 360)

        self.assertIsNone(controller.last_person_scan_distance)

    def test_turning_uses_smoothed_error_to_avoid_cutting_corners(self):
        controller = self.make_controller()
        detection = self.module.Detection(520, 120, 80, 120, 1.0)

        first = controller._compute_command(detection, 640, 360)
        second = controller._compute_command(detection, 640, 360)

        self.assertLess(abs(first.angular_z), abs(second.angular_z))

    def test_far_lateral_target_follows_trail_before_current_position(self):
        controller = self.make_controller()
        centered = self.module.Detection(280, 120, 80, 120, 1.0)
        lateral = self.module.Detection(520, 120, 80, 120, 1.0)

        for _ in range(controller.path_follow_delay_frames + controller.path_node_stable_frames):
            controller._compute_command(centered, 640, 360)
        first_lateral = controller._compute_command(lateral, 640, 360)
        for _ in range(controller.path_follow_delay_frames + controller.path_node_stable_frames + 1):
            later_lateral = controller._compute_command(lateral, 640, 360)

        self.assertLess(abs(first_lateral.angular_z), 0.04)
        self.assertGreater(abs(later_lateral.angular_z), abs(first_lateral.angular_z))

    def test_jittering_detection_does_not_pollute_path_nodes(self):
        controller = self.make_controller()
        centered = self.module.Detection(280, 120, 80, 120, 1.0)
        jitter = self.module.Detection(520, 120, 80, 120, 1.0)

        for _ in range(controller.path_follow_delay_frames + controller.path_node_stable_frames):
            controller._compute_command(centered, 640, 360)
        history_before = list(controller.target_center_history)
        controller._compute_command(jitter, 640, 360)

        self.assertEqual(list(controller.target_center_history), history_before)
        self.assertLess(abs(controller.smoothed_center_error), controller.center_deadband)

    def test_centered_target_clears_previous_turning_memory(self):
        controller = self.make_controller()
        lateral = self.module.Detection(500, 120, 80, 120, 1.0)
        centered = self.module.Detection(280, 120, 80, 120, 1.0)

        for _ in range(controller.path_follow_delay_frames + controller.path_node_stable_frames + 3):
            controller._compute_command(lateral, 640, 360)
        controller.prev_angular = controller.last_command.angular_z if hasattr(controller, "last_command") else 0.4
        command = controller._compute_command(centered, 640, 360)

        self.assertLess(abs(command.angular_z), 0.08)
        self.assertLess(abs(controller.smoothed_center_error), controller.center_deadband)

    def test_lateral_motion_records_turn_intent_direction(self):
        controller = self.make_controller()
        left_sequence = [
            self.module.Detection(260, 120, 80, 120, 1.0),
            self.module.Detection(220, 120, 80, 120, 1.0),
            self.module.Detection(170, 120, 80, 120, 1.0),
            self.module.Detection(120, 120, 80, 120, 1.0),
        ]

        for detection in left_sequence:
            controller._compute_command(detection, 640, 360)

        self.assertEqual(controller.pending_turn_direction, -1)

    def test_lost_target_search_uses_recorded_turn_intent(self):
        controller = self.make_controller()
        controller.pending_turn_direction = -1

        command = controller._compute_search_command()

        self.assertLess(command.angular_z, 0.0)

    def test_lost_target_drives_to_estimated_turn_point_then_stops(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = self.module.time.time()
        controller.last_frame_stamp = self.module.time.time()
        controller.last_front_distance = controller.front_stop_distance + 0.5
        controller.pending_turn_direction = -1
        controller.turn_node_remaining_distance = 0.20
        controller.prev_linear = 0.12
        controller.prev_angular = 0.3
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [])

        controller.on_timer(None)
        first = controller.cmd_pub.messages[-1]
        controller.turn_node_remaining_distance = 0.0
        controller.on_timer(None)
        second = controller.cmd_pub.messages[-1]

        self.assertGreater(first.linear.x, 0.0)
        self.assertEqual(first.angular.z, 0.0)
        self.assertEqual(second.linear.x, 0.0)
        self.assertEqual(second.angular.z, 0.0)
        self.assertIn("到达转弯点", controller.last_command.detail)

    def test_turn_intent_distance_uses_last_visible_height(self):
        controller = self.make_controller()

        far = controller._estimate_turn_node_distance(0.22)
        near = controller._estimate_turn_node_distance(0.50)

        self.assertGreater(far, near)

    def test_turn_node_uses_odom_distance_before_stopping(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = self.module.time.time()
        controller.last_frame_stamp = self.module.time.time()
        controller.last_front_distance = controller.front_stop_distance + 0.5
        controller.pending_turn_direction = -1
        controller.turn_node_remaining_distance = 0.20
        controller.turn_node_target_distance = 0.20
        controller.turn_node_start_odom = (1.0, 1.0)
        controller.current_odom = (1.05, 1.0)
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [])

        controller.on_timer(None)
        first = controller.cmd_pub.messages[-1]
        controller.current_odom = (1.21, 1.0)
        controller.on_timer(None)
        second = controller.cmd_pub.messages[-1]

        self.assertGreater(first.linear.x, 0.0)
        self.assertEqual(first.angular.z, 0.0)
        self.assertEqual(second.linear.x, 0.0)
        self.assertEqual(second.angular.z, 0.0)
        self.assertIn("到达转弯点", controller.last_command.detail)

    def test_turn_intent_can_be_overwritten_by_opposite_motion(self):
        controller = self.make_controller()
        controller.pending_turn_direction = -1
        right_sequence = [
            self.module.Detection(340, 120, 80, 120, 1.0),
            self.module.Detection(390, 120, 80, 120, 1.0),
            self.module.Detection(440, 120, 80, 120, 1.0),
            self.module.Detection(490, 120, 80, 120, 1.0),
        ]

        for detection in right_sequence:
            controller._compute_command(detection, 640, 360)

        self.assertEqual(controller.pending_turn_direction, 1)
        self.assertGreater(controller._compute_search_command().angular_z, 0.0)

    def test_centered_target_does_not_record_turn_intent(self):
        controller = self.make_controller()
        centered = self.module.Detection(280, 120, 80, 120, 1.0)

        for _ in range(controller.turn_intent_frames + 1):
            controller._compute_command(centered, 640, 360)

        self.assertEqual(controller.pending_turn_direction, 0)

    def test_start_follow_resets_previous_turn_memory(self):
        controller = self.make_controller()
        controller.smoothed_center_error = 0.8
        controller.last_target_center_x = 580
        controller.last_turn_sign = -1
        controller.target_center_history.extend([560.0, 570.0])

        controller.start_follow(None)

        self.assertEqual(controller.smoothed_center_error, 0.0)
        self.assertIsNone(controller.last_target_center_x)
        self.assertEqual(controller.last_turn_sign, 0)
        self.assertEqual(len(controller.target_center_history), 0)

    def test_target_jump_resets_trail_before_following_new_target(self):
        controller = self.make_controller()
        centered = self.module.Detection(280, 120, 80, 120, 1.0)
        left = self.module.Detection(-220, 120, 80, 120, 1.0)

        for _ in range(controller.path_follow_delay_frames + 1):
            controller._compute_command(centered, 640, 360)
        command = controller._compute_command(left, 640, 360)

        self.assertLess(command.angular_z, 0.0)
        self.assertLessEqual(len(controller.target_center_history), 1)

    def test_larger_new_target_is_not_hidden_by_old_center_bias(self):
        controller = self.make_controller()
        controller.last_target_center_x = 320
        old_nearby = self.module.Detection(280, 140, 50, 70, 0.6)
        larger_new = self.module.Detection(470, 80, 150, 190, 0.9)

        selected = controller._select_detection([old_nearby, larger_new])

        self.assertEqual(selected, larger_new)
        self.assertIsNone(controller.last_target_center_x)
        self.assertEqual(len(controller.target_center_history), 0)

    def test_publish_stop_resets_trail_and_turn_memory(self):
        controller = self.make_controller()
        controller.smoothed_center_error = -0.6
        controller.last_target_center_x = 60
        controller.last_turn_sign = 1
        controller.target_center_history.extend([70.0, 80.0])

        controller.publish_stop("test stop")

        self.assertEqual(controller.smoothed_center_error, 0.0)
        self.assertIsNone(controller.last_target_center_x)
        self.assertEqual(controller.last_turn_sign, 0)
        self.assertEqual(len(controller.target_center_history), 0)

    def test_lost_target_search_turns_without_forward_motion(self):
        controller = self.make_controller()
        command = controller._compute_search_command()

        self.assertEqual(command.linear_x, 0.0)
        self.assertNotEqual(command.angular_z, 0.0)
        self.assertFalse(command.detected)

    def test_short_lost_target_coasts_before_searching(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = self.module.time.time()
        controller.last_frame_stamp = self.module.time.time()
        controller.last_front_distance = controller.front_stop_distance + 0.5
        controller.prev_linear = 0.18
        controller.prev_angular = 0.2
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [])

        controller.on_timer(None)

        self.assertGreater(controller.cmd_pub.messages[-1].linear.x, 0.0)
        self.assertLessEqual(controller.cmd_pub.messages[-1].linear.x, controller.lost_coast_max_linear)
        self.assertEqual(controller.cmd_pub.messages[-1].angular.z, 0.0)
        self.assertIn("直线滑行", controller.last_command.detail)

    def test_repeated_lost_target_coasts_before_searching(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = self.module.time.time()
        controller.last_frame_stamp = self.module.time.time()
        controller.last_front_distance = controller.front_stop_distance + 0.5
        controller.prev_linear = 0.18
        controller.prev_angular = 0.4
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [])

        for _ in range(controller.lost_coast_frames):
            controller.on_timer(None)

        self.assertGreater(controller.cmd_pub.messages[-1].linear.x, 0.0)
        self.assertEqual(controller.cmd_pub.messages[-1].angular.z, 0.0)
        self.assertIn("直线滑行", controller.last_command.detail)

    def test_lost_target_front_obstacle_stops_before_coasting(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = self.module.time.time()
        controller.last_frame_stamp = self.module.time.time()
        controller.last_front_distance = controller.front_stop_distance - 0.01
        controller.prev_linear = 0.18
        controller.prev_angular = 0.4
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [])

        controller.on_timer(None)

        self.assertEqual(controller.cmd_pub.messages[-1].linear.x, 0.0)
        self.assertEqual(controller.cmd_pub.messages[-1].angular.z, 0.0)
        self.assertIn("front obstacle", controller.last_command.detail)

    def test_turn_node_stops_when_front_scan_has_no_valid_distance(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = self.module.time.time()
        controller.last_frame_stamp = self.module.time.time()
        controller.last_front_distance = None
        controller.pending_turn_direction = -1
        controller.turn_node_remaining_distance = 0.2
        controller.turn_node_target_distance = 0.2
        controller.turn_node_start_odom = (0.0, 0.0)
        controller.current_odom = (0.0, 0.0)
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [])

        controller.on_timer(None)

        self.assertEqual(controller.cmd_pub.messages[-1].linear.x, 0.0)
        self.assertEqual(controller.cmd_pub.messages[-1].angular.z, 0.0)
        self.assertIn("front scan unavailable", controller.last_command.detail)

    def test_turn_intent_lost_target_stops_after_lost_frame_limit(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = self.module.time.time()
        controller.last_frame_stamp = self.module.time.time()
        controller.last_front_distance = controller.front_stop_distance + 0.5
        controller.pending_turn_direction = -1
        controller.turn_node_remaining_distance = 0.20
        controller.lost_frames = controller.stop_lost_frames
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [])

        controller.on_timer(None)

        self.assertEqual(controller.cmd_pub.messages[-1].linear.x, 0.0)
        self.assertEqual(controller.cmd_pub.messages[-1].angular.z, 0.0)
        self.assertIn("No person detection", controller.last_command.detail)

    def test_fallback_search_keeps_last_turn_direction(self):
        controller = self.make_controller()
        controller.pending_turn_direction = 0
        controller.last_turn_sign = -1

        command = controller._compute_search_command()

        self.assertLess(command.angular_z, 0.0)

    def test_lost_target_front_obstacle_margin_slows_coasting(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = self.module.time.time()
        controller.last_frame_stamp = self.module.time.time()
        controller.last_front_distance = controller.front_stop_distance + controller.obstacle_slow_margin * 0.5
        controller.prev_linear = 0.18
        controller.prev_angular = 0.4
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [])

        controller.on_timer(None)

        self.assertGreater(controller.cmd_pub.messages[-1].linear.x, 0.0)
        self.assertLessEqual(controller.cmd_pub.messages[-1].linear.x, controller.lost_coast_max_linear * 0.5 + 1e-9)

    def test_following_with_stale_scan_publishes_stop_before_motion(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = 0.0
        controller.last_frame_stamp = self.module.time.time()
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [self.module.Detection(280, 120, 80, 120, 1.0)])

        controller.on_timer(None)

        self.assertEqual(controller.cmd_pub.messages[-1].linear.x, 0.0)
        self.assertEqual(controller.cmd_pub.messages[-1].angular.z, 0.0)
        self.assertIn("激光数据超时", controller.last_command.detail)

    def test_search_with_stale_scan_publishes_stop_before_turning(self):
        controller = self.make_controller()
        controller.follow_enabled = True
        controller.last_scan_stamp = 0.0
        controller.last_frame_stamp = self.module.time.time()
        controller.pull_frame = lambda: types.SimpleNamespace(shape=(360, 640, 3))
        controller._detect_people = lambda frame: (frame, [])

        controller.on_timer(None)

        self.assertEqual(controller.cmd_pub.messages[-1].linear.x, 0.0)
        self.assertEqual(controller.cmd_pub.messages[-1].angular.z, 0.0)
        self.assertIn("激光数据超时", controller.last_command.detail)

    def test_cleanup_releases_camera_and_publishes_stop(self):
        controller = self.make_controller()
        capture = DummyCapture()
        controller.cap = capture

        controller.cleanup()

        self.assertTrue(capture.released)
        self.assertIsNone(controller.cap)
        self.assertEqual(controller.cmd_pub.messages[-1].linear.x, 0.0)

    def test_yolo_parameters_are_configured_for_darknet_backend(self):
        controller = self.make_controller()

        self.assertTrue(controller.yolo_cfg.endswith("yolov3-tiny.cfg"))
        self.assertTrue(controller.yolo_weights.endswith("yolov3-tiny.weights"))
        self.assertEqual(controller.yolo_input_size, 416)
        self.assertGreater(controller.yolo_confidence, 0.0)

    def test_follow_tuning_is_more_responsive_for_walking_person(self):
        controller = self.make_controller()

        self.assertEqual(controller.max_linear_speed, 0.36)
        self.assertEqual(controller.max_angular_speed, 0.90)
        self.assertEqual(controller.angular_gain, 0.85)
        self.assertEqual(controller.center_deadband, 0.16)
        self.assertEqual(controller.smoothing, 0.45)
        self.assertEqual(controller.lost_coast_frames, 6)

    def test_pose_model_parameters_match_factory_person_follow_package(self):
        controller = self.make_controller()

        self.assertTrue(controller.pose_weights.endswith("scripts/weights/person_detect.pth"))
        self.assertTrue(controller.pose_config.endswith("scripts/config/human_pose.json"))
        self.assertEqual(controller.pose_input_size, 224)
        self.assertEqual(controller.pose_height_rate, 3.0)
        self.assertEqual(controller.pose_width_rate, 5.5)
        self.assertEqual(controller.detector_order, "pose,yolo,hog")

    def test_pose_detection_empty_result_falls_back_to_hog(self):
        controller = self.make_controller()
        controller.detector_order = "pose,hog"
        controller.pose_model = object()
        controller._detect_people_with_pose = lambda frame: (frame, [])
        controller._detect_people_with_hog = lambda frame: (frame, [self.module.Detection(1, 2, 30, 80, 0.7)])

        frame, detections = controller._detect_people(types.SimpleNamespace(shape=(360, 640, 3)))

        self.assertEqual(detections[0].score, 0.7)

    def test_pose_detection_failure_falls_back_to_hog(self):
        controller = self.make_controller()
        controller.detector_order = "pose,hog"
        controller.pose_model = object()
        controller._detect_people_with_pose = lambda _frame: (_ for _ in ()).throw(RuntimeError("pose failed"))
        controller._detect_people_with_hog = lambda frame: (frame, [self.module.Detection(1, 2, 30, 80, 0.7)])

        frame, detections = controller._detect_people(types.SimpleNamespace(shape=(360, 640, 3)))

        self.assertFalse(controller.pose_model)
        self.assertTrue(controller.pose_disabled)
        self.assertEqual(detections[0].score, 0.7)


if __name__ == "__main__":
    unittest.main()
