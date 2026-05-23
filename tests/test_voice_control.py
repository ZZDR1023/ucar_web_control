import unittest
import math

from web_panel.voice_control import build_route_point_aliases, odom_target_for_step, parse_voice_command


class VoiceControlTest(unittest.TestCase):
    def test_stop_words_have_highest_priority(self):
        command = parse_voice_command("小车停止，别前进")

        self.assertEqual(command["action"], "stop")
        self.assertEqual(command["linear_x"], 0.0)
        self.assertEqual(command["angular_z"], 0.0)

    def test_basic_motion_commands_are_low_speed(self):
        cases = {
            "前进": ("cmd_vel", 0.10, 0.0),
            "后退": ("cmd_vel", -0.08, 0.0),
            "左转": ("cmd_vel", 0.0, 0.45),
            "右转": ("cmd_vel", 0.0, -0.45),
        }

        for text, expected in cases.items():
            with self.subTest(text=text):
                command = parse_voice_command(text)
                self.assertEqual(command["action"], expected[0])
                self.assertAlmostEqual(command["linear_x"], expected[1])
                self.assertAlmostEqual(command["angular_z"], expected[2])

    def test_compound_distance_and_turn_command_becomes_sequence(self):
        command = parse_voice_command("前进1米再左转")

        self.assertEqual(command["action"], "sequence")
        self.assertEqual(len(command["steps"]), 2)
        self.assertEqual(command["steps"][0]["kind"], "move")
        self.assertAlmostEqual(command["steps"][0]["distance_m"], 1.0)
        self.assertAlmostEqual(command["steps"][0]["linear_x"], 0.10)
        self.assertEqual(command["steps"][1]["kind"], "turn")
        self.assertAlmostEqual(command["steps"][1]["angle_deg"], 90.0)
        self.assertGreater(command["duration_s"], 0.0)

    def test_single_distance_command_is_finite_sequence(self):
        command = parse_voice_command("后退0.5米")

        self.assertEqual(command["action"], "sequence")
        self.assertEqual(command["steps"][0]["kind"], "move")
        self.assertAlmostEqual(command["steps"][0]["distance_m"], 0.5)
        self.assertLess(command["steps"][0]["linear_x"], 0.0)

    def test_turn_angle_command_is_finite_sequence(self):
        command = parse_voice_command("右转45度")

        self.assertEqual(command["action"], "sequence")
        self.assertEqual(command["steps"][0]["kind"], "turn")
        self.assertAlmostEqual(command["steps"][0]["angle_deg"], 45.0)
        self.assertLess(command["steps"][0]["angular_z"], 0.0)

    def test_turn_step_odom_target_defaults_to_requested_angle(self):
        command = parse_voice_command("左转90度")

        target = odom_target_for_step(command["steps"][0])

        self.assertAlmostEqual(target["angle_rad"], math.radians(90.0))

    def test_route_point_aliases_include_chinese_ordinals(self):
        aliases = build_route_point_aliases("t1", 0)

        self.assertIn("t1中的1号巡逻点", aliases)
        self.assertIn("t1中的一号巡逻点", aliases)
        self.assertIn("t1第一个点", aliases)
        self.assertNotIn("1号巡逻点", aliases)

    def test_current_route_point_aliases_include_generic_ordinals(self):
        aliases = build_route_point_aliases("t1", 2, include_generic=True)

        self.assertIn("t1中的3号巡逻点", aliases)
        self.assertIn("3号巡逻点", aliases)
        self.assertIn("第三个点", aliases)

    def test_patrol_commands_are_distinct_from_navigation(self):
        self.assertEqual(parse_voice_command("开始巡逻")["action"], "patrol_start")
        self.assertEqual(parse_voice_command("暂停巡逻")["action"], "patrol_pause")
        self.assertEqual(parse_voice_command("继续巡逻")["action"], "patrol_resume")
        self.assertEqual(parse_voice_command("停止巡逻")["action"], "patrol_stop")

    def test_person_follow_commands_are_distinct_from_motion(self):
        self.assertEqual(parse_voice_command("跟着我")["action"], "person_follow_start")
        self.assertEqual(parse_voice_command("开始人体跟随")["action"], "person_follow_start")
        self.assertEqual(parse_voice_command("结束跟随")["action"], "person_follow_stop")

    def test_named_goal_matches_saved_patrol_points(self):
        locations = [
            {"name": "门口", "x": 1.2, "y": -0.3, "yaw": 0.5},
            {"name": "桌边", "x": 0.4, "y": 1.1, "yaw": 1.57},
        ]

        command = parse_voice_command("去门口", locations=locations)

        self.assertEqual(command["action"], "goal")
        self.assertEqual(command["target_name"], "门口")
        self.assertAlmostEqual(command["x"], 1.2)
        self.assertAlmostEqual(command["y"], -0.3)
        self.assertAlmostEqual(command["yaw"], 0.5)

    def test_named_goal_matches_route_point_alias(self):
        locations = [
            {
                "name": "巡逻点 1",
                "aliases": build_route_point_aliases("t1", 0),
                "x": 1.0,
                "y": 2.0,
                "yaw": 0.3,
            },
        ]

        command = parse_voice_command("去t1中的1号巡逻点", locations=locations)

        self.assertEqual(command["action"], "goal")
        self.assertEqual(command["target_name"], "巡逻点 1")
        self.assertAlmostEqual(command["x"], 1.0)

    def test_named_goal_matches_chinese_route_point_alias(self):
        locations = [
            {
                "name": "巡逻点 1",
                "aliases": build_route_point_aliases("t1", 0),
                "x": 1.0,
                "y": 2.0,
                "yaw": 0.3,
            },
        ]

        command = parse_voice_command("去t1中的一号巡逻点", locations=locations)

        self.assertEqual(command["action"], "goal")
        self.assertEqual(command["target_name"], "巡逻点 1")

    def test_generic_patrol_point_matches_current_route_alias(self):
        locations = [
            {
                "name": "巡逻点 3",
                "aliases": build_route_point_aliases("t1", 2, include_generic=True),
                "x": 3.0,
                "y": 2.0,
                "yaw": 0.3,
            },
        ]

        command = parse_voice_command("去3号巡逻点", locations=locations)

        self.assertEqual(command["action"], "goal")
        self.assertEqual(command["target_name"], "巡逻点 3")
        self.assertAlmostEqual(command["x"], 3.0)

    def test_unknown_command_reports_error_without_motion(self):
        command = parse_voice_command("唱一首歌")

        self.assertEqual(command["action"], "unknown")
        self.assertFalse(command["ok"])
        self.assertIn("未识别", command["message"])


if __name__ == "__main__":
    unittest.main()
