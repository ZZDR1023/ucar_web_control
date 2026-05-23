import unittest

import numpy as np

from web_panel.line_follow import LineFollowConfig, LineFollower


def blank_frame(width=320, height=240, color=(180, 180, 180)):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :] = color
    return frame


def draw_vertical_line(frame, x_center, width, color):
    half = width // 2
    frame[:, max(0, x_center - half):min(frame.shape[1], x_center + half + 1)] = color
    return frame


def draw_horizontal_line(frame, y_center, height, color):
    half = height // 2
    frame[max(0, y_center - half):min(frame.shape[0], y_center + half + 1), :] = color
    return frame


def draw_curved_lane(frame, top_center, bottom_center, road_width=80, boundary_width=6):
    height, width = frame.shape[:2]
    half_road = road_width // 2
    half_boundary = boundary_width // 2
    for y in range(height):
        t = y / float(height - 1)
        center = int(round(top_center + (bottom_center - top_center) * t))
        left = max(0, center - half_road)
        right = min(width, center + half_road)
        frame[y, left:right] = (150, 150, 150)
        frame[y, max(0, left - half_boundary):min(width, left + half_boundary + 1)] = (0, 190, 190)
        frame[y, max(0, right - half_boundary):min(width, right + half_boundary + 1)] = (0, 190, 190)
    return frame


class LineFollowerTest(unittest.TestCase):
    def test_default_config_targets_black_road(self):
        cfg = LineFollowConfig()

        self.assertEqual(cfg.line_color, "black")
        self.assertEqual(cfg.h_min, 0.0)
        self.assertEqual(cfg.h_max, 179.0)
        self.assertEqual(cfg.s_min, 0.0)
        self.assertEqual(cfg.s_max, 80.0)
        self.assertEqual(cfg.v_min, 0.0)
        self.assertEqual(cfg.v_max, 200.0)
        self.assertEqual(cfg.angular_gain, 0.60)
        self.assertNotIn("black_threshold", cfg.to_dict())

    def test_yellow_config_uses_yellow_hsv_defaults(self):
        cfg = LineFollowConfig(line_color="yellow")

        self.assertGreaterEqual(cfg.h_min, 10.0)
        self.assertLessEqual(cfg.h_max, 50.0)
        self.assertGreater(cfg.s_min, 0.0)
        self.assertGreater(cfg.v_min, 0.0)

    def test_legacy_black_threshold_maps_to_hsv_value_max(self):
        cfg = LineFollowConfig.from_dict({"line_color": "black", "black_threshold": 120})

        self.assertEqual(cfg.v_max, 120.0)

    def test_center_black_line_has_near_zero_offset(self):
        frame = draw_vertical_line(blank_frame(), 160, 28, (0, 0, 0))
        follower = LineFollower(LineFollowConfig(line_color="black"))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.offset, 0.0, delta=0.05)
        self.assertGreater(result.area, 1000)
        self.assertGreater(result.linear_x, 0.0)

    def test_left_black_line_has_negative_offset_and_turns_left(self):
        frame = draw_vertical_line(blank_frame(), 80, 28, (0, 0, 0))
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.5))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertLess(result.offset, -0.35)
        self.assertGreater(result.angular_z, 0.0)

    def test_small_straight_offset_keeps_zero_angular_speed(self):
        frame = draw_vertical_line(blank_frame(), 168, 28, (0, 0, 0))
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.5))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.offset, 0.05, delta=0.03)
        self.assertEqual(result.angular_z, 0.0)
        self.assertGreater(result.linear_x, 0.0)

    def test_sharp_turn_crawls_forward_with_limited_angular_speed(self):
        frame = draw_vertical_line(blank_frame(), 75, 28, (0, 0, 0))
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertLess(result.offset, -0.50)
        self.assertEqual(result.linear_x, 0.0)
        self.assertGreater(result.angular_z, 0.0)
        self.assertLessEqual(abs(result.angular_z), 0.42)

    def test_far_curve_does_not_pull_centered_near_field_into_turn(self):
        frame = blank_frame()
        for y in range(frame.shape[0]):
            t = y / float(frame.shape[0] - 1)
            center = int(95 + 65 * t)
            frame[y, max(0, center - 16):min(frame.shape[1], center + 17)] = (0, 0, 0)
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.offset, 0.0, delta=0.08)
        self.assertEqual(result.angular_z, 0.0)
        self.assertGreater(result.linear_x, 0.0)

    def test_ambiguous_branch_does_not_reverse_previous_turn_direction(self):
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, v_max=120))
        first = draw_vertical_line(blank_frame(color=(170, 170, 170)), 95, 28, (20, 20, 20))
        first_result = follower.process(first)
        self.assertTrue(first_result.detected)
        self.assertGreater(first_result.angular_z, 0.0)

        second = blank_frame(color=(170, 170, 170))
        draw_vertical_line(second, 95, 28, (20, 20, 20))
        draw_vertical_line(second, 205, 34, (20, 20, 20))
        second_result = follower.process(second)

        self.assertTrue(second_result.detected)
        self.assertLess(second_result.offset, 0.0)
        self.assertGreaterEqual(second_result.angular_z, 0.0)

    def test_curved_yellow_boundaries_nudge_centered_near_field_into_curve(self):
        frame = draw_curved_lane(blank_frame(color=(225, 225, 225)), top_center=110, bottom_center=160)
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertGreater(result.linear_x, 0.0)
        self.assertLess(result.offset, -0.07)
        self.assertGreater(result.angular_z, 0.0)

    def test_forbidden_contact_overrides_previous_turn_prediction(self):
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, v_max=120))
        first = draw_vertical_line(blank_frame(color=(170, 170, 170)), 95, 28, (20, 20, 20))
        self.assertGreater(follower.process(first).angular_z, 0.0)

        blocked = blank_frame(color=(170, 170, 170))
        draw_vertical_line(blocked, 205, 34, (20, 20, 20))
        draw_horizontal_line(blocked, 236, 6, (0, 190, 190))
        result = follower.process(blocked)

        self.assertTrue(result.forbidden_blocked)
        self.assertEqual(result.linear_x, 0.0)

    def test_right_forbidden_contact_turns_left_away_from_line(self):
        frame = blank_frame(color=(170, 170, 170))
        draw_vertical_line(frame, 225, 34, (20, 20, 20))
        frame[228:240, 235:319] = (0, 190, 190)
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, v_max=120, min_area=100))

        result = follower.process(frame)

        self.assertTrue(result.forbidden_blocked)
        self.assertEqual(result.linear_x, 0.0)
        self.assertGreater(result.angular_z, 0.0)

    def test_left_forbidden_contact_turns_right_away_from_line(self):
        frame = blank_frame(color=(170, 170, 170))
        draw_vertical_line(frame, 95, 34, (20, 20, 20))
        frame[228:240, 0:85] = (0, 190, 190)
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, v_max=120, min_area=100))

        result = follower.process(frame)

        self.assertTrue(result.forbidden_blocked)
        self.assertEqual(result.linear_x, 0.0)
        self.assertLess(result.angular_z, 0.0)

    def test_visible_branch_path_is_not_blocked_by_bottom_boundary_without_path_contact(self):
        frame = blank_frame(color=(170, 170, 170))
        frame[118:206, 35:210] = (120, 120, 120)
        frame[225:240, 0:200] = (0, 190, 190)
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, v_max=130, min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertLess(result.offset, -0.15)
        self.assertGreater(result.angular_z, 0.0)
        self.assertEqual(result.linear_x, 0.0)

    def test_visible_lower_branch_path_rotates_without_forward_motion_until_near_field(self):
        frame = blank_frame(color=(170, 170, 170))
        frame[118:206, 35:210] = (120, 120, 120)
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, v_max=130, min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertGreater(result.center_y, 160)
        self.assertLess(result.offset, -0.15)
        self.assertGreater(result.angular_z, 0.0)
        self.assertEqual(result.linear_x, 0.0)
        self.assertGreaterEqual(abs(result.angular_z), 0.40)

    def test_path_ahead_without_near_field_rotates_in_place(self):
        frame = blank_frame()
        frame[110:175, 145:185] = (0, 0, 0)
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, min_area=100))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertEqual(result.linear_x, 0.0)
        self.assertNotEqual(result.angular_z, 0.0)

    def test_repeated_no_near_field_left_right_search_locks_one_direction(self):
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, min_area=100))
        left = blank_frame()
        left[110:175, 42:82] = (0, 0, 0)
        right = blank_frame()
        right[110:175, 238:278] = (0, 0, 0)

        self.assertGreater(follower.process(left).angular_z, 0.0)
        self.assertGreater(follower.process(right).angular_z, 0.0)
        locked = follower.process(left)
        after_lock = follower.process(right)

        self.assertGreater(locked.angular_z, 0.0)
        self.assertGreater(after_lock.angular_z, 0.0)
        self.assertEqual(after_lock.linear_x, 0.0)

    def test_locked_branch_search_ignores_transient_opposite_near_field(self):
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, min_area=100))
        left_far = blank_frame()
        left_far[110:175, 42:82] = (0, 0, 0)
        right_far = blank_frame()
        right_far[110:175, 238:278] = (0, 0, 0)
        right_near = draw_vertical_line(blank_frame(), 245, 34, (0, 0, 0))
        centered_near = draw_vertical_line(blank_frame(), 160, 96, (0, 0, 0))

        follower.process(left_far)
        follower.process(right_far)
        follower.process(left_far)
        locked = follower.process(right_near)
        recovered = follower.process(centered_near)

        self.assertGreater(locked.angular_z, 0.0)
        self.assertEqual(locked.linear_x, 0.0)
        self.assertGreater(recovered.angular_z, 0.0)
        self.assertEqual(recovered.linear_x, 0.0)

    def test_branch_lock_overrides_opposite_forbidden_avoidance(self):
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, v_max=120, min_area=100))
        follower._activate_branch_search(1.0)
        frame = blank_frame(color=(170, 170, 170))
        draw_vertical_line(frame, 95, 34, (20, 20, 20))
        frame[228:240, 0:85] = (0, 190, 190)

        result = follower.process(frame)

        self.assertTrue(result.forbidden_blocked)
        self.assertEqual(result.linear_x, 0.0)
        self.assertGreater(result.angular_z, 0.0)
        self.assertAlmostEqual(result.angular_z, 0.42, delta=0.01)

    def test_branch_lock_turn_rate_follows_angular_gain(self):
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.25, min_area=100))
        follower._activate_branch_search(1.0)
        right_near = draw_vertical_line(blank_frame(), 245, 34, (0, 0, 0))

        result = follower.process(right_near)

        self.assertEqual(result.linear_x, 0.0)
        self.assertAlmostEqual(result.angular_z, 0.20, delta=0.01)

    def test_branch_lock_does_not_let_transient_offset_poison_turn_sign(self):
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, min_area=100))
        follower._activate_branch_search(1.0)
        opposite_near = draw_vertical_line(blank_frame(), 245, 34, (0, 0, 0))
        follower.process(opposite_near)

        self.assertGreater(follower.last_turn_sign, 0.0)

        centered_near = draw_vertical_line(blank_frame(), 160, 96, (0, 0, 0))
        follower.process(centered_near)
        follower.process(centered_near)
        follower.process(centered_near)

        shrinking_center = blank_frame()
        shrinking_center[:, 140:181] = (0, 0, 0)
        result = follower.process(shrinking_center)

        self.assertGreater(result.angular_z, 0.0)
        self.assertEqual(result.linear_x, 0.0)

    def test_branch_lock_releases_by_stable_near_field_without_time_gate(self):
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, min_area=100))
        left_far = blank_frame()
        left_far[110:175, 42:82] = (0, 0, 0)
        right_far = blank_frame()
        right_far[110:175, 238:278] = (0, 0, 0)
        centered_near = draw_vertical_line(blank_frame(), 160, 96, (0, 0, 0))

        follower.process(left_far)
        follower.process(right_far)
        follower.process(left_far)
        first = follower.process(centered_near)
        second = follower.process(centered_near)
        released = follower.process(centered_near)

        self.assertEqual(first.linear_x, 0.0)
        self.assertEqual(second.linear_x, 0.0)
        self.assertEqual(released.angular_z, 0.0)
        self.assertGreater(released.linear_x, 0.0)

    def test_shrinking_visible_path_keeps_turning_in_last_branch_direction(self):
        follower = LineFollower(LineFollowConfig(line_color="black", angular_gain=0.6, min_area=100))
        left_branch = blank_frame()
        left_branch[:, 70:170] = (0, 0, 0)
        first = follower.process(left_branch)
        self.assertGreater(first.angular_z, 0.0)

        shrinking_center = blank_frame()
        shrinking_center[:, 140:181] = (0, 0, 0)
        result = follower.process(shrinking_center)

        self.assertTrue(result.detected)
        self.assertGreater(result.angular_z, 0.0)
        self.assertEqual(result.linear_x, 0.0)
        self.assertGreaterEqual(abs(result.angular_z), 0.40)

    def test_wide_near_field_black_road_allows_forward_motion(self):
        frame = blank_frame(color=(225, 225, 225))
        frame[110:185, 120:200] = (115, 115, 115)
        frame[185:, :] = (105, 105, 105)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=1000, roi_bottom=1.0))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertGreater(result.linear_x, 0.0)

    def test_reflective_dark_gray_black_line_is_detected_with_threshold(self):
        frame = draw_vertical_line(blank_frame(color=(155, 155, 155)), 155, 28, (105, 105, 105))
        follower = LineFollower(LineFollowConfig(line_color="black", v_max=120))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.center_x, 155, delta=4)

    def test_black_path_tracing_ignores_dark_blob_far_from_bottom_center(self):
        frame = blank_frame(color=(160, 160, 160))
        draw_vertical_line(frame, 160, 20, (20, 20, 20))
        frame[160:230, 5:75] = (15, 15, 15)
        follower = LineFollower(LineFollowConfig(line_color="black", v_max=120))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.center_x, 160, delta=6)
        self.assertLess(abs(result.offset), 0.08)

    def test_blank_frame_search_uses_stronger_turn_rate(self):
        follower = LineFollower(LineFollowConfig(line_color="black"))

        result = follower.process(blank_frame())

        self.assertFalse(result.detected)
        self.assertAlmostEqual(abs(result.angular_z), 0.24, delta=0.01)

    def test_black_path_is_blocked_by_red_solid_line(self):
        frame = blank_frame(color=(170, 170, 170))
        draw_vertical_line(frame, 120, 20, (20, 20, 20))
        draw_horizontal_line(frame, 225, 8, (0, 0, 220))
        follower = LineFollower(LineFollowConfig(line_color="black", v_max=120))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertTrue(result.forbidden_blocked)
        self.assertEqual(result.linear_x, 0.0)
        self.assertGreater(result.angular_z, 0.0)

    def test_red_line_above_bottom_contact_band_does_not_block(self):
        frame = blank_frame(color=(170, 170, 170))
        draw_vertical_line(frame, 160, 20, (20, 20, 20))
        draw_horizontal_line(frame, 215, 8, (0, 0, 220))
        follower = LineFollower(LineFollowConfig(line_color="black", v_max=120))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertGreater(result.linear_x, 0.0)

    def test_far_yellow_line_in_top_of_roi_does_not_block_immediate_path(self):
        frame = blank_frame(color=(170, 170, 170))
        frame[:, 95:225] = (150, 150, 150)
        draw_horizontal_line(frame, 116, 8, (0, 190, 190))
        follower = LineFollower(LineFollowConfig(line_color="black", v_max=170, min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertGreater(result.linear_x, 0.0)

    def test_black_path_can_cross_red_dashed_line(self):
        frame = blank_frame(color=(170, 170, 170))
        draw_vertical_line(frame, 160, 20, (20, 20, 20))
        for x in range(0, frame.shape[1], 45):
            frame[182:188, x:x + 12] = (0, 0, 220)
        follower = LineFollower(LineFollowConfig(line_color="black", v_max=120))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertGreater(result.linear_x, 0.0)

    def test_gray_drivable_road_is_detected_as_black_path_under_glare(self):
        frame = blank_frame(color=(225, 225, 225))
        frame[:, 95:225] = (150, 150, 150)
        frame[:, 90:96] = (0, 190, 190)
        frame[:, 224:230] = (0, 190, 190)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertAlmostEqual(result.center_x, 160, delta=10)

    def test_bright_reflective_gray_road_is_detected_by_default_hsv_value_max(self):
        frame = blank_frame(color=(225, 225, 225))
        frame[:, 90:230] = (190, 190, 190)
        frame[:, 82:88] = (0, 190, 190)
        frame[:, 232:238] = (0, 0, 220)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.center_x, 160, delta=10)
        self.assertGreater(result.linear_x, 0.0)

    def test_hsv_mask_does_not_auto_tighten_wide_centered_road_to_dark_side_patch(self):
        frame = blank_frame(color=(225, 225, 225))
        frame[:, 30:290] = (190, 190, 190)
        frame[:, 40:80] = (80, 80, 80)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.center_x, 160, delta=10)
        self.assertEqual(result.angular_z, 0.0)

    def test_thin_bottom_edge_artifact_does_not_pull_target_off_wide_road(self):
        frame = blank_frame(width=640, height=480, color=(225, 225, 225))
        y1 = int(frame.shape[0] * 0.45)
        y2 = int(frame.shape[0] * 0.95)
        frame[y1:y2, 20:280] = (120, 120, 120)
        for idx, y in enumerate(range(y2 - 1, y1 + 80, -1)):
            t = idx / float((y2 - 1) - (y1 + 80))
            x = int(round(472 + (322 - 472) * t))
            frame[y, max(0, x - 1):min(frame.shape[1], x + 2)] = (120, 120, 120)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=250, angular_gain=0.6))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertLess(result.center_x, 205)
        self.assertEqual(result.linear_x, 0.0)
        self.assertGreater(result.angular_z, 0.0)

    def test_wide_side_branch_can_seed_path_when_no_center_road_is_visible(self):
        frame = blank_frame(width=640, height=480, color=(225, 225, 225))
        y1 = int(frame.shape[0] * 0.45)
        y2 = int(frame.shape[0] * 0.95)
        frame[y1:y2, 0:190] = (120, 120, 120)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=250, angular_gain=0.6))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertLess(result.center_x, 130)
        self.assertEqual(result.linear_x, 0.0)
        self.assertGreater(result.angular_z, 0.0)

    def test_black_mode_does_not_follow_colored_lane_marking_as_path(self):
        frame = blank_frame(color=(225, 225, 225))
        frame[:, 70:250] = (190, 190, 190)
        frame[:, 90:104] = (25, 115, 125)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertGreater(result.center_x, 135)
        self.assertLess(result.center_x, 185)

    def test_close_parallel_yellow_lane_lines_do_not_block_center_path(self):
        frame = blank_frame(color=(225, 225, 225))
        frame[:, 135:185] = (150, 150, 150)
        frame[:, 128:134] = (0, 190, 190)
        frame[:, 186:192] = (0, 190, 190)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertGreater(result.linear_x, 0.0)

    def test_black_road_uses_yellow_boundaries_for_lane_center(self):
        frame = blank_frame(color=(225, 225, 225))
        frame[:, 100:181] = (80, 80, 80)
        frame[:, 80:88] = (0, 190, 190)
        frame[:, 232:240] = (0, 190, 190)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertAlmostEqual(result.center_x, 160, delta=6)

    def test_black_road_uses_yellow_and_red_boundaries_for_lane_center(self):
        frame = blank_frame(color=(225, 225, 225))
        frame[:, 115:176] = (80, 80, 80)
        frame[:, 78:86] = (0, 190, 190)
        frame[:, 234:242] = (0, 0, 220)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=1000))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertAlmostEqual(result.center_x, 160, delta=6)

    def test_hsv_value_max_can_reject_gray_floor(self):
        frame = draw_vertical_line(blank_frame(color=(155, 155, 155)), 155, 28, (105, 105, 105))
        follower = LineFollower(LineFollowConfig(line_color="black", v_max=95))

        result = follower.process(frame)

        self.assertFalse(result.detected)

    def test_yellow_line_is_detected(self):
        frame = draw_vertical_line(blank_frame(), 170, 32, (0, 220, 220))
        follower = LineFollower(LineFollowConfig(line_color="yellow"))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.center_x, 170, delta=4)

    def test_dim_yellow_line_is_detected(self):
        frame = draw_vertical_line(blank_frame(), 150, 32, (25, 115, 125))
        follower = LineFollower(LineFollowConfig(line_color="yellow"))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.center_x, 150, delta=4)

    def test_blank_frame_reports_lost_line_and_stop_command(self):
        follower = LineFollower(LineFollowConfig(line_color="black"))

        result = follower.process(blank_frame())

        self.assertFalse(result.detected)
        self.assertEqual(result.linear_x, 0.0)
        self.assertNotEqual(result.angular_z, 0.0)
        self.assertGreaterEqual(result.linear_x, 0.0)

    def test_bottom_center_yellow_without_drivable_path_stops_without_turning(self):
        frame = blank_frame(color=(170, 170, 170))
        draw_horizontal_line(frame, 236, 6, (0, 190, 190))
        follower = LineFollower(LineFollowConfig(line_color="black"))

        result = follower.process(frame)

        self.assertFalse(result.detected)
        self.assertTrue(result.forbidden_blocked)
        self.assertEqual(result.linear_x, 0.0)
        self.assertEqual(result.angular_z, 0.0)

    def test_bottom_side_yellow_lane_boundary_does_not_block_center_contact(self):
        frame = blank_frame(color=(170, 170, 170))
        draw_vertical_line(frame, 160, 40, (120, 120, 120))
        frame[232:240, 0:55] = (0, 190, 190)
        frame[232:240, 265:320] = (0, 190, 190)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=500))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertGreater(result.linear_x, 0.0)

    def test_scattered_dark_patches_do_not_count_as_line(self):
        frame = blank_frame()
        frame[150:170, 30:70] = (0, 0, 0)
        frame[190:215, 190:235] = (0, 0, 0)
        follower = LineFollower(LineFollowConfig(line_color="black", min_area=100))

        result = follower.process(frame)

        self.assertFalse(result.detected)
        self.assertEqual(result.linear_x, 0.0)

    def test_config_clamps_motion_limits(self):
        cfg = LineFollowConfig(linear_speed=1.0, angular_gain=5.0, roi_top=0.9, roi_bottom=0.2)

        self.assertEqual(cfg.linear_speed, 0.10)
        self.assertEqual(cfg.angular_gain, 0.60)
        self.assertLess(cfg.roi_top, cfg.roi_bottom)


if __name__ == "__main__":
    unittest.main()
