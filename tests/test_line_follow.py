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


class LineFollowerTest(unittest.TestCase):
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

    def test_reflective_dark_gray_black_line_is_detected_with_threshold(self):
        frame = draw_vertical_line(blank_frame(color=(155, 155, 155)), 155, 28, (105, 105, 105))
        follower = LineFollower(LineFollowConfig(line_color="black", black_threshold=120))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.center_x, 155, delta=4)

    def test_black_path_tracing_ignores_dark_blob_far_from_bottom_center(self):
        frame = blank_frame(color=(160, 160, 160))
        draw_vertical_line(frame, 160, 20, (20, 20, 20))
        frame[160:230, 5:75] = (15, 15, 15)
        follower = LineFollower(LineFollowConfig(line_color="black", black_threshold=120))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertAlmostEqual(result.center_x, 160, delta=6)
        self.assertLess(abs(result.offset), 0.08)

    def test_black_path_is_blocked_by_red_solid_line(self):
        frame = blank_frame(color=(170, 170, 170))
        draw_vertical_line(frame, 160, 20, (20, 20, 20))
        draw_horizontal_line(frame, 185, 8, (0, 0, 220))
        follower = LineFollower(LineFollowConfig(line_color="black", black_threshold=120))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertTrue(result.forbidden_blocked)
        self.assertEqual(result.linear_x, 0.0)
        self.assertEqual(result.angular_z, 0.0)

    def test_black_path_can_cross_red_dashed_line(self):
        frame = blank_frame(color=(170, 170, 170))
        draw_vertical_line(frame, 160, 20, (20, 20, 20))
        for x in range(0, frame.shape[1], 45):
            frame[182:188, x:x + 12] = (0, 0, 220)
        follower = LineFollower(LineFollowConfig(line_color="black", black_threshold=120))

        result = follower.process(frame)

        self.assertTrue(result.detected)
        self.assertFalse(result.forbidden_blocked)
        self.assertGreater(result.linear_x, 0.0)

    def test_black_threshold_can_reject_gray_floor(self):
        frame = draw_vertical_line(blank_frame(color=(155, 155, 155)), 155, 28, (105, 105, 105))
        follower = LineFollower(LineFollowConfig(line_color="black", black_threshold=95))

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
        self.assertEqual(result.angular_z, 0.0)

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
