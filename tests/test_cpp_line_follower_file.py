import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
CPP_FILE = ROOT / "src" / "line_follower.cpp"


class CppLineFollowerFileTest(unittest.TestCase):
    def test_cpp_file_contains_contour_and_dash_logic(self):
        text = CPP_FILE.read_text(encoding="utf-8")

        self.assertIn("class LineFollower", text)
        self.assertIn("findContours", text)
        self.assertIn("MORPH_CLOSE", text)
        self.assertIn("lost_frames_", text)
        self.assertIn("max_lost_frames_", text)
        self.assertIn("last_twist_", text)
        self.assertIn("printUTF8", text)
        self.assertIn("printSpeedInfo", text)
        self.assertIn("image_transport", text)
        self.assertIn("/cmd_vel", text)

    def test_cpp_file_uses_ros_params_for_tunable_values(self):
        text = CPP_FILE.read_text(encoding="utf-8")

        self.assertIn('nh_.param("image_topic"', text)
        self.assertIn('nh_.param("branch_choice"', text)
        self.assertIn('nh_.param("max_lost_frames"', text)
        self.assertIn('nh_.param("line_threshold"', text)

    def test_cpp_file_has_centerline_dash_and_yellow_guard_parameters(self):
        text = CPP_FILE.read_text(encoding="utf-8")

        for token in (
            "roi_top_ratio_",
            "close_kernel_width_",
            "close_kernel_height_",
            "computeLayeredCenter",
            "detectYellowGuard",
            "turn_slowdown_",
            "min_curve_speed_",
            "yellow_guard_enabled_",
            "yellow_h_min_",
            "yellow_h_max_",
        ):
            self.assertIn(token, text)

    def test_cpp_file_has_sliding_window_lookahead_control(self):
        text = CPP_FILE.read_text(encoding="utf-8")

        for token in (
            "struct SlidingWindowResult",
            "chooseBandWindow",
            "computeSlidingWindowCenter",
            "lookahead_layer_",
            "lookahead_weight_",
            "curve_slowdown_",
            "max_curve_delta_",
            "sliding_min_width_",
            "curve_delta",
            "layers_found",
        ):
            self.assertIn(token, text)

    def test_cpp_file_has_straight_hold_for_delayed_curve_entry(self):
        text = CPP_FILE.read_text(encoding="utf-8")

        for token in (
            "straight_hold_enabled_",
            "straight_hold_remaining_",
            "straight_hold_frames_",
            "straight_hold_near_error_",
            "straight_hold_curve_delta_",
            "straight_hold_speed_",
            "shouldHoldStraight",
            "STRAIGHT_HOLD",
        ):
            self.assertIn(token, text)

    def test_cpp_file_has_branch_lock_and_crosswalk_gap_handling(self):
        text = CPP_FILE.read_text(encoding="utf-8")

        for token in (
            "branch_lock_enabled_",
            "branch_lock_remaining_",
            "branch_approach_hold_",
            "branch_commit_frames_",
            "branch_split_delta_",
            "detectBranchSplit",
            "selectCommittedBranchTarget",
            "BRANCH_APPROACH",
            "BRANCH_COMMITTED",
            "white_gap_enabled_",
            "detectWhiteCrosswalk",
            "white_gap_remaining_",
            "WHITE_GAP",
            "white_gap_frames_",
            "white_min_pixels_",
        ):
            self.assertIn(token, text)

    def test_cpp_file_has_persistent_branch_route_memory(self):
        text = CPP_FILE.read_text(encoding="utf-8")

        for token in (
            "branch_route_memory_remaining_",
            "branch_route_memory_frames_",
            "branch_exit_hold_remaining_",
            "branch_exit_hold_frames_",
            "branch_target_weight_",
            "branch_max_angular_scale_",
            "branchChoiceSign",
            "startBranchRouteMemory",
            "BRANCH_ROUTE_MEMORY",
            "BRANCH_EXIT_HOLD",
        ):
            self.assertIn(token, text)


if __name__ == "__main__":
    unittest.main()
