import unittest

import numpy as np

from simple_line_follower import command_from_mask, memory_command


class SimpleLineFollowerCoreTest(unittest.TestCase):
    def test_centered_mask_drives_straight(self):
        mask = np.zeros((80, 160), dtype=np.uint8)
        mask[:, 70:90] = 255

        linear, angular, found, error = command_from_mask(mask, kp=0.005, linear_speed=0.06, max_angular=0.35)

        self.assertTrue(found)
        self.assertAlmostEqual(error, 0.0, delta=2.0)
        self.assertAlmostEqual(linear, 0.06)
        self.assertAlmostEqual(angular, 0.0, delta=0.02)

    def test_left_mask_turns_left(self):
        mask = np.zeros((80, 160), dtype=np.uint8)
        mask[:, 20:40] = 255

        linear, angular, found, error = command_from_mask(mask, kp=0.005, linear_speed=0.06, max_angular=0.35)

        self.assertTrue(found)
        self.assertGreater(error, 0.0)
        self.assertEqual(linear, 0.06)
        self.assertGreater(angular, 0.0)
        self.assertLessEqual(abs(angular), 0.35)

    def test_empty_mask_stops(self):
        mask = np.zeros((80, 160), dtype=np.uint8)

        linear, angular, found, error = command_from_mask(mask, kp=0.005, linear_speed=0.06, max_angular=0.35)

        self.assertFalse(found)
        self.assertEqual(error, 0.0)
        self.assertEqual(linear, 0.0)
        self.assertEqual(angular, 0.0)

    def test_fork_can_follow_left_component_instead_of_average(self):
        mask = np.zeros((80, 160), dtype=np.uint8)
        mask[:, 15:35] = 255
        mask[:, 125:145] = 255

        linear, angular, found, error = command_from_mask(
            mask,
            kp=0.005,
            linear_speed=0.06,
            max_angular=0.35,
            branch_choice="left",
        )

        self.assertTrue(found)
        self.assertEqual(linear, 0.06)
        self.assertGreater(error, 40.0)
        self.assertGreater(angular, 0.0)

    def test_fork_can_follow_right_component_instead_of_average(self):
        mask = np.zeros((80, 160), dtype=np.uint8)
        mask[:, 15:35] = 255
        mask[:, 125:145] = 255

        linear, angular, found, error = command_from_mask(
            mask,
            kp=0.005,
            linear_speed=0.06,
            max_angular=0.35,
            branch_choice="right",
        )

        self.assertTrue(found)
        self.assertEqual(linear, 0.06)
        self.assertLess(error, -40.0)
        self.assertLess(angular, 0.0)

    def test_short_dash_gap_reuses_last_command_then_stops(self):
        last_command = (0.06, 0.12)

        linear, angular, lost_frames = memory_command(
            found=False,
            command=(0.0, 0.0),
            last_command=last_command,
            lost_frames=0,
            lost_frame_limit=3,
        )
        self.assertEqual((linear, angular, lost_frames), (0.06, 0.12, 1))

        linear, angular, lost_frames = memory_command(
            found=False,
            command=(0.0, 0.0),
            last_command=last_command,
            lost_frames=3,
            lost_frame_limit=3,
        )
        self.assertEqual((linear, angular, lost_frames), (0.0, 0.0, 4))


if __name__ == "__main__":
    unittest.main()
