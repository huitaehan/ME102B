import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from base_guidance import guidance_from_boxes


def box(cx, height=100, confidence=0.9, category=0):
    return SimpleNamespace(xyxy=[[cx - 10, 50, cx + 10, 50 + height]],
                           conf=[confidence], cls=[category])


class BaseGuidanceTests(unittest.TestCase):
    def guide(self, boxes, **kwargs):
        return guidance_from_boxes(
            boxes, frame_width=640, horizontal_focal_px=490,
            vertical_focal_px=490, flame_height_cm=5, **kwargs)

    def test_left_and_right_sign_and_magnitude(self):
        for cx, expected in ((160, 1), (480, -1)):
            with self.subTest(cx=cx):
                target = self.guide([box(cx)])
                self.assertAlmostEqual(target.turn_deg,
                                       expected * math.degrees(math.atan(160 / 490)))

    def test_center_and_deadband(self):
        self.assertTrue(self.guide([box(320)]).aligned)
        self.assertTrue(self.guide([box(330)]).aligned)
        self.assertFalse(self.guide([box(380)]).aligned)

    def test_closest_target_controls_both_distance_and_bearing(self):
        target = self.guide([box(160, height=50, confidence=0.99),
                             box(480, height=100, confidence=0.8)])
        self.assertEqual(target.center_x, 480)
        self.assertAlmostEqual(target.distance_m, 0.245)
        self.assertLess(target.turn_deg, 0)

    def test_no_target_and_other_classes(self):
        self.assertIsNone(self.guide([]))
        self.assertIsNone(self.guide([box(160, category=1)]))
        self.assertIsNone(self.guide([box(160, confidence=0.4)]))
        self.assertIsNone(self.guide([box(160, height=0)]))

    def test_mounting_yaw_is_relative_to_base(self):
        self.assertAlmostEqual(self.guide([box(320)], camera_yaw_deg=10).turn_deg, 10)

    def test_invalid_calibration_is_rejected(self):
        for kwargs in ({'deadband_deg': -1}, {'camera_yaw_deg': float('nan')}):
            with self.assertRaises(ValueError):
                self.guide([], **kwargs)

    def test_search_command_uses_heading_plus_error(self):
        path = Path(__file__).resolve().parents[1] / 'Search and Drive' / 'Search.py'
        spec = importlib.util.spec_from_file_location('search_planner', path)
        planner = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(planner)
        self.assertEqual(planner.fire_yaw_command(45, -15), 'YAW,30.000')
        self.assertEqual(planner.fire_yaw_command(175, 15), 'YAW,-170.000')


if __name__ == '__main__':
    unittest.main()
