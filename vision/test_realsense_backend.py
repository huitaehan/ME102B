import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import sys

import numpy as np

from realsense_backend import depth_scene
import fire_serial_bridge
import realsense_backend


def box(cx=320, confidence=0.9, category=0):
    return SimpleNamespace(xyxy=[[cx-50, 100, cx+50, 300]],
                           conf=[confidence], cls=[category])


def deproject(pixel, z):
    return ((pixel[0]-320)/500*z, (pixel[1]-240)/500*z, z)


class DepthTests(unittest.TestCase):
    def scene(self, boxes=None, depth=None, **kwargs):
        if boxes is None:
            boxes = [box()]
        if depth is None:
            depth = np.full((480, 640), 1.0)
        return depth_scene(boxes, depth, deproject, **kwargs)

    def test_measured_depth_replaces_assumed_flame_height(self):
        scene = self.scene()
        self.assertTrue(scene.range_valid)
        self.assertAlmostEqual(scene.guidance.distance_m, 1.0, places=5)
        self.assertEqual(scene.fire_message(), 'FIRE,1,1.0000,0.9000\n')

    def test_invalid_depth_is_not_no_fire(self):
        scene = self.scene(depth=np.zeros((480, 640)))
        self.assertFalse(scene.range_valid)
        self.assertIsNone(scene.guidance)
        self.assertIsNone(scene.fire_message())
        self.assertEqual(self.scene(boxes=[]).fire_message(), 'FIRE,0,0,0\n')

    def test_median_resists_sparse_spikes_and_holes(self):
        depth = np.full((480, 640), 1.2)
        depth[240:245, 310:315] = 0
        depth[246:250, 310:315] = 4
        self.assertAlmostEqual(self.scene(depth=depth).detections[0].depth_m, 1.2)

    def test_mixed_surface_patch_is_rejected(self):
        depth = np.ones((480, 640))
        depth[:, 320:] = 3
        scene = self.scene(depth=depth)
        self.assertFalse(scene.range_valid)
        self.assertEqual(scene.detections[0].reason, 'Mixed-depth patch')

    def test_unknown_fire_prevents_selection_of_known_farther_fire(self):
        depth = np.ones((480, 640))
        depth[:, :250] = 0
        scene = self.scene([box(160), box(480)], depth)
        self.assertIsNone(scene.guidance)
        self.assertIsNone(scene.fire_message())

    def test_nearest_measured_horizontal_range_controls_bearing(self):
        depth = np.full((480, 640), 2.0)
        depth[:, 400:] = 1.0
        scene = self.scene([box(160, 0.99), box(480, 0.8)], depth)
        self.assertGreater(scene.guidance.center_x, 320)
        self.assertLess(scene.guidance.turn_deg, 0)
        self.assertLess(scene.guidance.distance_m, 1.1)

    def test_horizontal_range_is_neither_z_nor_slant_range(self):
        scene = depth_scene([box()], np.ones((480, 640)), lambda p, z: (0.3, 0.4, 1.0))
        self.assertAlmostEqual(scene.guidance.distance_m, math.hypot(0.3, 1.0))
        self.assertAlmostEqual(scene.guidance.yaw_error_deg, -math.degrees(math.atan(0.3)))

    def test_pitch_rotation_projects_to_ground_plane(self):
        scene = depth_scene([box()], np.ones((480, 640)), lambda p, z: (0, 0, 1),
                            camera_pitch_deg=30)
        self.assertAlmostEqual(scene.guidance.distance_m, math.cos(math.radians(30)))

    def test_clipped_box_never_wraps_depth_indices(self):
        b = SimpleNamespace(xyxy=[[-100, -100, 10, 10]], conf=[0.9], cls=[0])
        self.assertFalse(self.scene([b]).range_valid)

    def test_missing_nan_out_of_range_samples_are_invalid(self):
        for value in (float('nan'), float('inf'), 0, 0.05, 10):
            with self.subTest(value=value):
                self.assertFalse(self.scene(depth=np.full((480, 640), value)).range_valid)

    def test_nonfire_class_is_ignored(self):
        self.assertEqual(self.scene([box(category=1)]).fire_message(), 'FIRE,0,0,0\n')


class BridgeDepthTests(unittest.TestCase):
    def run_bridge(self, depth):
        sent = []
        stopped = []

        class Camera:
            captures = 0

            def capture(self):
                self.captures += 1
                if self.captures > 1:
                    raise KeyboardInterrupt
                return np.zeros((480, 640, 3), dtype=np.uint8), depth, deproject

            def stop(self):
                stopped.append(True)

        class Link:
            in_waiting = 0

            def write(self, message):
                sent.append(message)

            def flush(self):
                pass

            def close(self):
                pass

        fake_model = lambda *args, **kwargs: [SimpleNamespace(boxes=[box()])]
        modules = {
            'cv2': SimpleNamespace(),
            'serial': SimpleNamespace(Serial=lambda *a, **k: Link()),
            'ultralytics': SimpleNamespace(YOLO=lambda path: fake_model),
        }
        with (patch.dict(sys.modules, modules),
              patch.object(realsense_backend, 'RealSenseCamera', Camera),
              patch.object(fire_serial_bridge.time, 'sleep'),
              patch.object(sys, 'argv', ['main.py', '--realsense', '--port', 'MOCK'])):
            try:
                fire_serial_bridge.main()
            finally:
                self.sent, self.stopped = sent, stopped

    def test_bridge_sends_measured_range_without_focal_calibration(self):
        self.run_bridge(np.ones((480, 640)))
        self.assertEqual(self.sent, [b'FIRE,1,1.0000,0.9000\n', b'STOP\n'])
        self.assertTrue(self.stopped)

    def test_bridge_stops_on_invalid_depth_without_sending_no_fire(self):
        with self.assertRaisesRegex(RuntimeError, 'depth is invalid'):
            self.run_bridge(np.zeros((480, 640)))
        self.assertEqual(self.sent, [b'STOP\n'])
        self.assertTrue(self.stopped)


if __name__ == '__main__':
    unittest.main()
