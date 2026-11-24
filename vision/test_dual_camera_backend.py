import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import dual_camera_backend
from dual_camera_backend import load_calibration, project_depth_to_ai, fresh_pair, LatestCapture
from imx500_backend import preview_boxes_from_outputs
from realsense_backend import DepthScene
import fire_serial_bridge


def calibration():
    return dict(version=1, ai_size=[640, 480], depth_size=[640, 480],
                ai_camera_id='imx500-test', realsense_serial='test',
                ai_scaler_crop=[0, 0, 4056, 3040], ai_raw_size=[2028, 1520],
                ai_K=np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]]),
                ai_distortion=np.zeros(5), R_depth_to_ai=np.eye(3),
                t_depth_to_ai_m=np.zeros(3))


class PairingTests(unittest.TestCase):
    def test_nearest_depth_is_chosen_from_recent_history(self):
        ai = [(1, 10.0, ('image', []))]
        depth = [(1, 10.01, 'matched'), (2, 10.09, 'newest')]
        pair = fresh_pair(ai, depth, 10.10, 0)
        self.assertEqual(pair[1][2], 'matched')

    def test_used_stale_and_excessively_skewed_frames_are_rejected(self):
        ai = [(1, 10.0, ('image', []))]
        self.assertIsNone(fresh_pair(ai, [(1, 10.01, 'depth')], 10.10, 1))
        self.assertIsNone(fresh_pair(ai, [(1, 10.25, 'depth')], 10.26, 0))
        self.assertIsNone(fresh_pair(ai, [(1, 10.15, 'depth')], 10.18, 0))

    def test_sensor_frames_without_inference_do_not_erase_valid_zero_detections(self):
        # A zero-detection inference is valid. A missing inference is not.
        finished = threading.Event()
        values = iter([('image1', []), ('image2', None), ('image3', None)])
        def capture():
            try:
                return next(values)
            except StopIteration:
                finished.set()
                release.wait(timeout=1)
                return ('image4', None)
        release = threading.Event()
        worker = LatestCapture(capture, accept=lambda v: v[1] is not None)
        try:
            self.assertTrue(finished.wait(timeout=1))
            self.assertEqual(worker.get()[2], ('image1', []))
            self.assertEqual(worker.frames, 3)
            self.assertEqual(worker.accepted, 1)
            self.assertEqual(len(worker.snapshot()), 1)
        finally:
            worker.stopped.set()
            release.set()
            worker.thread.join(timeout=1)


class ProjectionTests(unittest.TestCase):
    def test_baseline_changes_pixel_correspondence(self):
        data = calibration()
        data['t_depth_to_ai_m'] = np.array([0.1, 0, 0])
        mapped = project_depth_to_ai([[0, 0, 1]], data)
        self.assertEqual(mapped[240, 370], 1)
        self.assertEqual(mapped[240, 320], 0)

    def test_nearest_projected_surface_wins(self):
        mapped = project_depth_to_ai([[0, 0, 2], [0, 0, 1]], calibration())
        self.assertEqual(mapped[240, 320], 1)
        self.assertEqual(np.count_nonzero(mapped), 1)

    def test_invalid_out_of_view_and_behind_camera_are_dropped(self):
        mapped = project_depth_to_ai([[0, 0, -1], [10, 0, 1], [0, 0, 0],
                                      [float('nan'), 0, 1]], calibration())
        self.assertEqual(np.count_nonzero(mapped), 0)

    def test_rigid_rotation_uses_depth_to_ai_direction(self):
        data = calibration()
        data['R_depth_to_ai'] = np.diag([-1., -1., 1.])
        mapped = project_depth_to_ai([[0.1, 0, 1]], data)
        self.assertEqual(mapped[240, 270], 1)

    def test_invalid_calibration_is_refused(self):
        data = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in calibration().items()}
        with tempfile.TemporaryDirectory() as folder:
            file = Path(folder)/'calibration.json'
            file.write_text(json.dumps(data))
            self.assertEqual(load_calibration(file)['ai_size'], [640, 480])
            data['R_depth_to_ai'][0][0] = -1  # reflection, not a rigid rotation
            file.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, 'proper rotation'):
                load_calibration(file)

    def test_nms_mapping_uses_picamera_crop_converter(self):
        inputs = []
        def convert(coords):
            inputs.append(coords)
            return (100, 80, 40, 60)
        boxes = preview_boxes_from_outputs(
            [[[64, 128, 320, 384]], [0.9], [0], [1]],
            box_order='xyxy', box_units='pixels', input_size=(640, 640),
            convert_coords=convert)
        self.assertEqual(inputs[0], (0.2, 0.1, 0.6, 0.5))
        self.assertEqual(boxes[0].xyxy[0], (100, 80, 140, 140))

    def test_missing_inference_is_not_clear(self):
        self.assertIsNone(preview_boxes_from_outputs(
            None, box_order='xyxy', box_units='normalized', input_size=(640, 640),
            convert_coords=lambda coords: (0, 0, 0, 0)))

    def test_bridge_uses_dual_backend_without_cpu_yolo(self):
        sent = []
        preview_events = []
        camera_opens = []
        class Camera:
            frames = 0
            def __init__(self, *args, **kwargs):
                camera_opens.append(True)
            def capture(self):
                self.frames += 1
                if self.frames > 1:
                    raise KeyboardInterrupt
                return np.zeros((480, 640, 3)), DepthScene(())
            def stop(self):
                pass
        class Preview:
            def __init__(self, port):
                preview_events.append(('open', port))
            def publish(self, frame, scene):
                preview_events.append(('frame', scene))
            def close(self):
                preview_events.append(('close', None))
        class Link:
            in_waiting = 0
            def readline(self):
                return b'CAPS,RANGE_WAIT_V1\n'
            def write(self, data):
                sent.append(data)
            def flush(self):
                pass
            def close(self):
                pass
        def forbidden_yolo(*a, **k):
            raise AssertionError('CPU YOLO must not be loaded in IMX500/depth mode')
        with tempfile.TemporaryDirectory() as folder:
            model = Path(folder)/'model.rpk'
            model.write_bytes(b'test placeholder; no hardware test')
            argv = ['main.py', '--port', 'MOCK', '--dual-depth', '--imx500-model', str(model),
                    '--dual-calibration', 'test.json', '--imx500-box-order', 'xyxy',
                    '--imx500-box-units', 'normalized', '--preview-port', '8080']
            modules = dict(serial=SimpleNamespace(Serial=lambda *a, **k: Link()),
                           ultralytics=SimpleNamespace(YOLO=forbidden_yolo),
                           bridge_preview=SimpleNamespace(BridgePreview=Preview))
            with (patch.dict(sys.modules, modules), patch.object(sys, 'argv', argv),
                  patch.object(fire_serial_bridge.time, 'sleep'),
                  patch.object(dual_camera_backend, 'DualCamera', Camera)):
                fire_serial_bridge.main()
        self.assertEqual(sent, [b'CAPS\n', b'FIRE,0,0,0\n', b'STOP\n'])
        self.assertEqual(len(camera_opens), 1)
        self.assertEqual([e[0] for e in preview_events], ['open', 'frame', 'close'])
        self.assertEqual(preview_events[1][1].fire_message(), 'FIRE,0,0,0\n')


if __name__ == '__main__':
    unittest.main()
