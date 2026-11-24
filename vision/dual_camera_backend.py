"""IMX500 accelerator detections + calibrated D435i depth-only geometry.

No Ultralytics/PyTorch import or CPU neural-network inference is used here.
"""

import json
from collections import deque
from pathlib import Path
import threading
import time

import numpy as np

if __package__:
    from .imx500_backend import preview_boxes_from_outputs
    from .realsense_backend import depth_scene
else:
    from imx500_backend import preview_boxes_from_outputs
    from realsense_backend import depth_scene


def load_calibration(path):
    data = json.loads(Path(path).read_text())
    if data.get('version') != 1 or data.get('ai_size') != [640, 480]:
        raise ValueError('Calibration must be version 1 for a 640x480 AI preview')
    if data.get('depth_size') != [640, 480] or not data.get('realsense_serial'):
        raise ValueError('Calibration must identify the 640x480 D435i depth stream')
    for name, shape in (('ai_K', (3, 3)), ('R_depth_to_ai', (3, 3)), ('t_depth_to_ai_m', (3,))):
        value = np.asarray(data[name], dtype=float)
        if value.shape != shape or not np.all(np.isfinite(value)):
            raise ValueError(f'Invalid calibration field {name}')
        data[name] = value
    data['ai_distortion'] = np.asarray(data['ai_distortion'], dtype=float).reshape(-1)
    if (len(data['ai_distortion']) not in (4, 5, 8, 12, 14)
            or not np.all(np.isfinite(data['ai_distortion']))):
        raise ValueError('Invalid AI lens distortion')
    k, r = data['ai_K'], data['R_depth_to_ai']
    if k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1]):
        raise ValueError('Invalid AI intrinsics')
    if not np.allclose(r.T@r, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(r), 1, atol=1e-4):
        raise ValueError('Depth-to-AI rotation must be a proper rotation matrix')
    if len(data.get('ai_scaler_crop', [])) != 4 or not data.get('ai_camera_id'):
        raise ValueError('Calibration must identify AI camera and scaler crop')
    return data


def project_depth_to_ai(vertices, calibration):
    """Z-buffer RealSense optical XYZ metres into the distorted AI preview.

    R,t transform D435i depth-camera coordinates to AI-camera coordinates.
    Occluded/out-of-view points are dropped; gaps stay invalid, never filled.
    """
    import cv2
    vertices = np.asarray(vertices, dtype=np.float32).reshape(-1, 3)
    good = np.all(np.isfinite(vertices), axis=1) & (vertices[:, 2] > 0)
    points = vertices[good] @ calibration['R_depth_to_ai'].T + calibration['t_depth_to_ai_m']
    points = points[np.all(np.isfinite(points), axis=1) & (points[:, 2] > 0)]
    width, height = calibration['ai_size']
    zbuffer = np.full((height, width), np.inf, dtype=np.float32)
    if len(points):
        uv, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3),
                                  calibration['ai_K'], calibration['ai_distortion'])
        uv = uv.reshape(-1, 2)
        good = (np.all(np.isfinite(uv), axis=1) & (uv[:, 0] >= 0) &
                (uv[:, 0] < width-0.5) & (uv[:, 1] >= 0) & (uv[:, 1] < height-0.5))
        pixels = np.rint(uv[good]).astype(int)
        np.minimum.at(zbuffer, (pixels[:, 1], pixels[:, 0]), points[good, 2])
    zbuffer[~np.isfinite(zbuffer)] = 0
    return zbuffer


class LatestCapture:
    """Bounded recent results; rejected frames never overwrite valid inference.

    Completion times bound software delivery skew only, not exposure synchrony.
    """
    def __init__(self, capture, accept=lambda value: True):
        self.capture = capture
        self.accept = accept
        self.latest = None
        self.history = deque(maxlen=8)
        self.frames = 0
        self.accepted = 0
        self.error = None
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            sequence = 0
            while not self.stopped.is_set():
                value = self.capture()
                sequence += 1
                with self.lock:
                    self.frames += 1
                    if self.accept(value):
                        self.accepted += 1
                        self.latest = (sequence, time.monotonic(), value)
                        self.history.append(self.latest)
        except Exception as exc:
            if not self.stopped.is_set():
                self.error = exc

    def get(self):
        if self.error:
            raise RuntimeError('Camera capture failed') from self.error
        with self.lock:
            return self.latest

    def snapshot(self):
        if self.error:
            raise RuntimeError('Camera capture failed') from self.error
        with self.lock:
            return list(self.history)


def fresh_pair(ai_frames, depth_frames, now, last_ai, max_age=0.20, max_skew=0.10):
    """Newest unused inference and nearest depth by software delivery time.

    Keeping history does not relax age limits or claim exposure synchronization.
    """
    for ai in reversed(ai_frames):
        if ai[0] <= last_ai or not 0 <= now-ai[1] < max_age:
            continue
        candidates = [d for d in depth_frames if 0 <= now-d[1] < max_age
                      and abs(ai[1]-d[1]) <= max_skew]
        if candidates:
            return ai, min(candidates, key=lambda d: abs(ai[1]-d[1]))
    return None


class DualCamera:
    def __init__(self, model, calibration, *, box_order, box_units, fire_class=0,
                 anchor='bottom', camera_yaw_deg=0, camera_pitch_deg=0, deadband_deg=3):
        import cv2
        import pyrealsense2 as rs
        from picamera2 import Picamera2
        from picamera2.devices.imx500 import IMX500

        if not Path(model).is_file():
            raise ValueError('Compiled fire-model RPK not found')
        self.calibration = load_calibration(calibration)
        self.rs, self.cv2 = rs, cv2
        self.workers = []
        self.ai = None
        self.ai_started = False
        self.pipeline = None
        self.rs_started = False
        self.last_ai = 0
        self.scene_options = dict(fire_class=fire_class, anchor=anchor,
                                  camera_yaw_deg=camera_yaw_deg,
                                  camera_pitch_deg=camera_pitch_deg, deadband_deg=deadband_deg)
        depth_scene([], [[0]], lambda p, z: (0, 0, z), **self.scene_options)
        try:
            self.imx = IMX500(str(model))
            info = next(c for c in Picamera2.global_camera_info() if c['Num'] == self.imx.camera_num)
            if info['Id'] != self.calibration['ai_camera_id']:
                raise ValueError('AI camera does not match calibration')
            self.ai = Picamera2(self.imx.camera_num)
            self.ai.configure(self.ai.create_video_configuration(
                main={'size': (640, 480), 'format': 'RGB888'}, queue=False))
            if list(self.ai.camera_configuration()['raw']['size']) != self.calibration.get('ai_raw_size'):
                raise ValueError('AI sensor mode differs from calibration')
            self.ai.set_controls({'ScalerCrop': tuple(self.calibration['ai_scaler_crop'])})
            self.ai.start()
            self.ai_started = True
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(self.calibration['realsense_serial'])
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)
            self.rs_started = True
            self.cloud = rs.pointcloud()

            def capture_ai():
                request = self.ai.capture_request()
                try:
                    metadata = request.get_metadata()
                    if list(metadata['ScalerCrop']) != self.calibration['ai_scaler_crop']:
                        raise ValueError('AI crop differs from calibration; recalibrate')
                    outputs = self.imx.get_outputs(metadata, add_batch=True)
                    boxes = preview_boxes_from_outputs(
                        outputs, box_order=box_order, box_units=box_units,
                        input_size=self.imx.get_input_size(), fire_class=fire_class,
                        convert_coords=lambda coords: self.imx.convert_inference_coords(
                            coords, metadata, self.ai))
                    return request.make_array('main').copy(), boxes
                finally:
                    request.release()

            def capture_depth():
                frame = self.pipeline.wait_for_frames(2000).get_depth_frame()
                if not frame:
                    raise RuntimeError('Missing D435i depth frame')
                points = self.cloud.calculate(frame)
                return np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3).copy()

            # Inference is not present on every sensor frame. Preserve each
            # inference-bearing image, including valid zero-detection results.
            self.workers.append(LatestCapture(capture_ai, accept=lambda v: v[1] is not None))
            self.workers.append(LatestCapture(capture_depth))
        except Exception:
            self.stop()
            raise

    def capture(self):
        startup = self.last_ai == 0
        deadline = time.monotonic()+(60 if startup else 3)
        if startup:
            print('Waiting for first IMX500 inference + depth pair (up to 60s)...', flush=True)
        while time.monotonic() < deadline:
            ai_frames, depth_frames = [w.snapshot() for w in self.workers]
            pair = fresh_pair(ai_frames, depth_frames, time.monotonic(), self.last_ai)
            if pair:
                ai, depth = pair
                image, boxes = ai[2]
                self.last_ai = ai[0]
                mapped = (project_depth_to_ai(depth[2], self.calibration) if boxes else
                          np.zeros((480, 640), dtype=np.float32))
                def deproject(pixel, z):
                    ray = self.cv2.undistortPoints(
                        np.array(pixel, dtype=float).reshape(1, 1, 2),
                        self.calibration['ai_K'], self.calibration['ai_distortion']).reshape(2)
                    return ray[0]*z, ray[1]*z, z
                return image, depth_scene(boxes, mapped, deproject, **self.scene_options)
            time.sleep(0.005)
        ai, depth = self.workers
        raise RuntimeError(
            'No fresh IMX500 inference/depth pair: '
            f'AI frames={ai.frames}, inference frames={ai.accepted}, depth frames={depth.accepted}. '
            'Check these streams and software delivery timing; stale data was not reused.')

    def stop(self):
        for worker in self.workers:
            worker.stopped.set()
        try:
            if self.rs_started:
                self.pipeline.stop()
                self.rs_started = False
        finally:
            try:
                if self.ai_started:
                    self.ai.stop()
                    self.ai_started = False
            finally:
                for worker in self.workers:
                    worker.thread.join(timeout=3)
                if self.ai is not None:
                    self.ai.close()
                    self.ai = None
