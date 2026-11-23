"""D435i RGB + aligned depth. A depth return is a surface, not proof of flame range."""

from dataclasses import dataclass
import math

if __package__:
    from .base_guidance import BaseGuidance
else:
    from base_guidance import BaseGuidance


@dataclass(frozen=True)
class DepthDetection:
    box: tuple
    confidence: float
    roi: tuple
    depth_m: object
    guidance: object
    reason: str = ''


@dataclass(frozen=True)
class DepthScene:
    detections: tuple

    @property
    def range_valid(self):
        # Do not silently choose a farther target when another fire has unknown range.
        return all(d.guidance is not None for d in self.detections)

    @property
    def guidance(self):
        if not self.range_valid:
            return None
        return min((d.guidance for d in self.detections),
                   key=lambda g: (g.distance_m, -g.confidence), default=None)

    def fire_message(self):
        if not self.range_valid:
            return None  # Unknown range is not a no-fire observation.
        if not self.detections:
            return 'FIRE,0,0,0\n'
        g = self.guidance
        return f'FIRE,1,{g.distance_m:.4f},{g.confidence:.4f}\n'


def depth_scene(boxes, depth_m, deproject, *, fire_class=0, anchor='bottom',
                min_depth_m=0.15, max_depth_m=5.0, min_samples=9,
                camera_yaw_deg=0.0, camera_pitch_deg=0.0, deadband_deg=3.0):
    """Measure central/lower box patches in a depth image aligned to RGB.

    Median rejects isolated spikes; reject holes, sparse data, and mixed-depth
    patches. None of these checks prove the surface is the burning material.
    deproject(pixel, z_metres) returns camera coordinates: right, down, forward.
    The returned solver distance is horizontal ground-plane range, not depth Z
    or Euclidean slant range. Camera pitch is positive up; yaw is positive left.
    """
    import numpy as np

    if anchor not in ('center', 'bottom'):
        raise ValueError('Depth anchor must be center or bottom')
    if not (math.isfinite(min_depth_m) and math.isfinite(max_depth_m)
            and 0 < min_depth_m < max_depth_m):
        raise ValueError('Invalid depth range')
    if min_samples < 1:
        raise ValueError('min_samples must be positive')
    if not all(math.isfinite(v) for v in (camera_yaw_deg, camera_pitch_deg, deadband_deg)):
        raise ValueError('Mounting angles and deadband must be finite')
    if abs(camera_pitch_deg) >= 90 or abs(camera_yaw_deg) >= 90 or not 0 <= deadband_deg < 90:
        raise ValueError('Invalid mounting angle or deadband')
    depth_m = np.asarray(depth_m)
    if depth_m.ndim != 2:
        raise ValueError('Expected a 2D aligned depth image in metres')
    height, width = depth_m.shape
    detections = []
    for box in boxes:
        confidence = float(box.conf[0])
        if int(box.cls[0]) != fire_class or not 0.40 < confidence <= 1:
            continue
        coords = tuple(map(float, box.xyxy[0]))
        if not all(math.isfinite(v) for v in coords):
            continue
        x1, y1, x2, y2 = coords
        if x2 <= x1 or y2 <= y1:
            continue
        # Sample within the box, never below/outside it. Show this ROI in preview.
        patch_x1, patch_x2 = x1 + 0.3*(x2-x1), x1 + 0.7*(x2-x1)
        lo, hi = (0.70, 0.95) if anchor == 'bottom' else (0.35, 0.65)
        roi = (max(0, min(width, math.floor(patch_x1))),
               max(0, min(height, math.floor(y1 + lo*(y2-y1)))),
               max(0, min(width, math.ceil(patch_x2))),
               max(0, min(height, math.ceil(y1 + hi*(y2-y1)))))
        left, top, right, bottom = roi
        patch = depth_m[top:bottom, left:right]
        valid = patch[np.isfinite(patch) & (patch >= min_depth_m) & (patch <= max_depth_m)]
        z = None
        guidance = None
        reason = 'Insufficient valid depth'
        if valid.size >= min_samples and valid.size >= patch.size * 0.5:
            z = float(np.median(valid))
            # Reject large depth discontinuities rather than averaging foreground/background.
            p10, p90 = np.percentile(valid, (10, 90))
            if p90-p10 <= max(0.05, 0.10*z):
                cx, cy = (left+right-1)/2, (top+bottom-1)/2
                x, y, forward = map(float, deproject([cx, cy], z))
                pitch = math.radians(camera_pitch_deg)
                horizontal_forward = forward*math.cos(pitch) + y*math.sin(pitch)
                distance = math.hypot(x, horizontal_forward)
                if (all(math.isfinite(v) for v in (x, y, forward, distance))
                        and horizontal_forward > 0 and distance > 0):
                    error = camera_yaw_deg - math.degrees(math.atan2(x, horizontal_forward))
                    turn = 0.0 if abs(error) <= deadband_deg else error
                    guidance = BaseGuidance(distance, confidence, cx, cy, error, turn)
                    reason = ''
                else:
                    reason = 'Invalid deprojected geometry'
            else:
                reason = 'Mixed-depth patch'
        detections.append(DepthDetection(coords, confidence, roi, z, guidance, reason))
    return DepthScene(tuple(detections))


class RealSenseCamera:
    """Matched RGB/depth frames from one device; imports SDK only when used."""

    def __init__(self):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline = rs.pipeline()
        self.started = False
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        try:
            profile = self.pipeline.start(config)
            self.started = True
            self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self.align = rs.align(rs.stream.color)
        except Exception:
            self.stop()
            raise

    def capture(self):
        import numpy as np
        frames = self.align.process(self.pipeline.wait_for_frames(5000))
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError('RealSense did not provide matched RGB and depth frames')
        image = np.asanyarray(color.get_data()).copy()
        metres = np.asanyarray(depth.get_data()).astype(np.float32) * self.scale
        if metres.shape != image.shape[:2]:
            raise RuntimeError('Depth is not aligned to the RGB frame')
        intrinsics = depth.profile.as_video_stream_profile().get_intrinsics()
        deproject = lambda pixel, z: self.rs.rs2_deproject_pixel_to_point(intrinsics, pixel, z)
        return image, metres, deproject

    def stop(self):
        if self.started:
            self.pipeline.stop()
            self.started = False
