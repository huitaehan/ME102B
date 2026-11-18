"""Camera target geometry for rover-base horizontal aiming (positive = left).

Assumes an unmirrored, level camera rigidly mounted to the chassis. Horizontal
focal length and camera yaw must be calibrated. These are relative angle errors,
not wheel PWM values or absolute headings in the search planner's yaw frame.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BaseGuidance:
    distance_m: float
    confidence: float
    center_x: float
    center_y: float
    yaw_error_deg: float
    turn_deg: float

    @property
    def aligned(self):
        return self.turn_deg == 0.0

    def diagnostic(self):
        """Diagnostic output only: current ESP32 firmware does not accept this."""
        return f'BASE_GUIDANCE,1,{self.yaw_error_deg:.3f},{self.turn_deg:.3f}'


def guidance_from_boxes(boxes, *, frame_width, horizontal_focal_px,
                        vertical_focal_px, flame_height_cm, fire_class=0,
                        camera_yaw_deg=0.0, deadband_deg=3.0):
    """Select the closest estimated fire and return relative chassis turn.

    Box coordinates must refer to the original frame, not resized YOLO input.
    Assumes the optical centre is frame_width / 2. A positive camera yaw points
    left of the rover forward axis. Distance retains the existing height model.
    """
    values = (frame_width, horizontal_focal_px, vertical_focal_px, flame_height_cm)
    if not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError('Frame width, focal lengths and flame height must be positive')
    if not math.isfinite(camera_yaw_deg) or abs(camera_yaw_deg) >= 90:
        raise ValueError('Camera yaw must be between -90 and 90 degrees')
    if not math.isfinite(deadband_deg) or not 0 <= deadband_deg < 90:
        raise ValueError('Deadband must be between 0 and 90 degrees')
    candidates = []
    for box in boxes:
        confidence = float(box.conf[0])
        if int(box.cls[0]) != fire_class or not 0.40 < confidence <= 1:
            continue
        x1, y1, x2, y2 = map(float, box.xyxy[0])
        if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
            continue
        if x2 <= x1 or y2 <= y1:
            continue
        center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
        error = camera_yaw_deg + math.degrees(math.atan2(
            frame_width / 2 - center_x, horizontal_focal_px))
        distance = flame_height_cm * vertical_focal_px / (y2 - y1) / 100
        turn = 0.0 if abs(error) <= deadband_deg else error
        candidates.append(BaseGuidance(distance, confidence, center_x, center_y,
                                       error, turn))
    return min(candidates, key=lambda t: (t.distance_m, -t.confidence), default=None)
