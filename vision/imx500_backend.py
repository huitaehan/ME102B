"""IMX500 NMS output adapter. Verify box conventions on the physical camera."""
import math


def observation_from_outputs(outputs, *, box_order, box_units, input_height,
                             flame_height_cm, focal_length_px, fire_class=0):
    """Decode boxes/scores/classes/count; missing inference is not a clear frame."""
    if outputs is None:
        return None
    import numpy as np
    if len(outputs) != 4:
        raise ValueError('Expected four NMS tensors: boxes, scores, classes, count')
    raw_boxes = np.asarray(outputs[0])
    if raw_boxes.ndim < 2 or raw_boxes.shape[-1] != 4:
        raise ValueError('Expected boxes with trailing dimension 4; verify tensor layout')
    boxes = raw_boxes.reshape(-1, 4)
    scores = np.asarray(outputs[1]).reshape(-1)
    classes = np.asarray(outputs[2]).reshape(-1)
    counts = np.asarray(outputs[3]).reshape(-1)
    if len(counts) != 1 or not np.isfinite(counts[0]):
        raise ValueError('Invalid NMS detection count')
    count = int(counts[0])
    if count != counts[0] or not 0 <= count <= min(len(boxes), len(scores), len(classes)):
        raise ValueError('NMS detection count exceeds tensor dimensions')
    best = None
    for box, score, category in zip(boxes[:count], scores[:count], classes[:count]):
        if not np.all(np.isfinite(box)) or not math.isfinite(float(score)):
            raise ValueError('Nonfinite detection')
        if not math.isfinite(float(category)) or category != int(category) or not 0 <= score <= 1:
            raise ValueError('Invalid class or confidence')
        if int(category) != fire_class or score <= 0.40:
            continue
        if box_order == 'yxyx':
            y1, x1, y2, x2 = map(float, box)
        else:
            x1, y1, x2, y2 = map(float, box)
        if x2 <= x1 or y2 <= y1:
            raise ValueError('Invalid bounding box geometry')
        height_px = (y2 - y1) * input_height if box_units == 'normalized' else y2 - y1
        # Focal calibration must use model-input pixels, not preview pixels.
        distance = flame_height_cm * focal_length_px / height_px / 100.0
        if best is None or score > best[0]:
            best = (float(score), distance)
    if best is None:
        return 'FIRE,0,0,0\n'
    return f'FIRE,1,{best[1]:.4f},{best[0]:.4f}\n'
