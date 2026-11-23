"""IMX500 NMS output adapter. Verify box conventions on the physical camera."""
import math


def preview_boxes_from_outputs(outputs, *, box_order, box_units, input_size,
                               convert_coords, fire_class=0, confidence_threshold=0.40):
    """Decode NMS and map normalized model yxyx through IMX500's crop mapper.

    convert_coords accepts normalized yxyx and returns preview x,y,width,height.
    Missing metadata returns None, never an empty no-fire observation.
    """
    if not math.isfinite(confidence_threshold) or not 0 <= confidence_threshold < 1:
        raise ValueError('Confidence threshold must be between 0 and 1')
    if outputs is None:
        return None
    import numpy as np
    from types import SimpleNamespace
    if len(outputs) != 4:
        raise ValueError('Expected boxes, scores, classes, count NMS tensors')
    if box_order not in ('xyxy', 'yxyx') or box_units not in ('normalized', 'pixels'):
        raise ValueError('Verify model box order and units')
    raw = np.asarray(outputs[0])
    if raw.ndim < 2 or raw.shape[-1] != 4:
        raise ValueError('Expected boxes with trailing dimension 4')
    boxes = raw.reshape(-1, 4)
    scores, classes, counts = [np.asarray(v).reshape(-1) for v in outputs[1:]]
    if (len(counts) != 1 or not np.isfinite(counts[0]) or counts[0] != int(counts[0])
            or not 0 <= int(counts[0]) <= min(len(boxes), len(scores), len(classes))):
        raise ValueError('Invalid NMS count')
    result = []
    width, height = input_size
    for box, score, category in zip(boxes[:int(counts[0])], scores, classes):
        if (not np.all(np.isfinite(box)) or not math.isfinite(float(score))
                or not 0 <= score <= 1 or not math.isfinite(float(category))
                or category != int(category)):
            raise ValueError('Invalid detection values')
        if category != fire_class or score <= confidence_threshold:
            continue
        if box_order == 'xyxy':
            x1, y1, x2, y2 = map(float, box)
        else:
            y1, x1, y2, x2 = map(float, box)
        if box_units == 'pixels':
            x1, x2, y1, y2 = x1/width, x2/width, y1/height, y2/height
        if x2 <= x1 or y2 <= y1:
            raise ValueError('Invalid detection box')
        x, y, w, h = convert_coords((y1, x1, y2, x2))
        if w > 0 and h > 0:
            result.append(SimpleNamespace(xyxy=[(x, y, x+w, y+h)],
                                          conf=[float(score)], cls=[int(category)]))
    return result


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
