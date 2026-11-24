"""YOLO -> ESP32 USB serial bridge; run from repository root."""
import argparse
from collections import deque
import importlib.util
import math
from pathlib import Path
import time

if __package__:
    from .base_guidance import guidance_from_boxes
else:
    from base_guidance import guidance_from_boxes

SOLVER_PATH = Path(__file__).resolve().parents[1] / 'Calcs and Shooting' / 'Projectile Motion Calculation.py'
_spec = importlib.util.spec_from_file_location('projectile_solver', SOLVER_PATH)
solver = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(solver)


def solve_request(line, args):
    """Answer ESP32 trajectory requests using metres and degrees."""
    if not line.startswith('AIM_REQUEST,'):
        return None
    fields = line.split(',')
    request_id = fields[1] if len(fields) > 1 else '0'
    try:
        if len(fields) != 5 or not request_id.isdecimal():
            raise ValueError('Malformed request')
        distance, minimum, maximum = map(float, fields[2:])
        if not all(math.isfinite(v) for v in (distance, minimum, maximum)):
            raise ValueError('Nonfinite request')
        if distance <= 0 or not 0 <= minimum < maximum < 90:
            raise ValueError('Invalid geometry')
        if args.jet_speed_m_s is None:
            raise ValueError('Jet calibration missing')
        angle = solver.launch_angle(
            distance, args.target_height_m, args.jet_speed_m_s,
            args.nozzle_length_m, args.turret_x_m, args.turret_y_m,
            args.base_height_m, beta_min=minimum, beta_max=maximum)
        if not math.isfinite(angle) or not minimum <= angle <= maximum:
            raise ValueError('Invalid angle')
        return f'AIM_RESULT,{request_id},{angle:.4f}\n'
    except (ValueError, solver.OutOfRange, OverflowError, ZeroDivisionError):
        return f'AIM_ERROR,{request_id}\n'


def observation(boxes, flame_height_cm, focal_length_px):
    """Match repository's >0.40 threshold and pinhole distance estimate."""
    candidates = [b for b in boxes if float(b.conf[0]) > 0.40]
    if not candidates:
        return 'FIRE,0,0,0\n'
    box = max(candidates, key=lambda b: float(b.conf[0]))
    _, y1, _, y2 = [float(v) for v in box.xyxy[0]]
    height_px = max(y2 - y1, 1.0)
    metres = flame_height_cm * focal_length_px / height_px / 100.0
    return f'FIRE,1,{metres:.4f},{float(box.conf[0]):.4f}\n'


def mock_target_message(sequence, guidance):
    """Atomic bearing/distance observation for the explicitly simulated ESP32 run."""
    if guidance is None:
        return f'TARGET,{sequence},0,0,0,0\n'
    return (f'TARGET,{sequence},1,{guidance.distance_m:.4f},'
            f'{guidance.confidence:.4f},{guidance.yaw_error_deg:.3f}\n')


def require_capabilities(link, required, timeout=10.0):
    """Query without changing controller state; refuse an incompatible protocol."""
    required = set(required)
    capabilities = set()
    link.write(b'CAPS\n')
    deadline = time.monotonic() + timeout
    next_query = time.monotonic() + 0.5
    recent_lines = deque(maxlen=8)
    while time.monotonic() < deadline:
        if time.monotonic() >= next_query:
            # CAPS is read-only: retry if the first query arrived during boot.
            link.write(b'CAPS\n')
            next_query = time.monotonic() + 0.5
        line = link.readline().decode('ascii', errors='replace').strip()
        if line:
            recent_lines.append(line)
        if line.startswith('CAPS,'):
            capabilities.update(line.split(',')[1:])
            if required <= capabilities:
                return
    else:
        received = ' | '.join(recent_lines) if recent_lines else '(no serial response)'
        raise RuntimeError(
            'ESP32 does not advertise ' + ','.join(sorted(required-capabilities)) +
            ' within startup timeout. Upload the matching FireTurret firmware or '
            'check serial port/reset timing. Received: ' + received)


def start_mock_session(link, timeout=10.0, require_range_wait=False):
    """Start only a compatible explicit mock; RANGE_WAIT is needed for depth mode."""
    required = {'MOCK_BASE_V1'}
    if require_range_wait:
        required.add('RANGE_WAIT_V1')
    require_capabilities(link, required, timeout)
    # Start is stateful; unlike CAPS, do not blindly repeat it.
    link.write(b'MOCK_START\n')
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = link.readline().decode('ascii', errors='replace').strip()
        if line.startswith('MOCK_READY,'):
            print(line, flush=True)
            return
        if line.startswith('ERR,MOCK_START'):
            raise RuntimeError(line + '; reset ESP32 before another mock run')
    raise RuntimeError('ESP32 did not acknowledge MOCK_START; reset it before retrying')


def depth_message(scene, sequence=None):
    """Preserve three states: no fire, valid target, and fire with unknown range."""
    message = scene.fire_message()
    if message is None:
        return 'RANGE_UNKNOWN\n'
    if sequence is not None:
        return mock_target_message(sequence, scene.guidance)
    return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='e.g. /dev/ttyUSB0 or COM5')
    parser.add_argument('--camera', default='0', help='OpenCV camera index or URL')
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument('--picamera', action='store_true', help='Use Picamera2 on Pi')
    sources.add_argument('--imx500-model', type=Path, help='Run inference on AI Camera using this RPK')
    sources.add_argument('--realsense', action='store_true', help='YOLO on D435i RGB with aligned depth')
    parser.add_argument('--dual-depth', action='store_true',
                        help='Use IMX500 accelerator + D435i depth only; requires imx500-model and calibration')
    parser.add_argument('--dual-calibration', type=Path)
    parser.add_argument('--preview-port', type=int,
                        help='Optional browser preview of the same dual-depth observations, e.g. 8080')
    parser.add_argument('--depth-anchor', choices=('center', 'bottom'), default='bottom')
    parser.add_argument('--camera-pitch-deg', type=float, default=0.0,
                        help='D435i mounting pitch above horizontal for ground-plane range')
    parser.add_argument('--imx500-box-order', choices=('xyxy', 'yxyx'))
    parser.add_argument('--imx500-box-units', choices=('normalized', 'pixels'))
    parser.add_argument('--fire-class', type=int, default=0)
    parser.add_argument('--model', default=str(Path(__file__).resolve().parents[1] / 'models/best.pt'))
    parser.add_argument('--flame-height-cm', type=float, default=5.0)
    parser.add_argument('--focal-length-px', type=float,
                        help='Required except with --realsense, which uses SDK intrinsics')
    parser.add_argument('--start', action='store_true', help='Bench test only: requires firmware ALLOW_SERIAL_START=true')
    parser.add_argument('--jet-speed-m-s', type=float, help='Measured jet exit speed for physical aiming')
    parser.add_argument('--target-height-m', type=float, default=0.0)
    parser.add_argument('--nozzle-length-m', type=float, default=0.0)
    parser.add_argument('--turret-x-m', type=float, default=0.0, help='Pivot offset from camera distance reference')
    parser.add_argument('--turret-y-m', type=float, default=0.0)
    parser.add_argument('--base-height-m', type=float, default=0.0)
    parser.add_argument('--base-guidance', action='store_true',
                        help='Select closest fire and PRINT relative base turn (no wheel actuation)')
    parser.add_argument('--horizontal-focal-px', type=float,
                        help='Horizontal focal length at capture resolution; default assumes square pixels')
    parser.add_argument('--camera-yaw-deg', type=float, default=0.0)
    parser.add_argument('--alignment-deadband-deg', type=float, default=3.0)
    parser.add_argument('--mock-base', action='store_true',
                        help='Send TARGET observations to DEMO_MODE ESP32; simulate all actuators/sensors')
    args = parser.parse_args()
    if args.preview_port is not None and (not args.dual_depth or not 1 <= args.preview_port <= 65535):
        parser.error('--preview-port requires --dual-depth and a port between 1 and 65535')
    if args.dual_depth and (not args.imx500_model or not args.dual_calibration):
        parser.error('--dual-depth needs --imx500-model and --dual-calibration')
    if not args.realsense and not args.dual_depth and args.focal_length_px is None:
        parser.error('--focal-length-px is required unless using --realsense')
    if args.realsense or args.dual_depth:
        args.base_guidance = True
    if args.mock_base:
        args.base_guidance = True
        if args.start:
            parser.error('Use --mock-base without --start')
        if args.jet_speed_m_s is None:
            parser.error('--mock-base needs --jet-speed-m-s (an example value is allowed for mock only)')
    if args.base_guidance and args.imx500_model and not args.dual_depth:
        parser.error('Base guidance currently supports CPU image inference only')
    horizontal_focal = (args.horizontal_focal_px if args.horizontal_focal_px is not None
                        else args.focal_length_px)
    if args.base_guidance and not args.realsense and not args.dual_depth:
        try:
            guidance_from_boxes([], frame_width=640, horizontal_focal_px=horizontal_focal,
                                vertical_focal_px=args.focal_length_px,
                                flame_height_cm=args.flame_height_cm,
                                camera_yaw_deg=args.camera_yaw_deg,
                                deadband_deg=args.alignment_deadband_deg)
        except ValueError as exc:
            parser.error(str(exc))
    if args.imx500_model and (not args.imx500_box_order or not args.imx500_box_units):
        parser.error('Verify and supply --imx500-box-order and --imx500-box-units')
    if args.imx500_model and not args.imx500_model.is_file():
        parser.error('IMX500 RPK file does not exist; package the model on the Pi first')
    values = [args.flame_height_cm, args.target_height_m,
              args.nozzle_length_m, args.turret_x_m, args.turret_y_m, args.base_height_m]
    if args.focal_length_px is not None:
        values.append(args.focal_length_px)
    if not all(math.isfinite(v) for v in values) or args.nozzle_length_m < 0:
        parser.error('Geometry must be finite and nozzle length nonnegative')
    if args.jet_speed_m_s is not None and (not math.isfinite(args.jet_speed_m_s) or args.jet_speed_m_s <= 0):
        parser.error('Jet speed must be finite and positive')
    if args.flame_height_cm <= 0 or (args.focal_length_px is not None and args.focal_length_px <= 0):
        parser.error('Calibration values must be positive')
    if args.realsense:
        if __package__:
            from .realsense_backend import RealSenseCamera, depth_scene
        else:
            from realsense_backend import RealSenseCamera, depth_scene
        try:
            depth_scene([], [[0]], lambda pixel, z: (0, 0, z),
                        camera_yaw_deg=args.camera_yaw_deg,
                        camera_pitch_deg=args.camera_pitch_deg,
                        deadband_deg=args.alignment_deadband_deg)
        except ValueError as exc:
            parser.error(str(exc))
    if args.dual_depth:
        if __package__:
            from .dual_camera_backend import DualCamera
        else:
            from dual_camera_backend import DualCamera
    import cv2
    import serial
    model = None
    if not args.imx500_model:
        from ultralytics import YOLO
        model = YOLO(args.model)
    camera = None
    link = None
    preview = None
    try:
        if args.dual_depth:
            camera = DualCamera(
                args.imx500_model, args.dual_calibration,
                box_order=args.imx500_box_order, box_units=args.imx500_box_units,
                fire_class=args.fire_class, anchor=args.depth_anchor,
                camera_yaw_deg=args.camera_yaw_deg, camera_pitch_deg=args.camera_pitch_deg,
                deadband_deg=args.alignment_deadband_deg)
        elif args.realsense:
            camera = RealSenseCamera()
        elif args.imx500_model:
            from picamera2 import Picamera2
            from picamera2.devices.imx500 import IMX500
            if __package__:
                from .imx500_backend import observation_from_outputs
            else:
                from imx500_backend import observation_from_outputs
            imx500 = IMX500(str(args.imx500_model))
            camera = Picamera2(imx500.camera_num)
            camera.configure(camera.create_preview_configuration(
                main={'size': (640, 640), 'format': 'RGB888'}, buffer_count=12))
            camera.start()
            input_width, input_height = imx500.get_input_size()
        elif args.picamera:
            from picamera2 import Picamera2
            camera = Picamera2()
            camera.configure(camera.create_video_configuration(
                main={'size': (640, 640), 'format': 'RGB888'}))
            camera.start()
        else:
            source = int(args.camera) if args.camera.isdecimal() else args.camera
            camera = cv2.VideoCapture(source)
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not camera.isOpened():
                raise RuntimeError('Camera failed to open')
        if args.preview_port is not None:
            if __package__:
                from .bridge_preview import BridgePreview
            else:
                from bridge_preview import BridgePreview
            preview = BridgePreview(args.preview_port)
        link = serial.Serial(args.port, 115200, timeout=0.1, write_timeout=1)
        time.sleep(2)  # Allow ESP32 USB auto-reset to finish.
        if args.mock_base:
            start_mock_session(link, require_range_wait=args.dual_depth or args.realsense)
        elif args.dual_depth or args.realsense:
            require_capabilities(link, {'RANGE_WAIT_V1'})
        if args.start:
            link.write(b'START\n')
        sequence = 0
        range_ready = True
        last_depth_notice = 0.0
        while True:
            if args.dual_depth:
                frame, scene = camera.capture()
                if preview is not None:
                    preview.publish(frame, scene)
                guidance = scene.guidance
                range_ready = guidance is not None
                if not scene.range_valid:
                    if time.monotonic()-last_depth_notice >= 1:
                        print('WAITING_FOR_DEPTH: ' + '; '.join(
                            d.reason for d in scene.detections if d.guidance is None), flush=True)
                        last_depth_notice = time.monotonic()
                else:
                    print(guidance.diagnostic() if guidance else 'BASE_GUIDANCE,0,0,0', flush=True)
                if guidance:
                    print(f'VISION: range={guidance.distance_m:.3f} m '
                          f'conf={guidance.confidence:.2f} bearing={guidance.yaw_error_deg:+.2f} deg', flush=True)
                sequence += 1
                message = depth_message(scene, sequence if args.mock_base else None)
            elif args.realsense:
                frame, depth, deproject = camera.capture()
            elif args.imx500_model:
                metadata = camera.capture_metadata()
                outputs = imx500.get_outputs(metadata, add_batch=True)
                message = observation_from_outputs(
                    outputs, box_order=args.imx500_box_order,
                    box_units=args.imx500_box_units, input_height=input_height,
                    flame_height_cm=args.flame_height_cm,
                    focal_length_px=args.focal_length_px, fire_class=args.fire_class)
            elif args.picamera:
                frame = camera.capture_array('main')
            else:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError('Camera capture failed; no clear-fire frame sent')
            if not args.imx500_model:
                results = model(frame, verbose=False)
                boxes = [box for result in results for box in result.boxes]
                if args.realsense:
                    scene = depth_scene(
                        boxes, depth, deproject, fire_class=args.fire_class,
                        anchor=args.depth_anchor, camera_yaw_deg=args.camera_yaw_deg,
                        camera_pitch_deg=args.camera_pitch_deg,
                        deadband_deg=args.alignment_deadband_deg)
                    guidance = scene.guidance
                    range_ready = guidance is not None
                    if not scene.range_valid:
                        if time.monotonic()-last_depth_notice >= 1:
                            print('WAITING_FOR_DEPTH: no usable surface depth', flush=True)
                            last_depth_notice = time.monotonic()
                    else:
                        print(guidance.diagnostic() if guidance else 'BASE_GUIDANCE,0,0,0', flush=True)
                    sequence += 1
                    message = depth_message(scene, sequence if args.mock_base else None)
                elif args.base_guidance:
                    guidance = guidance_from_boxes(
                        boxes, frame_width=frame.shape[1], horizontal_focal_px=horizontal_focal,
                        vertical_focal_px=args.focal_length_px,
                        flame_height_cm=args.flame_height_cm, fire_class=args.fire_class,
                        camera_yaw_deg=args.camera_yaw_deg,
                        deadband_deg=args.alignment_deadband_deg)
                    if guidance is None:
                        message = 'FIRE,0,0,0\n'
                        print('BASE_GUIDANCE,0,0,0', flush=True)
                    else:
                        message = f'FIRE,1,{guidance.distance_m:.4f},{guidance.confidence:.4f}\n'
                        print(guidance.diagnostic(), flush=True)
                    if args.mock_base:
                        sequence += 1
                        message = mock_target_message(sequence, guidance)
                else:
                    message = observation(boxes, args.flame_height_cm, args.focal_length_px)
            if message is not None:
                link.write(message.encode('ascii'))
            while link.in_waiting:
                line = link.readline().decode('ascii', errors='replace').strip()
                print(line)
                if preview is not None and line.startswith('STATE,'):
                    preview.state_line = 'ESP32 ' + line
                # RANGE_UNKNOWN cancels a pending aim on ESP32. Do not solve a
                # queued request against its former distance during that wait.
                reply = solve_request(line, args) if range_ready else None
                if reply is not None:
                    link.write(reply.encode('ascii'))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if link is not None:
                try:
                    link.write(b'STOP\n')
                    link.flush()
                finally:
                    link.close()
        finally:
            try:
                if camera is not None:
                    if args.picamera or args.imx500_model or args.realsense:
                        camera.stop()
                    else:
                        camera.release()
            finally:
                if preview is not None:
                    preview.close()


if __name__ == '__main__':
    main()
