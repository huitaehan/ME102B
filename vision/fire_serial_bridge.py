"""YOLO -> ESP32 USB serial bridge; run from repository root."""
import argparse
import importlib.util
import math
from pathlib import Path
import time

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='e.g. /dev/ttyUSB0 or COM5')
    parser.add_argument('--camera', default='0', help='OpenCV camera index or URL')
    parser.add_argument('--picamera', action='store_true', help='Use Picamera2 on Pi')
    parser.add_argument('--imx500-model', type=Path, help='Run inference on AI Camera using this RPK')
    parser.add_argument('--imx500-box-order', choices=('xyxy', 'yxyx'))
    parser.add_argument('--imx500-box-units', choices=('normalized', 'pixels'))
    parser.add_argument('--fire-class', type=int, default=0)
    parser.add_argument('--model', default=str(Path(__file__).resolve().parents[1] / 'models/best.pt'))
    parser.add_argument('--flame-height-cm', type=float, default=5.0)
    parser.add_argument('--focal-length-px', type=float, required=True,
                        help='Repo defaults: 600 webcam; 490 Pi at 640x640. Calibrate.')
    parser.add_argument('--start', action='store_true', help='Bench test only: requires firmware ALLOW_SERIAL_START=true')
    parser.add_argument('--jet-speed-m-s', type=float, help='Measured jet exit speed for physical aiming')
    parser.add_argument('--target-height-m', type=float, default=0.0)
    parser.add_argument('--nozzle-length-m', type=float, default=0.0)
    parser.add_argument('--turret-x-m', type=float, default=0.0, help='Pivot offset from camera distance reference')
    parser.add_argument('--turret-y-m', type=float, default=0.0)
    parser.add_argument('--base-height-m', type=float, default=0.0)
    args = parser.parse_args()
    if args.imx500_model and (not args.imx500_box_order or not args.imx500_box_units):
        parser.error('Verify and supply --imx500-box-order and --imx500-box-units')
    if args.imx500_model and not args.imx500_model.is_file():
        parser.error('IMX500 RPK file does not exist; package the model on the Pi first')
    values = [args.flame_height_cm, args.focal_length_px, args.target_height_m,
              args.nozzle_length_m, args.turret_x_m, args.turret_y_m, args.base_height_m]
    if not all(math.isfinite(v) for v in values) or args.nozzle_length_m < 0:
        parser.error('Geometry must be finite and nozzle length nonnegative')
    if args.jet_speed_m_s is not None and (not math.isfinite(args.jet_speed_m_s) or args.jet_speed_m_s <= 0):
        parser.error('Jet speed must be finite and positive')
    if args.flame_height_cm <= 0 or args.focal_length_px <= 0:
        parser.error('Calibration values must be positive')
    import cv2
    import serial
    model = None
    if not args.imx500_model:
        from ultralytics import YOLO
        model = YOLO(args.model)
    camera = None
    link = None
    try:
        if args.imx500_model:
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
        link = serial.Serial(args.port, 115200, timeout=0.1, write_timeout=1)
        time.sleep(2)  # Allow ESP32 USB auto-reset to finish.
        if args.start:
            link.write(b'START\n')
        while True:
            if args.imx500_model:
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
                message = observation(boxes, args.flame_height_cm, args.focal_length_px)
            if message is not None:
                link.write(message.encode('ascii'))
            while link.in_waiting:
                line = link.readline().decode('ascii', errors='replace').strip()
                print(line)
                reply = solve_request(line, args)
                if reply is not None:
                    link.write(reply.encode('ascii'))
    except KeyboardInterrupt:
        pass
    finally:
        if link is not None:
            try:
                link.write(b'STOP\n')
                link.flush()
            finally:
                link.close()
        if camera is not None:
            if args.picamera or args.imx500_model:
                camera.stop()
            else:
                camera.release()


if __name__ == '__main__':
    main()
