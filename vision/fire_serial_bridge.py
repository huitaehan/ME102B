"""YOLO -> ESP32 USB serial bridge; run from repository root."""
import argparse
from pathlib import Path
import time


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
    parser.add_argument('--model', default=str(Path(__file__).resolve().parents[1] / 'models/best.pt'))
    parser.add_argument('--flame-height-cm', type=float, default=5.0)
    parser.add_argument('--focal-length-px', type=float, required=True,
                        help='Repo defaults: 600 webcam; 490 Pi at 640x640. Calibrate.')
    parser.add_argument('--start', action='store_true', help='Command automatic homing')
    args = parser.parse_args()
    if args.flame_height_cm <= 0 or args.focal_length_px <= 0:
        parser.error('Calibration values must be positive')
    import cv2
    import serial
    from ultralytics import YOLO
    model = YOLO(args.model)
    camera = None
    link = None
    try:
        if args.picamera:
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
            if args.picamera:
                frame = camera.capture_array('main')
            else:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError('Camera capture failed; no clear-fire frame sent')
            results = model(frame, verbose=False)
            boxes = [box for result in results for box in result.boxes]
            message = observation(boxes, args.flame_height_cm, args.focal_length_px)
            link.write(message.encode('ascii'))
            while link.in_waiting:
                print(link.readline().decode('ascii', errors='replace').strip())
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
            if args.picamera:
                camera.stop()
            else:
                camera.release()


if __name__ == '__main__':
    main()
