"""Picamera2 + CPU YOLO fire/distance preview, viewable through an SSH tunnel.

Uses the AI Camera as an image source; no compiled IMX500 model is required.
"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import math
from pathlib import Path
import threading
import time
import sys

# Permit direct execution from the repository root or any other directory.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from base_guidance import guidance_from_boxes


class PreviewHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith('/frame.jpg'):
            data = self.server.latest_jpeg
            if data is None:
                self.send_error(503, 'Waiting for the first inference frame')
                return
            content_type = 'image/jpeg'
        elif self.path == '/':
            data = (
                b'<!doctype html><title>Fire distance preview</title>'
                b'<h1>Fire detection and distance</h1>'
                b'<p>Check the distance source and validity shown in the image.</p>'
                b'<img id="feed" style="width:640px;max-width:100%;height:auto" alt="Live detections">'
                b'<script>const feed=document.getElementById("feed");'
                b'function refresh(){feed.src="/frame.jpg?t="+Date.now();}'
                b'feed.onload=()=>setTimeout(refresh,50);'
                b'feed.onerror=()=>setTimeout(refresh,500);refresh();</script>'
            )
            content_type = 'text/html; charset=utf-8'
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=(
        Path(__file__).resolve().parents[2] / 'models/best.pt'))
    parser.add_argument('--flame-height-cm', type=float, required=True)
    parser.add_argument('--focal-length-px', type=float, required=True,
                        help='Vertical focal length calibrated at 640x640 capture size')
    parser.add_argument('--fire-class', type=int, default=0)
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--imgsz', type=int, default=320,
                        help='YOLO input size: 320 for speed, 640 for smaller targets')
    parser.add_argument('--jpeg-quality', type=int, default=75)
    parser.add_argument('--exposure-us', type=int,
                        help='Optional fixed exposure in microseconds; disables auto exposure')
    parser.add_argument('--gain', type=float, default=1.0,
                        help='Analogue gain used with --exposure-us')
    parser.add_argument('--horizontal-focal-px', type=float,
                        help='Horizontal focal length; defaults to focal-length-px (square pixels)')
    parser.add_argument('--camera-yaw-deg', type=float, default=0.0,
                        help='Camera mounting yaw relative to rover forward, positive left')
    parser.add_argument('--alignment-deadband-deg', type=float, default=3.0)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (
            args.flame_height_cm, args.focal_length_px)):
        parser.error('Flame height and focal length must be finite and positive')
    if not args.model.is_file():
        parser.error(f'Model not found: {args.model}')
    if args.imgsz <= 0 or args.imgsz % 32:
        parser.error('imgsz must be a positive multiple of 32')
    if not 1 <= args.jpeg_quality <= 100:
        parser.error('jpeg-quality must be between 1 and 100')
    if args.exposure_us is not None and args.exposure_us <= 0:
        parser.error('exposure-us must be positive')
    if not math.isfinite(args.gain) or args.gain < 1:
        parser.error('gain must be finite and at least 1')
    horizontal_focal = (args.horizontal_focal_px if args.horizontal_focal_px is not None
                        else args.focal_length_px)
    try:
        guidance_from_boxes([], frame_width=640, horizontal_focal_px=horizontal_focal,
                            vertical_focal_px=args.focal_length_px,
                            flame_height_cm=args.flame_height_cm,
                            camera_yaw_deg=args.camera_yaw_deg,
                            deadband_deg=args.alignment_deadband_deg)
    except ValueError as exc:
        parser.error(str(exc))

    import cv2
    from picamera2 import Picamera2
    from ultralytics import YOLO

    model = YOLO(str(args.model))
    server = ThreadingHTTPServer(('127.0.0.1', args.port), PreviewHandler)
    server.latest_jpeg = None
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    camera = None
    try:
        camera = Picamera2()
        # Picamera2 RGB888 arrays have the BGR channel order OpenCV expects.
        camera.configure(camera.create_video_configuration(
            main={'size': (640, 640), 'format': 'RGB888'}, queue=False))
        if args.exposure_us is not None:
            camera.set_controls({'AeEnable': False, 'ExposureTime': args.exposure_us,
                                 'AnalogueGain': args.gain})
        camera.start()
        print(f'Preview: http://127.0.0.1:{args.port} (use SSH port forwarding)', flush=True)
        print('CPU inference; distance is an estimate. Ctrl+C to stop.', flush=True)
        last_report = 0.0
        while True:
            frame_start = time.monotonic()
            frame = camera.capture_array('main')
            inference_start = time.monotonic()
            result = model(frame, device='cpu', conf=0.40,
                           imgsz=args.imgsz, classes=[args.fire_class], verbose=False)[0]
            inference_ms = (time.monotonic() - inference_start) * 1000
            guidance = guidance_from_boxes(
                result.boxes, frame_width=frame.shape[1],
                horizontal_focal_px=horizontal_focal,
                vertical_focal_px=args.focal_length_px,
                flame_height_cm=args.flame_height_cm, fire_class=args.fire_class,
                camera_yaw_deg=args.camera_yaw_deg,
                deadband_deg=args.alignment_deadband_deg)
            reports = []
            for box in result.boxes:
                x1, y1, x2, y2 = map(float, box.xyxy[0])
                if y2 <= y1:
                    continue
                # Ultralytics maps boxes back to the original 640x640 frame,
                # so changing inference size does not change focal calibration.
                distance = args.flame_height_cm * args.focal_length_px / (y2 - y1)
                confidence = float(box.conf[0])
                left, top, right, bottom = map(int, (x1, y1, x2, y2))
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                cv2.circle(frame, ((left + right) // 2, (top + bottom) // 2),
                           4, (0, 255, 0), -1)
                label = f'Fire {confidence:.2f} | Est. distance {distance:.1f} cm'
                cv2.putText(frame, label, (8, 24 + 24 * len(reports)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                reports.append(label)
            if not reports:
                cv2.putText(frame, 'No fire detected', (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if guidance is not None:
                center = (int(guidance.center_x), int(guidance.center_y))
                cv2.circle(frame, center, 12, (255, 255, 0), 2)
                direction = 'LEFT' if guidance.turn_deg > 0 else 'RIGHT'
                steering = ('Base aligned' if guidance.aligned else
                            f'Base turn {direction} {abs(guidance.turn_deg):.1f} deg')
                cv2.putText(frame, steering, (8, 590), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255, 255, 0), 2)
            cv2.putText(frame, f'CPU inference: {inference_ms:.0f} ms', (8, 620),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            ok, encoded = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
            if not ok:
                raise RuntimeError('Could not encode preview image')
            server.latest_jpeg = encoded.tobytes()
            if time.monotonic() - last_report >= 1:
                status = '; '.join(reports) if reports else 'No fire detected'
                fps = 1 / max(time.monotonic() - frame_start, 1e-6)
                print(f'{status} | inference {inference_ms:.0f} ms | loop {fps:.1f} FPS',
                      flush=True)
                print(guidance.diagnostic() if guidance else 'BASE_GUIDANCE,0,0,0',
                      flush=True)
                last_report = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.close()
        server.shutdown()
        server.server_close()
        worker.join()


if __name__ == '__main__':
    main()
