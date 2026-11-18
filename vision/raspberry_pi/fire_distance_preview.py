"""Picamera2 + CPU YOLO fire/distance preview, viewable through an SSH tunnel.

Uses the AI Camera as an image source; no compiled IMX500 model is required.
"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import math
from pathlib import Path
import threading
import time


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
                b'<h1>Fire detection and estimated distance</h1>'
                b'<p>Distance depends on flame-height and camera calibration.</p>'
                b'<img id="feed" width="640" height="640" alt="Live detections">'
                b'<script>const feed=document.getElementById("feed");'
                b'function refresh(){feed.src="/frame.jpg?t="+Date.now();}'
                b'setInterval(refresh,500);refresh();</script>'
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
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (
            args.flame_height_cm, args.focal_length_px)):
        parser.error('Flame height and focal length must be finite and positive')
    if not args.model.is_file():
        parser.error(f'Model not found: {args.model}')

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
            main={'size': (640, 640), 'format': 'RGB888'}))
        camera.start()
        print(f'Preview: http://127.0.0.1:{args.port} (use SSH port forwarding)', flush=True)
        print('CPU inference; distance is an estimate. Ctrl+C to stop.', flush=True)
        last_report = 0.0
        while True:
            frame = camera.capture_array('main')
            result = model(frame, device='cpu', conf=0.40,
                           classes=[args.fire_class], verbose=False)[0]
            reports = []
            for box in result.boxes:
                x1, y1, x2, y2 = map(float, box.xyxy[0])
                if y2 <= y1:
                    continue
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
            ok, encoded = cv2.imencode('.jpg', frame)
            if not ok:
                raise RuntimeError('Could not encode preview image')
            server.latest_jpeg = encoded.tobytes()
            if time.monotonic() - last_report >= 1:
                print('; '.join(reports) if reports else 'No fire detected', flush=True)
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
