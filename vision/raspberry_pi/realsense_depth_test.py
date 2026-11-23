"""Manual D435i centre-patch depth test; no AI-camera calibration, YOLO or ESP32.

This does NOT automatically measure the target detected by the separate AI Camera.
"""

import argparse
from http.server import ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from realsense_backend import RealSenseCamera
from fire_distance_preview import PreviewHandler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--no-preview', action='store_true', help='Print depth in SSH terminal only')
    args = parser.parse_args()
    import cv2
    import numpy as np

    camera = server = thread = None
    try:
        camera = RealSenseCamera()
        if not args.no_preview:
            server = ThreadingHTTPServer(('127.0.0.1', args.port), PreviewHandler)
            server.latest_jpeg = None
            server.preview_title = 'D435i manual depth test'
            server.preview_caption = ('Point the centre crosshair at a solid target. '
                                      'This is not automatically linked to AI Camera detections.')
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            print(f'Preview http://127.0.0.1:{args.port}; forward this port through SSH.', flush=True)
        print('Manual centre patch; measured surface depth Z in metres. Ctrl+C to stop.', flush=True)
        last_report = 0.0
        while True:
            image, depth, _ = camera.capture()
            height, width = depth.shape
            cx, cy = width//2, height//2
            patch = depth[cy-5:cy+6, cx-5:cx+6]
            valid = patch[np.isfinite(patch) & (patch >= 0.15) & (patch <= 5.0)]
            label = 'Depth unavailable'
            if valid.size >= max(9, patch.size*0.5):
                z = float(np.median(valid))
                p10, p90 = np.percentile(valid, (10, 90))
                label = (f'Surface depth Z: {z:.3f} m ({z*100:.1f} cm)' if
                         p90-p10 <= max(0.05, 0.1*z) else 'Mixed surfaces: reposition crosshair')
            if server:
                cv2.rectangle(image, (cx-5, cy-5), (cx+5, cy+5), (0, 255, 255), 2)
                cv2.drawMarker(image, (cx, cy), (0, 255, 255), markerSize=24)
                cv2.putText(image, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 255, 255), 2)
                cv2.putText(image, 'Manual test - no automatic fire matching', (8, height-16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    raise RuntimeError('JPEG encoding failed')
                server.latest_jpeg = jpeg.tobytes()
            if time.monotonic()-last_report >= 1:
                print(label, flush=True)
                last_report = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if camera:
                camera.stop()
        finally:
            if server:
                server.shutdown()
                server.server_close()
                thread.join()


if __name__ == '__main__':
    main()
