"""Browser output for frames already captured by the serial bridge."""

from http.server import ThreadingHTTPServer
import threading

if __package__:
    from .raspberry_pi.fire_distance_preview import PreviewHandler
else:
    from raspberry_pi.fire_distance_preview import PreviewHandler


class BridgePreview:
    def __init__(self, port):
        self.server = ThreadingHTTPServer(('127.0.0.1', port), PreviewHandler)
        self.server.latest_jpeg = None
        self.server.preview_title = 'Live vision and ESP32 integration'
        self.server.preview_caption = ('Same observations used by the serial bridge. '
                                       'Mock mode simulates movement; centre the target manually.')
        self.state_line = 'Waiting for ESP32 state'
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        print(f'Bridge preview: http://127.0.0.1:{port} (use SSH tunnel)', flush=True)

    def publish(self, frame, scene):
        import cv2
        image = frame.copy()  # Never annotate the capture/inference buffer itself.
        for i, detection in enumerate(scene.detections):
            x1, y1, x2, y2 = map(int, detection.box)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            l, t, r, b = detection.roi
            cv2.rectangle(image, (l, t), (r, b), (255, 255, 0), 2)
            label = f'Fire conf {detection.confidence:.2f}'
            if detection.guidance:
                label += f' | horizontal range {detection.guidance.distance_m:.2f} m'
            else:
                label += ' | ' + detection.reason
            cv2.putText(image, label, (8, 24+24*i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 2)
        target = scene.guidance
        status = 'No accepted fire' if not scene.detections else 'Depth unavailable'
        if target:
            direction = 'LEFT' if target.turn_deg > 0 else 'RIGHT'
            status = ('ALIGNED' if target.aligned else
                      f'Turn {direction} {abs(target.turn_deg):.1f} deg')
            status += f' | bearing {target.yaw_error_deg:+.1f} deg'
            cv2.circle(image, (int(target.center_x), int(target.center_y)), 8, (255, 255, 0), 2)
        height = image.shape[0]
        for text, y in ((status, height-40), (self.state_line, height-16)):
            cv2.putText(image, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 2)
        ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            raise RuntimeError('Bridge preview JPEG encoding failed')
        self.server.latest_jpeg = encoded.tobytes()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()
