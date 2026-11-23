"""Test fire RPK on IMX500 accelerator, without D435i, CPU YOLO or ESP32.

Use --inspect for tensor summaries or --raw-preview to see framing while the
model runs. Detection boxes require explicit box-order/units. All previews use
the same localhost:8080 SSH tunnel as earlier viewers.
"""

import argparse
import math
from http.server import ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imx500_backend import preview_boxes_from_outputs
from fire_distance_preview import PreviewHandler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=(
        Path(__file__).resolve().parents[2]/'models/imx500/packaged/network.rpk'))
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--inspect', action='store_true', help='Print three inference tensor summaries and exit')
    mode.add_argument('--raw-preview', action='store_true',
                      help='Show camera image and print tensor summaries without guessing box format')
    parser.add_argument('--box-order', choices=('xyxy', 'yxyx'))
    parser.add_argument('--box-units', choices=('normalized', 'pixels'))
    parser.add_argument('--fire-class', type=int, default=0)
    parser.add_argument('--confidence', type=float, default=0.40,
                        help='Preview threshold only; does not change ESP32/bridge thresholds')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error(f'RPK not found: {args.model}')
    if not math.isfinite(args.confidence) or not 0 <= args.confidence < 1:
        parser.error('confidence must be between 0 and 1')
    if not args.inspect and not args.raw_preview and not (args.box_order and args.box_units):
        parser.error('First use --inspect; preview requires --box-order and --box-units')

    import cv2
    import numpy as np
    from picamera2 import Picamera2
    from picamera2.devices.imx500 import IMX500

    print('Loading RPK onto IMX500 accelerator; first firmware load can take several minutes.', flush=True)
    imx = IMX500(str(args.model))
    camera = Picamera2(imx.camera_num)
    server = thread = None
    try:
        camera.configure(camera.create_video_configuration(
            main={'size': (640, 480), 'format': 'RGB888'}, queue=False))
        camera.start()
        print(f'Network input size: {imx.get_input_size()}', flush=True)
        if not args.inspect:
            server = ThreadingHTTPServer(('127.0.0.1', args.port), PreviewHandler)
            server.latest_jpeg = None
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            print(f'Preview: http://127.0.0.1:{args.port}; IMX500 inference, no distance yet.', flush=True)
        summaries = 0
        last_report = 0.0
        last_inference = time.monotonic()
        last_boxes = None
        last_decoded = 0.0
        while True:
            request = camera.capture_request()
            try:
                metadata = request.get_metadata()
                outputs = imx.get_outputs(metadata, add_batch=True)
                image = request.make_array('main').copy() if not args.inspect else None
                if outputs is not None:
                    last_inference = time.monotonic()
                elif not args.raw_preview and time.monotonic()-last_inference > 60:
                    raise RuntimeError('No inference metadata for 60 seconds; check RPK/camera output')
                if args.inspect or (args.raw_preview and summaries < 3):
                    if outputs is not None:
                        print(f'Inference sample {summaries+1}: {len(outputs)} tensors', flush=True)
                        for index, output in enumerate(outputs):
                            array = np.asarray(output)
                            print(f'  tensor[{index}] shape={array.shape} first_values='
                                  f'{array.reshape(-1)[:12].tolist()}', flush=True)
                        summaries += 1
                        if args.inspect and summaries >= 3:
                            return
                    if args.inspect:
                        continue
                boxes = [] if args.raw_preview else preview_boxes_from_outputs(
                    outputs, box_order=args.box_order, box_units=args.box_units,
                    input_size=imx.get_input_size(), fire_class=args.fire_class,
                    confidence_threshold=args.confidence,
                    convert_coords=lambda coords: imx.convert_inference_coords(coords, metadata, camera))
            finally:
                request.release()
            if args.raw_preview:
                status = ('Raw preview | inference received' if outputs is not None else
                          'Raw preview | waiting for inference')
            else:
                now = time.monotonic()
                if boxes is not None:
                    last_boxes, last_decoded = boxes, now
                # Briefly display recent results between inference frames, with
                # an explicit age. Never reuse boxes in control/serial outputs.
                display_boxes = last_boxes if now-last_decoded <= 0.5 else None
                status = ('Waiting for fresh inference' if display_boxes is None else
                          f'IMX500: {len(display_boxes)} | conf>{args.confidence:.2f} | age {now-last_decoded:.2f}s')
                for box in display_boxes or []:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(image, f'Fire {box.conf[0]:.2f}', (max(0, x1), max(45, y1)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(image, status, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2)
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
            server.latest_jpeg = jpeg.tobytes()
            if time.monotonic()-last_report >= 1:
                print(status, flush=True)
                last_report = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            camera.close()
        finally:
            if server:
                server.shutdown()
                server.server_close()
                thread.join()


if __name__ == '__main__':
    main()
