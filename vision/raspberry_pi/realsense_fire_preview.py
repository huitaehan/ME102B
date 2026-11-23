"""Headless D435i RGB YOLO preview with aligned surface depth and base guidance."""

import argparse
from http.server import ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from realsense_backend import RealSenseCamera, depth_scene
from fire_distance_preview import PreviewHandler


def main(require_dual=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=Path(__file__).resolve().parents[2]/'models/best.pt')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--fire-class', type=int, default=0)
    parser.add_argument('--depth-anchor', choices=('center', 'bottom'), default='bottom')
    parser.add_argument('--camera-yaw-deg', type=float, default=0.0)
    parser.add_argument('--camera-pitch-deg', type=float, default=0.0)
    parser.add_argument('--list-devices', action='store_true')
    parser.add_argument('--imx500-model', type=Path,
                        help='Use AI Camera accelerator for detection and D435i depth only')
    parser.add_argument('--dual-calibration', type=Path)
    parser.add_argument('--imx500-box-order', choices=('xyxy', 'yxyx'))
    parser.add_argument('--imx500-box-units', choices=('normalized', 'pixels'))
    args = parser.parse_args()
    if require_dual and not args.imx500_model and not args.list_devices:
        parser.error('Dual preview requires --imx500-model; CPU detection is not used')
    if args.list_devices:
        import pyrealsense2 as rs
        devices = rs.context().query_devices()
        if len(devices) == 0:
            raise RuntimeError('No RealSense device found; check USB cable, power and udev permissions')
        for device in devices:
            print(device.get_info(rs.camera_info.name), device.get_info(rs.camera_info.serial_number))
        return
    if args.imx500_model and not all((args.dual_calibration, args.imx500_box_order, args.imx500_box_units)):
        parser.error('IMX500 mode requires dual-calibration and verified box order/units')
    if not args.imx500_model and not args.model.is_file():
        parser.error(f'Model not found: {args.model}')
    if args.imgsz <= 0 or args.imgsz % 32:
        parser.error('imgsz must be a positive multiple of 32')
    # Validate geometry arguments before opening camera/server.
    depth_scene([], [[0]], lambda p, z: (0, 0, z),
                camera_yaw_deg=args.camera_yaw_deg, camera_pitch_deg=args.camera_pitch_deg)
    import cv2
    model = None
    if not args.imx500_model:
        from ultralytics import YOLO
        model = YOLO(str(args.model))
    server = ThreadingHTTPServer(('127.0.0.1', args.port), PreviewHandler)
    server.latest_jpeg = None
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    camera = None
    try:
        if args.imx500_model:
            from dual_camera_backend import DualCamera
            camera = DualCamera(
                args.imx500_model, args.dual_calibration,
                box_order=args.imx500_box_order, box_units=args.imx500_box_units,
                fire_class=args.fire_class, anchor=args.depth_anchor,
                camera_yaw_deg=args.camera_yaw_deg, camera_pitch_deg=args.camera_pitch_deg)
        else:
            camera = RealSenseCamera()
        print(f'Preview http://127.0.0.1:{args.port}; depth ROI measures surfaces, not flame gas.', flush=True)
        last_report = 0.0
        while True:
            if args.imx500_model:
                image, scene = camera.capture()
            else:
                image, depth, deproject = camera.capture()
                result = model(image, device='cpu', imgsz=args.imgsz, conf=0.40,
                               classes=[args.fire_class], verbose=False)[0]
                scene = depth_scene(result.boxes, depth, deproject, fire_class=args.fire_class,
                                    anchor=args.depth_anchor, camera_yaw_deg=args.camera_yaw_deg,
                                    camera_pitch_deg=args.camera_pitch_deg)
            reports = []
            for i, detection in enumerate(scene.detections):
                x1, y1, x2, y2 = map(int, detection.box)
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                l, t, r, b = detection.roi
                cv2.rectangle(image, (l, t), (r, b), (255, 255, 0), 2)
                if detection.guidance is None:
                    label = 'DEPTH INVALID: ' + detection.reason
                else:
                    label = (f'Surface Z {detection.depth_m:.2f} m | '
                             f'Horizontal {detection.guidance.distance_m:.2f} m')
                reports.append(label)
                cv2.putText(image, label, (8, 24+24*i), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 2)
            g = scene.guidance
            status = 'No fire detected' if not scene.detections else 'Range unavailable'
            if g:
                status = f'Closest measured surface: {g.distance_m:.2f} m | base turn {g.turn_deg:+.1f} deg'
                cv2.circle(image, (int(g.center_x), int(g.center_y)), 8, (255, 255, 0), 2)
            cv2.putText(image, status, (8, 460), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 2)
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError('Preview JPEG encoding failed')
            server.latest_jpeg = jpeg.tobytes()
            if time.monotonic()-last_report >= 1:
                print(status + (' | ' + '; '.join(reports) if reports else ''), flush=True)
                last_report = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if camera:
                camera.stop()
        finally:
            server.shutdown()
            server.server_close()
            thread.join()


if __name__ == '__main__':
    main()
