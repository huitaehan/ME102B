"""Capture static checkerboard pairs, then calibrate D435i-depth -> AI-camera XYZ.

Capture AFTER both cameras are rigidly mounted. No fire RPK is needed to calibrate.
"""

import argparse
import json
from pathlib import Path
import time

import numpy as np


def capture_pairs(directory, pairs, interval):
    import cv2
    import pyrealsense2 as rs
    from picamera2 import Picamera2

    directory.mkdir(parents=True, exist_ok=True)
    if any(directory.iterdir()):
        raise ValueError('Use a new empty capture directory to avoid mixing calibrations')
    info = next((c for c in Picamera2.global_camera_info() if c['Model'] == 'imx500'), None)
    if info is None:
        raise RuntimeError('AI Camera not found')
    ai = Picamera2(info['Num'])
    pipeline = rs.pipeline()
    rs_started = False
    try:
        ai.configure(ai.create_video_configuration(
            main={'size': (640, 480), 'format': 'RGB888'}, queue=False))
        ai.start()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        profile = pipeline.start(config)
        rs_started = True
        extrinsics = profile.get_stream(rs.stream.depth).get_extrinsics_to(profile.get_stream(rs.stream.color))
        metadata = dict(
            ai_size=[640, 480], depth_size=[640, 480], ai_camera_id=info['Id'],
            realsense_serial=profile.get_device().get_info(rs.camera_info.serial_number),
            R_depth_to_color=np.array(extrinsics.rotation).reshape(3, 3, order='F').tolist(),
            t_depth_to_color_m=list(extrinsics.translation))
        print('Show a checkerboard to BOTH cameras. Change board position/tilt between pairs.', flush=True)
        print('Hold it stationary for each capture; cameras are not hardware synchronized.', flush=True)
        for index in range(pairs):
            print(f'Pair {index+1}/{pairs}: hold board still in {interval:g} seconds', flush=True)
            time.sleep(interval)
            request = ai.capture_request()
            try:
                image_ai = request.make_array('main').copy()
                crop = list(request.get_metadata()['ScalerCrop'])
            finally:
                request.release()
            image_rs = np.asanyarray(pipeline.wait_for_frames(3000).get_color_frame().get_data()).copy()
            if index == 0:
                metadata['ai_scaler_crop'] = crop
                metadata['ai_raw_size'] = list(ai.camera_configuration()['raw']['size'])
            elif crop != metadata['ai_scaler_crop']:
                raise RuntimeError('AI crop changed during calibration capture')
            for label, image in (('ai', image_ai), ('rs', image_rs)):
                if not cv2.imwrite(str(directory/f'{label}_{index:03d}.png'), image):
                    raise RuntimeError('Could not save calibration image')
        (directory/'capture.json').write_text(json.dumps(metadata, indent=2))
        print(f'Saved {pairs} pairs to {directory}. Next run --calibrate.', flush=True)
    finally:
        if rs_started:
            pipeline.stop()
        ai.close()


def calibrate(directory, output, columns, rows, square_mm):
    import cv2
    metadata = json.loads((directory/'capture.json').read_text())
    pattern = (columns, rows)
    grid = np.zeros((columns*rows, 3), np.float32)
    grid[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)*(square_mm/1000)
    objects, ai_corners, rs_corners = [], [], []
    for ai_path in sorted(directory.glob('ai_*.png')):
        rs_path = directory/ai_path.name.replace('ai_', 'rs_', 1)
        ai_image = cv2.imread(str(ai_path), cv2.IMREAD_GRAYSCALE)
        rs_image = cv2.imread(str(rs_path), cv2.IMREAD_GRAYSCALE)
        if ai_image is None or rs_image is None or ai_image.shape != (480, 640) or rs_image.shape != (480, 640):
            raise ValueError('Missing pair or incorrect image size')
        ok_ai, corners_ai = cv2.findChessboardCornersSB(ai_image, pattern)
        ok_rs, corners_rs = cv2.findChessboardCornersSB(rs_image, pattern)
        if ok_ai and ok_rs:
            # An asymmetric checkerboard layout/orientation marker helps avoid
            # corner-order ambiguities. Inspect pairs if residual is high.
            objects.append(grid.copy())
            ai_corners.append(corners_ai)
            rs_corners.append(corners_rs)
    if len(objects) < 15:
        raise ValueError(f'Only {len(objects)} usable pairs; collect at least 15 with varied board positions/tilts')
    ai_rms, kai, dai, _, _ = cv2.calibrateCamera(objects, ai_corners, (640, 480), None, None)
    rs_rms, krs, drs, _, _ = cv2.calibrateCamera(objects, rs_corners, (640, 480), None, None)
    rms, _, _, _, _, rotation, translation, _, _ = cv2.stereoCalibrate(
        objects, rs_corners, ai_corners, krs, drs, kai, dai, (640, 480),
        flags=cv2.CALIB_FIX_INTRINSIC)
    if not np.isfinite(rms) or rms > 2.0:
        raise ValueError(f'Stereo RMS {rms:.3f} pixels too high; inspect corner order, board movement and capture coverage')
    rdc = np.asarray(metadata.pop('R_depth_to_color'))
    tdc = np.asarray(metadata.pop('t_depth_to_color_m'))
    data = dict(metadata, version=1, ai_K=kai.tolist(), ai_distortion=dai.reshape(-1).tolist(),
                R_depth_to_ai=(rotation@rdc).tolist(),
                t_depth_to_ai_m=(rotation@tdc+translation.reshape(3)).tolist(),
                calibration_rms_px=float(rms), pairs=len(objects), square_mm=square_mm)
    if output.exists():
        raise ValueError('Output exists; choose a new filename to preserve the previous calibration')
    output.write_text(json.dumps(data, indent=2))
    print(f'Saved {output}; AI RMS {ai_rms:.3f}, RGB RMS {rs_rms:.3f}, stereo RMS {rms:.3f} px')
    print('Verify projected surface depth at several known distances before aiming.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--capture', type=Path)
    mode.add_argument('--calibrate', type=Path)
    parser.add_argument('--pairs', type=int, default=25)
    parser.add_argument('--interval', type=float, default=3.0)
    parser.add_argument('--columns', type=int, default=9, help='Checkerboard INNER corners across')
    parser.add_argument('--rows', type=int, default=6, help='Checkerboard INNER corners down')
    parser.add_argument('--square-mm', type=float, default=25.0, help='Measured checkerboard square size')
    parser.add_argument('--output', type=Path, default=Path('dual_camera_calibration.json'))
    args = parser.parse_args()
    if (args.pairs < 15 or not np.isfinite(args.interval) or args.interval <= 0
            or args.columns < 3 or args.rows < 3 or not np.isfinite(args.square_mm) or args.square_mm <= 0):
        parser.error('Need >=15 pairs, positive interval/square size and >=3x3 inner corners')
    if args.capture:
        capture_pairs(args.capture, args.pairs, args.interval)
    else:
        calibrate(args.calibrate, args.output, args.columns, args.rows, args.square_mm)


if __name__ == '__main__':
    main()
