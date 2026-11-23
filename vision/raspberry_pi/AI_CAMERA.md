# AI Camera integration (hardware verification pending)

## Preferred dual-camera design: IMX500 detection + D435i depth only

This mode runs the fire model on the IMX500 accelerator. D435i provides depth
points only at runtime; its RGB stream and the Pi CPU neural-network detector
are not used. Pi handles tensor decoding, coordinate transforms, preview,
projectile calculations and serial communication. Performance is not measured.

Two cameras cannot share pixel coordinates just by matching image resolution.
Rigidly mount them with overlapping fields of view, finish manual focus, then
calibrate their relative rotation/translation and the AI lens. Any later change
to their mounting or lens focus requires new calibration. The earlier `best.pt`
preview used CPU inference, not the AI accelerator; this mode requires a valid
fire-model RPK (packaging instructions below) with verified NMS layout.

The `dual_camera_calibration.py` tool captures static checkerboard image pairs
from both RGB cameras. For example, a board with 9x6 INNER corners and measured
25 mm squares (replace these values with the actual board):

```sh
.venv/bin/python vision/raspberry_pi/dual_camera_calibration.py --capture calibration_pairs --pairs 25
.venv/bin/python vision/raspberry_pi/dual_camera_calibration.py --calibrate calibration_pairs --columns 9 --rows 6 --square-mm 25 --output dual_camera_calibration.json
```

Show the board to both cameras at varied distances, positions and tilts. Hold it
still for each pair; capture is not hardware synchronized. The tool accepts at
least 15 detected pairs and rejects high stereo reprojection error. Check the
printed residuals and verify overlay at several measured target distances;
low calibration error alone is not proof of good depth/target association.
Preserve the calibration file and image pairs locally. No default identity
transform or guessed pixel scaling is supplied.

Calibration composes a measured D435i-RGB-to-AI transform with the SDK's factory
depth-to-RGB extrinsics. Runtime enables ONLY the D435i depth stream, deprojects
it through SDK pointcloud geometry, transforms to AI coordinates, and projects
with the calibrated AI lens distortion. A nearest-surface z-buffer handles
overlap; missing points stay invalid. IMX500 boxes pass through Picamera2's
`convert_inference_coords` for the actual sensor crop before sampling this map.
The file is tied to both camera IDs, AI crop/sensor mode, and 640x480 streams.

Preview example ONLY for a verified pixel-yxyx model:

```sh
.venv/bin/python vision/raspberry_pi/dual_fire_preview.py --imx500-model models/imx500/packaged/network.rpk --dual-calibration dual_camera_calibration.json --imx500-box-order yxyx --imx500-box-units pixels
```

Use the existing SSH tunnel/browser at localhost:8080. Unlike the earlier CPU
preview, `dual_fire_preview.py` refuses to start without an IMX500 RPK. Cyan
rectangles show depth sampling regions. Surface association, flame transparency,
camera mounting pitch/yaw and invalid-depth handling are described in the root
README. Mount with zero roll; mounting pitch/yaw options refer to the AI camera.

ESP32 mock integration uses the same calibrated source (stop the preview first):

```sh
.venv/bin/python main.py --port /dev/ttyUSB0 --dual-depth --imx500-model models/imx500/packaged/network.rpk --dual-calibration dual_camera_calibration.json --imx500-box-order yxyx --imx500-box-units pixels --mock-base --jet-speed-m-s 5 --nozzle-length-m 0.1 --base-height-m 0.2 --target-height-m 0.2
```

Jet speed/geometry here are mock examples. Missing inference is never treated as
no fire. Fresh captures are kept in latest-frame buffers, and software delivery
age/skew are bounded (200/100 ms). These limits are NOT exposure synchronization;
validate initially with stationary targets and a stopped rover. Timing under
motion needs separate validation. Missing pairs time out; detected fire without
valid projected depth stops the bridge and sends STOP. No CPU YOLO fallback occurs.
No calibration, accelerator inference, frame rate or dual-camera hardware operation
has been verified on this Windows workspace.

The integrated bridge supports IMX500 inference without running Ultralytics on
the Pi CPU. Projectile calculation and serial communication still run on the Pi;
FireTurret remains the main ESP32 controller. The old imx500_detection.py viewer
is not used by this backend and its dictionary-based output decoding is unverified.

## Package on the Raspberry Pi

With the camera connected to a Pi 5 running current Raspberry Pi OS:

```sh
sudo apt update
sudo apt install imx500-all imx500-tools python3-picamera2 python3-numpy python3-opencv python3-serial
sudo reboot
```

From the repository root on the Pi:

```sh
imx500-package -i models/imx500/packerOut.zip -o models/imx500/packaged
```

This should generate `models/imx500/packaged/network.rpk`. Packaging must be run
on Raspberry Pi OS. Source: https://www.raspberrypi.com/documentation/accessories/ai-camera.html

## Verify before operating actuators

Test the packaged fire network without ESP32 or RealSense first:

```sh
.venv/bin/python vision/raspberry_pi/imx500_fire_preview.py --inspect
```

This prints three bounded summaries (tensor shapes and sample values). Use them
with the export metadata to establish units/order/class. Then test browser boxes,
for example ONLY if pixel-yxyx output is verified:

```sh
.venv/bin/python vision/raspberry_pi/imx500_fire_preview.py --box-order yxyx --box-units pixels
```

Use the existing port-8080 SSH tunnel/browser. This viewer uses the IMX500
accelerator and never imports Ultralytics. It does not calculate distance.
Check box placement with known fire/no-fire images before dual-camera calibration.

The existing archive contains a 640x640 input and four outputs: boxes, scores,
classes and detection count. This is compatible in principle with the adapter's
NMS interface, but the archive does not prove that it matches best.pt or establish
coordinate ordering/units. Confirm the model export provenance, compare known
fire and no-fire examples, and inspect output values on the camera. The bridge
requires explicit box-order and box-unit options rather than guessing.

Example ONLY IF the export is confirmed to use pixel yxyx boxes:

```sh
python main.py --port /dev/ttyUSB0 --imx500-model models/imx500/packaged/network.rpk --imx500-box-order yxyx --imx500-box-units pixels --focal-length-px 490 --jet-speed-m-s 5
```

For normalized boxes use `--imx500-box-units normalized`; for xyxy ordering use
`--imx500-box-order xyxy`. Fire class defaults to 0; confirm it or use
`--fire-class`. Example focal length and jet speed require measurement.
Focal calibration must correspond to model-input pixel coordinates (640x640 for
this archive), its field of view and crop. No preview-coordinate conversion is
applied. The camera should be fixed relative to the calibrated geometry.

The adapter validates tensor lengths, detection counts, finite values and box
geometry. Missing inference metadata sends no observation, allowing the ESP32's
existing freshness timeout to apply; it is never interpreted as fire extinguished.
An invalid tensor raises an error and bridge cleanup attempts STOP.

Keep actuators disconnected and use default FireTurret demo mode for first tests.
No RPK generation, camera inference or hardware validation has been performed
on the Windows development machine.
