# AI Camera integration (hardware verification pending)

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
