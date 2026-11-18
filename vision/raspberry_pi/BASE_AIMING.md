# Horizontal aiming with the drive base

The nozzle motor controls elevation. Horizontal aiming must rotate the rover.
The camera must be rigidly fixed to the chassis with a known yaw offset, an
unmirrored image, and a level horizontal image axis. Camera and nozzle should
share the forward direction; offsets/parallax need calibration at working range.

## Pi output

`vision/base_guidance.py` selects the closest estimated fire above confidence
0.40, then returns distance, confidence, image centre, and a relative base turn.
This selection is per frame, not persistent target tracking. Multiple similar
targets can switch priority. Distance uses the existing assumed-flame-height model.

```
yaw_error_deg = camera_yaw_deg + degrees(atan((image_width / 2 - cx) / fx))
```

Positive means turn left; negative means turn right. Inside the configurable
alignment deadband (default 3 degrees), the suggested turn is zero. `fx` is the
horizontal focal length in original capture pixels. It defaults to the vertical
focal length under the square-pixel assumption; calibrate it separately if needed.
Changing YOLO inference size does not change capture-frame coordinates.

The browser viewer highlights the selected target and displays the suggested turn:

```sh
python vision/raspberry_pi/fire_distance_preview.py --flame-height-cm 5 --focal-length-px 490 --horizontal-focal-px 490 --imgsz 640
```

Terminal diagnostics are:

```
BASE_GUIDANCE,1,-18.083,-18.083
BASE_GUIDANCE,1,1.000,0.000
BASE_GUIDANCE,0,0,0
```

Fields are detection-valid, raw error degrees, suggested turn degrees. The last
line means no target, not aligned. The current ESP32 does not accept this as a
command. It is deliberately printed only, not sent to actuators.

The integrated bridge can calculate the same values with `--base-guidance`:

```sh
python main.py --port /dev/ttyUSB0 --picamera --focal-length-px 490 --base-guidance
```

That flag selects the closest fire for both FIRE distance and steering diagnostics.
It does not turn the base or add horizontal alignment checks to the firmware.
Use demo mode while validating. Only one program can own the camera/serial port;
stop the preview before starting the bridge. IMX500 tensor inference does not yet
support this guidance flag; CPU inference is supported.

An explicit `--mock-base` mode now sends bearing and distance together to the
ESP32 demo controller and simulates base alignment before the real Pi projectile
calculation. See [the motor-free integration guide](../../firmware/esp32/MOCK_ROVER.md).
The standalone `mock_rover_demo.py` supplies a virtual scene without a camera.

## Existing search planner and missing execution

`Search and Drive/Search.py` sends `YAW` headings relative to the last `AIM_SET`.
`fire_yaw_command(current_yaw_deg, relative_turn_deg)` converts the measured
current heading plus the Pi's relative turn to that command frame. For example,
heading +45 degrees and fire correction -15 degrees produces `YAW,30.000`.
Sending `YAW,-15` directly in that example would be wrong.

`FireTurret.ino` still rejects physical YAW/DRIVE and has an empty `serviceDrive()`;
its separate `serviceMockBase()` is an explicit DEMO_MODE simulation only.
The owner confirmed two JGA25-370 motors, one DRV8871 per wheel, and wheel encoders
not yet wired to ESP32. Wheels have 80 mm nominal diameter. The turret uses
DRV8825. Pins/polarity, encoder electrical interface/counts, and track width remain
unknown. The mock uses clearly marked example track width and encoder resolution.
The vision angle cannot determine wheel PWM, wheel travel, or turn duration by
itself. For a differential/skid-steer base, wheel commands must be implemented
and calibrated on ESP32 with heading feedback or visual feedback.

The eventual ESP32 alignment service should turn with pump off, accept fresh
observations, slow/stop inside a deadband, and require stable alignment before
requesting the projectile solution. Stale/lost observations must stop the base;
STOP/FAULT must stop it too. Keep the base stopped while aiming/shooting.
Reobserve after turning rather than repeatedly adding the same relative error.
Those actuator services and interlocks are not implemented by this Pi change.
