# ESP32 fire turret

Open `FireTurret/FireTurret.ino` in Arduino IDE. Install the Espressif ESP32
board package and select your actual ESP32 board and USB port. No extra Arduino
libraries are used. This is configurable firmware, not hardware-validated code.

## Example wiring (classic ESP32 DevKit)

| Signal | GPIO | Connection |
|---|---:|---|
| STEP | 25 | STEP/DIR motor driver's STEP input |
| DIR | 26 | Driver DIR input |
| Enable | 27 | Driver active-low enable |
| Home switch | 33 | Switch to GND, internal pull-up |
| Pump control | 32 | Active-high MOSFET/relay driver input |

Use a current-limited stepper driver configured for the motor and microstepping,
a suitable motor supply, and a pump driver with inductive protection. GPIO cannot
power either motor or pump. Verify common grounding and 3.3 V compatible inputs.
Provide a hardware pull-down for the active-high pump control so it stays off
while the ESP32 resets. Motor acceleration is not implemented; use a conservative
step interval and verify no steps are lost under load.

## Before automatic operation

Set actual pin assignments, pump polarity, HOME_DIR_HIGH, STEPS_PER_DEGREE
(including microstepping/gearing), and mechanical MIN_ANGLE/MAX_ANGLE. The home
switch corresponds to 90 degrees above horizontal. Initial homing moves toward
that switch, times out after 15 seconds, then backs off to MAX_ANGLE. If already
pressed at startup, send START; otherwise a debounced switch press starts homing.
There is no separate power/start button in this hardware description.

Measure jet exit speed and set JET_SPEED_M_S, TARGET_HEIGHT_M, usable range,
and CALIBRATED=true only after controlled calibration. The trajectory calculation
assumes horizontal distance and an ideal projectile; camera slant range, water
breakup, variable pressure, and drag introduce error. No temperature is inferred.
The defaults deliberately fault before firing because jet speed is unknown.
Ensure horizontal alignment yourself: only nozzle elevation is actuated here.
The camera should remain fixed during aiming and observe the same target; the
bridge chooses the highest-confidence detection and does not track target identity.
Use a single-target scene. Distance assumes a known flame height; it is an estimate.

## Camera bridge

The existing scripts only display detections, so the new bridge sends one complete
observation per frame over USB serial. Run from `/workspace/ME102B` (or your local
repository root) on the machine with the camera and ESP32 physically connected:

```sh
python -m pip install ultralytics opencv-python pyserial
python vision/fire_serial_bridge.py --port /dev/ttyUSB0 --focal-length-px 600 --start
```

For Raspberry Pi with Picamera2 installed through the Pi OS packages:

```sh
python vision/fire_serial_bridge.py --port /dev/ttyUSB0 --picamera --focal-length-px 490 --start
```

This bridge uses `models/best.pt` with Ultralytics, including on Pi; it does not use the
IMX500 accelerator. The existing IMX500 script expects `fire_network.rpk`, which
is not checked into the repository as a standalone file, and its tensor decoding
needs verification on the actual camera. Keep calibration matched to actual frame
resolution: webcam path requests 640x480; Picamera2 path uses 640x640. Substitute
your actual serial port, flame height, focal calibration, and model as needed.
Close Arduino Serial Monitor before running the bridge. STOP or Ctrl+C turns off
the pump and latches FAULT; reset and home again to recover.

## State flow and limitations

WAIT_START -> HOMING -> SEARCH -> CHARACTERIZE -> CALCULATE_AIM -> SHOOT
-> FIRE_CHECK. Out-of-range targets enter DRIVE, which intentionally does nothing
until a drive system is added; only an updated in-range observation exits it.
SHOOT runs for at most 3000 ms, then stops the pump. Loss of a valid target can
stop it earlier. FIRE_CHECK waits 500 ms for settling, then uses new frames:
fire present returns to CHARACTERIZE; no detections for 2 seconds returns to
SEARCH. A camera timeout cannot count as extinguished. Search uses incoming
camera frames; horizontal scanning and rover movement are not implemented.
No vertical scanning is implemented because changing the camera/nozzle geometry
requires a defined mounting/calibration plan. Event checker and service functions
are explicitly labeled in the sketch for Deliverable 4.

Validation in the cloud can check logic and syntax, but flashing, switch direction,
stepper travel, camera inference, distance calibration, and pumping require the
physical hardware. Take the requested full-code screenshots in Arduino IDE after
configuring and compiling for your actual board.
