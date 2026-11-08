# ESP32 fire turret

Open `FireTurret/FireTurret.ino` in Arduino IDE. Install the Espressif ESP32
board package and select your actual ESP32 board and USB port. No extra Arduino
libraries are used. This is configurable firmware, not hardware-validated code.

## Sensor-only demo

`DEMO_MODE=true` is the current default. It disables actuator outputs and simulates
trajectory/aiming while preserving real switch and analog sensor checks. Follow
[DEMO.md](DEMO.md) for setup and recording instructions. The state diagram describes
the normal control flow; simulated camera/motion events are explicitly labeled in
the demo. Real actuator behavior below requires `DEMO_MODE=false` and calibration.

## Example wiring (classic ESP32 DevKit)

| Signal | GPIO | Connection |
|---|---:|---|
| STEP | 25 | STEP/DIR motor driver's STEP input |
| DIR | 27 | Driver DIR input |
| Enable | 22 | Driver active-low enable |
| Start button | 23 | Normally-open button to GND, internal pull-up |
| Home switch | 33 | Switch to GND, internal pull-up |
| Flame analog output | 34 | Sensor AO/S signal, ADC1 input; maximum 3.3 V |
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
that switch, times out after 15 seconds, then backs off to MAX_ANGLE.
Press the separate start button on GPIO23 to begin homing. The home limit switch
on GPIO33 only establishes the 90-degree position; it never starts the machine.
A start button held during boot must be released and pressed again. The limit
switch already being pressed when homing begins establishes home immediately.
Serial START is disabled by default; ALLOW_SERIAL_START can enable it for bench
tests. Home is an absolute 90-degree elevation, not a relative 90-degree move.

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
python vision/fire_serial_bridge.py --port /dev/ttyUSB0 --focal-length-px 600
```

For Raspberry Pi with Picamera2 installed through the Pi OS packages:

```sh
python vision/fire_serial_bridge.py --port /dev/ttyUSB0 --picamera --focal-length-px 490
```

This bridge uses `models/best.pt` with Ultralytics, including on Pi; it does not use the
IMX500 accelerator. The existing IMX500 script expects `fire_network.rpk`, which
is not checked into the repository as a standalone file, and its tensor decoding
needs verification on the actual camera. Keep calibration matched to actual frame
resolution: webcam path requests 640x480; Picamera2 path uses 640x640. Substitute
your actual serial port, flame height, focal calibration, and model as needed.
Close Arduino Serial Monitor before running the bridge. STOP or Ctrl+C turns off
the pump and latches FAULT; reset and home again to recover.

## State descriptions

Event checkers detect input changes, camera observations, timing, and motion
completion. Service functions control outputs, calculate aiming, and run state
logic. Both are explicitly labeled with comments in the Arduino sketch.

| Number | State | Services / actions | Event conditions and next state |
|---|---|---|---|
| 0 | Wait for start | Keep pump off; monitor separate start button on GPIO23. | A new debounced button press enters Homing. A button held at boot must first be released. The home switch cannot start the machine. |
| 1 | Homing | Move toward the limit switch at 90 degrees above ground. When reached, record zero steps as the 90-degree reference. | Debounced limit-switch contact enters Search and commands back-off to 85 degrees. Failure to reach the switch within 15 seconds enters Fault. |
| 2 | Search | Complete switch back-off; monitor fresh YOLO observations. No physical scanning is implemented. | After back-off, fresh fire enters Characterize. A switch still pressed after back-off enters Fault. A new start-button press stops motion and re-enters Homing, including during back-off. Otherwise remain in Search. |
| 3 | Characterize | Use camera distance and filtered analog infrared readings; no temperature measurement is available. | Only camera fire supported by calibrated infrared detection advances: within 0.10–2.00 m enters Calculate & aim; outside that range enters Drive. Otherwise remain here to recheck with pump off. Missing or stale fire returns to Search. |
| 4 | Drive | Keep pump off. Drive service is a placeholder and commands no rover movement. | A fresh in-range target returns to Characterize. Missing or stale fire returns to Search. Otherwise remain here. |
| 5 | Calculate & aim | Calculate the low-angle trajectory and command elevation stepper movement. | Completed commanded motion enters Shoot and turns on the pump. Missing calibration or invalid trajectory enters Fault. Target loss returns to Search; lost infrared support returns to Characterize; out-of-range fire enters Drive. |
| 6 | Shoot | Pump on for at most three seconds. | Three-second timer expiration enters Fire check. Missing, stale, or out-of-range camera fire can enter Fire check earlier. Infrared loss does not interrupt an active shot. Every exit turns the pump off. |
| 7 | Fire check | Pump off; settle for 500 ms, then evaluate new camera frames. | Fresh fire returns to Characterize. Fresh no-fire frames and clear infrared readings spanning two seconds return to Search. Invalid/stale infrared data or stale camera data enters Fault. Infrared-only detection keeps checking with the pump off. |
| 8 | Fault | Pump off; disable stepper driver. | Reset the ESP32 to return to Wait for start, then press the button to home again. |

A serial `STOP` command enters Fault from any state. Serial `START` is ignored
unless `ALLOW_SERIAL_START=true` is explicitly configured for bench testing.
The bridge's optional `--start` flag is only for that bench-test configuration;
normal runs omit it. An unexpected raw limit-switch contact while moving toward
home outside Homing also enters Fault. Camera observations become stale after
1.5 seconds. Motion completion is based on commanded steps, not encoder feedback.

## State flow and limitations

WAIT_START -> HOMING -> SEARCH -> CHARACTERIZE -> CALCULATE_AIM -> SHOOT
-> FIRE_CHECK. Out-of-range targets enter DRIVE, which intentionally does nothing
until a drive system is added; only an updated in-range observation exits it.
SHOOT runs for at most 3000 ms, then stops the pump. Loss of a valid target can
stop it earlier. FIRE_CHECK waits 500 ms for settling, then uses new frames:
camera fire returns to CHARACTERIZE; both sensors clear for 2 seconds returns to
SEARCH. A camera timeout cannot count as extinguished. Search uses incoming
camera frames; horizontal scanning and rover movement are not implemented.
No vertical scanning is implemented because changing the camera/nozzle geometry
requires a defined mounting/calibration plan. Event checker and service functions
are explicitly labeled in the sketch for Deliverable 4.

Validation in the cloud can check logic and syntax, but flashing, switch direction,
stepper travel, camera inference, distance calibration, and pumping require the
physical hardware. Take the requested full-code screenshots in Arduino IDE after
configuring and compiling for your actual board.

## Re-homing and physical power

A new debounced start-button press in Search restarts Homing. Holding the button
through the first homing sequence does not trigger another cycle. Button presses
in other active states are ignored and are not queued for later. Serial START
remains a bench-test option in Wait for start only.

Physical power removal is not a software state. A master switch must remove power
from the relevant ESP32, pump, and motor supplies. Powering off only the ESP32 is
not equivalent to disconnecting the actuator supplies. When power is restored,
firmware initializes in Wait for start and requires a new button press and homing.
Hardware must hold the pump control inactive while the ESP32 is unpowered/reset.
See [the updated state diagram](state-diagram.md).

## Analog flame sensor setup

Use the module's analog signal output, not a digital threshold output, on GPIO34
(ADC1 on the classic ESP32). Connect sensor GND to ESP32 GND. Confirm your exact
DFRobot module's supply range and output specifications before powering it. If
3.3 V operation is supported, use it only when the output is also specified within
ESP32 input limits. A 5 V analog output needs a suitable divider or level interface;
never connect it directly to an ESP32 ADC. Do not assume a disconnected analog
input is detectable: it can float. Keep the sensor connected and verify its response.

Mount the sensor toward the camera target and validate its distance/viewing angle.
It senses infrared intensity, not temperature or definitive proof of fire. Sunlight,
lighting, water, and alignment can affect readings. Camera and IR disagreement
holds Characterize with the pump off rather than declaring the fire extinguished.
No new numbered state was added. Driving and scanning remain unimplemented.

With pump and motor driver disconnected, upload the sketch and open Serial Monitor
at 115200 baud. Every 500 ms it prints:

```text
FLAME,<average ADC count>,<ready 0/1>,<confirmed 0/1>
```

The ESP32 averages eight samples taken 20 ms apart (12-bit counts, 0–4095).
Observe readings with no flame, normal room/sunlight conditions, and a controlled
flame at the intended working distances. Do not use an arbitrary default threshold.
Set FLAME_HIGH_MEANS_FIRE to match whether readings rise or fall near flame.
Choose FLAME_ON_THRESHOLD and FLAME_OFF_THRESHOLD with a gap to prevent chatter:
for rising signals ON must be greater than OFF; for falling signals ON must be
less than OFF. Defaults 2400/2000 are examples only. After measuring, set
FLAME_CALIBRATED=true and upload again. Jet calibration CALIBRATED is separate
and must also be completed before shooting. Confirmation is not available until
all eight samples are collected; stale/invalid calibration cannot authorize pumping.

In Fire check, infrared detection resets the no-fire timer even between camera
frames. Camera fire returns to Characterize; infrared-only detection keeps the
pump off in Fire check. Only fresh no-fire frames spanning two seconds while
infrared remains clear return to Search. The Pi protocol FIRE messages are unchanged;
the analog sensor is read locally by the ESP32, and FLAME reports are sent back.

Infrared confirmation is required before shooting, but is ignored during the
active Shoot state because water may disturb the reading. The three-second timer,
camera conditions, and STOP still apply. Infrared is evaluated again in Fire check
after the settling interval.
