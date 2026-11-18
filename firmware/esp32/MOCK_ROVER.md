# Motor-free rover integration test

Upload `FireTurret/FireTurret.ino` with `DEMO_MODE=true`. This is the integrated
controller, not SensorDemo. No wheel driver or encoder connection is needed.
The mock assigns no wheel GPIO, and DEMO_MODE prevents stepper/pump output writes.
Keep actuator drivers disconnected. No physical drive implementation is included.

Explicit `MOCK_START` enables simulated homing and IR confirmation. Without this
command, the existing demo still uses its real switches and flame sensor. The
mock does not verify real sensors, mechanics, motor wiring, or successful shooting.
`MOCK_START` is rejected outside DEMO_MODE and after leaving Wait for start.
Reset ESP32 before a fresh mock session; STOP latches Fault as usual.

## Scripted complete test (no camera needed)

Close Serial Monitor and stop other programs using the ESP32 serial port. On Pi:

```sh
cd ~/ME102B
.venv/bin/python vision/mock_rover_demo.py --port /dev/ttyUSB0
```

Use the actual port from `python -m serial.tools.list_ports`. On Windows, use the
local Python environment and `--port COM5` (or the actual COM port).

The script checks `MOCK_BASE_V1`, then starts an explicit mock session. It supplies
a virtual target initially 20 degrees left and 0.8 metres away. It uses simulated
heading reports to update the virtual camera bearing as the rover turns. The
ESP32 must settle within 3 degrees for at least 500 ms over multiple fresh frames,
then request the real Python projectile solver. Solver calibration here is
illustrative (5 m/s, 0.1 m nozzle, pivot/target height 0.2 m). A simulated nozzle
move and three-second simulated shot follow, then the script clears the target.
Success prints:

```
PASS: align -> Pi projectile solve -> simulated shot -> fire check -> search
```

Test the opposite direction with `--yaw-deg -20`. The script sends STOP on exit.
If you see `ERR,MOCK_START,NOT_ALLOWED`, reset ESP32 and rerun.

## Live camera observations

Stop the browser preview before starting this bridge:

```sh
.venv/bin/python main.py --port /dev/ttyUSB0 --picamera --focal-length-px 490 --jet-speed-m-s 5 --nozzle-length-m 0.1 --base-height-m 0.2 --target-height-m 0.2 --mock-base
```

This sends the closest target's bearing and distance atomically. Camera readings
are real; all actuators, homing and IR confirmation are simulated. A stationary
camera does not turn when the simulation turns. For live tests, manually rotate
the unpowered chassis/camera toward the target, or use the scripted virtual scene
above to demonstrate a complete turn without moving anything. The browser viewer
is a separate diagnostic and does not send TARGET messages.

## Mock geometry and messages

The wheel's nominal diameter is 80 mm. **200 mm track width and 360 counts per
wheel revolution are simulation placeholders, not measured JGA25-370 values.**
For an in-place left turn, left-wheel travel is negative and right-wheel travel
positive. If wheel-centre spacing is `b` and turn is `theta` radians:

```
right travel = b * theta / 2
left travel  = -b * theta / 2
counts      = travel / (pi * 0.080) * counts_per_wheel_revolution
```

The mock integrates a bounded angular rate at each loop. It never writes PWM.
Example with placeholder 0.2 m track and 360 counts/rev: a 90-degree turn would
require -0.1571/+0.1571 m wheel travel, or -225/+225 counts.

115200 baud, newline-delimited:

```
Pi -> ESP32: CAPS
ESP32 -> Pi: CAPS,MOCK_BASE_V1,TARGET,MOCK_START
Pi -> ESP32: MOCK_START
ESP32 -> Pi: MOCK_READY,...
Pi -> ESP32: TARGET,1,1,0.8000,0.9500,20.000
ESP32 -> Pi: BASE_MOCK,heading_deg,left_counts,right_counts,left_m_s,right_m_s
ESP32 -> Pi: AIM_REQUEST,request_id,distance_m,min_angle_deg,max_angle_deg
Pi -> ESP32: AIM_RESULT,request_id,elevation_deg
```

TARGET fields: increasing sequence, detected, distance metres, confidence,
relative bearing degrees (left positive). No target is `TARGET,2,0,0,0,0`.
Old/invalid observations are rejected without refreshing freshness. Legacy FIRE
messages are rejected during a mock session to prevent bypassing alignment.
State 9 is mock base alignment; previous state numbers 0 through 8 are unchanged.

Lost/stale observations stop simulated wheel motion and return to Search.
An alignment timeout, solver timeout/error, or STOP enters Fault. Fresh off-centre
observations cancel elevation aiming or interrupt a simulated shot. Physical
base turning still needs driver wiring, encoder decoding/calibration, and these
interlocks integrated with the real motor service.

## Validation

Python protocol and solver tests: `python -m unittest discover -s vision -p 'test_*.py'`.
The C++ host harness includes the actual firmware and checks turn direction,
80 mm wheel geometry, alignment, aiming handoff, stale/old input, timeouts, STOP,
and zero output pin setup/writes. On a machine with g++:

```sh
g++ -std=c++11 -I firmware/esp32/tests firmware/esp32/tests/mock_base_test.cpp -o mock_base_test
./mock_base_test
```

The test header is outside the sketch folder and is not used by Arduino builds.
Compilation for the actual ESP32 and USB integration still require your board.
