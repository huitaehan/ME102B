# Sensor-only demonstration

The sketch now defaults to `DEMO_MODE=true`. Upload
`FireTurret/FireTurret.ino` using Arduino IDE and use Serial Monitor at 115200 baud,
Newline. Keep the pump and motor driver physically disconnected for this test.
At startup verify `DEMO MODE - actuator outputs disabled` appears.

## What is real and what is simulated

- Real: start button GPIO23, home limit switch GPIO33, analog flame sensor GPIO34,
  debounce, analog filtering/thresholds, state transitions, and shooting timer.
- Simulated: stepper motion, back-off and aiming completion (one second), a fixed
  45-degree trajectory, and intended pump actions. Actuator GPIOs are not initialized or written in demo mode. Keep all actuator
  drivers physically disconnected; unconfigured pins do not guarantee an OFF level.
- Optional simulated camera: serial commands below generate observations every
  100 ms. The real Pi bridge can still send FIRE messages; receiving a valid FIRE
  message disables the simulated source. Use only one serial client at a time.
- Driving remains unimplemented. Entering Drive does not demonstrate real driving.

## Wire and calibrate first

Wire the normally-open start button from GPIO23 to GND, home switch NO/COM from
GPIO33 to GND, and the analog sensor output to GPIO34 with common ground. Confirm
the sensor supply specification and ensure its output never exceeds 3.3 V.
See README.md for wiring and analog calibration details.

The demo deliberately does NOT bypass the analog sensor. First observe
`FLAME,<average>,<ready>,<confirmed>`, measure flame/no-flame readings, set
FLAME_HIGH_MEANS_FIRE and the ON/OFF thresholds, then set FLAME_CALIBRATED=true
and upload again. The provided thresholds are examples. Jet calibration is
bypassed ONLY in demo mode. With an uncalibrated analog sensor the sequence waits
in Characterize; do not mistake this for a failure of the demo.

## Serial commands

| Command | Effect |
|---|---|
| SIM_NEAR | Repeated simulated camera fire at 0.8 m, confidence 0.9 |
| SIM_FAR | Repeated simulated camera fire at 3.0 m, confidence 0.9 |
| SIM_CLEAR | Repeated simulated camera frames with no fire |
| SIM_OFF | Stop simulated frames; camera data expires after 1.5 seconds |
| STOP | Enter Fault; reset the ESP32 to recover |

These camera settings persist until changed; they are not one-frame messages.
SIM_OFF does not send a no-fire frame. No command replaces the physical start
button, limit switch, or analog sensor. Serial START stays disabled by default.

## Suggested recording sequence

1. Start with SIM_CLEAR. Press the physical start button: Homing (1).
2. Press the home switch for at least 50 ms, then release promptly: Search (2).
   Back-off completes after one simulated second. Leaving the switch pressed
   causes a fault once back-off completes.
3. Release and press Start again to demonstrate Search -> Homing; trigger and
   release the home switch again.
4. Send SIM_NEAR with the analog sensor reporting no flame: Characterize (3)
   waits for infrared confirmation with pump off.
5. Send SIM_FAR and present a controlled flame to the analog sensor: Drive (4).
6. Send SIM_NEAR while maintaining infrared confirmation: Characterize (3),
   Calculate & aim (5), then Shoot (6). Read the printed simulated actions aloud.
7. During Shoot, removing infrared support alone does not interrupt the shot.
   After three seconds the state enters Fire check (7).
8. For a retry, leave SIM_NEAR active and restore/maintain analog confirmation:
   after settling the machine returns to Characterize and shoots again.
9. To finish, send SIM_CLEAR and remove the analog flame stimulus. After 500 ms
   settling and fresh clear observations spanning two seconds, it returns to Search.
   Sending SIM_CLEAR during Shoot stops the shot early via the camera condition;
   send it after Fire check begins to demonstrate the full three-second timer.
10. Send STOP to demonstrate Fault (8). Separately, reset and press Start without
    pressing Home for 15 seconds to demonstrate homing timeout.

Record the sensor actions and Serial Monitor together. State messages retain
`STATE,<number>` and add readable `DEMO STATE` / `DEMO ACTION` lines. Explain that
camera inputs and actuators are simulated; confirm instructor acceptance of
camera simulation. This is not a test of physical aiming, pumping, or detection.

## Return to hardware operation

Set DEMO_MODE=false and complete both analog and jet/mechanical calibration before
connecting actuators. SIM commands are ignored with demo mode off, real STEP/DIR
operation resumes, and the calibrated projectile calculation is required again.

## Diagnostic startup version

The complete sketch prints BOOT CHECK 1 through 5 before the demo banner.
Serial starts before input setup. If resets continue, capture the last checkpoint.
Motor/pump GPIOs are untouched in demo mode; keep those devices disconnected.
The example motor pins are STEP=25, DIR=27, ENABLE=22; GPIO26 is no longer used.
A latching start switch works between GPIO23 and GND: open it, then close it to
start. A closed switch at boot must first be opened. Opening it is not a stop or
power-off command. The complete diagnostic sketch includes setup(), loop(),
all state-machine functions, and all global variables; open/copy the entire file.
