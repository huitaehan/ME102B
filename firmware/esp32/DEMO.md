# Input-only demo (use this for the current sensor test)

Open `SensorDemo/SensorDemo.ino`, not `FireTurret/FireTurret.ino`.
This separate sketch assigns only START_PIN=21, HOME_PIN=33, FLAME_PIN=34.
It contains no motor/pump pin assignments and no digital output writes.
GPIO21 is the confirmed MI/MISO start-switch connection on the Adafruit ESP32
Feather V2. The older setup used GPIO23. Physical operation remains unverified.

Wire start switch GPIO21 to GND, limit switch GPIO33 to GND (NO/COM), and a
verified 0–3.3 V analog sensor output to GPIO34 with common ground. Keep devices
unplugged for the initial boot check. Use Serial Monitor at 115200 baud, Newline.
Opening then closing the latching start switch requests Homing. Opening it does
not stop operation or remove power. Trigger/release Home to reach Search.

Every 500 ms STATUS shows the numbered/named state, start ON/OFF, home
PRESSED/RELEASED, analog average, calibration readiness, and infrared confirmation.
SWITCH CHANGE reports debounced switch edges; STATE CHANGE reports the previous
and new states alongside both switch positions. Existing STATE,<number> lines remain. Calibrate the real analog sensor as described
below before expecting Characterize to advance. SIM_NEAR/FAR/CLEAR/OFF optionally
simulate camera frames; aiming and pumping are printed simulations only.
The full state-machine sequence and timers follow the guide below; actuator GPIO
configuration and normal-mode instructions apply only to the older FireTurret sketch.
SensorDemo is always a simulation; it has no hardware-actuation mode.

---

# Sensor-only demonstration

The sketch now defaults to `DEMO_MODE=true`. Upload
`FireTurret/FireTurret.ino` using Arduino IDE and use Serial Monitor at 115200 baud,
Newline. Keep the pump and motor driver physically disconnected for this test.
At startup verify `DEMO MODE - actuator outputs disabled` appears.

## What is real and what is simulated

- Real: start button GPIO21, home limit switch GPIO33, analog flame sensor GPIO34,
  debounce, analog filtering/thresholds, state transitions, and shooting timer.
- Simulated: stepper motion, back-off and aiming completion (one second), a fixed
  45-degree trajectory, and intended pump actions. Actuator GPIOs are not initialized or written in demo mode. Keep all actuator
  drivers physically disconnected; unconfigured pins do not guarantee an OFF level.
- Optional simulated camera: serial commands below generate observations every
  100 ms. The real Pi bridge can still send FIRE messages; receiving a valid FIRE
  message disables the simulated source. Use only one serial client at a time.
- Driving remains unimplemented. Entering Drive does not demonstrate real driving.

## Wire and calibrate first

Wire the normally-open start button from GPIO21 to GND, home switch NO/COM from
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
A latching start switch works between GPIO21 and GND: open it, then close it to
start. A closed switch at boot must first be opened. Opening it is not a stop or
power-off command. The complete diagnostic sketch includes setup(), loop(),
all state-machine functions, and all global variables; open/copy the entire file.

## Simulated analog flame input

SensorDemo now starts with a simulated analog input. Its timer starts automatically
on first entry to Characterize; the value stays low for three seconds before
rising. Commands accept upper/lowercase and surrounding spaces. Send `SIM_FLAME`
to restart the timer manually. The profile is an
explicitly simulated analog source: 3 seconds around 600 ADC counts, a 4-second
rise to 3200, 8 seconds around 3200, a 3-second fall, then remain around 600.
Small pseudorandom variations of +/-50 counts are added. Repeating SIM_FLAME
restarts the timing. STATUS labels the source as SIMULATED or REAL.

The simulated source uses fixed rising thresholds 2400/2000 and does not require
real sensor calibration. It never changes FLAME_CALIBRATED. REAL_FLAME restores
real analogRead and the real calibration requirements. Switching sources clears
the averaging buffer, and eight new samples must arrive before readiness.

For the demo, home with the physical switches, send SIM_NEAR, then SIM_FLAME.
Characterize advances when the simulated IR reading reaches its threshold.
Send SIM_FAR instead to demonstrate Drive. Send SIM_CLEAR and wait for simulated
IR to fall/clear to demonstrate Fire check -> Search. Simulated flame readings do
not themselves clear the camera; SIM_NEAR remains active until changed.
This is software simulation, not evidence of a working analog sensor. Disclose
it in the video and confirm instructor acceptance for the analog-sensor requirement.

Use `sim_far` to select 3 m and reach Drive after analog confirmation; `sim_near`
selects 0.8 m and leads to Calculate & aim. The camera source must still be selected
explicitly. `real_flame` restores real input and calibration requirements.

## Updated simulated approach sequence

For the latest SensorDemo, send `sim_far` after homing. Out-of-range camera fire
now enters Drive immediately, before requiring analog confirmation. The simulated
flame profile restarts on Drive entry. After three seconds of simulated driving,
the simulated camera distance changes automatically from 3.0 m to 0.8 m, and the
analog profile starts rising. The machine briefly returns to Characterize to
check the new distance and waits for infrared confirmation, then enters Calculate
& aim and Shoot. This is a simulated approach; no physical driving occurs.
No need to send sim_near to leave Drive in this sequence. Real camera FIRE inputs
are never automatically rewritten; they require an actual in-range observation.
`real_flame` still requires real sensor confirmation; no analog values are changed
when that source is selected. Full hardware-controller behavior is unchanged.

## Drive-then-flame revision 2 (current behavior)

Startup identifies `DEMO VERSION: DRIVE-THEN-FLAME-2`. After homing, send sim_far.
State 4 stays active while simulating driving for three seconds with low flame
readings. Then the simulated camera target becomes 0.8 m away and flame readings
start rising immediately. Remain in State 4 until the filtered reading reaches
2400 and the camera is fresh/in range, then go directly to State 5 (no return to
State 3). STATUS now includes distance and DRIVING/WAIT_FOR_FLAME phase. Simulated
approach never changes real camera data or real flame readings. This supersedes
the earlier approach description that returned from Drive to Characterize.
