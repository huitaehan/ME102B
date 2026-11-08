# Hardware configuration

Confirmed by the project owner:

- Controller: Adafruit ESP32 Feather V2 (ESP32-PICO-MINI-02).
- Vision computer: Raspberry Pi 5 with Raspberry Pi AI Camera (IMX500).
- NEMA17 controls nozzle elevation. Robot drive motors provide horizontal aiming.
- Start switch: Feather MI/MISO, GPIO21, to GND.
- Home switch: GPIO33 to NO, GND to COM.
- Analog flame signal: A2/GPIO34; sensor VCC to 3V, GND to GND.
  Confirm sensor specifications support 3.3 V supply and analog output.

Upload `firmware/esp32/FireTurret/FireTurret.ino` for integrated control.
Its start pin matches SensorDemo. Motor/pump pins remain provisional; the supplied
wiring covers inputs only. Demo mode remains enabled and calibration flags remain off.

The Pi bridge defaults to CPU inference. An optional `--imx500-model` backend
runs the packaged network on the AI Camera. See `vision/raspberry_pi/AI_CAMERA.md`.
The supplied conversion archive has four NMS tensors; model identity, box layout,
and detection accuracy must be verified on the physical camera before use.

Still needed for physical implementation:

- NEMA17 driver model, STEP/DIR/enable wiring, microstepping, gearing, direction,
  home angle and mechanical travel limits.
- Wheel motor types, driver models and wiring, encoder specifications, wheel
  diameter and track width. Confirm whether an IMU is available for heading.
- Pump model/supply, switching driver and wiring, active level.
- Whether the camera is fixed to the chassis or moves with nozzle elevation.
- Nozzle geometry, measured jet speed, camera distance calibration, and analog
  flame/no-flame measurements.

There is no independent yaw motor or confirmed camera-tilt motor. Do not adopt the
experimental sketch's extra axes or pin map. Future YAW commands must control the
base heading, while elevation aims the nozzle. Heading zero and feedback still
need a defined implementation.
