# ME102B — Fire detection and water turret

YOLO runs on a computer or Raspberry Pi. A USB serial bridge sends detections
and estimated distances to an ESP32, which reads an analog flame sensor and controls nozzle elevation and the pump.

## Repository layout

```text
firmware/esp32/
  FireTurret/FireTurret.ino       Arduino state machine
  README.md                     Wiring, calibration, and state behavior
vision/
  fire_serial_bridge.py          YOLO detections -> ESP32 over USB
  webcam_detection.py            Standalone OpenVINO webcam viewer
  requirements.txt              Python dependencies
  raspberry_pi/
    imx500_detection.py          Original IMX500 camera viewer
    system-packages.txt          Original Pi OS package commands
models/
  best.pt                       Trained YOLO weights used by the bridge
  best_openvino_model/          OpenVINO export used by the webcam viewer
  imx500/packerOut.zip          IMX500 conversion artifacts
assets/
  sample_fire.jpg               Sample image
```

## Sensor-only demo

The sketch defaults to `DEMO_MODE=true`: physical motor and pump outputs are disabled,
while real buttons and the calibrated analog flame sensor still operate. See the
[demo instructions](firmware/esp32/DEMO.md) for Serial Monitor commands and a recording
sequence. Set `DEMO_MODE=false` and finish calibration for actual actuation.

## ESP32 firmware

Open [FireTurret.ino](firmware/esp32/FireTurret/FireTurret.ino) in Arduino IDE.
Install the Espressif ESP32 board package and select your board and serial port.
Read the [hardware and calibration instructions](firmware/esp32/README.md) before
uploading. Pin assignments are examples; automatic shooting remains disabled
until mechanical limits and measured water-jet calibration are configured.
An analog infrared flame sensor on GPIO34 must support the camera detection
before aiming/shooting. Calibrate its thresholds and set FLAME_CALIBRATED=true;
example defaults do not authorize operation. Read the flame-sensor setup section
in the wiring guide. This sensor does not measure temperature.
Drive and horizontal scanning are not implemented.

Start operation by pressing the separate button on GPIO23. During homing, the
NEMA17 moves the turret toward 90 degrees above the ground until the limit switch
on GPIO33 is pressed. This establishes the absolute home reference; it does not
command a relative 90-degree turn from an unknown starting position. The turret
then backs off to 85 degrees and searches incoming camera observations.
The home limit switch never starts the machine. Serial `START` is disabled by
default. See the [numbered state descriptions](firmware/esp32/README.md#state-descriptions)
for the event conditions and services.

## Vision and serial bridge

From the repository root, create a Python environment and install dependencies:

```sh
python -m venv .venv
# Linux / macOS
source .venv/bin/activate
# Windows PowerShell instead: .venv\Scripts\Activate.ps1
python -m pip install -r vision/requirements.txt
```

Run YOLO and send detections to the connected ESP32 (replace the serial port):

```sh
python vision/fire_serial_bridge.py --port /dev/ttyUSB0 --focal-length-px 600
```

Use `COM5` or your actual port on Windows. Close Arduino Serial Monitor first.
Start the bridge before pressing the physical start button. Opening the serial
port may reset the ESP32. Do not use `--start` for normal operation; that option
only works when firmware `ALLOW_SERIAL_START` is enabled for bench tests.
The bridge defaults to `models/best.pt`; `--model` accepts another model path.
Distance uses assumed flame height and calibrated focal length, so calibrate for
your camera resolution and target. Ctrl+C sends STOP; firmware latches FAULT
and requires reset and homing before reuse.

To run only the original webcam visualization:

```sh
python vision/webcam_detection.py
```

For Raspberry Pi camera input, install Picamera2 using your Raspberry Pi OS
packages and make it available to your Python environment, then use:

```sh
python vision/fire_serial_bridge.py --port /dev/ttyUSB0 --picamera --focal-length-px 490
```

The bridge runs Ultralytics inference, not the IMX500 accelerator. The separate
`vision/raspberry_pi/imx500_detection.py` viewer requires Picamera2/IMX500 support
and `models/imx500/fire_network.rpk`. That runtime model is not included;
`packerOut.zip` contains conversion artifacts rather than an RPK. Its output
decoding also needs verification against the actual camera/model.

## Validation status

Firmware host logic checks covered pump timing, fire checking, vision timeout,
homing timeout, and STOP. Python syntax and bridge distance formatting were
checked. Compilation with the ESP32 board package, camera inference, motor
travel, pumping, and physical calibration still require the actual setup.
