# ESP32 state machine

Arrow labels are event conditions and transition actions. Notes describe services.
Power Off is a physical system condition, not an ESP32 enum value.

```mermaid
stateDiagram-v2
    direction TB
    state "Power Off" as Off
    [*] --> Off
    Off --> Powered: Power applied
    Powered --> Off: Master power removed

    state "Powered system" as Powered {
        [*] --> Wait
        state "0 — Wait for start" as Wait
        state "1 — Homing" as Home
        state "2 — Search" as Search
        state "3 — Characterize" as Characterize
        state "4 — Drive" as Drive
        state "5 — Calculate & aim" as Aim
        state "6 — Shoot" as Shoot
        state "7 — Fire check" as Check
        state "8 — Fault" as Fault

        Wait --> Home: New start-button press
        Home --> Search: 90° limit switch pressed / set home and back off to 85°
        Home --> Fault: 15-second homing timeout
        Search --> Home: New start-button press / pump OFF and cancel motion
        Search --> Characterize: Back-off complete and fresh fire detected
        Search --> Fault: Limit switch still pressed after back-off
        Characterize --> Aim: Camera fire and IR confirmed, within range
        Characterize --> Drive: Camera fire and IR confirmed, outside range
        Characterize --> Search: Fire absent or data stale
        Drive --> Characterize: Fresh fire now within range
        Drive --> Search: Fire absent or data stale
        Aim --> Shoot: Valid trajectory and aiming complete / pump ON
        Aim --> Fault: Calibration missing or trajectory invalid
        Aim --> Characterize: Infrared confirmation lost
        Aim --> Drive: Fire outside range
        Aim --> Search: Fire absent or data stale
        Shoot --> Check: 3 seconds elapsed / pump OFF
        Shoot --> Check: Fire absent, out of range, data stale, or IR lost / pump OFF
        Check --> Characterize: Fresh fire after 500 ms settling
        Check --> Search: Fresh no-fire frames and IR clear span 2 seconds after settling
        Check --> Fault: Camera stale or infrared data not ready
        Fault --> Wait: ESP32 reset

        note right of Home
            SERVICE: step toward 90° switch
            CHECKERS: switch debounce and timeout
        end note
        note right of Search
            SERVICE: complete back-off, monitor YOLO
            CHECKERS: new button press and fresh fire
            No physical scanning implemented
        end note
        note right of Drive
            Drive service not implemented
            Pump remains OFF
        end note
        note right of Aim
            SERVICES: trajectory calculation and stepper control
            CHECKERS: range, visibility, infrared support, steps complete
        end note
        note right of Check
            SERVICE: pump OFF
            CHECKERS: settling timer, new frames, infrared clear, no-fire duration
        end note
        note right of Fault
            Pump OFF; stepper driver disabled
            Global fault events: serial STOP or unexpected
            home-switch contact while moving toward home
        end note
    }
```

- Start button: GPIO23 to GND; home switch: GPIO33 to GND; both debounced.
- A held button does not repeatedly trigger homing. Presses outside Wait/Search
  are ignored; serial START is disabled by default.
- Camera data becomes stale after 1.5 seconds; usable range is 0.10–2.00 m.
- A reset from any powered state restarts in Wait for start. Physical power-off
  requires switching the relevant supplies; software does not cut supply power.

- Characterize waits with pump off when vision and infrared disagree or the analog
  sensor is uncalibrated. IR alone does not authorize aiming or pumping.
- GPIO34 samples analog infrared intensity; it does not measure temperature.
- The average uses eight samples at 20 ms intervals, with calibrated hysteresis.
  IR-only detection in Fire check keeps the pump off and resets the clear timer.
