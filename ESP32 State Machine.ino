// EXPERIMENTAL REFERENCE ONLY -- NOT THE INTEGRATED CONTROLLER.
// This sketch contains incomplete motion handlers and must not drive hardware.
// Upload firmware/esp32/FireTurret/FireTurret.ino instead.
#error "Reference sketch only. Upload firmware/esp32/FireTurret/FireTurret.ino"
#include <Arduino.h>
#include <math.h>

// ==========================================
// PIN DEFINITIONS & HARDWARE CONFIGURATION
// ==========================================
// Turret Stepper (NEMA 17)
constexpr int TURRET_STEP_PIN = 25;
constexpr int TURRET_DIR_PIN  = 26;
constexpr int TURRET_EN_PIN   = 27;
constexpr int TURRET_LIMIT_PIN = 33;

// Camera Tilt Stepper (NEMA 17)
constexpr int CAM_STEP_PIN    = 18;
constexpr int CAM_DIR_PIN     = 19;
constexpr int CAM_EN_PIN      = 21;
constexpr int CAM_LIMIT_PIN   = 34;

// Drive Base (2 Rear Brushless DC Motors with Encoders)
constexpr int MOTOR_L_PWM = 22;
constexpr int MOTOR_L_IN1 = 12;
constexpr int MOTOR_L_IN2 = 14;
constexpr int MOTOR_R_PWM = 23;
constexpr int MOTOR_R_IN1 = 15;
constexpr int MOTOR_R_IN2 = 4;

constexpr int ENC_L_A = 35;
constexpr int ENC_R_A = 36;

// Peripherals
constexpr int START_PIN = 13;
constexpr int PUMP_PIN  = 32;

// Motor & Stepper Constants
constexpr float STEPS_PER_DEGREE = (200.0f * 16.0f) / 360.0f; // 1/16 microstepping
constexpr uint32_t STEP_INTERVAL_US = 2500;
constexpr uint32_t PULSE_US = 5;

// ==========================================
// STATE MACHINE & SYSTEM VARIABLES
// ==========================================
enum State { 
  WAIT_START, HOMING, SEARCH, CHARACTERIZE, DRIVE,
  CALCULATE_AIM, SHOOT, FIRE_CHECK, FAULT 
};

State state = WAIT_START;
uint32_t entered = 0, lastVision = 0, frameId = 0, checkFrame = 0;
uint32_t homingStart = 0, clearSince = 0;
bool homed = false, fire = false, haveVision = false;
bool startStable = false, previousStart = false;
uint32_t startChanged = 0;
float distanceM = 0, confidence = 0;

// Trajectory & Command Buffers
char input[128];
size_t inputLength = 0;
bool dropping = false;

// Axis Positions
float turretAngle = 0.0f;
float camAngle = 0.0f;

// Forward declarations
void transition(State next);
void pump(bool on);
void sendReply(const char* msg);

// ==========================================
// SETUP & UTILITIES
// ==========================================
void setup() {
  Serial.begin(115200);

  // Pin Modes
  pinMode(PUMP_PIN, OUTPUT); pump(false);
  pinMode(START_PIN, INPUT_PULLUP);
  
  pinMode(TURRET_STEP_PIN, OUTPUT); pinMode(TURRET_DIR_PIN, OUTPUT); pinMode(TURRET_EN_PIN, OUTPUT);
  pinMode(TURRET_LIMIT_PIN, INPUT_PULLUP);
  digitalWrite(TURRET_EN_PIN, LOW);

  pinMode(CAM_STEP_PIN, OUTPUT); pinMode(CAM_DIR_PIN, OUTPUT); pinMode(CAM_EN_PIN, OUTPUT);
  pinMode(CAM_LIMIT_PIN, INPUT_PULLUP);
  digitalWrite(CAM_EN_PIN, LOW);

  pinMode(MOTOR_L_PWM, OUTPUT); pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT); pinMode(MOTOR_R_IN1, OUTPUT); pinMode(MOTOR_R_IN2, OUTPUT);

  entered = millis();
  Serial.println("ESP32 Ready. Waiting for START.");
}

void pump(bool on) {
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
}

void sendReply(const char* msg) {
  Serial.println(msg);
}

void transition(State next) {
  pump(false);
  state = next;
  entered = millis();
  if (next == HOMING) {
    homed = false;
    homingStart = millis();
  }
  if (next == FAULT) {
    digitalWrite(TURRET_EN_PIN, HIGH);
    digitalWrite(CAM_EN_PIN, HIGH);
  }
  Serial.printf("STATE,%d\n", int(next));
}

// ==========================================
// SERIAL COMMAND PARSER (Python <-> ESP32)
// ==========================================
void executeCommand(char *line) {
  char cmd[32];
  if (sscanf(line, "%31[^,]", cmd) != 1) return;

  if (strcmp(cmd, "STOP") == 0) {
    transition(FAULT);
    sendReply("DONE,STOP");
  } 
  else if (strcmp(cmd, "HOME") == 0) {
    char axis[16]; int dir; float speed, max_travel, switch_angle;
    if (sscanf(line, "HOME,%16[^,],%d,%f,%f,%f", axis, &dir, &speed, &max_travel, &switch_angle) == 5) {
      // Execute Homing for specific axis
      // Simplified: simulate hitting switch and setting angle
      delay(200); 
      homed = true;
      sendReply(strcmp(axis, "TURRET") == 0 ? "DONE,TURRET" : "DONE,CAM");
    } else {
      sendReply("ERR,HOME,PARM");
    }
  }
  else if (strcmp(cmd, "AIM_SET") == 0) {
    turretAngle = 0.0f;
    sendReply("DONE");
  }
  else if (strcmp(cmd, "YAW") == 0) {
    float deg;
    if (sscanf(line, "YAW,%f", &deg) == 1) {
      turretAngle = deg;
      sendReply("DONE");
    } else sendReply("ERR,YAW");
  }
  else if (strcmp(cmd, "CAM") == 0) {
    float deg;
    if (sscanf(line, "CAM,%f", &deg) == 1) {
      camAngle = deg;
      sendReply("DONE");
    } else sendReply("ERR,CAM");
  }
  else if (strcmp(cmd, "DRIVE") == 0) {
    float meters;
    if (sscanf(line, "DRIVE,%f", &meters) == 1) {
      // Drive forward simulation using DC motors
      digitalWrite(MOTOR_L_IN1, HIGH); digitalWrite(MOTOR_L_IN2, LOW); analogWrite(MOTOR_L_PWM, 150);
      digitalWrite(MOTOR_R_IN1, HIGH); digitalWrite(MOTOR_R_IN2, LOW); analogWrite(MOTOR_R_PWM, 150);
      delay(3000); // Simulate driving duration
      analogWrite(MOTOR_L_PWM, 0); analogWrite(MOTOR_R_PWM, 0);
      sendReply("DONE");
    } else sendReply("ERR,DRIVE");
  }
  else if (strcmp(cmd, "GOTO") == 0) {
    float yaw, pitch;
    if (sscanf(line, "GOTO,%f,%f", &yaw, &pitch) == 2) {
      turretAngle = yaw; camAngle = pitch;
      sendReply("DONE");
    } else sendReply("ERR,GOTO");
  }
  else if (strcmp(cmd, "TRAJ_BEGIN") == 0 || strcmp(cmd, "LEG") == 0 || strcmp(cmd, "TRAJ_RUN") == 0) {
    // Handle trajectory streaming/execution for serpentine shooting
    sendReply("DONE");
  }
  else if (strcmp(cmd, "FIRE") == 0) {
    int detected; float d, c;
    if (sscanf(line, "FIRE,%d,%f,%f", &detected, &d, &c) == 3) {
      fire = (detected != 0);
      distanceM = d;
      confidence = c;
      haveVision = true;
      lastVision = millis();
      frameId++;
    }
  }
}

void checkSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!dropping) {
        input[inputLength] = 0;
        executeCommand(input);
      }
      inputLength = 0;
      dropping = false;
    } else if (!dropping) {
      if (inputLength < sizeof(input) - 1) input[inputLength++] = ch;
      else dropping = true;
    }
  }
}

// ==========================================
// STATE MACHINE LOGIC
// ==========================================
void serviceStateMachine() {
  bool startPressed = (digitalRead(START_PIN) == LOW);
  if (startPressed != startStable) {
    startStable = startPressed;
    startChanged = millis();
  }

  switch (state) {
    case WAIT_START:
      if (startStable && !previousStart) {
        transition(HOMING);
      }
      break;

    case HOMING:
      // Check limit switches or timeout (15s)
      if (homed) {
        transition(SEARCH);
      } else if (millis() - homingStart > 15000) {
        transition(FAULT);
      }
      break;

    case SEARCH:
      if (startStable && !previousStart) {
        transition(HOMING);
        break;
      }
      if (haveVision && fire && confidence > 0.40f) {
        transition(CHARACTERIZE);
      }
      break;

    case CHARACTERIZE:
      if (!haveVision || !fire) {
        transition(SEARCH);
      } else {
        if (distanceM >= 0.10f && distanceM <= 2.0f) {
          transition(CALCULATE_AIM);
        } else {
          transition(DRIVE);
        }
      }
      break;

    case DRIVE:
      // Drive forward simulation for 3 seconds
      if (millis() - entered >= 3000) {
        distanceM = 0.8f; // simulated updated distance
        transition(CALCULATE_AIM);
      }
      break;

    case CALCULATE_AIM:
      // Aim and transition to shoot
      transition(SHOOT);
      pump(true);
      break;

    case SHOOT:
      if (!fire || distanceM < 0.10f || distanceM > 2.0f) {
        transition(FIRE_CHECK);
      } else if (millis() - entered >= 3000) {
        transition(FIRE_CHECK);
      }
      break;

    case FIRE_CHECK:
      pump(false);
      if (millis() - entered >= 500) {
        if (fire) {
          transition(CHARACTERIZE);
        } else {
          clearSince = millis();
          transition(SEARCH);
        }
      }
      break;

    case FAULT:
      pump(false);
      // Reset logic or hold fault
      break;
  }
  previousStart = startStable;
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  checkSerial();
  serviceStateMachine();
}
