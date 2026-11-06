#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ESP32 DevKit example pins; verify your board and wiring before use.
// STEP/DIR driver required (A4988/DRV8825 etc.); never connect a NEMA17 directly.
constexpr int STEP_PIN=25, DIR_PIN=26, ENABLE_PIN=27;
constexpr int START_PIN=23, HOME_PIN=33, PUMP_PIN=32;
constexpr bool ALLOW_SERIAL_START=false; // Enable only for explicit bench testing.
constexpr bool HOME_DIR_HIGH=true, PUMP_ACTIVE_HIGH=true;
// Separate normally-open start button and home switch each connect their pin to GND.
// Both use internal pull-ups; driver enable is assumed active LOW.
constexpr bool CALIBRATED=false; // Set true only after mechanical and jet calibration.
constexpr float STEPS_PER_DEGREE=200.0f*16.0f/360.0f; // Include gear ratio.
constexpr float MIN_ANGLE=5, MAX_ANGLE=85; // Homing switch is at 90 degrees.
constexpr float MIN_RANGE_M=0.10f, MAX_RANGE_M=2.0f;
constexpr float JET_SPEED_M_S=0.0f; // MEASURE; zero deliberately disables firing.
constexpr float TARGET_HEIGHT_M=0.0f; // Target height relative to nozzle.
// Camera distance must approximate HORIZONTAL range; adjust bridge if tilted.
constexpr uint32_t STEP_INTERVAL_US=2500, PULSE_US=5;
constexpr uint32_t VISION_TIMEOUT_MS=1500, HOMING_TIMEOUT_MS=15000;
constexpr uint32_t SHOOT_MS=3000, SETTLE_MS=500, CLEAR_MS=2000;

enum State { WAIT_START, HOMING, SEARCH, CHARACTERIZE, DRIVE,
             CALCULATE_AIM, SHOOT, FIRE_CHECK, FAULT };
State state=WAIT_START;
uint32_t entered=0, lastVision=0, frameId=0, checkFrame=0;
uint32_t nextStep=0, pulseStarted=0, switchChanged=0, clearSince=0;
bool pulseHigh=false, homed=false, fire=false, haveVision=false;
bool switchRaw=false, switchStable=false;
bool startRaw=false, startStable=false, previousStart=false;
uint32_t startChanged=0;
bool releasedBeforeStart=false, checkSawFrame=false, aimSet=false;
long position=0, target=0;
float distanceM=0, confidence=0;
char input[96]; size_t inputLength=0; bool dropping=false;

// SERVICE FUNCTION: electrical outputs and actuator control.
void pump(bool on) { digitalWrite(PUMP_PIN, (on==PUMP_ACTIVE_HIGH)?HIGH:LOW); }
// SERVICE FUNCTION: cancel the requested stepper movement.
void stopMotion() { target=position; }
// SERVICE FUNCTION: change state, stop outputs, reset timers, and report the state.
void transition(State next) {
  pump(false); stopMotion(); state=next; entered=millis(); aimSet=false;
  if(next==FIRE_CHECK) { checkFrame=frameId; checkSawFrame=false; clearSince=0; }
  if(next==FAULT) digitalWrite(ENABLE_PIN,HIGH);
  Serial.printf("STATE,%d\n",int(next));
}
// EVENT CHECKER: debounce the separate start button (active LOW).
void checkStartButton() {
  bool pressed=digitalRead(START_PIN)==LOW;
  if(pressed!=startRaw) { startRaw=pressed; startChanged=millis(); }
  if(millis()-startChanged>=30) startStable=startRaw;
}
// EVENT CHECKER: debounce the 90-degree homing switch.
void checkLimitSwitch() {
  bool pressed=digitalRead(HOME_PIN)==LOW;
  if(pressed!=switchRaw) { switchRaw=pressed; switchChanged=millis(); }
  if(millis()-switchChanged>=30) switchStable=switchRaw;
}
// EVENT CHECKER: accept one complete observation per camera frame.
// FIRE,0,0,0  or FIRE,1,<distance_metres>,<confidence>
// START is accepted only with ALLOW_SERIAL_START enabled for bench testing.
// Normal operation requires the separate physical start button; STOP latches FAULT.
void acceptLine(char *line) {
  if(!strcmp(line,"STOP")) { transition(FAULT); return; }
  if(!strcmp(line,"START")) {
    if(ALLOW_SERIAL_START && state==WAIT_START) transition(HOMING);
    return;
  }
  int detected=-1, consumed=0; float d=0,c=0;
  if(sscanf(line,"FIRE,%d,%f,%f%n",&detected,&d,&c,&consumed)!=3 || line[consumed]) return;
  if((detected!=0 && detected!=1) || !isfinite(d) || !isfinite(c) || c<0 || c>1) return;
  if(detected && (d<=0 || c<=0.40f)) return;
  fire=detected; distanceM=d; confidence=c;
  haveVision=true; lastVision=millis(); ++frameId;
}
// EVENT CHECKER: collect serial lines and dispatch complete commands/detections.
void checkSerial() {
  // Bounded work so serial traffic cannot starve stepping or the pump timer.
  for(int n=0;n<128 && Serial.available();++n) {
    char ch=Serial.read();
    if(ch=='\r') continue;
    if(ch=='\n') {
      if(!dropping) { input[inputLength]=0; acceptLine(input); }
      inputLength=0; dropping=false;
    } else if(!dropping) {
      if(inputLength<sizeof(input)-1) input[inputLength++]=ch;
      else dropping=true;
    }
  }
}
// EVENT CHECKER: true when an accepted camera observation is less than 1.5 s old.
bool visionFresh() { return haveVision && millis()-lastVision<VISION_TIMEOUT_MS; }
// EVENT CHECKER: true when a fresh observation reports fire.
bool targetVisible() { return visionFresh() && fire; }
// EVENT CHECKER: test estimated distance against the configured usable range.
bool targetInRange() { return distanceM>=MIN_RANGE_M && distanceM<=MAX_RANGE_M; }
// EVENT CHECKER: true when the commanded step count is reached and pulse is LOW.
// This checks commanded steps, not physical position feedback.
bool motionDone() { return position==target && !pulseHigh; }
// EVENT CHECKER: true when the current state has lasted at least ms milliseconds.
bool elapsed(uint32_t ms) { return millis()-entered>=ms; }

// SERVICE FUNCTION: nonblocking STEP pulses; positive position moves away from home.
void serviceStepper() {
  uint32_t now=micros();
  if(pulseHigh) {
    if(uint32_t(now-pulseStarted)>=PULSE_US) { digitalWrite(STEP_PIN,LOW); pulseHigh=false; }
    return;
  }
  bool home=state==HOMING;
  if(home && switchRaw) return; // Stop immediately; confirm with debounce in FSM.
  if(!home && (!homed || position==target || state==FAULT)) return;
  if(int32_t(now-nextStep)<0) return;
  bool towardHome=home || target<position;
  // Stop on raw switch when moving toward home; away motion must release it.
  if(!home && towardHome && switchRaw) { transition(FAULT); return; }
  digitalWrite(DIR_PIN,(towardHome==HOME_DIR_HIGH)?HIGH:LOW);
  delayMicroseconds(3); // Driver direction setup time only, not a motion delay.
  digitalWrite(STEP_PIN,HIGH); pulseHigh=true; pulseStarted=micros();
  nextStep=pulseStarted+STEP_INTERVAL_US;
  if(!home) position+=towardHome?-1:1;
}
// SERVICE FUNCTION: convert nozzle elevation into a target step count from home.
void aimAt(float angle) { target=lroundf((90.0f-angle)*STEPS_PER_DEGREE); }
// SERVICE FUNCTION: ideal projectile calculation, low-angle solution.
// Water jets require measured calibration; this model ignores drag/breakup.
bool calculateAngle(float range,float &degrees) {
  if(!CALIBRATED || JET_SPEED_M_S<=0 || range<=0) return false;
  const float g=9.81f, v2=JET_SPEED_M_S*JET_SPEED_M_S;
  float discriminant=v2*v2-g*(g*range*range+2*TARGET_HEIGHT_M*v2);
  if(discriminant<0) return false;
  degrees=atanf((v2-sqrtf(discriminant))/(g*range))*180.0f/PI;
  return isfinite(degrees) && degrees>=MIN_ANGLE && degrees<=MAX_ANGLE;
}
// SERVICE FUNCTION: drive deliberately unimplemented. Pump remains off in DRIVE.
void serviceDrive() { /* TODO: motor driver, horizontal alignment, updated range. */ }
// SERVICE FUNCTION: stationary camera search. No horizontal motor was specified.
void serviceSearch() { /* YOLO scans frames; no rover motion implemented. */ }

// STATE MACHINE DISPATCHER / SERVICE FUNCTION: evaluate events and run state actions.
// EVENT CHECKER functions report conditions; SERVICE FUNCTIONs perform actions.
// Some switch/frame conditions are checked inline below and labeled explicitly.
void serviceStateMachine() {
  switch(state) {
    case WAIT_START:
      // EVENT CHECKER (inline): start button released, then a new debounced press.
      // The home switch never starts the machine. A held button at boot must release.
      if(!startRaw && !startStable) releasedBeforeStart=true;
      if(releasedBeforeStart && startStable && !previousStart) transition(HOMING);
      break;
    case HOMING:
      // EVENT CHECKERS: home switch reached, or homing timer expired.
      // SERVICES: establish home position and command switch back-off.
      if(switchStable) {
        position=target=0; homed=true;
        transition(SEARCH); // Report state 2 and reset state-entry bookkeeping.
        aimAt(MAX_ANGLE); // Back off switch before searching.
      } else if(elapsed(HOMING_TIMEOUT_MS)) transition(FAULT);
      break;
    case SEARCH:
      // SERVICE: monitor camera search. EVENTS: back-off done, switch state, fire.
      serviceSearch();
      if(motionDone() && switchStable) { transition(FAULT); break; }
      if(motionDone() && targetVisible()) transition(CHARACTERIZE);
      break;
    case CHARACTERIZE:
      // EVENT CHECKERS: fresh fire and usable distance select the next state.
      // YOLO provides confidence and estimated distance, not temperature.
      if(!targetVisible()) transition(SEARCH);
      else transition(targetInRange()?CALCULATE_AIM:DRIVE);
      break;
    case DRIVE:
      // SERVICE: drive placeholder. EVENTS: target lost or now in range.
      serviceDrive();
      if(!targetVisible()) transition(SEARCH);
      else if(targetInRange()) transition(CHARACTERIZE);
      break;
    case CALCULATE_AIM:
      // SERVICES: calculate trajectory and command elevation.
      // EVENTS: target validity, calculation success, and completed step count.
      if(!targetVisible()) { transition(SEARCH); break; }
      if(!targetInRange()) { transition(DRIVE); break; }
      if(!aimSet) {
        float angle;
        if(!calculateAngle(distanceM,angle)) { transition(FAULT); break; }
        aimAt(angle); aimSet=true;
      }
      if(motionDone()) { transition(SHOOT); pump(true); }
      break;
    case SHOOT:
      // Pump was enabled on entry. EVENTS: target invalid or 3 s timer expired.
      if(!visionFresh() || !fire || !targetInRange()) transition(FIRE_CHECK);
      else if(elapsed(SHOOT_MS)) transition(FIRE_CHECK);
      break;
    case FIRE_CHECK:
      // EVENT CHECKERS: camera freshness and settling timer.
      // EVENT CHECKER (inline): new frame, fire present, or 2 s of no-fire frames.
      if(!visionFresh()) { transition(FAULT); break; }
      if(!elapsed(SETTLE_MS)) { checkFrame=frameId; break; }
      // Require NEW observations after stopping/settling; stale no-fire is not clear.
      if(frameId!=checkFrame) {
        checkFrame=frameId;
        if(fire) { transition(CHARACTERIZE); break; }
        if(!checkSawFrame) { checkSawFrame=true; clearSince=millis(); }
        if(millis()-clearSince>=CLEAR_MS) transition(SEARCH);
      }
      break;
    // SERVICE: keep the pump off while the fault remains latched.
    case FAULT: pump(false); break; // Reset ESP32 to recover; re-home required.
  }
  previousStart=startStable;
}
// INITIALIZATION: configure pins, safe pump output, serial link, and startup timer.
void setup() {
  pinMode(PUMP_PIN,OUTPUT); pump(false);
  pinMode(STEP_PIN,OUTPUT); digitalWrite(STEP_PIN,LOW);
  pinMode(DIR_PIN,OUTPUT); pinMode(ENABLE_PIN,OUTPUT);
  digitalWrite(ENABLE_PIN,LOW); pinMode(HOME_PIN,INPUT_PULLUP);
  pinMode(START_PIN,INPUT_PULLUP);
  startRaw=startStable=previousStart=(digitalRead(START_PIN)==LOW);
  releasedBeforeStart=!startRaw; startChanged=millis();
  Serial.begin(115200); entered=millis();
  Serial.println("WAIT_START: press the separate start button.");
}
// MAIN LOOP: poll event checkers, run the state machine, and service motor pulses.
void loop() {
  checkSerial(); checkStartButton(); checkLimitSwitch(); serviceStateMachine(); serviceStepper();
}
