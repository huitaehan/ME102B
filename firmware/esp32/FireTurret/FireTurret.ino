#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ESP32 DevKit example pins; verify your board and wiring before use.
// STEP/DIR driver required (A4988/DRV8825 etc.); never connect a NEMA17 directly.
constexpr int STEP_PIN=25, DIR_PIN=26, ENABLE_PIN=27;
constexpr int HOME_PIN=33, PUMP_PIN=32;
constexpr bool HOME_DIR_HIGH=true, PUMP_ACTIVE_HIGH=true;
// Switch connects HOME_PIN to GND; driver enable assumed active LOW.
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
bool switchRaw=false, switchStable=false, previousSwitch=false;
bool releasedBeforeStart=false, checkSawFrame=false, aimSet=false;
long position=0, target=0;
float distanceM=0, confidence=0;
char input[96]; size_t inputLength=0; bool dropping=false;

// SERVICE: electrical outputs and actuator control.
void pump(bool on) { digitalWrite(PUMP_PIN, (on==PUMP_ACTIVE_HIGH)?HIGH:LOW); }
void stopMotion() { target=position; }
void transition(State next) {
  pump(false); stopMotion(); state=next; entered=millis(); aimSet=false;
  if(next==FIRE_CHECK) { checkFrame=frameId; checkSawFrame=false; clearSince=0; }
  if(next==FAULT) digitalWrite(ENABLE_PIN,HIGH);
  Serial.printf("STATE,%d\n",int(next));
}
// EVENT CHECKER: debounce the 90-degree homing switch.
void checkLimitSwitch() {
  bool pressed=digitalRead(HOME_PIN)==LOW;
  if(pressed!=switchRaw) { switchRaw=pressed; switchChanged=millis(); }
  if(millis()-switchChanged>=30) switchStable=switchRaw;
}
// EVENT CHECKER: accept one complete observation per camera frame.
// FIRE,0,0,0  or FIRE,1,<distance_metres>,<confidence>
// START begins homing (also supported by a switch press); STOP latches FAULT.
void acceptLine(char *line) {
  if(!strcmp(line,"STOP")) { transition(FAULT); return; }
  if(!strcmp(line,"START")) {
    if(state==WAIT_START) transition(HOMING);
    return;
  }
  int detected=-1, consumed=0; float d=0,c=0;
  if(sscanf(line,"FIRE,%d,%f,%f%n",&detected,&d,&c,&consumed)!=3 || line[consumed]) return;
  if((detected!=0 && detected!=1) || !isfinite(d) || !isfinite(c) || c<0 || c>1) return;
  if(detected && (d<=0 || c<=0.40f)) return;
  fire=detected; distanceM=d; confidence=c;
  haveVision=true; lastVision=millis(); ++frameId;
}
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
// EVENT CHECKERS: freshness, range, timing, and motion completion.
bool visionFresh() { return haveVision && millis()-lastVision<VISION_TIMEOUT_MS; }
bool targetVisible() { return visionFresh() && fire; }
bool targetInRange() { return distanceM>=MIN_RANGE_M && distanceM<=MAX_RANGE_M; }
bool motionDone() { return position==target && !pulseHigh; }
bool elapsed(uint32_t ms) { return millis()-entered>=ms; }

// SERVICE: nonblocking STEP pulses; positive position moves away from home.
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
void aimAt(float angle) { target=lroundf((90.0f-angle)*STEPS_PER_DEGREE); }
// SERVICE: ideal projectile calculation, low-angle solution.
// Water jets require measured calibration; this model ignores drag/breakup.
bool calculateAngle(float range,float &degrees) {
  if(!CALIBRATED || JET_SPEED_M_S<=0 || range<=0) return false;
  const float g=9.81f, v2=JET_SPEED_M_S*JET_SPEED_M_S;
  float discriminant=v2*v2-g*(g*range*range+2*TARGET_HEIGHT_M*v2);
  if(discriminant<0) return false;
  degrees=atanf((v2-sqrtf(discriminant))/(g*range))*180.0f/PI;
  return isfinite(degrees) && degrees>=MIN_ANGLE && degrees<=MAX_ANGLE;
}
// SERVICE: drive deliberately unimplemented. Pump remains off in DRIVE.
void serviceDrive() { /* TODO: motor driver, horizontal alignment, updated range. */ }
// SERVICE: stationary camera search. No horizontal motor was specified.
void serviceSearch() { /* YOLO scans frames; no rover motion implemented. */ }

// SERVICE: state machine. Event checker calls select transitions.
void serviceStateMachine() {
  switch(state) {
    case WAIT_START:
      if(!switchStable) releasedBeforeStart=true;
      if(releasedBeforeStart && switchStable && !previousSwitch) transition(HOMING);
      break;
    case HOMING:
      if(switchStable) {
        position=target=0; homed=true;
        aimAt(MAX_ANGLE); // Back off switch before searching.
        state=SEARCH; entered=millis();
      } else if(elapsed(HOMING_TIMEOUT_MS)) transition(FAULT);
      break;
    case SEARCH:
      serviceSearch();
      if(motionDone() && switchStable) { transition(FAULT); break; }
      if(motionDone() && targetVisible()) transition(CHARACTERIZE);
      break;
    case CHARACTERIZE:
      // YOLO provides confidence and estimated distance, not temperature.
      if(!targetVisible()) transition(SEARCH);
      else transition(targetInRange()?CALCULATE_AIM:DRIVE);
      break;
    case DRIVE:
      serviceDrive();
      if(!targetVisible()) transition(SEARCH);
      else if(targetInRange()) transition(CHARACTERIZE);
      break;
    case CALCULATE_AIM:
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
      if(!visionFresh() || !fire || !targetInRange()) transition(FIRE_CHECK);
      else if(elapsed(SHOOT_MS)) transition(FIRE_CHECK);
      break;
    case FIRE_CHECK:
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
    case FAULT: pump(false); break; // Reset ESP32 to recover; re-home required.
  }
  previousSwitch=switchStable;
}
void setup() {
  pinMode(PUMP_PIN,OUTPUT); pump(false);
  pinMode(STEP_PIN,OUTPUT); digitalWrite(STEP_PIN,LOW);
  pinMode(DIR_PIN,OUTPUT); pinMode(ENABLE_PIN,OUTPUT);
  digitalWrite(ENABLE_PIN,LOW); pinMode(HOME_PIN,INPUT_PULLUP);
  Serial.begin(115200); entered=millis();
  Serial.println("WAIT_START: send START or press/release the home switch.");
}
void loop() {
  checkSerial(); checkLimitSwitch(); serviceStateMachine(); serviceStepper();
}
