#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

constexpr int STEP_PIN=25, DIR_PIN=26, ENABLE_PIN=27;
constexpr int START_PIN=23, HOME_PIN=33, PUMP_PIN=32;
constexpr bool ALLOW_SERIAL_START=false;
constexpr bool HOME_DIR_HIGH=true, PUMP_ACTIVE_HIGH=true;

constexpr bool CALIBRATED=false;
constexpr float STEPS_PER_DEGREE=200.0f*16.0f/360.0f;
constexpr float MIN_ANGLE=5, MAX_ANGLE=85;
constexpr float MIN_RANGE_M=0.10f, MAX_RANGE_M=2.0f;
constexpr float JET_SPEED_M_S=0.0f;
constexpr float TARGET_HEIGHT_M=0.0f;

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

// SERVICE FUNCTION
void pump(bool on) { digitalWrite(PUMP_PIN, (on==PUMP_ACTIVE_HIGH)?HIGH:LOW); }

// SERVICE FUNCTION
void stopMotion() { target=position; }

// SERVICE FUNCTION
void transition(State next) {
  pump(false); stopMotion(); state=next; entered=millis(); aimSet=false;
  if(next==HOMING) homed=false;
  if(next==FIRE_CHECK) { checkFrame=frameId; checkSawFrame=false; clearSince=0; }
  if(next==FAULT) digitalWrite(ENABLE_PIN,HIGH);
  Serial.printf("STATE,%d\n",int(next));
}

// EVENT CHECKER
void checkStartButton() {
  bool pressed=digitalRead(START_PIN)==LOW;
  if(pressed!=startRaw) { startRaw=pressed; startChanged=millis(); }
  if(millis()-startChanged>=30) startStable=startRaw;
}

// EVENT CHECKER
bool startButtonPressed() {
  return releasedBeforeStart && startStable && !previousStart;
}

// EVENT CHECKER
void checkLimitSwitch() {
  bool pressed=digitalRead(HOME_PIN)==LOW;
  if(pressed!=switchRaw) { switchRaw=pressed; switchChanged=millis(); }
  if(millis()-switchChanged>=30) switchStable=switchRaw;
}

// EVENT CHECKER
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

// EVENT CHECKER
void checkSerial() {

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

// EVENT CHECKER
bool visionFresh() { return haveVision && millis()-lastVision<VISION_TIMEOUT_MS; }

// EVENT CHECKER
bool targetVisible() { return visionFresh() && fire; }

// EVENT CHECKER
bool targetInRange() { return distanceM>=MIN_RANGE_M && distanceM<=MAX_RANGE_M; }

// EVENT CHECKER
bool motionDone() { return position==target && !pulseHigh; }

// EVENT CHECKER
bool elapsed(uint32_t ms) { return millis()-entered>=ms; }

// SERVICE FUNCTION
void serviceStepper() {
  uint32_t now=micros();
  if(pulseHigh) {
    if(uint32_t(now-pulseStarted)>=PULSE_US) { digitalWrite(STEP_PIN,LOW); pulseHigh=false; }
    return;
  }
  bool home=state==HOMING;
  if(home && switchRaw) return;
  if(!home && (!homed || position==target || state==FAULT)) return;
  if(int32_t(now-nextStep)<0) return;
  bool towardHome=home || target<position;

  if(!home && towardHome && switchRaw) { transition(FAULT); return; }
  digitalWrite(DIR_PIN,(towardHome==HOME_DIR_HIGH)?HIGH:LOW);
  delayMicroseconds(3);
  digitalWrite(STEP_PIN,HIGH); pulseHigh=true; pulseStarted=micros();
  nextStep=pulseStarted+STEP_INTERVAL_US;
  if(!home) position+=towardHome?-1:1;
}

// SERVICE FUNCTION
void aimAt(float angle) { target=lroundf((90.0f-angle)*STEPS_PER_DEGREE); }

// SERVICE FUNCTION
bool calculateAngle(float range,float &degrees) {
  if(!CALIBRATED || JET_SPEED_M_S<=0 || range<=0) return false;
  const float g=9.81f, v2=JET_SPEED_M_S*JET_SPEED_M_S;
  float discriminant=v2*v2-g*(g*range*range+2*TARGET_HEIGHT_M*v2);
  if(discriminant<0) return false;
  degrees=atanf((v2-sqrtf(discriminant))/(g*range))*180.0f/PI;
  return isfinite(degrees) && degrees>=MIN_ANGLE && degrees<=MAX_ANGLE;
}

// SERVICE FUNCTION
void serviceDrive() {   }

// SERVICE FUNCTION
void serviceSearch() {   }

// SERVICE FUNCTION
void serviceStateMachine() {
  if(!startRaw && !startStable) releasedBeforeStart=true;
  switch(state) {
    case WAIT_START:

      if(startButtonPressed()) transition(HOMING);
      break;
    case HOMING:

      if(switchStable) {
        position=target=0; homed=true;
        transition(SEARCH);
        aimAt(MAX_ANGLE);
      } else if(elapsed(HOMING_TIMEOUT_MS)) transition(FAULT);
      break;
    case SEARCH:

      if(startButtonPressed()) { transition(HOMING); break; }

      serviceSearch();
      if(motionDone() && switchStable) { transition(FAULT); break; }
      if(motionDone() && targetVisible()) transition(CHARACTERIZE);
      break;
    case CHARACTERIZE:

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

      if(frameId!=checkFrame) {
        checkFrame=frameId;
        if(fire) { transition(CHARACTERIZE); break; }
        if(!checkSawFrame) { checkSawFrame=true; clearSince=millis(); }
        if(millis()-clearSince>=CLEAR_MS) transition(SEARCH);
      }
      break;

    case FAULT: pump(false); break;
  }
  previousStart=startStable;
}

// SERVICE FUNCTION
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

// SERVICE FUNCTION
void loop() {
  checkSerial(); checkStartButton(); checkLimitSwitch(); serviceStateMachine(); serviceStepper();
}
