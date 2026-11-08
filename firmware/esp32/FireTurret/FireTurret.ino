#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

constexpr bool DEMO_MODE=true;
constexpr uint32_t DEMO_MOTION_MS=1000, DEMO_FRAME_MS=100;

constexpr int STEP_PIN=25, DIR_PIN=27, ENABLE_PIN=22;
// Adafruit ESP32 Feather V2: MI/MISO=21, home=33, A2=34.
constexpr int START_PIN=21, HOME_PIN=33, PUMP_PIN=32;
constexpr int FLAME_PIN=34;
constexpr bool FLAME_CALIBRATED=false;
constexpr bool FLAME_HIGH_MEANS_FIRE=true;
constexpr int FLAME_ON_THRESHOLD=2400, FLAME_OFF_THRESHOLD=2000;
constexpr uint32_t FLAME_SAMPLE_MS=20, FLAME_TIMEOUT_MS=200;
constexpr int FLAME_SAMPLE_COUNT=8;
constexpr bool ALLOW_SERIAL_START=false;
constexpr bool HOME_DIR_HIGH=true, PUMP_ACTIVE_HIGH=true;

constexpr bool CALIBRATED=false;
constexpr float STEPS_PER_DEGREE=200.0f*16.0f/360.0f;
constexpr float MIN_ANGLE=5, MAX_ANGLE=85;
constexpr float MIN_RANGE_M=0.10f, MAX_RANGE_M=2.0f;
constexpr uint32_t AIM_TIMEOUT_MS=3000;

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
uint32_t aimRequestId=0, aimRequestedAt=0;
bool aimPending=false;
float aimDistanceM=0;
int flameSamples[FLAME_SAMPLE_COUNT]={};
int flameIndex=0, flameCount=0, flameTotal=0, flameAverage=0;
bool infraredFire=false;
uint32_t lastFlameSample=0, lastFlameReport=0;
bool demoCamera=false, demoFire=false;
float demoDistanceM=0.8f;
uint32_t demoLastFrame=0, demoMotionStarted=0;
char input[96]; size_t inputLength=0; bool dropping=false;

// SERVICE FUNCTION: Turn the water pump on or off.
void pump(bool on) {
  if(DEMO_MODE) return;
  digitalWrite(PUMP_PIN, (on==PUMP_ACTIVE_HIGH)?HIGH:LOW);
}

// SERVICE FUNCTION: Stop the turret from moving.
void stopMotion() { target=position; }

// SERVICE FUNCTION: Change the state and turn the pump off.
void transition(State next) {
  pump(false); stopMotion(); state=next; entered=millis(); aimSet=false; aimPending=false;
  if(next==HOMING) homed=false;
  if(next==FIRE_CHECK) { checkFrame=frameId; checkSawFrame=false; clearSince=0; }
  if(next==FAULT && !DEMO_MODE) digitalWrite(ENABLE_PIN,HIGH);
  Serial.printf("STATE,%d\n",int(next));
  if(DEMO_MODE) {
    const char *names[]={"Wait for start","Homing","Search","Characterize","Drive",
                         "Calculate & aim","Shoot","Fire check","Fault"};
    Serial.printf("DEMO STATE: %s\n",names[int(next)]);
    if(next==HOMING) Serial.println("DEMO ACTION: motor disabled; press the real home switch.");
    if(next==DRIVE) Serial.println("DEMO ACTION: driving is not implemented.");
    if(next==SHOOT) Serial.println("DEMO ACTION: pump ON simulated; physical pump stays OFF.");
    else Serial.println("DEMO ACTION: pump OFF; actuator outputs disabled.");
  }
}

// EVENT CHECKER: Read the start button and filter out bouncing.
void checkStartButton() {
  bool pressed=digitalRead(START_PIN)==LOW;
  if(pressed!=startRaw) { startRaw=pressed; startChanged=millis(); }
  if(millis()-startChanged>=30) startStable=startRaw;
}

// EVENT CHECKER: Check if the start button was pressed again.
bool startButtonPressed() {
  return releasedBeforeStart && startStable && !previousStart;
}

// EVENT CHECKER: Check if the turret reached the 90-degree limit switch.
void checkLimitSwitch() {
  bool pressed=digitalRead(HOME_PIN)==LOW;
  if(pressed!=switchRaw) { switchRaw=pressed; switchChanged=millis(); }
  if(millis()-switchChanged>=30) switchStable=switchRaw;
}

// EVENT CHECKER: Read a camera message or a start/stop command.
void acceptLine(char *line) {
  if(!strcmp(line,"CAPS")) {
    Serial.println("CAPS,FIRE,AIM_REQUEST,AIM_RESULT,AIM_ERROR,STOP,ELEVATION_ONLY");
    return;
  }
  // The merged reference sketch acknowledged these without executing motion.
  // Reject them explicitly until the corresponding hardware services exist.
  const char *unsupported[]={"HOME","AIM_SET","YAW","CAM","DRIVE","GOTO",
                             "TRAJ_BEGIN","LEG","TRAJ_RUN"};
  for(const char *command:unsupported) {
    size_t length=strlen(command);
    if(!strncmp(line,command,length) && (line[length]==0 || line[length]==',')) {
      Serial.printf("ERR,%s,NOT_IMPLEMENTED\n",command);
      return;
    }
  }
  if(!strcmp(line,"STOP")) { transition(FAULT); return; }
  if(!strcmp(line,"START")) {
    if(ALLOW_SERIAL_START && state==WAIT_START) transition(HOMING);
    return;
  }
  if(DEMO_MODE && (!strcmp(line,"SIM_NEAR") || !strcmp(line,"SIM_FAR") ||
                   !strcmp(line,"SIM_CLEAR") || !strcmp(line,"SIM_OFF"))) {
    demoCamera=strcmp(line,"SIM_OFF")!=0;
    demoFire=strcmp(line,"SIM_CLEAR")!=0;
    demoDistanceM=!strcmp(line,"SIM_FAR")?3.0f:0.8f;
    Serial.printf("DEMO CAMERA: %s (simulated input)\n",line);
    return;
  }
  unsigned long replyId=0; float angle=0; int used=0;
  if(sscanf(line,"AIM_ERROR,%lu%n",&replyId,&used)==1 && !line[used]) {
    if(state==CALCULATE_AIM && aimPending && replyId==aimRequestId) transition(FAULT);
    return;
  }
  used=0;
  if(sscanf(line,"AIM_RESULT,%lu,%f%n",&replyId,&angle,&used)==2 && !line[used]) {
    if(state!=CALCULATE_AIM || !aimPending || replyId!=aimRequestId) return;
    if(millis()-aimRequestedAt>=AIM_TIMEOUT_MS || !isfinite(angle) ||
       angle<MIN_ANGLE || angle>MAX_ANGLE) { transition(FAULT); return; }
    if(!targetVisible() || !targetInRange() || !flameConfirmed() ||
       fabsf(distanceM-aimDistanceM)>0.05f) { transition(CHARACTERIZE); return; }
    aimPending=false; aimAt(angle); aimSet=true;
    return;
  }
  int detected=-1, consumed=0; float d=0,c=0;
  if(sscanf(line,"FIRE,%d,%f,%f%n",&detected,&d,&c,&consumed)!=3 || line[consumed]) return;
  if((detected!=0 && detected!=1) || !isfinite(d) || !isfinite(c) || c<0 || c>1) return;
  if(detected && (d<=0 || c<=0.40f)) return;
  demoCamera=false;
  fire=detected; distanceM=d; confidence=c;
  haveVision=true; lastVision=millis(); ++frameId;
}

// EVENT CHECKER: Check for new messages from the Raspberry Pi.
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

// SERVICE FUNCTION: Send simulated camera frames only when demo commands enable them.
void serviceDemoCamera() {
  if(!DEMO_MODE || !demoCamera || millis()-demoLastFrame<DEMO_FRAME_MS) return;
  demoLastFrame=millis();
  fire=demoFire; distanceM=demoDistanceM; confidence=demoFire?0.9f:0.0f;
  haveVision=true; lastVision=millis(); ++frameId;
}

// EVENT CHECKER: Check if the camera data is recent.
bool visionFresh() { return haveVision && millis()-lastVision<VISION_TIMEOUT_MS; }

// EVENT CHECKER: Check if the camera detects fire.
bool targetVisible() { return visionFresh() && fire; }

// EVENT CHECKER: Check if the fire is within shooting range.
bool targetInRange() { return distanceM>=MIN_RANGE_M && distanceM<=MAX_RANGE_M; }

// SERVICE FUNCTION: Read and average the analog flame sensor without waiting.
void serviceFlameSensor() {
  uint32_t now=millis();
  if(now-lastFlameSample<FLAME_SAMPLE_MS) return;
  lastFlameSample=now;
  flameTotal-=flameSamples[flameIndex];
  flameSamples[flameIndex]=analogRead(FLAME_PIN);
  flameTotal+=flameSamples[flameIndex];
  flameIndex=(flameIndex+1)%FLAME_SAMPLE_COUNT;
  if(flameCount<FLAME_SAMPLE_COUNT) ++flameCount;
  flameAverage=flameTotal/flameCount;
  if(FLAME_HIGH_MEANS_FIRE) {
    if(flameAverage>=FLAME_ON_THRESHOLD) infraredFire=true;
    else if(flameAverage<=FLAME_OFF_THRESHOLD) infraredFire=false;
  } else {
    if(flameAverage<=FLAME_ON_THRESHOLD) infraredFire=true;
    else if(flameAverage>=FLAME_OFF_THRESHOLD) infraredFire=false;
  }
}

// EVENT CHECKER: Check if the flame sensor is calibrated and has recent readings.
bool flameSensorReady() {
  bool thresholdsValid=FLAME_ON_THRESHOLD>=0 && FLAME_ON_THRESHOLD<=4095 &&
    FLAME_OFF_THRESHOLD>=0 && FLAME_OFF_THRESHOLD<=4095 &&
    (FLAME_HIGH_MEANS_FIRE ? FLAME_ON_THRESHOLD>FLAME_OFF_THRESHOLD :
                            FLAME_ON_THRESHOLD<FLAME_OFF_THRESHOLD);
  return FLAME_CALIBRATED && thresholdsValid && flameCount==FLAME_SAMPLE_COUNT &&
    millis()-lastFlameSample<FLAME_TIMEOUT_MS;
}

// EVENT CHECKER: Check if the infrared reading supports the camera detection.
bool flameConfirmed() { return flameSensorReady() && infraredFire; }

// SERVICE FUNCTION: Print analog readings so the flame threshold can be calibrated.
void reportFlameSensor() {
  if(millis()-lastFlameReport<500) return;
  lastFlameReport=millis();
  Serial.printf("FLAME,%d,%d,%d\n",flameAverage,int(flameSensorReady()),int(flameConfirmed()));
}

// EVENT CHECKER: Check if all requested motor steps are finished.
bool motionDone() { return position==target && !pulseHigh; }

// EVENT CHECKER: Check if the timer has finished.
bool elapsed(uint32_t ms) { return millis()-entered>=ms; }

// SERVICE FUNCTION: Send step pulses to move the turret motor.
void serviceStepper() {
  if(DEMO_MODE) {
    pulseHigh=false;
    if(homed && state!=HOMING && state!=FAULT && millis()-demoMotionStarted>=DEMO_MOTION_MS)
      position=target;
    return;
  }
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

// SERVICE FUNCTION: Set the angle the nozzle should point to.
void aimAt(float angle) {
  target=lroundf((90.0f-angle)*STEPS_PER_DEGREE);
  if(DEMO_MODE) {
    demoMotionStarted=millis();
    Serial.printf("DEMO ACTION: simulate nozzle aiming to %.1f degrees for 1 second.\n",angle);
  }
}

// SERVICE FUNCTION: Calculate the nozzle angle needed to reach the fire.
bool calculateAngle(float range,float &degrees) {
  if(DEMO_MODE) {
    degrees=45.0f;
    Serial.println("DEMO ACTION: use simulated 45-degree aim; trajectory calibration bypassed.");
    return range>0 && degrees>=MIN_ANGLE && degrees<=MAX_ANGLE;
  }
  return false; // Physical trajectories are calculated by the Pi solver.
}

// SERVICE FUNCTION: Leave driving blank until the drive system is added.
void serviceDrive() {   }

// SERVICE FUNCTION: Leave physical scanning blank; the camera looks for fire.
void serviceSearch() {   }

// SERVICE FUNCTION: Run the current state and decide which state comes next.
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
      else if(flameConfirmed()) transition(targetInRange()?CALCULATE_AIM:DRIVE);
      break;
    case DRIVE:

      serviceDrive();
      if(!targetVisible()) transition(SEARCH);
      else if(targetInRange()) transition(CHARACTERIZE);
      break;
    case CALCULATE_AIM:

      if(!targetVisible()) { transition(SEARCH); break; }
      if(!flameConfirmed()) { transition(CHARACTERIZE); break; }
      if(!targetInRange()) { transition(DRIVE); break; }
      if(!DEMO_MODE && (aimPending || aimSet) && fabsf(distanceM-aimDistanceM)>0.05f) {
        transition(CHARACTERIZE); break;
      }
      if(!aimSet && !aimPending) {
        if(DEMO_MODE) {
          float angle;
          if(!calculateAngle(distanceM,angle)) { transition(FAULT); break; }
          aimAt(angle); aimSet=true;
        } else {
          if(!CALIBRATED) { transition(FAULT); break; }
          ++aimRequestId; aimDistanceM=distanceM; aimRequestedAt=millis(); aimPending=true;
          Serial.printf("AIM_REQUEST,%lu,%.4f,%.2f,%.2f\n",
                        (unsigned long)aimRequestId,aimDistanceM,MIN_ANGLE,MAX_ANGLE);
        }
      }
      if(aimPending && millis()-aimRequestedAt>=AIM_TIMEOUT_MS) { transition(FAULT); break; }
      if(aimSet && motionDone()) { transition(SHOOT); pump(true); }
      break;
    case SHOOT:

      if(!visionFresh() || !fire || !targetInRange()) transition(FIRE_CHECK);
      else if(elapsed(SHOOT_MS)) transition(FIRE_CHECK);
      break;
    case FIRE_CHECK:

      if(!visionFresh() || !flameSensorReady()) { transition(FAULT); break; }
      if(!elapsed(SETTLE_MS)) { checkFrame=frameId; break; }
      if(infraredFire) { checkSawFrame=false; clearSince=0; }

      if(frameId!=checkFrame) {
        checkFrame=frameId;
        if(fire) { transition(CHARACTERIZE); break; }
        if(infraredFire) break;
        if(!checkSawFrame) { checkSawFrame=true; clearSince=millis(); }
        if(millis()-clearSince>=CLEAR_MS) transition(SEARCH);
      }
      break;

    case FAULT: pump(false); break;
  }
  previousStart=startStable;
}

// SERVICE FUNCTION: Set up the pins and serial connection when the ESP32 starts.
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("BOOT CHECK 1: setup started"); Serial.flush();
  if(!DEMO_MODE) {
    pinMode(PUMP_PIN,OUTPUT); pump(false);
    pinMode(STEP_PIN,OUTPUT); digitalWrite(STEP_PIN,LOW);
    pinMode(DIR_PIN,OUTPUT); pinMode(ENABLE_PIN,OUTPUT);
    digitalWrite(ENABLE_PIN,LOW);
  }
  Serial.println("BOOT CHECK 2: actuator setup skipped in demo"); Serial.flush();
  pinMode(HOME_PIN,INPUT_PULLUP);
  Serial.println("BOOT CHECK 3: home input ready"); Serial.flush();
  pinMode(START_PIN,INPUT_PULLUP);
  Serial.println("BOOT CHECK 4: start input ready"); Serial.flush();
  pinMode(FLAME_PIN,INPUT); analogReadResolution(12);
  Serial.println("BOOT CHECK 5: analog input ready"); Serial.flush();
  startRaw=startStable=previousStart=(digitalRead(START_PIN)==LOW);
  releasedBeforeStart=!startRaw; startChanged=millis();
  entered=millis();
  if(DEMO_MODE) {
    Serial.println("DEMO MODE - actuator outputs disabled. Real flame calibration still required.");
    Serial.println("DEMO COMMANDS: SIM_NEAR, SIM_FAR, SIM_CLEAR, SIM_OFF, STOP");
    Serial.println("DEMO STATE: Wait for start");
  }
  Serial.println("WAIT_START: open the start switch, then close it to start.");
}

// SERVICE FUNCTION: Keep checking inputs and running the state machine and motor.
void loop() {
  serviceFlameSensor(); checkSerial(); serviceDemoCamera(); checkStartButton(); checkLimitSwitch(); serviceStateMachine(); serviceStepper(); reportFlameSensor();
  delay(1);
}
