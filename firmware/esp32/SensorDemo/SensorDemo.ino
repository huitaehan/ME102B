#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

constexpr bool DEMO_MODE=true;
constexpr uint32_t DEMO_MOTION_MS=1000, DEMO_FRAME_MS=100;

constexpr int START_PIN=21, HOME_PIN=33;
constexpr int FLAME_PIN=34;
constexpr bool FLAME_CALIBRATED=false;
constexpr bool FLAME_HIGH_MEANS_FIRE=true;
constexpr int FLAME_ON_THRESHOLD=2400, FLAME_OFF_THRESHOLD=2000;
constexpr uint32_t FLAME_SAMPLE_MS=20, FLAME_TIMEOUT_MS=200;
constexpr int FLAME_SAMPLE_COUNT=8;
constexpr bool ALLOW_SERIAL_START=false;

constexpr float MIN_ANGLE=5, MAX_ANGLE=85;
constexpr float MIN_RANGE_M=0.10f, MAX_RANGE_M=2.0f;

constexpr uint32_t VISION_TIMEOUT_MS=1500, HOMING_TIMEOUT_MS=15000;
constexpr uint32_t SHOOT_MS=3000, SETTLE_MS=500, CLEAR_MS=2000;

enum State { WAIT_START, HOMING, SEARCH, CHARACTERIZE, DRIVE,
             CALCULATE_AIM, SHOOT, FIRE_CHECK, FAULT };
State state=WAIT_START;
uint32_t entered=0, lastVision=0, frameId=0, checkFrame=0;
uint32_t switchChanged=0, clearSince=0;
bool pulseHigh=false, homed=false, fire=false, haveVision=false;
bool switchRaw=false, switchStable=false;
bool startRaw=false, startStable=false, previousStart=false;
uint32_t startChanged=0;
bool releasedBeforeStart=false, checkSawFrame=false, aimSet=false;
long position=0, target=0;
float distanceM=0, confidence=0;
int flameSamples[FLAME_SAMPLE_COUNT]={};
int flameIndex=0, flameCount=0, flameTotal=0, flameAverage=0;
bool infraredFire=false;
uint32_t lastFlameSample=0, lastFlameReport=0;
bool demoCamera=false, demoFire=false;
float demoDistanceM=0.8f;
uint32_t demoLastFrame=0, demoMotionStarted=0;
char input[96]; size_t inputLength=0; bool dropping=false;

// SERVICE FUNCTION: Print the simulated pump action without using any output pins.
void reportPump(bool on) {
  static bool lastOn=false;
  if(on!=lastOn) Serial.println(on?"SIMULATED ACTION: pump ON":"SIMULATED ACTION: pump OFF");
  lastOn=on;
}

// SERVICE FUNCTION: Stop the turret from moving.
void stopMotion() { target=position; }

// SERVICE FUNCTION: Change the state and turn the pump off.
void transition(State next) {
  reportPump(false); stopMotion(); state=next; entered=millis(); aimSet=false;
  if(next==HOMING) homed=false;
  if(next==FIRE_CHECK) { checkFrame=frameId; checkSawFrame=false; clearSince=0; }
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
  Serial.printf("INPUTS: start=%d home=%d analog=%d ready=%d confirmed=%d\n",
    int(startStable),int(switchStable),flameAverage,int(flameSensorReady()),int(flameConfirmed()));
}

// EVENT CHECKER: Check if all requested motor steps are finished.
bool motionDone() { return position==target && !pulseHigh; }

// EVENT CHECKER: Check if the timer has finished.
bool elapsed(uint32_t ms) { return millis()-entered>=ms; }

// SERVICE FUNCTION: Simulate motion completion after one second.
void serviceDemoMotion() {
  pulseHigh=false;
  if(homed && state!=HOMING && state!=FAULT && millis()-demoMotionStarted>=DEMO_MOTION_MS)
    position=target;
}

// SERVICE FUNCTION: Set the angle the nozzle should point to.
void aimAt(float angle) {
  target=lroundf(angle*100.0f);
  if(DEMO_MODE) {
    demoMotionStarted=millis();
    Serial.printf("DEMO ACTION: simulate nozzle aiming to %.1f degrees for 1 second.\n",angle);
  }
}

// SERVICE FUNCTION: Choose a simulated nozzle angle for the input-only demo.
bool calculateAngle(float range,float &degrees) {
  degrees=45.0f;
  Serial.println("SIMULATED ACTION: use a 45-degree aim; no trajectory is measured.");
  return range>0 && degrees>=MIN_ANGLE && degrees<=MAX_ANGLE;
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
      if(!aimSet) {
        float angle;
        if(!calculateAngle(distanceM,angle)) { transition(FAULT); break; }
        aimAt(angle); aimSet=true;
      }
      if(motionDone()) { transition(SHOOT); reportPump(true); }
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

    case FAULT: reportPump(false); break;
  }
  previousStart=startStable;
}

// SERVICE FUNCTION: Set up only the three sensor inputs and Serial Monitor.
void setup() {
  Serial.begin(115200); delay(1000);
  Serial.println("INPUT-ONLY DEMO: no motor or pump GPIOs are assigned."); Serial.flush();
  pinMode(HOME_PIN,INPUT_PULLUP);
  Serial.println("BOOT CHECK: home input GPIO33 ready"); Serial.flush();
  pinMode(START_PIN,INPUT_PULLUP);
  Serial.println("BOOT CHECK: start input GPIO21 ready"); Serial.flush();
  pinMode(FLAME_PIN,INPUT); analogReadResolution(12);
  Serial.println("BOOT CHECK: analog input GPIO34 ready"); Serial.flush();
  startRaw=startStable=previousStart=(digitalRead(START_PIN)==LOW);
  releasedBeforeStart=!startRaw; startChanged=millis(); entered=millis();
  Serial.println("SIMULATED CAMERA: SIM_NEAR, SIM_FAR, SIM_CLEAR, SIM_OFF; STOP enters Fault.");
  Serial.println("WAIT_START: open the start switch, then close it to start.");
}

// SERVICE FUNCTION: Read sensors, run the state machine, and print simulated actions.
void loop() {
  serviceFlameSensor(); checkSerial(); serviceDemoCamera(); checkStartButton(); checkLimitSwitch(); serviceStateMachine(); serviceDemoMotion(); reportFlameSensor();
  delay(1);
}