// Run on a machine with g++:
// g++ -std=c++11 -I firmware/esp32/tests firmware/esp32/tests/mock_base_test.cpp -o mock_base_test
// ./mock_base_test
#include "Arduino.h"
#include <assert.h>
#include <math.h>
#include <string.h>

// Arduino IDE normally generates these forward declarations for the .ino.
bool targetVisible();
bool targetInRange();
bool flameConfirmed();
bool flameSensorReady();
void aimAt(float);
#include "../FireTurret/FireTurret.ino"

static void send(const char *line) {
  char copy[96]; snprintf(copy,sizeof(copy),"%s",line); acceptLine(copy);
}

static void startRun() {
  state=WAIT_START; mockRun=false;
  send("MOCK_START");
  assert(mockRun && state==SEARCH && homed);
}

static void observe(uint32_t sequence, float error, bool detected=true) {
  char message[96];
  snprintf(message,sizeof(message),"TARGET,%lu,%d,%.4f,%.4f,%.3f",
           (unsigned long)sequence,int(detected),detected?0.8:0.0,
           detected?0.95:0.0,static_cast<double>(error));
  send(message);
}

static void tick() {
  ++mockMillis;
  serviceFlameSensor(); serviceStateMachine(); serviceMockBase(); serviceStepper();
}

static void reachAim(float targetBearing) {
  uint32_t sequence=0;
  bool sawMotion=false;
  uint32_t deadline=millis()+10000;
  while(millis()<deadline && !aimPending) {
    if(millis()%100==0) observe(++sequence,targetBearing-baseHeadingDeg);
    tick();
    if(mockLeftSpeed!=0) {
      sawMotion=true;
      assert(mockLeftSpeed*mockRightSpeed<0);
      assert((mockLeftSpeed<0)==(targetBearing>0));
    }
  }
  assert(sawMotion && aimPending && state==CALCULATE_AIM && baseAligned());
  assert(fabsf(baseHeadingDeg-targetBearing)<=BASE_TOLERANCE_DEG);
  assert(mockLeftSpeed==0 && mockRightSpeed==0);
  assert(fabsf(mockLeftM+mockRightM)<1e-5);
  // Confirm the nominal 80mm wheel geometry matches the integrated heading.
  float rightCounts=mockRightM*MOCK_COUNTS_PER_WHEEL_REV/(PI*MOCK_WHEEL_DIAMETER_M);
  float expected=baseHeadingDeg*MOCK_TRACK_WIDTH_M*MOCK_COUNTS_PER_WHEEL_REV/
                 (360*MOCK_WHEEL_DIAMETER_M);
  assert(fabsf(rightCounts-expected)<0.01);
}

int main() {
  setup();
  assert(!mockRun);  // The normal demo never silently bypasses real sensors.
  send("TARGET,1,1,0.8,0.95,20");
  assert(!haveVision && !mockRun);
  startRun();
  reachAim(20);
  // Old/malformed input must not keep the camera fresh.
  uint32_t previousVision=lastVision, previousFrame=frameId;
  observe(targetSequence,80);
  send("TARGET,999,1,0.8,0.95,nan");
  assert(lastVision==previousVision && frameId==previousFrame);

  char reply[80];
  snprintf(reply,sizeof(reply),"AIM_RESULT,%lu,15",(unsigned long)aimRequestId);
  send(reply);
  assert(aimSet && !aimPending);
  while(state!=SHOOT) {
    if(millis()%100==0) observe(targetSequence+1,20-baseHeadingDeg);
    tick();
    assert(state!=FAULT && millis()-demoMotionStarted<3000);
  }
  // Horizontal target shift interrupts the simulated shot.
  observe(targetSequence+1,15);
  tick();
  assert(state==FIRE_CHECK);

  // Unknown depth pauses active aiming immediately and cancels queued replies.
  startRun();
  reachAim(20);
  uint32_t cancelledRequest=aimRequestId;
  send("RANGE_UNKNOWN");
  assert(state==WAIT_RANGE && fire && !rangeKnown && !aimPending && !aimSet);
  assert(distanceM==0 && !targetInRange() && !baseAligned());
  assert(mockLeftSpeed==0 && mockRightSpeed==0 && position==target);
  snprintf(reply,sizeof(reply),"AIM_RESULT,%lu,15",(unsigned long)cancelledRequest);
  send(reply);
  assert(state==WAIT_RANGE && !aimSet);
  // Repeated missing ranges and later camera silence both stay idle, not Fault.
  send("RANGE_UNKNOWN");
  uint32_t pausedAt=millis();
  while(millis()-pausedAt<VISION_TIMEOUT_MS+100) tick();
  assert(state==WAIT_RANGE && !aimPending);
  observe(targetSequence+1,0);
  tick();
  assert(state==CHARACTERIZE && rangeKnown); // resumes from fresh range only
  send("RANGE_UNKNOWN");
  observe(targetSequence+1,0,false);
  tick();
  assert(state==SEARCH && !fire); // true no-fire has its own meaning

  // Unknown range interrupts Shoot, but must not clear a STOP fault or start gate.
  startRun();
  observe(1,0);
  state=SHOOT; aimSet=true;
  send("RANGE_UNKNOWN");
  assert(state==WAIT_RANGE && !aimSet && !rangeKnown);
  send("STOP");
  send("RANGE_UNKNOWN");
  assert(state==FAULT);
  state=WAIT_START; mockRun=false;
  send("RANGE_UNKNOWN");
  assert(state==WAIT_START && !rangeKnown);

  startRun();
  reachAim(20);
  // A request without a matching Pi reply must fault after its deadline.
  uint32_t requestTime=aimRequestedAt;
  while(state!=FAULT && millis()-requestTime<AIM_TIMEOUT_MS+100) {
    if(millis()%100==0) observe(targetSequence+1,20-baseHeadingDeg);
    tick();
  }
  assert(state==FAULT);

  startRun();
  reachAim(-20);
  assert(baseHeadingDeg<0 && mockLeftM>0 && mockRightM<0);
  send("STOP");
  assert(state==FAULT && mockLeftSpeed==0 && mockRightSpeed==0);
  send("MOCK_START");
  assert(state==FAULT);  // STOP remains latched.

  startRun();
  observe(1,30);
  while(millis()-lastVision<VISION_TIMEOUT_MS+10) tick();
  assert(state==SEARCH && mockLeftSpeed==0 && mockRightSpeed==0);

  startRun();
  // A stuck image continually reporting the same off-centre error cannot align.
  uint32_t startedAt=millis();
  while(state!=FAULT && millis()-startedAt<BASE_TIMEOUT_MS+1000) {
    if(millis()%100==0) observe(targetSequence+1,30);
    tick();
  }
  assert(state==FAULT);
  assert(outputWrites==0 && outputPinSetups==0);
  puts("PASS: mock base direction, geometry, alignment, aim handoff, stale input, timeouts, STOP, no outputs");
}
