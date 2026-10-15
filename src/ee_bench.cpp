#include "ee_bench.h"
#ifdef EE_BENCH
#include "hardware.h"
#include "config.h"
#include <stdio.h>

static bool running = false;
static int phase = 0;
static float startPos, endPos, limitA;
static uint32_t started, phaseStarted, lastClient, lastSequence, timeoutMs;
static bool moving = false;
static uint32_t txFailures;

static void finish(const char* reason) {
  // Idle stops powered actuation; it is not a mechanical brake.
  odrv.setState(ODriveAxisState::AXIS_STATE_IDLE);
  running = false;
  moving = false;
  Serial.print("EE:DONE:"); Serial.println(reason);
}

bool eeBenchCommand(const String& command) {
  if (!command.startsWith("EE ")) return false;
  if (command == "EE HELLO") {
    Serial.println("EE:HELLO:1");
  } else if (command == "EE STOP") {
    finish("ABORTED");
  } else if (command == "EE PING") {
    lastClient = millis();
    Serial.println("EE:PONG");
  } else if (command.startsWith("EE RUN ")) {
    float speed; unsigned long duration; char extra;
    float a, b, current;
    int count = sscanf(command.c_str(), "EE RUN %f %f %f %f %lu %c", &a, &b, &speed, &current, &duration, &extra);
    if (running || count != 5 || !isfinite(a) || !isfinite(b) || !isfinite(speed) || !isfinite(current) ||
        a < MOTOR_MAX || a > MOTOR_MIN || b < MOTOR_MAX || b > MOTOR_MIN || fabsf(a-b) < 1.0f ||
        speed <= 0 || speed > GROUND_TEST_VELOCITY_LIMIT || current <= 0 || current > GROUND_TEST_CURRENT_LIMIT_A ||
        duration < 1000 || duration > 120000) {
      Serial.println("EE:ERROR:INVALID_PARAMETERS_OR_BUSY"); return true;
    }
    if (!odriveReady() || !eeTelemetryFresh() || fabsf(motorpos-a) > 0.5f) {
      Serial.println("EE:ERROR:NOT_READY_OR_NOT_AT_START"); return true;
    }
    startPos=a; endPos=b; limitA=current; timeoutMs=duration;
    odrv.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    odrv.setLimits(speed, current);
    // Replace any target left by an interrupted trial before enabling power.
    odrvPosition(startPos);
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    phase=0; moving=false; running=true;
    started=phaseStarted=lastClient=millis(); lastSequence=eeTelemetrySequence();
    txFailures=odriveTxFailureCount();
    Serial.println("EE:STARTED");
  } else {
    Serial.println("EE:ERROR:UNKNOWN_COMMAND");
  }
  return true;
}

void eeBenchUpdate() {
  if (!running) return;
  uint32_t now=millis();
  if (now-lastClient > 2000) { finish("CLIENT_TIMEOUT"); return; }
  if (axisError || !odriveHeartbeatFresh() || !eeTelemetryFresh()) { finish("TELEMETRY_OR_AXIS_FAULT"); return; }
  if (odriveTxFailureCount() != txFailures) { finish("CAN_TX_FAULT"); return; }
  if (moving && lastHeartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) { finish("AXIS_LEFT_CLOSED_LOOP"); return; }
  if (now-phaseStarted > timeoutMs) { finish("MOTION_TIMEOUT"); return; }
  if (fabsf(motorcurrent) >= limitA * 0.98f) { finish("CURRENT_LIMIT"); return; }
  if (motorpos < MOTOR_MAX-0.5f || motorpos > MOTOR_MIN+0.5f) { finish("POSITION_LIMIT"); return; }
  if (!moving) {
    if (lastHeartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) return;
    odrvPosition(phase == 0 ? endPos : startPos);
    moving=true;
  }
  uint32_t seq=eeTelemetrySequence();
  if (seq != lastSequence) {
    lastSequence=seq;
    Serial.print("EE:DATA:"); Serial.print(now-started); Serial.print(',');
    Serial.print(phase == 0 ? "outbound" : "return"); Serial.print(',');
    Serial.print(motorpos,6); Serial.print(','); Serial.print(motorvel,6); Serial.print(',');
    Serial.print(motorcurrent,6); Serial.print(','); Serial.print(batteryVoltage,4); Serial.print(',');
    Serial.println(seq);
    float target=phase == 0 ? endPos : startPos;
    if (fabsf(motorpos-target) <= 0.2f && fabsf(motorvel) < 0.2f) {
      if (phase == 1) { finish("COMPLETE"); return; }
      phase=1; phaseStarted=now; odrvPosition(startPos);
    }
  }
}
#else
bool eeBenchCommand(const String&) { return false; }
void eeBenchUpdate() {}
#endif
