#include "hardware.h"

#include <SPI.h>
#include <Wire.h>

#include "MCP2515.h"
#include "ODriveEnums.h"
#include "config.h"

// ---------- Global hardware instances ----------
BARO baro;
IMU imu;
RGBLed led(LED_R, LED_G, LED_B);
ODriveCAN odrv(wrap_can_intf(CAN), ODRV_NODE_ID);
Heartbeat_msg_t lastHeartbeat;

// ---------- ODrive telemetry ----------
float motorvel = 0.0f;
float motorpos = 0.0f;
float motorcurrent = 0.0f;
float motor_cmd_pos = 0.0f;
float batteryVoltage = 0.0f;
uint32_t axisError = 0;

static uint32_t lastFeedbackMs = 0;
static uint32_t eeLastPairMs = 0;
static uint32_t eePairSequence = 0;
bool eeTelemetryFresh() { return eePairSequence && (uint32_t)(millis()-eeLastPairMs) < 200; }
uint32_t eeTelemetrySequence() { return eePairSequence; }
static uint32_t lastHeartbeatMs = 0;
static uint32_t lastEnableRequestMs = 0;
static uint32_t lastPositionDiagnosticMs = 0;
static uint32_t lastTelemetryRequestMs = 0;
static uint32_t lastTelemetryDiagnosticMs = 0;
static uint32_t lastBatterySampleMs = 0;
static uint32_t lastBatteryDiagnosticMs = 0;
static uint32_t odriveRxFrames = 0;
static uint32_t odriveTxFailures = 0;
static uint32_t odriveTelemetryTimeouts = 0;
// ODrive heartbeat rate is configurable and is commonly 1 Hz.  350 ms made
// a healthy controller appear stale between heartbeat frames.
static constexpr uint32_t ODRIVE_HEARTBEAT_TIMEOUT_MS = 2500;
static constexpr uint32_t ODRIVE_FEEDBACK_TIMEOUT_MS = 350;
static constexpr uint32_t ODRIVE_ENABLE_RETRY_MS = 100;
static bool preflightPassed = false;
static bool batteryTelemetryEnabled = false;

// MCP2515 callbacks execute from the interrupt handler.  Do not perform SPI,
// Serial, or ODrive decoding there; queue the frame and process it in the main
// loop instead (the standalone 9967b65 test used this same pattern).
static constexpr uint8_t CAN_RX_QUEUE_SIZE = 16;
static volatile CanMsg canRxQueue[CAN_RX_QUEUE_SIZE];
static volatile uint8_t canRxHead = 0;
static volatile uint8_t canRxTail = 0;
static volatile uint32_t canRxDropped = 0;

// Use the MCP2515 library's receive callback, as in the known-good 878c9ac
// implementation.  Relying only on parsePacket() polling can miss the INT
// driven receive path on this library/board combination.
static void canCallback(int packetSize) {
  uint8_t length = static_cast<uint8_t>(packetSize > 8 ? 8 : packetSize);
  uint8_t nextHead = static_cast<uint8_t>((canRxHead + 1) % CAN_RX_QUEUE_SIZE);
  if (nextHead == canRxTail) {
    ++canRxDropped;
    while (CAN.available()) CAN.read();
    return;
  }

  volatile CanMsg& msg = canRxQueue[canRxHead];
  msg.id = static_cast<uint32_t>(CAN.packetId());
  msg.len = length;
  for (uint8_t i = 0; i < length; ++i) msg.buffer[i] = CAN.read();
  while (CAN.available()) CAN.read();
  canRxHead = nextHead;
}

void onCanMessage(const CanMsg& msg) {
  ++odriveRxFrames;
  odrv.onReceive(msg.id, msg.len, msg.buffer);
}

static void drainCanRxQueue() {
  while (true) {
    CanMsg msg;
    noInterrupts();
    if (canRxTail == canRxHead) {
      interrupts();
      return;
    }
    const volatile CanMsg& queued = canRxQueue[canRxTail];
    msg.id = queued.id;
    msg.len = queued.len;
    for (uint8_t i = 0; i < msg.len; ++i) msg.buffer[i] = queued.buffer[i];
    canRxTail = static_cast<uint8_t>((canRxTail + 1) % CAN_RX_QUEUE_SIZE);
    interrupts();
    onCanMessage(msg);
  }
}

static void odriveFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  motorpos = msg.Pos_Estimate;
  motorvel = msg.Vel_Estimate;
  lastFeedbackMs = millis();
}

static void odriveCurrents(Get_Iq_msg_t& msg, void* user_data) {
  motorcurrent = msg.Iq_Measured;
}

static void odriveHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  lastHeartbeat = msg;
  axisError = msg.Axis_Error;
  lastHeartbeatMs = millis();
}

void ledWrite(float r, float g, float b) {
  led.setColor(r, g, b);
}

bool hardwarePreflightCheck() {
  bool imuOk = imu.isInitialized();
  bool baroOk = baro.isConnected();
  bool imuSampleOk = false;
  bool baroSampleOk = false;

  for (int i = 0; i < 10; ++i) {
    imu.update();
    imuSampleOk = imu.hasValidSample();
    if (baro.update()) baroSampleOk = baro.hasValidSample();
    delay(5);
  }

  // Exercise CAN RX and the heartbeat before declaring ODrive usable.
  serviceOdrive();
  uint32_t deadline = millis() + 500;
  while ((int32_t)(millis() - deadline) < 0) {
    serviceOdrive();
    if (odriveHeartbeatFresh() && axisError == 0) break;
    delay(5);
  }
  bool odriveOk = odriveHeartbeatFresh() && axisError == 0;

  Serial.print("PREFLIGHT: IMU=");
  Serial.print(imuOk && imuSampleOk ? "OK" : "FAIL");
  Serial.print(" ACCEL_XYZ_GYRO_XYZ=");
  Serial.print(imuSampleOk ? "OK" : "FAIL");
  Serial.print(" DPS368=");
  Serial.print(baroOk && baroSampleOk ? "OK" : "FAIL");
  Serial.print(" ODRIVE=");
  Serial.println(odriveOk ? "OK" : "FAIL");
  preflightPassed = imuOk && imuSampleOk && baroOk && baroSampleOk && odriveOk;
  return preflightPassed;
}

bool hardwarePreflightPassed() { return preflightPassed; }

void setupHardware() {
  Serial.begin(115200);

  analogReadResolution(12);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  updateBatteryVoltage();

  // Aqua means the board is alive; white is reserved for descent.
  ledWrite(0.0f, 0.15f, 0.10f);

  // DPS368 barometer (I2C). begin() probes both valid DPS368 addresses.
  if (!baro.begin()) {
    Serial.println("WARNING: DPS368 initialization failed");
    baro.printDiagnostics();
  } else {
    Serial.print("DPS368 OK, baseline pressure Pa: ");
    Serial.println(baro.getBaselinePressure(), 2);
    baro.printDiagnostics();
  }

  // ICM42688 IMU (SPI1)
  while (!imu.begin()) {
    ledWrite(0.1f, 0.0f, 0.0f);
    Serial.println("IMU not found!");
    delay(1000);
  }

  // CAN (MCP2515 on SPI0)
  SPI.setMISO(CAN_MISO);
  SPI.setMOSI(CAN_MOSI);
  SPI.setSCK(CAN_SCK);
  SPI.begin(false);

  odrv.onFeedback(odriveFeedback, NULL);
  odrv.onStatus(odriveHeartbeat, NULL);
  odrv.onCurrents(odriveCurrents, NULL);

  CAN.setPins(CAN_CS, CAN_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);

  if (!CAN.begin(CAN_BAUDRATE)) {
    while (1) {
      ledWrite(0.1f, 0.0f, 0.0f);
      Serial.println("CAN not found!");
      delay(1000);
    }
  }

  // Use the same polling receive path as the known-good standalone test.
  // Do not install the interrupt callback here: ODriveCAN::request() waits
  // through pumpEvents(), and an ISR queue would hide RTR responses from that
  // synchronous wait until after its timeout.
  // CAN.onReceive(canCallback);

  Serial.println("CAN setup complete");

  // Normal flight control uses position commands. The ODrive must already be
  // calibrated and able to enter closed-loop control.
  odrv.clearErrors();
  odrv.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
                         ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  #ifdef EE_BENCH
  odrv.setState(ODriveAxisState::AXIS_STATE_IDLE);
#else
  odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
#endif
}

void updateBatteryVoltage() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastBatterySampleMs) < 10) return;
  lastBatterySampleMs = now;
  const float pinVoltage = analogRead(BATTERY_VOLTAGE_PIN) *
                           (BATTERY_ADC_REFERENCE_V / BATTERY_ADC_MAX);
  batteryVoltage = pinVoltage *
                   ((BATTERY_DIVIDER_R1_OHM + BATTERY_DIVIDER_R2_OHM) /
                    BATTERY_DIVIDER_R2_OHM);
}

void setBatteryTelemetryEnabled(bool enabled) {
  batteryTelemetryEnabled = enabled;
  lastBatteryDiagnosticMs = millis();
}

void EnableOdrv() {
  // Kept for API compatibility. CAN/MCP2515 access is owned by core 0;
  // serviceOdrive() in loop() performs the enable/retry operation there.
}

bool odriveHeartbeatFresh() {
  return lastHeartbeatMs != 0 &&
         (uint32_t)(millis() - lastHeartbeatMs) <= ODRIVE_HEARTBEAT_TIMEOUT_MS;
}

bool odriveReady() {
  return odriveHeartbeatFresh() && axisError == 0;
}

bool odriveCurrentLimitExceeded() {
  return fabsf(motorcurrent) >= GROUND_TEST_CURRENT_LIMIT_A;
}

uint32_t odriveRxFrameCount() { return odriveRxFrames; }
uint32_t odriveTxFailureCount() { return odriveTxFailures; }
uint32_t odriveTelemetryTimeoutCount() { return odriveTelemetryTimeouts; }

void serviceOdrive() {
  updateBatteryVoltage();
  if (batteryTelemetryEnabled) {
    uint32_t batteryNow = millis();
    if ((uint32_t)(batteryNow - lastBatteryDiagnosticMs) >= 1000) {
      lastBatteryDiagnosticMs = batteryNow;
      Serial.print("BATTERY_VOLTAGE:");
      Serial.println(batteryVoltage, 2);
    }
  }
  // Poll the MCP2515, matching the known-good standalone implementation.
  // This also lets synchronous RTR telemetry requests receive their response.
  pumpEvents(CAN);

  // Request telemetry explicitly. A callback alone does not make ODrive
  // publish Get_Iq or Encoder_Estimates; these are RTR responses unless
  // cyclic CAN messages have been configured on the controller.
  uint32_t telemetryNow = millis();
  if ((uint32_t)(telemetryNow - lastTelemetryRequestMs) >= 20) {
    lastTelemetryRequestMs = telemetryNow;
    Get_Encoder_Estimates_msg_t feedback;
    bool feedbackReceived = odrv.getFeedback(feedback, 5);
    if (feedbackReceived) {
      // ODrive reports both fields in turns: position [rev], velocity [rev/s].
      // Never overwrite the last good sample with a timed-out default object.
      motorpos = feedback.Pos_Estimate;
      motorvel = feedback.Vel_Estimate;
    } else {
      ++odriveTelemetryTimeouts;
    }
    Get_Iq_msg_t currents;
    bool currentsReceived = odrv.getCurrents(currents, 5);
    if (currentsReceived) {
      motorcurrent = currents.Iq_Measured;
      if (feedbackReceived) { eeLastPairMs=millis(); ++eePairSequence; }
    } else {
      ++odriveTelemetryTimeouts;
    }
    if ((uint32_t)(telemetryNow - lastTelemetryDiagnosticMs) >= 1000) {
      lastTelemetryDiagnosticMs = telemetryNow;
      Serial.print("ODRIVE_TELEMETRY: feedback=");
      Serial.print(feedbackReceived ? "OK" : "TIMEOUT");
      Serial.print(" pos_rev=");
      Serial.print(motorpos, 4);
      Serial.print(" vel_rev_s=");
      Serial.print(motorvel, 4);
      Serial.print(" iq=");
      Serial.print(currentsReceived ? "OK" : "TIMEOUT");
      Serial.print(" iq_A=");
      Serial.print(motorcurrent, 4);
      Serial.print(" rx_frames=");
      Serial.print(odriveRxFrames);
      Serial.print(" rtr_timeouts=");
      Serial.print(odriveTelemetryTimeouts);
      Serial.print(" tx_failures=");
      Serial.println(odriveTxFailures);
    }
  }

  uint32_t nowMs = millis();
  bool feedbackFresh = lastFeedbackMs != 0 &&
                       (uint32_t)(nowMs - lastFeedbackMs) <=
                           ODRIVE_FEEDBACK_TIMEOUT_MS;
  #ifndef EE_BENCH
  if (odriveHeartbeatFresh() &&
      (!feedbackFresh || lastHeartbeat.Axis_State !=
                              ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) &&
      (uint32_t)(nowMs - lastEnableRequestMs) >= ODRIVE_ENABLE_RETRY_MS) {
    lastEnableRequestMs = nowMs;
    odrv.clearErrors();
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
#endif
}

void odrvPosition(float pos) {
  motor_cmd_pos = pos;
  // Heartbeat is status telemetry, not a transmit prerequisite.  The
  // standalone 9967b65 controller sent setpoints even while waiting for the
  // first heartbeat.  Gating here made the motor hold its old position
  // forever when RX/heartbeat delivery was unavailable.
  bool sent = odrv.setPosition(pos);
  if (!sent) ++odriveTxFailures;
  // Report command transmission once per second. This distinguishes a
  // working sweep from a CAN TX failure without flooding the flight log.
  uint32_t nowMs = millis();
  if (!sent || (uint32_t)(nowMs - lastPositionDiagnosticMs) >= 1000) {
    lastPositionDiagnosticMs = nowMs;
    Serial.print("ODRIVE_POS_CMD: target=");
    Serial.print(pos, 3);
    Serial.print(" tx=");
    Serial.print(sent ? "OK" : "FAIL");
    Serial.print(" heartbeat=");
    Serial.print(odriveHeartbeatFresh() ? "FRESH" : "STALE");
    Serial.print(" axis_state=");
    Serial.print(lastHeartbeat.Axis_State);
    Serial.print(" error=0x");
    Serial.println(axisError, HEX);
  }
}
