#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

#include "ODriveCAN.h"
#include "ODriveMCPCAN.hpp"
#include "baro.h"
#include "imu.h"
#include "rgb.h"

// ---------- RGB LED PINS ----------
#define LED_R 24
#define LED_G 22
#define LED_B 23

// ---------- CAN (MCP2515 on SPI0) PINS ----------
#define CAN_MISO 16
#define CAN_CS 17
#define CAN_SCK 18
#define CAN_MOSI 19
#define CAN_INT 20
#define MCP2515_CLK_HZ 20000000  // 20 MHz crystal (see lib/CAN, a fork of
                                 // sandeepmistry/CAN with 20 MHz support)
#define CAN_BAUDRATE 250000
#define ODRV_NODE_ID 0

// Battery monitor: the divider is R1 from battery+ to GPIO27 and R2 from
// GPIO27 to ground. ADC1 on RP2350 is GPIO27. Keep the resistor values here so
// the logged voltage conversion is explicit and easy to calibrate.
#define BATTERY_VOLTAGE_PIN 27
#define BATTERY_ADC_MAX 4095.0f
#define BATTERY_ADC_REFERENCE_V 3.3f
#define BATTERY_DIVIDER_R1_OHM 17800.0f
#define BATTERY_DIVIDER_R2_OHM 10000.0f

// Global hardware instances (defined in hardware.cpp)
extern BARO baro;
extern IMU imu;
extern RGBLed led;
extern ODriveCAN odrv;

// ODrive telemetry (updated from CAN callbacks)
extern float motorvel;
extern float motorpos;
extern float motorcurrent;
extern float motor_cmd_pos;
extern float batteryVoltage;
extern uint32_t axisError;
extern Heartbeat_msg_t lastHeartbeat;

void setupHardware();
void updateBatteryVoltage();
void setBatteryTelemetryEnabled(bool enabled);
void ledWrite(float r, float g, float b);
bool hardwarePreflightCheck();
bool hardwarePreflightPassed();

// ODrive helpers
void EnableOdrv();
void odrvPosition(float pos);
void serviceOdrive();
bool odriveHeartbeatFresh();
bool odriveReady();
bool odriveCurrentLimitExceeded();
uint32_t odriveRxFrameCount();
uint32_t odriveTxFailureCount();
uint32_t odriveTelemetryTimeoutCount();

#endif  // HARDWARE_H
