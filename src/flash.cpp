#include "flash.h"

#include <LittleFS.h>

#include "orientation.h"
#include "state.h"
#include "ee_bench.h"

// Binary data structure (76 bytes per record).
// Keep this layout synchronized with desktop/main.js (<I16fII).
struct __attribute__((packed)) FlightRecord {
  uint32_t time_ms;
  float altitude_m;
  float velocity_ms;
  float accel_bias_ms2;
  float raw_accel_ms2;
  float vertical_accel_ms2;
  float raw_baro_m;
  float motor_pos;
  float motor_vel;
  float motor_cmd_pos;
  float roll_rad;
  float pitch_rad;
  float yaw_rad;
  float Cd;
  float desired_Cd;
  float motor_current;
  float battery_voltage;
  uint32_t state;
  uint32_t axis_error;
};

static_assert(sizeof(FlightRecord) == 76,
              "FlightRecord must match the <I16fII ground-station format");

// Single-producer (core 1 estimator), single-consumer (core 0 logger) queue.
// Records are produced at 100 Hz, so this gives over 20 seconds of margin.
#define BUFFER_SIZE 2048
#define BUFFER_MASK (BUFFER_SIZE - 1)
static FlightRecord writeBuffer[BUFFER_SIZE];
static volatile uint16_t bufferHead = 0;
static volatile uint16_t bufferTail = 0;
static volatile uint32_t droppedSamples = 0;

static File dataFile;
static unsigned long logStartTime = 0;
static int flightNumber = 0;
static String commandBuffer = "";
static unsigned long lowStorageWarningStart = 0;
static bool lowStorageWarningActive = false;
static bool logWriteFault = false;
static uint32_t lastFlushMs = 0;

#define LOW_STORAGE_THRESHOLD (4 * 1024 * 1024)  // 4MB
#define WARNING_DURATION 10000                   // 10 seconds

static bool fsInitialized = false;  // Track if filesystem is mounted

void initFlash() {
  // Mount filesystem if not already mounted
  if (!fsInitialized) {
    if (!LittleFS.begin()) {
      Serial.println("Failed to mount filesystem");
      return;
    }
    fsInitialized = true;
  }

  // A new test/flight is a new producer session.  Do not let stale records
  // from an aborted session be written into the next file.
  if (dataFile) dataFile.close();
  noInterrupts();
  bufferHead = 0;
  bufferTail = 0;
  interrupts();
  droppedSamples = 0;

  // Check storage
  FSInfo fs_info;
  LittleFS.info(fs_info);
  size_t freeSpace = fs_info.totalBytes - fs_info.usedBytes;

  Serial.print("Flash storage: ");
  Serial.print(freeSpace / 1024);
  Serial.print(" KB free / ");
  Serial.print(fs_info.totalBytes / 1024);
  Serial.println(" KB total");

  if (freeSpace < LOW_STORAGE_THRESHOLD) {
    Serial.println("WARNING: Low storage (<4MB free)");
    lowStorageWarningActive = true;
    lowStorageWarningStart = millis();
  }

  // Find next available flight number (but don't create file yet)
  flightNumber = 0;  // Reset to 0 to enumerate from the beginning
  while (true) {
    String filename = "/flight_" + String(flightNumber) + ".bin";
    if (!LittleFS.exists(filename)) {
      break;
    }
    flightNumber++;
  }

  Serial.print("Ready to log flight ");
  Serial.println(flightNumber);
  // File will be created on first log entry (when STATE_BOOST/CONTROL/DESCENT
  // starts)
  logStartTime = 0;
  logWriteFault = false;
  lastFlushMs = 0;
}

bool checkStorageWarning() {
  if (lowStorageWarningActive) {
    unsigned long elapsed = millis() - lowStorageWarningStart;
    if (elapsed >= WARNING_DURATION) {
      lowStorageWarningActive = false;
    }
    return true;
  }
  return false;
}

void logFlightData(float altitude, float velocity, float accelBias,
                   float rawAccel, float verticalAccel, float rawBaro, float motorPos,
                   float motorVel, float motorCmdPos, float Cd, float desiredCd,
                   float motorCurrent, float batteryVoltage, uint32_t axisError) {
  // Capture only. This function may be called on core 1, so it must not open,
  // write, or flush LittleFS. The core-0 loop drains the queue below.
  uint16_t head = bufferHead;
  uint16_t nextHead = (uint16_t)((head + 1u) & BUFFER_MASK);
  if (nextHead == bufferTail) {
    ++droppedSamples;
    return;
  }

  float roll, pitch, yaw;
  GetOrientation(&roll, &pitch, &yaw);

  writeBuffer[head].time_ms = millis();
  writeBuffer[head].altitude_m = altitude;
  writeBuffer[head].velocity_ms = velocity;
  writeBuffer[head].accel_bias_ms2 = accelBias;
  writeBuffer[head].raw_accel_ms2 = rawAccel;
  writeBuffer[head].vertical_accel_ms2 = verticalAccel;
  writeBuffer[head].raw_baro_m = rawBaro;
  writeBuffer[head].motor_pos = motorPos;
  writeBuffer[head].motor_vel = motorVel;
  writeBuffer[head].motor_cmd_pos = motorCmdPos;
  writeBuffer[head].roll_rad = roll;
  writeBuffer[head].pitch_rad = pitch;
  writeBuffer[head].yaw_rad = yaw;
  writeBuffer[head].Cd = Cd;
  writeBuffer[head].desired_Cd = desiredCd;
  writeBuffer[head].motor_current = motorCurrent;
  writeBuffer[head].battery_voltage = batteryVoltage;
  writeBuffer[head].state = (uint32_t)currentState;
  writeBuffer[head].axis_error = axisError;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  bufferHead = nextHead;
}

void serviceFlightLog() {
  if (!fsInitialized || logWriteFault || bufferTail == bufferHead) return;

  if (!dataFile) {
    String filename = "/flight_" + String(flightNumber) + ".bin";
    dataFile = LittleFS.open(filename, "w");
    if (!dataFile) {
      Serial.println("Failed to open file for writing");
      return;
    }
    Serial.print("Started logging to: ");
    Serial.println(filename);
  }

  // Bound the work per main-loop pass so CAN servicing remains responsive.
  int recordsWritten = 0;
  while (bufferTail != bufferHead && recordsWritten < 32) {
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    FlightRecord& record = writeBuffer[bufferTail];
    if (logStartTime == 0) logStartTime = record.time_ms;
    uint32_t relativeTime = record.time_ms - logStartTime;
    FlightRecord output = record;
    output.time_ms = relativeTime;
    size_t written = dataFile.write((uint8_t*)&output, sizeof(FlightRecord));
    if (written != sizeof(FlightRecord)) {
      logWriteFault = true;
      Serial.println("FLASH:WRITE_ERROR: record not fully written");
      return;
    }
    bufferTail = (uint16_t)((bufferTail + 1u) & BUFFER_MASK);
    ++recordsWritten;
  }
  // Flush periodically rather than once for every service pass.  This keeps
  // the core-0 loop responsive while still bounding loss after power failure.
  uint32_t now = millis();
  if (recordsWritten > 0 && (lastFlushMs == 0 ||
      (uint32_t)(now - lastFlushMs) >= 100)) {
    dataFile.flush();
    lastFlushMs = now;
  }
}

void flushLogBuffer() {
  if (!fsInitialized) return;
  if (logWriteFault) {
    Serial.println("FLASH:FLUSH_ABORTED: previous write failed");
    return;
  }
  // serviceFlightLog() can fail to open the file (for example, a full or
  // unmounted filesystem). Never spin forever in a state transition.
  if (bufferTail != bufferHead && !dataFile) {
    serviceFlightLog();
    if (!dataFile || logWriteFault) return;
  }
  while (bufferTail != bufferHead && !logWriteFault) serviceFlightLog();
}

void finalizeFlightLog() {
  flushLogBuffer();
  if (dataFile) {
    dataFile.close();
    Serial.println("FLASH:LOG_CLOSED");
  }
}

void handleFlashCommands() {
  // Early return if filesystem not mounted yet
  if (!fsInitialized) {
    // Try to mount filesystem (won't create files, just mount)
    if (LittleFS.begin()) {
      fsInitialized = true;
    } else {
      return;  // Filesystem not ready, skip command processing
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      commandBuffer.trim();

      if (commandBuffer == "LIST") {
        Dir dir = LittleFS.openDir("/");
        while (dir.next()) {
          if (dir.fileName().startsWith("flight_")) {
            Serial.print("FLASH:FLIGHT:");
            Serial.print(dir.fileName());

            // Show file size in human-readable format
            size_t fileSize = dir.fileSize();
            Serial.print(" (");
            if (fileSize >= 1024 * 1024) {
              Serial.print(fileSize / (1024.0 * 1024.0), 2);
              Serial.print(" MB");
            } else if (fileSize >= 1024) {
              Serial.print(fileSize / 1024.0, 2);
              Serial.print(" KB");
            } else {
              Serial.print(fileSize);
              Serial.print(" bytes");
            }
            Serial.print(")");

            // Mark if this is the current active flight (only if file is
            // actually open for logging)
            String currentFile = "flight_" + String(flightNumber) + ".bin";
            if ((dir.fileName() == currentFile ||
                 dir.fileName() == ("/" + currentFile)) &&
                dataFile) {
              Serial.print(" [ACTIVE]");
            }
            Serial.println();
          }
        }
        Serial.println("FLASH:END");
      } else if (commandBuffer == "CURRENT") {
        Serial.print("FLASH:CURRENT:");
        if (dataFile) {
          Serial.println(flightNumber);
        } else {
          Serial.println("NONE");
        }
      } else if (commandBuffer.startsWith("GET ")) {
        int flightNum = commandBuffer.substring(4).toInt();
        String filename = "/flight_" + String(flightNum) + ".bin";

        // Close current file if it's the one being downloaded
        bool needReopen = false;
        if (flightNum == flightNumber && dataFile) {
          dataFile.flush();  // Ensure all data is written before closing
          dataFile.close();
          needReopen = true;
        }

        File f = LittleFS.open(filename, "r");
        if (f) {
          // The payload is arbitrary binary and can legitimately contain the
          // bytes in a text sentinel such as "FLASH:END".  Announce its exact
          // size so clients can read it without scanning the payload.
          Serial.print("FLASH:DATA_START:");
          Serial.println(f.size());
          // Send in chunks to avoid blocking
          uint8_t chunk[256];
          while (f.available()) {
            int bytesRead = f.read(chunk, sizeof(chunk));
            Serial.write(chunk, bytesRead);
          }
          f.close();
          Serial.println("FLASH:END");
        } else {
          Serial.println("FLASH:ERROR: File not found");
        }

        // Reopen current file if we closed it
        if (needReopen) {
          String currentFilename = "/flight_" + String(flightNumber) + ".bin";
          dataFile = LittleFS.open(currentFilename, "a");
        }
      } else if (commandBuffer.startsWith("DELETE ")) {
        int flightNum = commandBuffer.substring(7).toInt();
        // Don't delete the currently active flight (only if file is actually
        // open for logging)
        if (flightNum == flightNumber && dataFile) {
          Serial.println("FLASH:ERROR: Cannot delete active flight");
        } else {
          String filename = "/flight_" + String(flightNum) + ".bin";
          if (LittleFS.remove(filename)) {
            Serial.println("FLASH:DELETED");
          } else {
            Serial.println("FLASH:ERROR: Delete failed");
          }
        }
      } else if (commandBuffer == "INFO") {
        FSInfo fs_info;
        LittleFS.info(fs_info);
        Serial.print("FLASH:STORAGE: ");

        // Format used bytes
        if (fs_info.usedBytes >= 1024 * 1024) {
          Serial.print(fs_info.usedBytes / (1024.0 * 1024.0), 2);
          Serial.print(" MB");
        } else if (fs_info.usedBytes >= 1024) {
          Serial.print(fs_info.usedBytes / 1024.0, 2);
          Serial.print(" KB");
        } else {
          Serial.print(fs_info.usedBytes);
          Serial.print(" bytes");
        }

        Serial.print(" / ");

        // Format total bytes
        if (fs_info.totalBytes >= 1024 * 1024) {
          Serial.print(fs_info.totalBytes / (1024.0 * 1024.0), 2);
          Serial.print(" MB");
        } else if (fs_info.totalBytes >= 1024) {
          Serial.print(fs_info.totalBytes / 1024.0, 2);
          Serial.print(" KB");
        } else {
          Serial.print(fs_info.totalBytes);
          Serial.print(" bytes");
        }

        Serial.println(" used");
      } else if (commandBuffer == "BATTERY MONITOR ON") {
        setBatteryTelemetryEnabled(true);
        Serial.println("BATTERY_MONITOR:ON");
      } else if (commandBuffer == "BATTERY MONITOR OFF") {
        setBatteryTelemetryEnabled(false);
        Serial.println("BATTERY_MONITOR:OFF");
      } else if (commandBuffer == "BARO STATUS") {
        baro.printDiagnostics();
      } else if (eeBenchCommand(commandBuffer)) {
        // EE commands own bench motion in the EE build.
      } else if (commandBuffer == "GROUND_TEST START") {
#ifdef EE_BENCH
        Serial.println("GROUND_TEST:ERROR:USE_EE_RUN");
#else
        if (startGroundTest()) {
          Serial.println("GROUND_TEST:ARMED: Shake the rocket to start recording");
        } else {
          Serial.println("GROUND_TEST:ERROR: ODrive heartbeat/error/state not ready");
        }
      #endif
      } else if (commandBuffer == "GROUND_TEST ABORT") {
        abortGroundTest();
        Serial.println("GROUND_TEST:ABORTED");
      } else if (commandBuffer == "GROUND_TEST STATUS") {
        Serial.print("GROUND_TEST:STATE:");
        Serial.println(stateName(currentState));
      }

      commandBuffer = "";
    } else {
      commandBuffer += c;
    }
  }
}
