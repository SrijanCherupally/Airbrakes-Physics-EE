#include <Arduino.h>

#include "state.h"
#include "flash.h"
#include "ee_bench.h"

// Make setup blocking so core1 waits for hardware init.
semaphore_t setup_done;

void setup() {
  sem_init(&setup_done, 0, 1);

  setupHardware();
  stateInit();
  // LittleFS is core-0-owned, just like CAN. Mount it before either core can
  // enqueue samples; the estimator must never initialize or write the FS.
  initFlash();

  // Preflight is diagnostic only for now. Do not inhibit the state machine:
  // ground-test commands and state diagnostics must remain available even if
  // one startup sample is unavailable.
  bool preflightOk = hardwarePreflightCheck();
  ledWrite(0.0f, 0.15f, 0.10f);
  Serial.print("PREFLIGHT:");
  Serial.println(preflightOk ? "PASS (diagnostic only)" : "WARN (diagnostic only)");

  // Flash logging will be initialized when entering STATE_PAD

  // Indicate setup done
  sem_release(&setup_done);
}

void loop() {
  handleFlashCommands();
  serviceOdrive();
  serviceFlightLog();

  // Update state machine
#ifdef EE_BENCH
  eeBenchUpdate();
#else
  stateUpdate();
#endif
}

void setup1() {
  // wait for core0 setup to finish
  sem_acquire_blocking(&setup_done);
}

void loop1() {
  // Update filter
#ifdef EE_BENCH
  delay(2);
#else
  estimatorUpdate();
#endif
}
