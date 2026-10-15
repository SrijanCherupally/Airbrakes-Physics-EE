# Airbrakes Physics EE: added load vs current and RPM

Research question: How does added load on the airbrake mechanism affect motor current and rotational speed during a repeatable deployment stroke?

The independent variable is the total added mass attached to the airbrakes. The dependent variables are measured ODrive q-axis current and motor RPM. Keep the same attachment point(s), force direction, travel, speed limit, controller gains, current limit, battery conditions and lubrication for a single investigation. Record temperature and apparatus changes in session notes. This uses the physical module, without a rocket flight.

## Electronics and firmware ancestry

This repository is derived directly from SrijanCherupally/Airbrakes-V3. The original source and history are retained. `UPSTREAM_REVISION.txt` records the exact starting revision. Existing firmware, Electron ground station, drivers and sensor libraries remain available.

The EE build reuses the RP2350/Pico 2, ODrive node 0, CAN at 250 kbit/s, and the vendored CAN library patched for the 20 MHz MCP2515 crystal. It retains the original IMU, barometer, RGB LED and battery ADC initialization. A missing IMU still blocks setup, as in V3. Wiring is defined in `include/hardware.h`; do not substitute a generic Pico pin map.

| Interface | Inherited wiring/configuration |
| --- | --- |
| MCP2515 SPI | MISO 16, CS 17, SCK 18, MOSI 19, INT 20 |
| USB serial | 115200 baud |
| ODrive CAN | node 0, 250000 bit/s |
| Battery divider | GPIO27, 17.8 kOhm / 10 kOhm |
| Motor travel bounds | -42 to 0 revolutions, inherited calibration |

`PhysicsEE` is a separate PlatformIO build. It bypasses the flight state machine and the estimator loop. It starts the ODrive in IDLE and uses `EE RUN` instead of the old shake trigger. CAN telemetry is acquired by the existing `serviceOdrive()`; EE publishes only pairs for which both encoder and measured-current requests succeeded. The sequence number lets analysis reject duplicate/stale rows. Encoder and current reads are sequential, not simultaneous. The original `AirbrakesV3` build remains available.

## Launch

From the repository root, with Python 3.10+:

```powershell
python -m pip install -r ee/requirements.txt
python ee/experiment.py gui
```

Or double-click `ee/launch.bat`. Simulation and analysis use the Python standard library; physical capture needs pyserial. The GUI uses Tkinter.

1. Leave **Simulated experiment** checked for a dry run. Edit the load list and repetitions, create a session, and run each trial. Open the report to verify the workflow.
2. For physical tests, build and upload the dedicated firmware:

```powershell
python -m platformio run -e PhysicsEE
python -m platformio run -e PhysicsEE -t upload
```

3. Close the original Electron app and other serial monitors so this app owns the port. Uncheck simulation, select the Pico port, and record the load attachment geometry and total added mass in the settings.
4. Verify actual mechanism endpoint calibration. Default test travel is **0 to -10 rev**, speed limit **2 rev/s**, and current limit **1 A**. These are starting values, not validated settings for your apparatus. Firmware restricts positions to V3's -42…0 rev bounds and accepts up to its 3 A / 50 rev/s limits. Physical calibration must establish whether these bounds are appropriate.
5. Position the mechanism at the configured start before running. The software does not home it or reset the encoder. Each trial verifies the measured start position, moves outward, and returns. It releases powered actuation afterward. Secure/support weights so releasing the motor does not drop or backdrive the mechanism; ODrive IDLE is not a mechanical brake.
6. Fit the load requested by **Run next trial**, then start. Change weights between trials. The load order is randomized within each repetition block. The mass recorded is manually entered, not sensed.
7. **STOP** requests ODrive IDLE. The firmware also requests IDLE on loss of client pings (2 s), stale paired telemetry (200 ms), stale heartbeat, axis error, CAN position-send failure, current-limit approach, or motion timeout. CAN/USB failures can prevent software stop commands from reaching the motor; retain a physical power disconnect. Firmware limits are software bounds, not physical limit switches. Failed/aborted trials remain saved and excluded from analysis.

Current and RPM are shown live. A trial is marked COMPLETE only after both commanded endpoints are reached at low speed. Current limiting can terminate a loaded trial; it must not be treated as a normal observation. Capture does not automatically change gains, current limits or stroke between load levels.

## Results and analysis

Each experiment is saved under `ee_results/<timestamp>_physical` or `_simulated`:

```text
session.json                settings, planned loads, repetitions and trial statuses
trial_001/metadata.json      load, repetition, source, parameters, final status
trial_001/data.csv           raw timestamped fresh telemetry
analysis.json               run summaries, grouped repeat statistics, regressions, exclusions
trial_summary.csv           one row per trial and direction
report.html                 offline plots and tables
```

Use **Resume session** to continue pending trials. Existing settings are immutable for that session; the editor applies to new sessions only. Interrupted RUNNING trials remain excluded, preserving their original data. To repeat an unsuccessful condition, create another session with the same settings and load. All data is saved locally on the computer; EE streams are not written to onboard flash.

```powershell
python ee/experiment.py demo
python ee/experiment.py analyze ee_results/<session-folder>
python -m unittest discover -s ee -p "test_*.py"
```

Analysis keeps outbound and return separate. It uses a configurable central position window (default middle 60% of travel) and excludes near-stationary samples. It reports mean absolute current, current spread and peak, mean absolute RPM, speed spread, battery voltage and timing gaps. Signed current and speed remain in the raw CSV. RPM = `motor_vel * 60`, because ODrive reports rev/s.

For each mass and direction, uncertainty from repeatability is calculated across independent trial means. The telemetry sample count is not the number of experimental repeats. Regressions use one mean per independent trial, report slope standard error and R-squared, and are descriptive: they do not correct for apparatus/systematic uncertainties. No formal uncertainty propagation from the recorded mass uncertainty or current calibration is implemented. Graphs show trial points; grouped tables provide SD and standard error.

A central travel window does not guarantee steady motion. Inspect raw traces and speed variability warnings before choosing the final analysis interval. Short strokes may never attain the velocity limit. A position controller can keep RPM almost constant as load rises, then saturate: a nearly flat RPM trend is a meaningful possible outcome. The speed setting is a velocity limit, not a constant-RPM guarantee.

The measured current is ODrive **Iq**, not battery current. Do not calculate electrical energy as battery voltage times Iq. Added mass gives applied weight `mg`; linkage geometry and force direction determine how that translates into screw load. No torque constant, lead-screw dimensions or mechanical advantage is invented here. Record attachment geometry and calibrate current/speed if absolute uncertainties are needed.

Simulated trials use an illustrative current-growth/speed-droop model with seeded noise. They exercise the workflow and analysis; they do not predict this hardware or constitute experimental evidence.

## EE serial protocol, version 1

```text
EE HELLO -> EE:HELLO:1
EE RUN <start_rev> <end_rev> <speed_rev_s> <current_limit_a> <phase_timeout_ms>
EE PING -> EE:PONG
EE STOP -> EE:DONE:ABORTED
EE:STARTED
EE:DATA:<elapsed_ms>,<outbound|return>,<position_rev>,<velocity_rev_s>,<Iq_A>,<battery_V>,<fresh_pair_sequence>
EE:DONE:<COMPLETE|failure_reason>
EE:ERROR:<reason>
```

Firmware and host can be built/tested without a connected mechanism. Physical actuation and sensor accuracy still need bench verification.

Sources: [Airbrakes-V3](https://github.com/SrijanCherupally/Airbrakes-V3), [ODrive control documentation](https://docs.odriverobotics.com/v/latest/manual/control.html). Original discussion: Codex chat **Design airbrakes experiments**, September 24, 2026, ID `01a0d583-0f0a-7213-81c6-47d9d9ac7758`.
