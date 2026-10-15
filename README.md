# Airbrakes V3

## Physics EE experiment adaptation

This derived repository adds a bench experiment for **added airbrake load vs motor current and RPM**, based on the original V3 electronics and CAN drivers.

Start with [PHYSICS_EE.md](PHYSICS_EE.md). Launch the experiment app with `python ee/experiment.py gui` or `ee/launch.bat`. Use the **PhysicsEE** PlatformIO environment for physical trials. Results, metadata and analysis are stored together under `ee_results/`. The original V3 ground station remains available through `npm start`.

Airbrakes V3 is a research and development project containing embedded firmware, a desktop ground-station application, simulation utilities, and flight-data tooling for an active-airbrakes test platform.

The repository is organized so that firmware, desktop software, simulation, and recorded data can be developed and analyzed independently.

> **Project note:** This repository is experimental engineering software. Hardware testing should be performed only under the applicable team, competition, and adult-supervision requirements.

## What is in the repository?

| Area | Purpose |
| --- | --- |
| `src/` | Embedded firmware implementation |
| `include/` | Shared firmware interfaces and configuration |
| `desktop/` | Electron-based ground-station UI |
| `sim/` | Simulation and generated analysis artifacts |
| `tools/` | Standalone development and diagnostic utilities |
| `lib/` | Project-local libraries |
| `test/` | PlatformIO tests |
| `flight_data/` | Versioned flight-data files |

The firmware targets an RP2350/Pico 2-class controller and integrates inertial sensing, barometric sensing, motor-control communication, onboard logging, and estimation/control software. The desktop application provides a unified interface for development, diagnostics, configuration, data download, analytics, and replay.

## Start here

### Firmware development

Install [PlatformIO](https://platformio.org/) through VS Code or the standalone CLI, then run:

```bash
pio run
```

For board-specific programming and hardware-test procedures, follow the procedures maintained by your team rather than treating this README as a substitute for the project's hardware documentation.

### Ground-station development

From the repository root:

```bash
npm install
npm start
```

The desktop application is built with Electron. Its main responsibilities are separated between the main process, preload bridge, renderer, analytics dashboard, and replay view.

### Simulation and analysis

Simulation utilities live under `sim/`. Check the scripts in that directory for their current command-line interfaces. Generated artifacts should be treated as derived files rather than hand-edited source.

## Firmware architecture

The firmware is divided into several logical layers:

- **Hardware interfaces** — sensor, CAN, storage, and board-level interfaces.
- **Estimation** — sensor sampling, attitude estimation, and state estimation.
- **Flight state** — high-level state tracking and transitions.
- **Control** — model-based calculations and actuator commands.
- **Logging** — structured records written to onboard storage.
- **Configuration** — project constants and tunable parameters.

The implementation is primarily under `src/`, with public/shared interfaces under `include/`.

Useful starting points include:

| Component | Location |
| --- | --- |
| State definitions | `include/state.h` |
| Configuration | `include/config.h` |
| Hardware interfaces | `include/hardware.h` / `src/hardware.cpp` |
| Estimation | `src/estimator.cpp`, `src/kalman.cpp`, `src/orientation.cpp` |
| Control | `src/control.cpp` |
| Logging | `include/flash.h`, `src/flash.cpp` |

## Ground station architecture

The desktop application is split into focused components:

- `desktop/renderer.html` — application UI shell.
- `desktop/main.js` — Electron main-process integration and local-system bridge.
- `desktop/preload.js` — controlled renderer/main-process API bridge.
- `desktop/renderer.js` — renderer-side application behavior.
- `desktop/analytics.js` — flight-data analytics and charts.
- `desktop/flight3d.js` — interactive replay visualization.

Keeping these responsibilities separated makes it easier to change the UI without coupling it directly to firmware implementation details.

## Data and logs

Flight-data formats, storage layout, serial interfaces, and analysis workflows are documented separately in [`DATA.md`](DATA.md). GUI-specific information is in [`APP_INSTRUCTIONS.md`](APP_INSTRUCTIONS.md).

When changing a data record or serialized format, update the corresponding reader/writer code and documentation together. Prefer backward-compatible changes where practical.

## Documentation map

- [`APP_INSTRUCTIONS.md`](APP_INSTRUCTIONS.md) — ground-station installation and software usage.
- [`DATA.md`](DATA.md) — data formats, storage, downloads, and analysis.
- [`include/README`](include/README) — conventions for shared firmware headers.
- [`test/README`](test/README) — test-directory notes.
- [`CONTRIBUTING.md`](CONTRIBUTING.md) — development and pull-request guidelines.

## Repository conventions

- Keep generated artifacts separate from hand-maintained source.
- Keep hardware-specific constants in the configuration/interface layer rather than scattering them through implementation files.
- Update documentation when changing public interfaces, file formats, or developer workflows.
- Prefer small, focused commits and pull requests so changes are easy to review.
- Do not commit secrets, personal credentials, or unnecessary generated build output.

## Pull requests

Changes should normally be developed on a topic branch and proposed through a pull request against `main`. Describe the motivation, summarize the files changed, and note any tests or validation performed. GitHub's pull-request workflow is designed to keep proposed changes isolated and reviewable before they are merged.

## License

See [`LICENSE`](LICENSE) for the repository's license terms.
