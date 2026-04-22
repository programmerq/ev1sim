# EV1 Simulator Runtime

Minimal standalone C++ vehicle physics simulator using [Project Chrono](https://projectchrono.org/), Chrono::Vehicle, and Irrlicht visualization.

This is the **physics + visualization plant** for a future EV1 electronics simulation toolchain. It renders a drivable wheeled vehicle on a flat terrain plane, accepts keyboard input, and exposes clean command/telemetry boundaries for future external control.

## Prerequisites

- **CMake** 3.18+
- **C++17** compiler (Clang, GCC, or MSVC)
- **Project Chrono** 9.x or 10.x built from source with the **Vehicle** and **Irrlicht** modules enabled
- **Irrlicht** (installed as a Chrono dependency)

### Installing Project Chrono from Source

```bash
# 1. Install Irrlicht (macOS)
brew install irrlicht

# On Ubuntu/Debian: sudo apt install libirrlicht-dev

# 2. Clone Chrono
git clone https://github.com/projectchrono/chrono.git
cd chrono
git checkout release/9.0   # or main for latest

# 3. Build with Vehicle + Irrlicht modules
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_MODULE_VEHICLE=ON \
  -DENABLE_MODULE_IRRLICHT=ON \
  -DIRRLICHT_INSTALL_DIR=$(brew --prefix irrlicht) \
  -DCMAKE_INSTALL_PREFIX=$HOME/chrono-install

cmake --build . -j$(sysctl -n hw.ncpu)
cmake --install .
```

The install directory (`$HOME/chrono-install`) will contain a `lib/cmake/Chrono/` directory — that's your `Chrono_DIR`.

## Building ev1sim

```bash
cd /path/to/ev1sim
mkdir build && cd build

cmake .. -DChrono_DIR=$HOME/chrono-install/lib/cmake/Chrono
cmake --build . -j$(sysctl -n hw.ncpu)
```

On first configure, CMake will also fetch **nlohmann/json** and **Catch2** (for tests) automatically via FetchContent.

### Disable tests (faster build)

```bash
cmake .. -DChrono_DIR=... -DBUILD_TESTS=OFF
```

## Running

```bash
# From the build directory (config paths are relative to working dir)
cd /path/to/ev1sim
./build/ev1sim

# Or with overrides
./build/ev1sim --config config/low_mu.json
./build/ev1sim --vehicle hmmwv --surface low_mu
./build/ev1sim --step-size 0.001 --realtime false
```

### CLI Flags

| Flag | Values | Default |
|------|--------|---------|
| `--config <path>` | Path to JSON config | `config/default.json` |
| `--vehicle <name>` | `sedan`, `hmmwv` | `sedan` |
| `--surface <name>` | `asphalt`, `concrete`, `low_mu` | `asphalt` |
| `--step-size <s>` | Physics step in seconds | `0.002` |
| `--realtime <bool>` | `true`, `false` | `true` |
| `--friction <val>` | Custom friction coefficient | (from surface preset) |
| `--render-fps <n>` | Target render FPS | `60` |
| `--hud <bool>` | Show on-screen HUD | `true` |
| `--headless` | Run without a window (no Irrlicht, no rendering) | off |
| `--max-time <s>` | Exit after this many seconds of sim time (`0` = no limit) | `0` |
| `--scripted-accel-brake` | Enable built-in accel → hold → brake scenario | off |
| `--target-kph <n>` | Target speed for the scripted scenario (implies enabled) | `40` |
| `--hold-time <s>` | Hold target speed for this long before braking | `1.0` |

CLI flags override config file values.

### Headless mode

```bash
# Physics + telemetry + external-sim I/O, no window.
# Exits on max-time or SIGINT.
./build/ev1sim --headless --max-time 10

# Faster than real-time, useful in CI:
./build/ev1sim --headless --max-time 10 --realtime false
```

Headless mode is intended for scripted scenario runs against the external
electronic simulator.  Keyboard input is not available; without a scripted
driver the command stays at zero throttle/brake/steering.

`--headless` requires at least one way to terminate automatically — either
`--max-time <s>` or a scripted scenario.  Running `--headless` with
neither is a usage error (exit `64`) rather than a silent hang.

#### Exit codes

| Code | Meaning |
|------|---------|
| `0`  | Successful completion (scripted `Done`, window closed, or `max_time` with no scripted scenario) |
| `2`  | `max_time_s` expired while a scripted scenario was still running — scenario timeout |
| `130`| SIGINT (Ctrl-C) |
| `64` | Usage error (e.g. `--headless` with no way to terminate) |
| `1`  | Fatal exception during run |

### Built-in accel → hold → brake scenario

A minimal scripted driver with three phases:

1. **Accel** — full throttle until vehicle speed reaches `--target-kph`
2. **Hold** — maintain target speed for `--hold-time` seconds (default `1.0`)
3. **Brake** — full brake until speed falls below `stop_threshold_mps` (0.1 m/s)

When the Brake phase completes, the sim exits `0`.  The sample config
[`config/accel_brake.json`](config/accel_brake.json) runs the scenario
headless on a flat rigid plane.

```bash
./build/ev1sim --config config/accel_brake.json

# Compose with CLI flags (defaults to the Milford level — see caveat below):
./build/ev1sim --headless --realtime false --target-kph 30 --hold-time 1 --max-time 30
```

**Terrain caveat.** The scenario's Brake phase ends when forward speed
falls below `stop_threshold_mps`.  On the default Milford level the
vehicle sits on a slope, so full brakes don't fully arrest it and the
Brake phase may never complete.  For reliable CI use, point the
scenario at a flat terrain — either via `terrain.type: "rigid_plane"`
in the config (as `config/accel_brake.json` does) or via
`--level <flat-level.json>`.  The `--max-time` backstop will flag this
case with exit code `2` rather than hanging.

The scripted driver also runs in windowed mode when enabled, which is
handy for visually debugging a scenario before sending it to CI.

Phase transitions are logged to stdout (`[ScriptedDriver] Phase -> Hold`).

## Scenario levels

Level JSON files in [`level/`](level/) describe the terrain under the
vehicle.  Each patch has a `type` (`plane` or `mesh`), a friction
coefficient, and optional texture.  `plane` patches take `size` (L × W,
metres) and `center` (x, y, z); `mesh` patches reference an OBJ file.
`type` defaults to `mesh` when omitted, so existing mesh-based levels
need no changes.

| Level | Purpose | Shape | Companion config |
|-------|---------|-------|------------------|
| [`milford.json`](level/milford.json) | Proving-ground mesh (slope caveat above) | asphalt + grass OBJ meshes | `config/default.json` |
| [`flat_ice_transition.json`](level/flat_ice_transition.json) | Traction / ABS across a high-µ → low-µ transition | 100 m asphalt → 100 m ice, 20 m wide | [`config/ice_transition.json`](config/ice_transition.json) |
| [`flat_split_mu.json`](level/flat_split_mu.json) | Split-µ ABS / stability — left wheels on ice, right on asphalt | two 200 m × 10 m strips along the Y axis | [`config/split_mu.json`](config/split_mu.json) |

Both new scenarios run headlessly with the built-in accel → hold → brake
driver:

```bash
./build/ev1sim --config config/ice_transition.json   # exit 0 on success
./build/ev1sim --config config/split_mu.json         # exit 0 on success
```

For interactive (keyboard) inspection:

```bash
./build/ev1sim --level level/flat_ice_transition.json
```

### `environment` config block (optional)

```jsonc
"environment": {
  "time_of_day": "day",           // "day" | "dusk" | "night" — ambient + sun preset
  "ambient":     [0.4, 0.4, 0.4], // RGB override, wins over preset
  "sun_elevation_deg": 55.0,      // override, wins over preset
  "ambient_temp_c": 20.0          // stub — no consumer yet (see TODO)
}
```

Headless runs ignore this block.

### Planned scenarios (TODO)

These need either new geometry or a scripted steering phase and are
deferred:

- **Flat skidpad + constant-steer scripted phase** — enables headless
  circle/steady-state handling tests.
- **Moose test / double-lane-change** — needs scripted-steer waypoints.
- **Bumpy / washboard road** — heightmap OBJ or Chrono `RigidTerrain`
  bmp heightmap for suspension response.
- **Wet asphalt (~0.5) and snow (~0.2) transition levels** — additional
  friction presets paired with `flat_ice_transition.json`.
- **Brake / tire temperature hooks** — Chrono has no native thermal
  model; `Config::Environment::ambient_temp_c` is reserved for future
  consumers.  No code reads it yet.
- **Milford rework** — re-spawn on a flat apron so brake tests are
  reliable there too.

## Controls

| Key | Action |
|-----|--------|
| W | Throttle (ramps up while held) |
| S | Brake (ramps up while held) |
| A | Steer left (ramps, self-centers on release) |
| D | Steer right (ramps, self-centers on release) |
| Space | Toggle parking brake |
| R | Reset vehicle to spawn position |
| C | Cycle camera mode |
| Esc | Quit |

## Camera Modes

Cycled with **C**:

1. **Chase** — behind and above the vehicle
2. **Hood** — on the hood looking forward
3. **Rear** — behind the vehicle looking backward
4. **Top-Down** — overhead view
5. **Free-Look** — mouse orbit (drag to rotate, scroll to zoom)

## Architecture

```
main.cpp ─► SimApp ─┬─► VehicleWorld ─► Chrono system, vehicle, terrain
                     ├─► KeyboardInputController ─► DriverCommand
                     ├─► CommandDriver (ChDriver bridge)
                     ├─► CameraManager
                     └─► Telemetry
```

**Key abstractions for future extension:**

- **`DriverCommand`** — framework-agnostic command struct with `front_brake`/`rear_brake` split (both set identically from keyboard for now, ready for independent front/rear control)
- **`VehicleState`** — framework-agnostic telemetry struct with speed, wheel speeds, chassis acceleration, yaw rate, etc.
- **`InputController`** — abstract interface. `KeyboardInputController` is the first implementation. Future: `SocketInputController`, `WheelInputController`, `PlaybackInputController`
- **`CommandDriver`** — bridges `DriverCommand` → Chrono's `ChDriver` interface

### Where to add future socket-based input

1. Create `SocketInputController` implementing `InputController`
2. In `SimApp` constructor, select it based on a config/CLI flag instead of `KeyboardInputController`
3. No changes needed to `VehicleWorld`, `CommandDriver`, or rendering code

### Where to add future telemetry output

1. `VehicleState` already contains all fields needed by external electronics simulation
2. Add a `TelemetryExporter` that serializes `VehicleState` to shared memory, a socket, or a file
3. Call it alongside the existing `Telemetry::Record()` in `SimApp::Run()`

### Future per-axle brake control

`DriverCommand` already has `front_brake` and `rear_brake` fields. To enable independent control:

1. Modify `CommandDriver::Synchronize()` to apply torques per-axle via Chrono's brake subsystem API instead of using the single `m_braking` channel
2. Future regen braking maps to negative propulsion torque (add a `propulsion_torque` field to `DriverCommand`)

## Chrono Version Notes

- Targets **Chrono 9.x** or **10.x** (10.0 released March 2026)
- Uses `Sedan` / `HMMWV_Full` wrapper classes from `chrono_models`
- Uses `ChContactMethod::SMC` (smooth contact) with TMeasy tire model
- Vehicle JSON definitions are stable across 8.x → 9.x → 10.x
- If your Chrono version has different header paths or class names (e.g., the visualization class), check the Chrono CHANGELOG for your version

## Running Tests

```bash
cd build
ctest --output-on-failure
```

Tests cover: config JSON parsing, CLI overrides, default values, keyboard ramping logic, parking brake toggle, one-shot reset/camera-cycle behavior.

## Project Structure

```
ev1sim/
  CMakeLists.txt
  README.md
  config/
    default.json          # Standard asphalt parking lot
    low_mu.json           # Low-friction surface preset
  src/
    main.cpp              # CLI parsing, create SimApp, run
    SimApp.h/cpp          # Main loop, subsystem lifecycle
    VehicleWorld.h/cpp    # Chrono system, vehicle, terrain, physics
    VehicleState.h        # Telemetry output struct (no Chrono deps)
    VehiclePose.h         # Camera positioning struct (no Chrono deps)
    DriverCommand.h       # Control input struct (no Chrono deps)
    CommandDriver.h/cpp   # ChDriver bridge (DriverCommand → Chrono)
    InputController.h     # Abstract input interface
    KeyboardInputController.h/cpp
    CameraManager.h/cpp   # 5-mode camera system
    Config.h/cpp          # JSON + CLI config
    Telemetry.h/cpp       # Logging + HUD overlay
  tests/
    test_config.cpp
    test_driver_command.cpp
    test_keyboard_input.cpp
```
