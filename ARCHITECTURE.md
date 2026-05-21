# Architecture

This simulator is the **physics plant** for a future EV1 electronics
toolchain.  It is deliberately shaped so that an external electrical
simulator (ECUs, wiring, load dynamics) can drive the bulbs/horn and
read the panel sensors while this process owns vehicle dynamics and
rendering.

The boundaries below exist so that a test harness, a recorded trace
replayer, or the external electrical simulator can each play the role
of "driver" or "electrical system" without the rest of the program
noticing.

## Process layout

```
                +---------------------------+
                |          SimApp           |
                |  (Run / RunHeadless loop) |
                +-----+-----+-----+---------+
                      |     |     |
     +----------------+     |     +-----------------+
     |                      |                       |
     v                      v                       v
 KeyboardInput       VehicleWorld (Chrono)     ExternalSimConnector
 / ScriptedDriver     Vehicle + Terrain        (shm, electricsim proto)
     |                      ^                       ^
     |                      |                       |
     +----DriverCommand---->+                       |
                            |                       |
                            +----VehicleState-------+
                                 (panels, bulbs)
```

- Inputs (`KeyboardInputController`, `ScriptedDriver`, or a future
  remote driver) produce a `DriverCommand`.
- `VehicleWorld` applies the command each physics step and exposes a
  `VehicleState` snapshot after each advance.
- `ExternalSimConnector` latches inbound bulb/horn commands from the
  external electrical sim and publishes outbound panel-sensor state.
- `SimApp` is the only component that knows about all three; everything
  else stays on its own side of the boundary.

## The command boundary — `DriverCommand`

File: [src/DriverCommand.h](src/DriverCommand.h)

A plain struct with no Chrono or Irrlicht types.  Anything that can
produce one of these is a valid driver:

| Field             | Range     | Meaning                                 |
|-------------------|-----------|-----------------------------------------|
| `throttle`        | 0..1      | Requested propulsion                    |
| `front_brake`     | 0..1      | Service brake, front axle               |
| `rear_brake`      | 0..1      | Service brake, rear axle                |
| `steering`        | -1..1     | Positive = left turn (Chrono convention)|
| `parking_brake`   | bool      |                                         |
| `reset_vehicle`   | bool      | One-shot: respawn at spawn point        |
| `horn_low`        | bool      | 400 Hz tone request                     |
| `horn_high`       | bool      | 500 Hz tone request                     |

Current producers:
- [`KeyboardInputController`](src/KeyboardInputController.h) — WASD / space / etc.
- [`ScriptedDriver`](src/ScriptedDriver.h) — accel-hold-brake headless scenario.

Future producers (socket driver, replay) should emit this struct and
nothing else — no direct access to `VehicleWorld` or the Chrono vehicle.

## The state boundary — `VehicleState`

File: [src/VehicleState.h](src/VehicleState.h)

A plain struct, again Chrono-free, capturing a single simulation step:
pose, velocity, accelerations, wheel speeds, applied-command echo.  This
is what telemetry logs, what the scripted driver reads to decide its
phase, and what a future IPC layer would serialise.

Any consumer that does not need Chrono types should take a
`const VehicleState&` — never a `ChWheeledVehicle&`.

## The electrical boundary — `ExternalSimConnector`

Files: [src/ExternalSimConnector.h](src/ExternalSimConnector.h),
[src/VehicleLights.h](src/VehicleLights.h),
[src/VehiclePanels.h](src/VehiclePanels.h)

This is the interface to the external electric simulator.  It is a
one-process wrapper over the electricsim shared-memory bus and is safe
to drive every frame whether or not a peer is actually connected.

**Endpoints — the registry lives in electricsim, not here.**  The
authoritative, complete signal registry is electricsim's
`docs/3d_sim_contract.md` (its cross-reference table) plus the
`electricsim/src/io/ev1_*_signals.hpp` headers.  Do **not** treat this
file as the registry: an earlier version of it listed only the
4000..4033 bulb/horn/panel block and silently drifted behind reality as
dozens of signals were added.  ev1sim's live list is whatever
`ExternalSimConnector::Endpoints()` returns; the "covers every device
exactly once" test pins the exact count.

Signals fall into three ID blocks:

| Block      | Segment      | What lives there                                    |
|------------|--------------|-----------------------------------------------------|
| 4000..4199 | chassis bus  | bulbs/horn (4000-4021), panel sensors (4030-4033), motor/throttle/brake/**sim-time** (4070-4075), wiper/HVAC/ambient (4080-4091), IPC telltales (4130-4145), BTCM actuators (4147-4154) |
| 5000..5899 | main harness | per-module ECU signals **owned by electricsim** (BTCM 5000-5099, …) that ev1sim subscribes to |
| 6900..6999 | main harness | driver inputs + 3D-sim-contract physics/sensors **ev1sim publishes** (6900-6991) |

Bulb IDs 4000..4016 follow the electricsim `LightIdx` enum exactly;
4017..4018 extend the range for the two tail-filament bulbs (`LRTL`,
`RRTL`) that ev1sim models but electricsim does not.

**Build-time optionality.**  If the electricsim repo is present next
to this one (or pointed at via `-DELECTRICSIM_DIR=<path>`), the
connector compiles against its transport and publishes a live bus.  If
not, it falls back to a stub that reports `Status::Unavailable` and
silently drops frames — every caller site in the codebase already
handles this, so `--external-sim on` without electricsim is harmless.

**Protocol-level boundary.**  The connector trades exclusively in
`(signal_id, value)` deltas over the shared-memory bus.  Anything
below that — wire framing, CRC, register maps, the bit-level layout
of any serial bus the external simulator uses internally to drive
its own ECUs — lives entirely inside the external simulator and is
**not** mirrored here.  This file and the source tree refer to
signals only by numeric ID and high-level semantics (speed, voltage,
telltale active/inactive, per-corner solenoid open/closed); the
transport is deliberately opaque to anything more specific.  Keep it
that way when adding endpoints.

## Run modes

`SimApp::Run()` dispatches on `Config::simulation.headless`:

- **Interactive (`RunWithVisualization`)** — Irrlicht window, keyboard
  driver, Chrono chase camera, HUD overlays, horn audio on macOS.
- **Headless (`RunHeadless`)** — no Irrlicht device, no OpenGL
  context.  Physics, telemetry, and the external-sim connector still
  run.  A run requires a terminator (`max_time_s > 0` or a scripted
  scenario); see `config/accel_brake.json` and `config/headless_smoke.json`.

Both modes return one of:

| Code | Meaning                                                          |
|------|------------------------------------------------------------------|
| 0    | `kExitSuccess` — scenario finished or window closed               |
| 2    | `kExitUsage` — headless without a terminator                      |
| 3    | `kExitTimeout` — `max_time_s` reached with scripted still running |
| 130  | `kExitInterrupted` — SIGINT in headless                           |

These are meant to be consumed by CI.

## Adding a new driver source

1. Produce `DriverCommand` values from your source.
2. Pass them to `VehicleWorld::GetDriver().SetCommand(cmd)` before
   each `Synchronize`/`Advance` pair.
3. Do **not** touch Chrono types directly from the driver — the point
   of the boundary is that `ScriptedDriver` and `KeyboardInputController`
   both already run against it and anything new should too.

## Adding a new electrical endpoint

1. **Allocate the ID in the contract first.**  `electricsim/docs/3d_sim_contract.md`
   plus the `electricsim/src/io/ev1_*_signals.hpp` headers are the single
   source of truth.  Pick the right block (chassis 4000-range, module
   main-harness 5000-range, or driver/3D-sim 6900-range) and a free ID;
   never reuse one.  (4070 is motor RPM, **not** sim-time — that exact
   mistake already shipped in a stale TODO note before this guard
   existed.)
2. Mirror the constant in `ExternalSimConnector.cpp` with a
   `// Locked in lockstep with electricsim/src/io/...` comment.  For a
   **chassis** signal, also add it to the `static_assert` block (gated on
   `EV1SIM_HAVE_EXTERNAL_SIM`) that cross-checks every ev1sim chassis ID
   against `electricsim::io::kSigChassis*` at compile time — drift then
   becomes a build error, not a runtime mystery.
3. Register it in `ExternalSimConnector::Endpoints()` and add get/set
   methods mirroring the existing bulb/horn/panel patterns.
4. Update [tests/test_external_sim_connector.cpp](tests/test_external_sim_connector.cpp)
   — the "covers every device exactly once" test will fail until the
   new endpoint is accounted for.
