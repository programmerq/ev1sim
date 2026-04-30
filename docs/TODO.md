# ev1sim — TODO

Catch-all for physical-world / render / sim follow-ups deferred from focused
work.  When you finish an item, delete it from here.

## Physical-world components (next batches)

The `PhysicalWorld` pattern landed with `CombinationSwitch` as the first
instance.  Each item below is a new component that joins it — keyboard or
future-UI input on one side, chassis-segment signal publishing on the other.

- [ ] **Hazard switch** (latched push-button on column).  Defer keyboard
  binding; needs a floating UI panel for click-to-toggle (see floating-UI
  item below).
- [x] **Turn signal stalk** — left/right detents with return-to-center
  behavior (auto-cancel from steering travel implemented).  Shares the same
  physical column as the combination switch.
- [x] **Wiper switch** — off, intermittent, low, high (+ delay).  Right
  column stalk.  V key cycles OFF→INT→LOW→HIGH→OFF; M key = wash momentary.
  Publishes kSigDriverWiperSwitch (6958) + kSigDriverWiperWashRequest (6959).
  Consumer wiring (BPM/wiper relay) is a TODO on the electricsim side.
- [x] **Cruise control stalk** — set, resume, cancel, +/-.  Left column
  stalk on EV1.  G=SET, Y=RESUME, N=CANCEL, +(=)=SPEED_UP, -=SPEED_DOWN.
  Publishes kSigDriverCruiseSet/Resume/Cancel/SpeedUp/SpeedDown (6953-6957).
  Consumer wiring (RSA/PIM cruise logic) is a TODO on the electricsim side.
- [x] **IPC trip-reset button** — single momentary on the cluster.  I key.
  Publishes kSigDriverIpcTripResetButton (6952).  Consumer wiring (IPC
  odometer reset) is a TODO on the electricsim side.
- [ ] **RSA exterior keypad** — door-pillar 5-button (1/2, 3/4, 5/6, 7/8,
  9/0) for keyless entry.
- [ ] **RSA interior buttons** — PRND select, EPB switch, power window
  switches.  These are RSA's internal HMI; ev1sim simulates the user
  pressing them and publishes the resulting signals to RSA.
- [ ] **PRND keyboard cycling** — up/down keys to advance the PRND
  selector (the SimApp `SetDriverGearSelector(3)` placeholder needs
  replacement with an actual cycle).  When RSA's solenoid blocks the
  shift (e.g. shift-out-of-Park without brake), ev1sim shows an
  unobtrusive cue.
- [ ] **Seatbelt sensors** (driver, passenger) — reed switches in the
  buckles.  Defer keyboard binding; needs floating UI panel.
- [ ] **Ambient temp sensor** — pull current outdoor temp from a free
  weather API at startup; fall back to 68°F ± 5–10°F variance based on
  time of day.  Future: read from a real temp sensor on the physical sim
  rig.
- [ ] **HVAC controls** — temp setpoint, fan speed, mode selector, AC
  button, defrost button.

## Door / lock state

- [x] **Door lock state** modeled in `PhysicalWorld::DoorLocks` (driver,
  passenger, trunk; defaults UNLOCKED).  No keyboard binding — toggles will
  land via the floating-UI panel.
- [ ] **Door lock bus signal pinning** — `DoorLocks` state is not yet published
  on the chassis bus.  Add per-door lock state signals when an electricsim
  consumer (RSA central locking, key fob) wants to read them.
- [ ] **Motor current calculation** — motor current (Amps) is not published;
  Chrono's `ChEngineSimpleMap` does not expose a current output directly.
  Needs investigation of how to compute I from torque + motor characteristics.
  A placeholder TODO: add `vehicle.dynamics.motor_current_a` (chassis ID 4072)
  once the Chrono API path is confirmed.
- [x] **EV1 powertrain model fidelity audit (initial pass).**  Done in
  [docs/ev1_chrono_audit.md](ev1_chrono_audit.md).  Fixed final drive
  10.0 → 10.946:1, brake torque 1800 → 800 N·m/wheel, chassis mass
  1217 → 1199 kg (Gen 2 NiMH target), and front/rear track widths.
  Coastdown experiment (`config/scenarios/coastdown.json`) shows
  ~2× excess rolling resistance and ~2× excess v² dissipation
  remain — captured in audit §11.  See follow-up tasks below.
- [x] **TMeasy full-parameterization migration (audit §13, 2026-04-30).**
  Switched the tire JSON from "Maximum Bearing Capacity" shortcut to
  the full `Parameters` block to (a) sidestep a Chrono quirk that was
  silently shrinking the tire to ~7" effective radius via an
  argument-mismatch in `GuessPassCar70Par`, and (b) expose
  longitudinal slip stiffness for direct tuning.  After migration
  F_rr dropped from 3.0× → 1.34× spec; CdA went the other way
  (3.4× → 6.5×) because the corrected geometry surfaced TMeasy slip
  dynamics that were previously masked by the wrong scale.
- [ ] **Slip-dynamics drag calibration (deferred).**  CdA still 6.5× spec
  in coastdown after the geometry fix.  The remaining excess lives in
  TMeasy's internal slip-curve shape; the exposed `Initial Slopes
  dFx/dsx` knob doesn't usefully reduce it (lowering trades F_rr for
  CdA, raising past 2× hits numerical instability).  Defer until we
  have measured EV1 slip-curve data or migrate to a Pacejka tire model.
- [x] **Rear EMB brake actuator + self-energizing drum model.**
  `src/BrakeDrum.h` implements the self-energizing torque model
  (`T = μ·F·R·(1 + α·smooth_sign(ω))`) with a smooth ramp through
  ω=0 so the simulator stays well-behaved at standstill.  ev1sim's
  `ExternalSimConnector` subscribes to `kSigRearMotorLR/RR` (5014/5015);
  `SimApp::ApplyRearEmbBrake` converts cmd → shoe force → drum torque
  per wheel and applies via `VehicleWorld::ApplyRearBrakePerWheel`.
  Stale-fallback to local pedal mirrors the front ABS pattern.
- [ ] **EMB shoe-force integrator (refinement).**  Current model
  treats the BTCM cmd (-1 / 0 / +1) as a proportional force command.
  More faithful: integrate cmd × motor_speed × dt to track shoe
  position, then derive force from spring-like compliance.  Defer
  until measured EV1 EMB response data lands.
- [x] **Brake pedal-feel / master-cylinder pressure model.**
  `BrakePedal` PhysicalWorld component with two-stage position→pressure
  curve.  Publishes `kSigChassisBrakeMasterPressureKpa` (4074) on the
  chassis bus.  BTCM consumer wiring is the next step.
- [x] **BTCM consume master-cylinder pressure (4074).**  Done in
  `electricsim/ev1/btcm/controller.cpp` — chassis-bus subscriber
  prefers pressure (200 kPa threshold = ~10 % travel into the soft
  fluid stage) and falls back to Q8 deadband when pressure is stale
  or never received.  Pressure consumer has a defensive bounds check
  ([0, 50000] kPa) to ignore corrupted frames; see `SHM transport
  intermittency` below.
- [ ] **SHM transport intermittent float corruption (infrastructure).**
  Multi-process runs (ev1sim + PIM + RSA + BTCM all attached to the
  same chassis bus) occasionally deliver corrupted float values to
  late-attached readers.  Symptom: BTCM reads kSigChassisBrakeMaster-
  PressureKpa as values like 1.7e25 instead of 13940.  The wire bytes
  appear to mismatch what `MakeFloatDelta()` wrote.  Workaround for
  now: `SharedMemoryTransport` create=false on the controller side +
  `scripts/run_abs_compare.sh` waits for ev1sim's "connected to main
  harness bus" log + cleans `/electricsim_*_bus` segments via
  `shm_unlink` before each scenario.  Some intermittency remains.
  Deeper fix would be to add a checksum byte to `MakeFloatDelta` /
  read-side validation in `f32_from_payload`, or audit the ring
  buffer's writer/reader synchronization for races.
- [ ] **BTCM ABS algorithm doesn't reliably engage in headless runs.**
  Even with the full controller stack (PIM + RSA + BTCM) and ev1sim
  driving wheel-omega + brake-pressure, the BTCM firmware's ABS
  state machine doesn't transition out of POST defaults during a
  hard-brake event.  Front wheels lock to slip=1.0; the BTCM-on vs
  BTCM-off comparison runs (via `scripts/run_abs_compare.sh`) end
  up nearly identical.  Likely root cause is firmware-side: the AVR
  ABS algorithm needs more inputs than just brake_pedal + tone-ring
  pulses (probably run_1 stable, accel signal, etc.) before it'll
  engage.  Worth a focused firmware diagnostic batch.
- [ ] **ABS rumble feedback for force-feedback rigs (deferred).**
  When ABS engages, the BTCM sol_*_iso/sol_*_dmp signals cycle.
  Surfacing this as a `kSigChassisAbsActive` boolean would let a
  game-controller adapter drive a rumble motor for tactile feedback.
  Out of scope until game-controller input/FFB lands.
- [ ] **Suspension spring/damper rate verification.**  Front and rear
  share the same 143000 N/m spring rate.  EV1's weight distribution
  is reported ~50/50, so identical rates are *less* suspicious than
  initially thought, but they should still be verified with a
  `bounce_test.json` scenario (drop-from-height + measure ride
  frequency) — Olley's rule typically wants rear slightly stiffer to
  control pitch, even with even weight distribution.  Keep until
  measured.
- [ ] **Front MacPherson strut + rear trailing-arm templates.**
  Current Chrono setup uses `DoubleWishbone` for both axles; real EV1
  is MacPherson front / trailing-arm (twist-beam) rear.  Substantial
  rebuild; deferred until either (a) Chrono `MacPhersonStrut` JSON
  sample available, or (b) we have measured EV1 hardpoints.
- [ ] **Body aerodynamics.**  No `ChAerodynamicLoad` on the chassis;
  the EV1's signature 0.19 Cd is currently lumped into tire dissipation.
  Add explicit drag once the rolling/slip dissipation is calibrated.
- [ ] **Brake bias front/rear.**  Currently both axles share
  `EV1_BrakeSimple.json` at 800 N·m/wheel.  Real EV1 has front discs
  + rear drums with stronger front bias.  Split into
  `EV1_BrakeSimple_Front.json` (800) and `_Rear.json` (500) when
  tuning braking dynamics matters.

## Outputs ev1sim could render (defer all)
- [ ] Wiper motor visual sweep (model update needed).
- [ ] Defrost grid visualization.
- [ ] HVAC blower (audio + load).
- [ ] Door lock solenoid (audio + visual click).
- [ ] Trunk animation (T key already toggles state; no visual today).

## Floating UI panel
- [ ] **Mouse capture / camera escape.** Need a way to release the camera
  to interact with floating UI elements.
- [ ] **Switch-list overlay** — clickable panel listing each PhysicalWorld
  component's current state, with click-to-toggle.  Unblocks hazard, door
  locks, key fob, seatbelt sensors and any other low-frequency input that
  doesn't deserve a keybind.

## Bus-mediated physics
- [x] **Front-brake torque from BTCM ABS solenoid state (DONE).** ev1sim
  subscribes to `kSigSolFL/FR_ISO/DMP` (5010-5013) on the main harness
  segment.  Each tick, iso/dump tuples are decoded to APPLY/HOLD/DUMP phase
  and applied per-wheel via `VehicleWorld::ApplyFrontBrakePerWheel()`.
  Falls back to local brake when BTCM data is stale (>200 ms); one-time
  INFO log on each freshness transition per wheel.  HUD shows
  `ABS: FL=APPLY FR=APPLY` when BTCM is live.
- [ ] **Rear-EMB clamp-position model needed for full rear-wheel modulation.**
  BTCM publishes EMB motor commands (`kSigRearMotorLR/RR`, signed floats)
  but nothing directly mappable to torque without a clamp-position model.
  Until that model is added, rear brake = local `rear_brake` (no per-wheel
  modulation).
- [x] **Throttle bus path (PIM)** — `kSigChassisThrottleCmdQ8` (4073) is
  published by PIM each tick (passthrough or cruise-controlled).  ev1sim
  subscribes via `ExternalSimConnector::GetThrottleCmd(window)`,
  `SimApp::ApplyElectronicsThrottle` overrides `cmd.throttle` when fresh,
  falls back to local pedal at 200 ms staleness.  Selected via
  `vehicle_dynamics.driver = "electronics"`.  Cruise demo exercises the
  full loop end-to-end (`config/scenarios/cruise_demo_electronics.json`).
- [ ] Same pattern for **steering assist** etc.

## Sim-time sync — ev1sim publisher side

To unblock electricsim from running in lockstep with faster-than-realtime
ev1sim runs, ev1sim should publish a sim-time tick on the chassis bus
(`kSigChassisSimTimeNs`, suggested chassis ID 4070).  Cross-repo design
lives in electricsim/docs/TODO.md under "Sim-time sync".  ev1sim's part:
publish the current Chrono sim-time on every step, encoded as uint64_t
little-endian nanoseconds, on the chassis segment.

## Future: physical sim rig
- [ ] Adapt the user's physical driving sim rig with a lookalike RSA, seat
  belt sensor, IPC display, chime/horn sound effects.  Hook real HVAC
  controls.  ev1sim's PhysicalWorld pattern should grow a hardware
  backend that maps real switches to chassis-bus signals (replacing
  keyboard input).
