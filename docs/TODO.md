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
- [ ] **EV1 powertrain model fidelity audit.** The current Chrono powertrain
  doesn't necessarily match the real EV1 motor characteristics — peak/cont.
  torque, power-vs-rpm curve, regen behavior, transaxle gearing, etc.
  Steady-state cruise behavior in the electronics-driven scenarios is
  suspicious (cruise-hold experiments show the plant doesn't track the
  setpoint cleanly), which may be a controller issue OR a plant issue.
  Validate the powertrain map against published EV1 specs (~102 kW peak,
  150 N·m, single-speed reduction ~10.946:1, regen up to ~25 kW) before
  tuning the cruise-control gains in `ev1/pim/pim_throttle.{h,c}`.  Until
  then, leave the conservative P-only controller alone.

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
