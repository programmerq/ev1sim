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
- [x] **RSA exterior keypad** — `RsaExteriorKeypad` component in
  `PhysicalWorld`.  5 mouse-clickable buttons (1/2, 3/4, 5/6, 7/8, 9/0)
  in FloatingUiPanel with hover state via Irrlicht button skin (color-only).
  Convenience macro button "Enter 111111" queues a full 6-digit sequence.
  Publishes `kSigDriverRsaExteriorKeypad[1..5]` (6985-6989) each tick.
  Consumer-side RSA wiring is a TODO (see electricsim/docs/TODO.md).
- [ ] **RSA interior buttons** — PRND select, EPB switch, power window
  switches.  These are RSA's internal HMI; ev1sim simulates the user
  pressing them and publishes the resulting signals to RSA.
- [ ] **PRND keyboard cycling** — up/down keys to advance the PRND
  selector (the SimApp `SetDriverGearSelector(3)` placeholder needs
  replacement with an actual cycle).  When RSA's solenoid blocks the
  shift (e.g. shift-out-of-Park without brake), ev1sim shows an
  unobtrusive cue.
- [x] **Power windows** — `PhysicalWorld::PowerWindows` models driver +
  passenger window × up/down (4 momentary bools).  Signals 6980-6983 pinned
  and published on the main harness segment.  PowerWindows have no keyboard
  binding; floating UI panel will drive them when its widget set expands.
- [x] **Seatbelt sensors** (driver, passenger) — reed switches in the
  buckles.  `PhysicalWorld::Seatbelts` holds independent driver/passenger
  bool state (default: both buckled).  Driver seatbelt publishes
  `kSigDriverSeatbeltBuckled` (6964); passenger seatbelt publishes
  `kSigDriverSeatbeltBuckledPassenger` (6965) — both on the main harness
  segment.  FloatingUiPanel toggles added (Wave 2).
  Consumer wiring (IPC seatbelt-light telltale) is a TODO on the
  electricsim side.
- [x] **Ambient temp sensor** — naive almanac-style diurnal sinusoid model
  (`AmbientTempSensor` in `PhysicalWorld`).  Publishes `kSigChassisAmbientTempC`
  (4090) and `kSigChassisAmbientHumidityPct` (4091) on the chassis bus each tick
  (epsilon-gated).  Time-of-day from system clock; fully deterministic for a
  given run time with the default config (mean 18°C, ±8°C swing, peak at 14h).
  No live weather API this round — see follow-up below.
- [ ] **Live weather API integration** (free service, with naive-almanac fallback
  if API is down) — current implementation is naive-almanac only.
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

## Outputs ev1sim could render (defer all)
- [ ] Wiper motor visual sweep (model update needed).
- [ ] Defrost grid visualization.
- [ ] HVAC blower (audio + load).
- [ ] Door lock solenoid (audio + visual click).
- [ ] Trunk animation (T key already toggles state; no visual today).

## Door handles + exterior keypad follow-up

- [ ] **Hover-magnify polish for keypad buttons** — currently uses Irrlicht
  button hover state which is hover-color only (no geometric scale-up). A
  geometric magnify would require custom IGUIElement drawing or a separate
  overlay node; deferred.
- [ ] **Door handle test scenarios** — config flag `auto_unlock_at_start: true`
  is already de facto since DoorLocks defaults to UNLOCKED.  Expose an
  explicit config or scenario JSON option to set initial lock state so
  user-interactive tests that need to iterate don't have to unlock first.

## Floating UI panel
- [x] **Mouse capture / camera escape (DONE).**  TAB toggles UI mode.
  `CameraManager::SetGrabbingMouse(false)` early-returns from `OnEvent`
  when UI mode is active so mouse clicks reach the Irrlicht GUI.
  Cursor shown/hidden via `ICursorControl::setVisible()`.
- [x] **Infrastructure landed.**  `FloatingUiPanel` module (vertical button
  column, translucent background, label-fn + click-callback per button).
  Three first instances live on the panel:
  - Hazard toggle (X keyboard path still works in parallel)
  - Door locks: Lock All/Unlock All + Driver/Passenger/Trunk individual toggles
  - Charge coupler: PLUGGED/UNPLUGGED (replaces stub-false)
- [x] **Wave 2 UI buttons landed** — the following items now have panel
  buttons:
  - Wiper cycle (OFF→INT→LOW→HIGH→OFF) + momentary Wash
  - Cruise stalk: SET, RESUME, CANCEL, +, - (5 momentary buttons)
  - IPC trip-reset (momentary)
  - Seatbelt driver toggle (BUCKLED/UNBUCKLED)
  - Seatbelt passenger toggle (BUCKLED/UNBUCKLED)
- [ ] **Remaining UI candidates** — next floating-panel rounds:
  - Trunk open/close (T key covers it but panel would be friendlier)
  - Hood open/close
  - Power windows (driver, passenger) — signals 6980-6983 pinned, no
    panel buttons yet
  - Pedals: brake pressure slider, accelerator slider (currently
    keyboard-only — no analog range exposed in FloatingUiPanel)
  - Steering wheel angle indicator / override
  - IPC LCD brightness and LED telltale overrides
  - RSA LEDs (door-lock feedback, entry-accepted, entry-denied)
  - HVAC: temp setpoint, fan speed, mode, AC, defrost
  - Shifter (PRND) — currently keyboard-only

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
- [ ] Same pattern for **throttle** (BTCM/PIM), **steering assist** etc.

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
