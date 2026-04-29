# ev1sim — TODO

Catch-all for physical-world / render / sim follow-ups deferred from focused
work.  When you finish an item, delete it from here.

## Project Chrono startup state
- [ ] **Start with parking brake set, vehicle off.** Currently Chrono boots
  with the vehicle in an "engine on" state, which doesn't match the EV1's
  expected RSA startup sequence (RSA controls the OFF/ACC/RUN/START state
  via internal logic, not a key cylinder).  Set Chrono's initial state so
  brakes are clamped and propulsion is disabled until RSA tells us to go.
  Symptom this might explain: vehicle behaves "engine running" before any
  ECU pipeline is ready, masking real ECU control issues.

## Physical-world components (next batches)

The `PhysicalWorld` pattern landed with `CombinationSwitch` as the first
instance.  Each item below is a new component that joins it — keyboard or
future-UI input on one side, chassis-segment signal publishing on the other.

- [ ] **Hazard switch** (latched push-button on column).  Defer keyboard
  binding; needs a floating UI panel for click-to-toggle (see floating-UI
  item below).
- [ ] **Turn signal stalk** — left/right detents with return-to-center
  behavior.  Shares the same physical column as the combination switch.
  Needs `kSigDriverTurnSignalLeft/Right` pinned in
  `electricsim/src/io/ev1_driver_inputs.hpp`.
- [ ] **Wiper switch** — off, intermittent, low, high (+ delay).  Right
  column stalk.
- [ ] **Cruise control stalk** — set, resume, cancel, +/-.  Left column
  stalk on EV1.
- [ ] **IPC trip-reset button** — single momentary on the cluster.
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
- [ ] **Brake torque from BTCM actuator state.** Today the user's brake
  input drives Chrono's brake torque directly.  Migrate to: BTCM publishes
  per-wheel actuator state (solenoid iso/dump tuple, EMB motor command,
  wheel cylinder pressures `kSigPressLF/RF`); ev1sim consumes and uses
  these to drive Chrono's brake torque per wheel.  Local brake input
  becomes the fallback when bus state is stale.  This is what makes
  ABS modulation visible in the vehicle's dynamics.
- [ ] Same pattern for **throttle** (BTCM/PIM), **steering assist** etc.

## Future: physical sim rig
- [ ] Adapt the user's physical driving sim rig with a lookalike RSA, seat
  belt sensor, IPC display, chime/horn sound effects.  Hook real HVAC
  controls.  ev1sim's PhysicalWorld pattern should grow a hardware
  backend that maps real switches to chassis-bus signals (replacing
  keyboard input).
