# ev1sim — TODO

Catch-all for physical-world / render / sim follow-ups deferred from focused
work.  When you finish an item, delete it from here.

## Physical-world components (next batches)

The `PhysicalWorld` pattern landed with `CombinationSwitch` as the first
instance.  Each item below is a new component that joins it — keyboard or
future-UI input on one side, chassis-segment signal publishing on the other.

- [x] **Hazard switch** — `PhysicalWorld::HazardSwitch` (latched push-button).
  X key toggles it and the FloatingUiPanel has a click-to-toggle button (Wave 1).
  Derived hazard-active publishes on the chassis cavity `kSigTurnHazSw_HazardOut`
  (4045); LHJB overrides both turn sides from it.
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
- [x] **RSA interior buttons** — PRND select (`PhysicalWorld::PrndSelector`)
  and power-window switches (`PowerWindows`, 6980-6983) are modelled and
  published.  EPB is covered by the park-brake path (`DriverCommand.parking_brake`
  → set/release edges `kSigDriverParkBrakeSetRequest/ReleaseRequest` 6946/6947);
  a dedicated `ElectronicParkBrake` HMI component is optional polish — see
  call-out below.
- [x] **PRND keyboard cycling** — `,`/`.` keys (`prnd_up`/`prnd_down` actions)
  advance `PrndSelector::cycle_up/down`; SimApp publishes the real selector
  position via `SetDriverGearSelector` (the `(3)` placeholder is gone).  RSA's
  shift-block cue is surfaced from `GetRsaShiftBlocked()` (4088) in a
  FloatingUiPanel status row.  Scenario JSON supports `prnd_up`/`prnd_down`.
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
- [x] **HVAC controls** — `PhysicalWorld::HvacControls` models the driver
  climate panel: temp setpoint (°C, clamped 16–30), fan (OFF/LOW/MED/HIGH),
  mode (FACE/BILEVEL/FEET/DEFROST), A/C request, and front-defrost request.
  Published to HTCM on the chassis bus (`4124` setpoint f32, `4125` fan,
  `4126` mode, `4127` A/C, `4128` defrost) each tick, publish-on-change.  No
  keyboard binding — the floating-UI panel will drive the setters when its
  widget set expands (mirrors PowerWindows).  Consumer wiring (HTCM HVAC
  inputs; IDs allocated ev1sim-side first) is a TODO on the electricsim side.

## Door / lock state

- [x] **Door lock state** modeled in `PhysicalWorld::DoorLocks` (driver,
  passenger, trunk; defaults UNLOCKED).  No keyboard binding — toggles will
  land via the floating-UI panel.
- [x] **Door lock bus signal pinning** — `DoorLocks` state is published on the
  chassis bus as `vehicle.body.door_lock_state.{driver,passenger,trunk}`
  (4155-4157, uint8 0=unlocked/1=locked), publish-on-change, mirroring the
  RSA cmd encoding (4084/4085).  Closes the `door_lock_motor` loop so RSA/IPC
  central-locking can confirm the actuated state.  IDs allocated ev1sim-side
  first; electricsim consumer wiring is a TODO on that side.
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
- [x] **SHM transport "intermittent float corruption" — root-caused.**
  Was NOT a transport bug.  Controllers' bus-drain loops did not
  filter `FrameType::SignalDefine` frames, which carry an ASCII
  signal-name payload sharing the same `signal_id` as the data
  frames they describe.  Reading the first 4 bytes of
  `"vehicle.brake.master_cylinder_pressure_kpa..."` as a float
  produced 1.756e25 every time — looked like corruption, was actually
  a missing typecheck.  Fix landed in BTCM/PIM/IPC/APM/RSA/RHJB/LHJB
  controller drains: skip non-`DeltaBatch` frames before typed
  decode (matches the existing ev1sim + scan_tool + BPM + scripted
  pattern).  Defensive single-creator SHM init also landed
  (`O_CREAT|O_EXCL` + wait-for-magic) so the multi-creator init race
  can't corrupt the mutex/cond state, even though it wasn't the
  actual bug here.
- [x] **BTCM ABS algorithm engages — confirmed working.**  After the
  SignalDefine filter fix, the BTCM AVR firmware's ABS state machine
  cycles through HOLD/DUMP phases ~720 times during the
  `abs_hard_brake.json` brake event.  The
  `scripts/run_abs_compare.sh + compare_abs_runs.py` output now
  shows clear differences between BTCM-on and BTCM-off:
    - BTCM-on rear EMB peak slip ≈ 0.66 vs free-rolling 0.43 (rear
      brakes engage and contribute to deceleration)
    - Stopping distance BTCM-on 19.16 m vs BTCM-off 19.29 m
      (marginal on dry asphalt — expected; ABS shines on low-mu)
    - Front time-locked drops slightly with ABS but not dramatically
      because dry asphalt's high friction limit barely benefits from
      modulation.
- [x] **Low-mu ABS scenario.**  `config/abs_low_mu.json` (uniform ice, µ=0.10)
  + `config/scenarios/abs_low_mu_stop.json` give the slippery-surface stop where
  the friction-limited deceleration is well below the locked deceleration.  Wired
  into the ABS regression sweep (`scripts/abs_regression.sh`, `run_abs_compare.sh`)
  with baseline thresholds in `scripts/abs_baseline.txt` (fl/fr locked < 50 %,
  phases > 25).  (Comparison run + charts require a local Chrono + electricsim
  build.)
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
- [x] **Defrost grid visualization** — HTCM chassis-bus signal 4083 subscribed;
  FloatingUiPanel renders "Defrost: ON/OFF" status row (no rear-window graphic yet).
- [x] **HVAC blower status** — HTCM chassis-bus signal 4082 subscribed;
  FloatingUiPanel renders "Blower: OFF/LOW/MED/HIGH" status row (audio/load deferred).
- [ ] Door lock solenoid (audio + visual click).
- [ ] Trunk animation (T key already toggles state; no visual today).

## Actuator/plant peripherals (specs landed; consumer wiring deferred)

The chassis-bus contract + plant models for three electricsim-driven actuators
landed (signal IDs, endpoints, decode, `PhysicalWorld` plant classes, unit
tests, and [docs/peripherals.md](peripherals.md)), and `SimApp::ConsumeBody
ActuatorPeripherals(dt)` now drives all three plants each tick in both loops.
Only the render/audio surfacing remains (tracked in the "defer all" section above):

- [x] **`door_lock_motor` spec + consume (4092-4095)** — `PhysicalWorld::Door
  LockMotor` ×2 (LH/RH); SimApp drives both motors from 4092-4095 (preferring
  the legs over the 4084/4085 mirror) and reflects end-of-travel into `DoorLocks`
  (now also republished as 4155-4157).  Remaining: door-lock solenoid
  audio/visual click (render — deferred above).
- [x] **`sounder` spec + consume (4096)** — `PhysicalWorld::Sounder`; SimApp
  advances it from 4096 each tick and detects the click edge.  Remaining: play
  the TURN/HAZ click in the audio backend (a `SounderAudio` backend like
  `HornAudio` — deferred).
- [x] **`power_steering_pump_motor` spec + consume (4097 in / 4098 out)** —
  `PhysicalWorld::PowerSteeringPumpMotor`; SimApp drives the pump from 4097 and
  publishes the HV interlock-closed boolean (4098) each tick.  Optional future:
  per-phase BLDC model on molex.A/B/C.

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
- [x] **Headlamp + turn-signal active-state display rows** — derived from
  the existing electricsim bulb feed line cache (signals 4000-block, no new
  bus subscriptions).  FloatingUiPanel now shows:
  - "Headlamps: OFF / LOW / HIGH" from LLBH/RLBH (low) and LHBH/RHBH (high)
  - "L Turn: ON / OFF" from LFTS|LRTS (flashes per tick — visible as rapid oscillation)
  - "R Turn: ON / OFF" from RFTS|RRTS (same pattern)
  These cover three IPC LCD telltale slots (HIGH_BEAM, LEFT_TURN, RIGHT_TURN)
  that the IPC supervisor declares but does not yet drive via its LCD output
  path.  The fourth declared-but-deferred IPC LCD slot, WAIT (charge-wait
  indicator), has no equivalent bulb feed line and remains fully deferred
  until electricsim adds a separate charge-state signal.
- [x] **HVAC control panel** — FloatingUiPanel rows for the driver climate
  controls (`PhysicalWorld::HvacControls`, published to HTCM on 4124-4128): a
  setpoint display row + buttons for Temp +/- (0.5 °C), fan cycle, mode cycle,
  A/C toggle, and defrost toggle.  Pure `FormatHvac*Label` helpers are unit-
  tested; the panel wiring lives in `SimApp`.
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
- [x] **Throttle bus path (PIM)** — `kSigChassisThrottleCmdQ8` (4073) is
  published by PIM each tick (passthrough or cruise-controlled).  ev1sim
  subscribes via `ExternalSimConnector::GetThrottleCmd(window)`,
  `SimApp::ApplyElectronicsThrottle` overrides `cmd.throttle` when fresh,
  falls back to local pedal at 200 ms staleness.  Selected via
  `vehicle_dynamics.driver = "electronics"`.  Cruise demo exercises the
  full loop end-to-end (`config/scenarios/cruise_demo_electronics.json`).
- [x] **Steering bus-input** — `kSigChassisSteeringCmd` (4076, float32 norm
  -1..+1), the symmetric counterpart of the throttle path.  Connector
  `GetSteeringCmd(window)` + `SimApp::ApplyElectronicsSteering` override the
  local steering in "electronics" drive mode when fresh, else fall back.
  Allocated ev1sim-side first: **no electricsim producer today** — a closed-loop
  / physical-rig steering input, distinct from the PSCM power-steering pump
  peripheral.  (Power-steering *assist* itself is the `power_steering_pump_motor`.)

## Sim-time sync — ev1sim publisher side

- [x] **Publish sim-time on the chassis bus (DONE).**  ev1sim publishes
  `kSigChassisSimTimeNs` (chassis ID **4075** — *not* 4070, which is
  already motor RPM; the old "suggested 4070" note here was wrong and
  would have collided) every physics step as uint64 little-endian
  nanoseconds, sourced from `VehicleWorld::GetSimTime()` and encoded via
  `MakeU64Delta` in the per-tick chassis dynamics frame.  This unblocks
  electricsim's `SimClock` master handoff — BTCM/PIM/IPC already
  subscribe and stay in wall-clock mode until the first 4075 sample
  arrives.  Cross-repo design: electricsim/docs/TODO.md "Sim-time sync".

## Future: physical sim rig
- [ ] Adapt the user's physical driving sim rig with a lookalike RSA, seat
  belt sensor, IPC display, chime/horn sound effects.  Hook real HVAC
  controls.  ev1sim's PhysicalWorld pattern should grow a hardware
  backend that maps real switches to chassis-bus signals (replacing
  keyboard input).
