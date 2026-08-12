# ev1sim — TODO

Catch-all for physical-world / render / sim follow-ups deferred from focused
work.  When you finish an item, delete it from here.

## IPC + RSA display rework (planned)

Rework ev1sim's HUD to surface the IPC instrument cluster + RSA center console
as in-app displays (2D panels first, 3D later), fed by **decoding the GM-8192
`*_TX` frame cells off the wire** (Palm-EV1-dashboard style) rather than
per-value chassis cells. Display-only first; interactivity later. Full plan +
ratified decisions: [`docs/ipc_rsa_display_plan.md`](ipc_rsa_display_plan.md).

- [ ] **Phase 1 — IPC cluster, 2D, snoop-fed (display-only)**: a `BusSnoop`
  frame-decode layer → `DashSnapshot`; `InstrumentClusterPanel` + pure label
  helpers; frame-decode + label + `[e2e]` tests.
- [ ] **Phase 2 — RSA console, 2D, snoop-fed (display-only)**: `RsaConsolePanel`
  mirroring electricsim `rsa_panel_catalog.yaml`.
- [ ] **Phase 3 — 3D in-scene** cluster + console (same snapshot feed).
- [ ] **Phase 4 — interactivity**: clickable RSA inputs; resolve multi-writer
  arbitration with electricsim's RSA controller first.

## Physical-world components (next batches)

The `PhysicalWorld` pattern landed with `CombinationSwitch` as the first
instance.  Each item below is a new component that joins it — keyboard or
future-UI input on one side, chassis-segment signal publishing on the other.

- [ ] **Live weather API integration** (free service, with naive-almanac fallback
  if API is down) — current implementation is naive-almanac only.
- [ ] **`power_steering_pump_motor` per-phase BLDC model (optional).**  The pump
  plant (`ev1sim::PowerSteeringPumpMotor`, chassis 4097 in / 4098 interlock-closed
  out) is currently a lumped-motor model; a per-phase BLDC model on `molex.A/B/C`
  is an optional future refinement.

## Powertrain

- [ ] **No top-speed limiter anywhere (the ~80 mph software cap).**
  ev1sim's plant has no speed limiter at all, and it should not have one —
  @source:manual propulsion p250 puts the cap in the PIM: "The PIM will limit
  vehicle speed in the forward direction to 129 km/h (80 mph)", enforced by
  decreasing the torque current (48 km/h / 30 mph in reverse).  So the work is
  on the **electricsim PIM** side, not here: a torque-command taper keyed on
  **motor shaft speed**, which is what the real PCM watches (prop p326 — the
  speed/direction sensor is "attached to the end of the rotor shaft"; p336 —
  the PCM "uses and monitors the motor shaft speed signal … including vehicle
  speed signal generation, torque control, and cruise control").  Not road
  speed: the BTCM owns true road speed and reaches the PCM as a torque retard
  request, not a speed feed.
  ev1sim's side of that is mostly done — 4070 now publishes the unclamped
  shaft speed (`SimApp::PublishMotorState`), so a PIM-side limiter has an
  honest signal to act on.  Still open on this side: 4070 can never go
  negative, because the Chrono transmission is permanently in forward gear
  (nothing calls `SetDriveMode`) and `ChAutomaticTransmissionSimpleMap` takes
  `std::abs(driveshaft_speed)` anyway, while the chassis-signal contract
  describes 4070 as signed with negative meaning reverse.  Reverse is not
  modelled at all today, so nothing is wrong yet — but a reverse limiter
  (30 mph, same p250 sentence) needs a signed 4070 first.

- [ ] **Two contradictory drive ceilings across the two repos.**  This repo now
  carries 16 000 RPM, sourced from propulsion p328, as the speed above which the
  propulsion system makes no torque.  electricsim still carries 13 000 RPM in
  `ev1/pim/pim_drive_plant.h` labelled "redline guard", with a header note that
  "the vehicle's top speed is road-load-limited, not rev-limited".  Both halves
  of that note are now contradicted: 13 000 RPM is the ~80 mph *software cap*
  restated in RPM, not a redline, and top speed is set by that cap rather than
  by road load.  The number wants reclassifying rather than deleting — it is a
  reasonable stand-in for the software cap, just mislabelled.  Related:
  `electricsim/docs/pim_hv_inverter_research_brief.md` records as an open
  question that "'~13,000 rpm' is not confirmed by any authoritative source …
  treat any >10,000 rpm redline as [EV1-INFER] unless a manual page says
  otherwise" — propulsion p328 is that page, so the question can be closed.
  **Same file, same shape, second number (2026-08-11):** `pim_drive_plant.h`
  also carries a **150 N·m** torque clamp.  That is the secondary-web-figure
  peak this repo has now replaced with the 141 N·m propulsion p250 states, so
  the two repos' drives now disagree about peak torque by 6 % as well as about
  the speed ceiling.  Both halves are one reclassification pass on that one
  header; recorded here rather than reached across repos.
  **Third number (2026-08-12):** that clamp is **symmetric** —
  `cmd_torque_mnm` is clamped to `±PIM_DP_MOTOR_MAX_TORQUE_MNM` in `_tick`, so
  the *regen* side is bounded at 150 N·m and by no power limit at all.  At
  10 000 RPM that permits ~157 kW of regeneration against the 10 950 W
  propulsion p60/p209 states ("The maximum allowed regeneration is 365 volts
  DC and 30 amps DC") — ~14×, and on the repo that models the PIM, which is
  the module the manual attributes the limit to.  It is the same defect §3.1
  fixes here, one repo over and larger.  Note the ev1sim fix does not
  propagate: this file is electricsim's own plant.  Still recorded rather than
  reached across.

## ABS scenarios

- [x] **The three uniform-surface stops brake on the barrier tick.**
  Done 2026-08-12.  `abs_high_mu_stop`, `abs_hard_brake` and
  `abs_brake_and_steer` scheduled `set_brake` at 7.0 / 7.0 / 6.0 s, all behind
  a `wait_for_speed` barrier releasing at 15.028 / 15.028 / 12.053 s, so full
  brake landed on the exact tick the throttle dropped to zero.  Brakes moved to
  18.0 / 18.0 / 15.0 s — a 2.97 / 2.97 / 2.96 s settle — with brake values, mu
  values, the 0.30 s brake-to-steer offset and the hold durations (18 / 18 / 19
  s) all unchanged; `max_time_s` extended to keep the same 5 s tail after brake
  release.  `scripts/scenario_runway_report.py` now covers all seven ABS
  scenarios rather than the four transition ones, and
  `config/abs_hard_brake.json` was added because `abs_hard_brake` had no config
  wrapper at all — which is part of why nobody had measured it.

- [x] **The settle table's barrier-release times are frozen measurements that
  nothing re-derives.**  Done 2026-08-12.  The table moved out of
  `tests/test_scenario.cpp` into `config/scenarios/measured_settle.json` — one
  copy, read by the `[Runway]` cases, by the coverage guard, and by
  `scripts/scenario_runway_report.py`, which now **re-derives**
  `measured_release_s` and `measured_crossing_s` from a headless run and exits
  non-zero when a derived value sits more than 0.05 s from the recorded one.
  `--update` re-records what it measured, so a legitimate plant move is a
  command rather than a hand-edit; `--selftest` drives the comparison across
  its own tolerance (0.025 / 0.045 / 0.100 s against 0.050) and is registered
  as the `runway_table_selftest` ctest, which runs no simulation.
  Two things found while doing it.  The report **could not be run the way its
  own docstring documented**: `scripts/scenario_runway_report.py` with no
  scenario names exited 2 without simulating anything, because `argparse`
  checks a `nargs="*"` list default against `choices` as a single value.  And
  the table conflated two different reasons for a `-1` crossing — "no boundary
  exists" (the three uniform stops) and "the crossing is inside the brake event
  by design" (`abs_mu_jump`) — which the JSON now distinguishes, so the
  re-derivation can check the crossings that are entry conditions and skip the
  one that is the subject of its test.
  The real re-derivation is seven headless runs of a few minutes each (~18 min
  measured), so it is opt-in locally — `make test-runway-table`, or
  `ctest -L slow` when configured `-DEV1SIM_SLOW_TESTS=ON`.
  **In CI it runs on the CRON lane**, not the PR path: the `schedule` /
  `workflow_dispatch` branch of `chrono-smoke`, which already builds the
  `ev1sim` binary it needs.  Adding 18+ minutes to a lane that gates every PR
  is the wrong direction, and on a Chrono cache miss it could exceed the job
  ceiling and turn a correctness check into a flaky one; nobody waits on the
  cron.  A failure surfaces as a red scheduled run carrying an `::error::`
  annotation that names the fix, a job-summary block, and GitHub's default mail
  for a failed scheduled workflow; the report log is kept as an artifact for 30
  days.  The script prints a completion marker and the step refuses to pass
  without it, so a run killed by the timeout cannot leave a log that reads like
  a clean one.  `workflow_dispatch` exists so a red cron can be re-checked
  without waiting three days for the next.

- [x] **The coast map exceeds the manual's stated regeneration bound above
  ~8400 RPM.**  Done 2026-08-12, and the framing above was wrong in the way
  that mattered.  It is not a category error that the bound is electrical and
  the map "mechanical drag": @source:manual propulsion p57, "COAST DOWN
  FUNCTION" says the zero-pedal negative torque IS commanded regenerative
  braking ("The coast down feature uses a calibrated amount of regenerative
  braking... The PCM controls coast down by providing a negative torque current
  as a function of drive motor shaft speed/direction sensor rate"), so the
  printed limit on p60/p209 applies to it directly and the curve was in
  straight violation — 5.5× the ceiling at its worst, not merely implausible.
  Curve re-cut as constant power above an 8350 RPM knee; no breakpoint at or
  below 8000 RPM changed value.  That is **not** the same as leaving the
  2026-04-30 calibration alone — coastdown spans 3586–10 758 RPM, so 34 % of
  its fitted range moved and the fit's CdA shifted 8×.  Details, the conditions
  the map still does not represent, and the coastdown effect are in
  `docs/ev1_chrono_audit.md` §3.1 and §11.1.

## ABS scenario integration artefacts (need a built electricsim)

- [ ] **Re-run the BTCM-on/off sweep and regenerate the engineering report.**
  `docs/reports/abs_scenarios.md` is generated by `scripts/abs_report.py` from
  a six-scenario `scripts/run_abs_compare.sh` sweep, and its header still says
  _Generated 2026-05-02_.  It therefore predates PR #52 (throttle ramps, and
  `abs_low_mu_stop` moving to an asphalt launch on packed snow) **and** the
  2026-08-11 branch (motor corner point; `abs_mu_jump` / `abs_split_mu` /
  `abs_diagonal_mu` runways).  Every headline number in it is measured against
  inputs that no longer exist.  Not hand-editable — it is a generated file, and
  regenerating it needs PIM/BTCM/RSA binaries built from electricsim, which the
  unit suite deliberately does not depend on.  `scripts/abs_baseline.txt` needs
  the same re-capture and carries a stale marker saying so.

## Door / lock state

- [ ] **Slip-dynamics drag calibration (deferred).**  CdA still 6.5× spec
  in coastdown after the geometry fix.  The remaining excess lives in
  TMeasy's internal slip-curve shape; the exposed `Initial Slopes
  dFx/dsx` knob doesn't usefully reduce it (lowering trades F_rr for
  CdA, raising past 2× hits numerical instability).  Defer until we
  have measured EV1 slip-curve data or migrate to a Pacejka tire model.
  (The `Body aerodynamics` follow-up — "trim tire rolling/slip dissipation
  now that drag is no longer lumped into it" — is this same item.)
  **Updated 2026-08-12:** re-fit this against the corrected coast map, not the
  old one, and run it as `./build/ev1sim --config config/coastdown.json`.
  The pre-2026-08-12 coast curve's torque rose with speed, and a two-parameter
  `F_rr + CdA·v²` fit has nowhere to put a rising term but CdA — so §11's "2×
  on both terms" was partly an artefact of an unsourced curve.  Bounding the
  coast map moves **CdA 0.914 → 0.621 m²** (2.54× → 1.72× spec) and **F_rr
  188.9 → 231.7 N**.  Below the knee, where the two maps are identical, both
  fits agree to 0.3 N and 0.003 m² — use that sub-knee control
  (`--v-max 23.28`) as the check that any future fit is measuring the plant and
  not the window.  The excess is reduced, **not** explained away: CdA is still
  1.7× spec, and an earlier version of this entry wrongly concluded the residual
  was "not aero-shaped" on the strength of a time-truncated fit.
  `scripts/fit_coastdown.py` no longer has that 30 s cap: pass it both CSVs and
  it fits them over the speed range they share, names it, and refuses when
  there isn't one.  Its numbers also now carry a spread, and a window that
  cannot separate F_rr from CdA says so instead of printing a confident value —
  which is what makes "CdA is 1.7× spec" a claim worth acting on and the
  retracted "0.09 m², a quarter of spec" one that never was (it was
  0.088 ± 0.145 m²).
- [ ] **EMB shoe-force integrator (refinement).**  Current model
  treats the BTCM cmd (-1 / 0 / +1) as a proportional force command.
  More faithful: integrate cmd × motor_speed × dt to track shoe
  position, then derive force from spring-like compliance.  Defer
  until measured EV1 EMB response data lands.  (Per-wheel rear
  modulation already ships: `ApplyRearEmbBrake`, `src/SimApp.cpp:1100-1157`,
  consumes `kSigRearMotorLR/RR` and drives each wheel via
  `BrakeDrum::torque_magnitude_nm`; the command→force step at
  `SimApp.cpp:1126` is still a crude linear map — this refinement replaces
  it with a faithful clamp/shoe-position model.  This item is the surviving
  tracker for that residual clamp/shoe-position work.)
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
  **Provenance flag (2026-08-12):** this entry credits
  `run_abs_compare.sh` with running `abs_hard_brake`, but until this date
  that script's whitelist rejected `hard_brake` and `config/abs_hard_brake.json`
  did not exist — so whatever produced these numbers, it was not the invocation
  named here.  The stopping distances below (19.16 / 19.29 m) are also an order
  of magnitude off this branch's measured ~102 m for the same scenario, which
  says they came from a different scenario, a different plant, or both.  Left
  as the historical record rather than rewritten; do not treat these figures as
  current.
    - BTCM-on rear EMB peak slip ≈ 0.66 vs free-rolling 0.43 (rear
      brakes engage and contribute to deceleration)
    - Stopping distance BTCM-on 19.16 m vs BTCM-off 19.29 m
      (marginal on dry asphalt — expected; ABS shines on low-mu)
    - Front time-locked drops slightly with ABS but not dramatically
      because dry asphalt's high friction limit barely benefits from
      modulation.
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
- [ ] **Rear axle template: the EV1 rear is a beam axle, not a wishbone.**
  Chassis manual p. 271: a one-piece rear axle (aluminum end castings,
  tubular center section) on five links — two upper leading links, two
  lower trailing links, and a track bar — with two coil springs and two
  shocks.  `EV1_RearDoubleWishbone.json` models control arms, uprights
  and tie rods that do not exist on the car, and models no track bar.
  Chrono 9.0.1's JSON factory does accept two candidates: `SolidAxle`
  carries exactly the EV1's link set (two upper links, two lower links,
  a track bar) but is a *steered* axle — knuckles, tie rod, drag link,
  bell crank — which the rear is not; `GenericWheeledSuspension` is
  fully data-driven and could express the layout as written.  The
  blocker is not the template, it is that neither can be filled in: no
  EV1 rear hardpoint is published (see the next item), so a swap would
  replace one set of invented coordinates with another.  Deferred; the
  file labels itself a stand-in in its `//PROVENANCE` block.
  *(The front does **not** need replacing: chassis p. 251 describes a
  short/long arm independent front suspension, which is what
  `DoubleWishbone` models.  Earlier notes here claimed MacPherson front
  and twist-beam rear; both were wrong.)*
- [ ] **EV1 suspension hardpoints — no source found.**  Every coordinate
  in both suspension JSONs is Chrono's `sedan` sample rescaled to the
  printed track, and both files say so.  A search of both service
  manuals and all four parts-catalog sections turned up a wrench size,
  a tap size and hub runout limits — no geometry.  Reopen only if a
  measured car, a drawing, or a dimensioned figure turns up.

## Outputs ev1sim could render (defer all)
- [ ] Wiper motor visual sweep (model update needed).
- [ ] Door lock solenoid **visual** click — the audio click already plays via
  `SounderAudio::PlayClick` (fired on motor end-of-travel in `SimApp`); only the
  on-screen visual remains.
- [ ] Trunk animation (T key already toggles state; no visual today).

## Door handles + exterior keypad follow-up

- [ ] **Hover-magnify polish for keypad buttons** — currently uses Irrlicht
  button hover state which is hover-color only (no geometric scale-up). A
  geometric magnify would require custom IGUIElement drawing or a separate
  overlay node; deferred.

## Floating UI panel
- [ ] **Remaining UI candidates** — next floating-panel rounds:
  - Trunk open/close (T key covers it but panel would be friendlier)
  - Hood open/close **(done)** — modeled as a two-stage latch (Hood: CLOSED →
    POPPED → OPEN). Single primary-latch ajar sensor (6962) trips as soon as it
    pops, so POPPED/OPEN are indistinguishable on the bus while ev1sim tracks the
    full state. Panel: status row + Interior Release / Raise / Lower-Latch
    buttons; F = pop/close shortcut. VehiclePanels.HOOD delegates to Hood.
  - Power windows (driver, passenger) **(done)** — momentary press-and-hold
    panel buttons (Drv/Pass Up/Down) drive signals 6980-6983 via the new
    FloatingUiPanel hold-button support (polls IGUIButton::isPressed())
  - Pedals: brake pressure slider, accelerator slider (currently
    keyboard-only — no analog range exposed in FloatingUiPanel)
  - Steering angle indicator **(done)** — display row, front road-wheel angle
    (rad→deg, L/R); analog override still TODO
  - IPC LCD brightness and LED telltale overrides
  - IPC WAIT (charge-wait) telltale row — blocked until electricsim adds a
    separate charge-state signal.  (The other three IPC LCD telltale slots —
    HIGH_BEAM, LEFT_TURN, RIGHT_TURN — are already surfaced from the bulb feed
    line cache.)
  - RSA LEDs (door-lock feedback, entry-accepted, entry-denied)
  - Shifter (PRND) **(done)** — "Gear Up/Down" panel buttons added (mirror the
    `,` / `.` keys); display row was already present

## Bus-mediated physics
- [ ] **Regen ↔ friction blending (S15 — documented gap, do NOT fabricate).**
  Friction braking is regen-agnostic and regen (PIM's motor-current readout,
  chassis 4072 `kSigChassisMotorCurrentA`, derived/published by electricsim's
  PIM — see `src/ExternalSimConnector.cpp:244-252`) is a display-only ammeter
  readout that never feeds wheel torque,
  so the **regen-cutout fail-safe is untestable in-sim** (no regen torque
  to cut, no blend to hand off to friction).  Documented fully in
  [docs/ev1_chrono_audit.md](ev1_chrono_audit.md) §15.  (The earlier
  `src/MotorCurrent.h` limitation block was removed when PIM took over
  pack-current derivation — commit `4d49a55`.)  Blocked on
  EV1 regen-torque-vs-speed + blend-handover data the manuals don't
  publish — modelling it would mean inventing constants.  When traceable
  structure (or a justified structure-only `@design` model) lands: add a
  signed `propulsion_torque` channel to `DriverCommand` (regen = negative
  torque, per ARCHITECTURE.md "Future per-axle brake control"), sum
  regen + friction to the driver demand in a BTCM-side blend controller,
  backfill friction on regen cutout — then the fail-safe becomes a real
  closed-loop test.

## Future: physical sim rig
- [ ] Adapt the user's physical driving sim rig with a lookalike RSA, seat
  belt sensor, IPC display, chime/horn sound effects.  Hook real HVAC
  controls.  ev1sim's PhysicalWorld pattern should grow a hardware
  backend that maps real switches to chassis-bus signals (replacing
  keyboard input).

## Vendored stub hygiene

- [x] **Vendored electricsim stub carried upstream-internal references — scrubbed
  upstream, re-vendored here.**  `tests/electricsim_stub/src/io/wire_table.hpp`
  and `tests/electricsim_stub/src/net_host/conductor_publisher.hpp` picked up
  upstream review-finding identifiers and source paths when PR #48 re-vendored
  the stub set.  Both files are intentionally byte-identical to their upstream
  originals, and the stub's integrity check treats any local edit as drift, so
  the references could not be scrubbed in this repo — the fix landed upstream in
  electricsim first (tracked there as
  `BL-2026-08-09-vendored-stub-leaks-private-refs-to-ev1sim`), and this repo
  re-vendored both files from that commit.

  **The identifiers are gone.  The source paths stay, deliberately.**  Upstream
  ruled that a path naming a real source file is not a private reference — it
  points at a file, where a finding id points at a document a reader outside that
  repo cannot open — and that the same rule had already been applied when these
  files were scrubbed the first time.  So `src/net/net_types.hpp` and
  `config/nets/<module>.yaml` remain in `wire_table.hpp`, and the third path this
  entry implied was in these two files, `src/io/topology/feed_query.hpp`, turns
  out to sit in `tests/electricsim_stub/src/io/ev1_chassis_signals.hpp` instead.

- [ ] **`ev1_chassis_signals.hpp` still carries upstream section citations.**
  The scrub above covered two of the 17 vendored files.
  `tests/electricsim_stub/src/io/ev1_chassis_signals.hpp` was untouched by it and
  by the scrub before it, and still cites upstream planning documents by section
  number in several comments.  Same constraint as before — byte-identical by
  design, so it cannot be fixed here.  Upstream carries the worklist on
  `BL-2026-08-10-no-private-ref-audit-on-vendorable-files`, together with the
  automated check whose absence is why this keeps recurring; re-vendor that file
  here once it lands.
