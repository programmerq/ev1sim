# ev1sim — TODO

Catch-all for physical-world / render / sim follow-ups deferred from focused
work.  When you finish an item, delete it from here.

## IPC + RSA display rework (planned)

Rework ev1sim's HUD to surface the IPC instrument cluster + RSA center console
as in-app displays (2D panels first, 3D later), fed by **decoding the GM-8192
`*_TX` frame cells off the wire** (Palm-EV1-dashboard style) rather than
per-value chassis cells. Display-only first; interactivity later. Full plan +
ratified decisions: [`docs/ipc_rsa_display_plan.md`](ipc_rsa_display_plan.md).

- [x] **Phase 1 — IPC cluster, 2D, snoop-fed (display-only)**: `BusSnoop`
  (`src/BusSnoop.{h,cpp}`) decodes $41 / $40 / $20 / $90 / $91 / $21 off the PIM,
  BPM, BTCM and IPC `*_TX` cells into a freshness-gated `DashSnapshot`;
  `ExternalSimConnector::GetDashSnapshot()` adds the IPC lamp cells (4130-4145);
  `SimApp` draws `InstrumentClusterPanel` bottom-centre when the external sim
  is on. Decode, label, null-driver draw and `[e2e]` tests.
- [x] **Phase 2 — RSA console, 2D (display-only)**: `RsaConsolePanel`
  (`src/RsaConsolePanel.{h,cpp}`) draws the console indicators the external sim
  drives (OFF/ACC + RUN mode LEDs, P R N D, VALID CODE / keypad lamp, security,
  shift lock, BTSI solenoid, backlight) from the RSA output cells via
  `WireTruthChassis::rsa_console()`, with the external sim's wire-mode panel
  derivations. The catalog's `stub` HVAC/fan/clock/trip LEDs are left out.
  Drawn right of the cluster when the external sim is on.
- [x] **Phase 3 — 3D in-scene** cluster + console (same snapshot feed):
  `DashScreens3D` (`src/DashScreens3D.{h,cpp}`) draws the two 2D panels into
  render-target textures on quads fixed to the chassis frame (cluster at the
  dash top below the windshield, console low between the seats; positions
  read off the chassis visual mesh, no interior dimensions in hand). The new
  **Cabin** camera mode looks at both from the driver's seat.
- [x] **Phase 4 — interactivity**: the console panel's controls are
  clickable in UI mode (Tab), on the 2D panel and on the 3D console screen in
  the Cabin view (`src/RsaConsoleInput.{h,cpp}`). OFF/ACC, RUN and LOCK go
  through the keypad driver's mode-switch bits, P/R/N/D through the selector's
  pins, the five keys through the keypad driver (held 0.5 s or more = long
  press), and COAST DOWN toggles the driver's coast-down switch, lit from the
  PCM's $41 Coast Down LED request. Every click reuses a producer ev1sim
  already had (the keyboard's, or the coast-down cell's scenario hook), so
  ev1sim gains no new writer on any cell and the
  arbitration with electricsim's web RSA panel is unchanged.

## Physical-world components (next batches)

The `PhysicalWorld` pattern landed with `CombinationSwitch` as the first
instance.  Each item below is a new component that joins it — keyboard or
future-UI input on one side, chassis-segment signal publishing on the other.

- [x] **Live weather for the ambient almanac.**  `scripts/fetch_weather.py`
  fits a real day's hourly temperature and humidity (Open-Meteo) to the
  almanac's daily cycle and writes a weather file; a config names it under
  `environment.weather` (see `Config::Environment::Weather`).  The sim reads it
  at startup only and stays on sim time (`AmbientTempSensor::hour_of_day`);
  without a file, or when the script cannot reach the service, the almanac
  keeps its defaults.
- [ ] **`power_steering_pump_motor` per-phase BLDC model (optional).**  The pump
  plant (`ev1sim::PowerSteeringPumpMotor`, chassis 4097 in / 4098 interlock-closed
  out) is currently a lumped-motor model; a per-phase BLDC model on `molex.A/B/C`
  is an optional future refinement.

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
  The real re-derivation is seven headless scenario runs, so it is opt-in
  locally — `make test-runway-table`, or `ctest -L slow` when configured
  `-DEV1SIM_SLOW_TESTS=ON`.  Timing, measured both places: **3 min 02 s** on a
  GitHub runner with a warm cache, ~18 min on the developer machine it was
  first timed on.
  **In CI it is its own `settle-table` job on the CRON lane**, not the PR path
  (`if: schedule || workflow_dispatch`, `needs: chrono-smoke` for the warm
  Chrono cache).  Adding it to a lane that gates every PR is the wrong
  direction, and sharing a job with a cold Chrono build could exceed a ceiling
  and turn a correctness check into a flaky one; nobody waits on the cron.  Its
  own job also means a red names what broke rather than reddening the smoke
  job.  A failure surfaces as a red scheduled run carrying an `::error::`
  annotation that names the fix, a job-summary block, and GitHub's default mail
  for a failed scheduled workflow; `runway.log` is kept as an artifact for 30
  days.  The script prints a completion marker and the step refuses to pass
  without it, so a run killed by the timeout cannot leave a log that reads like
  a clean one — which is not hypothetical: the first dispatch crashed on a
  relative `--binary` path and the step correctly reported that nothing had
  been checked rather than a verdict.  `workflow_dispatch` exists so a red cron
  can be re-checked without waiting three days for the next.

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
## Outputs ev1sim could render
- [x] Wiper sweep: `WiperBlades3D` (`src/WiperBlades3D.{h,cpp}`) draws both
  wipers as arm and blade bars on the windshield (the chassis mesh has no wiper
  parts), swung to the blade position the RHJB's wiper-motor plant publishes
  (`CHASSIS_WIPER_SWEEP_POSITION`), so the park return after OFF shows too; the
  HUD's WIPER bar follows the same position. Parked blade-tip heights from the
  body manual (p. 126); pivots, lengths and sweep angle are `@design`.
- [x] Door lock visual: the body diagram (bottom right) has a lock knob inside
  each door and the trunk, sliding with the door-lock motor's stroke (amber
  mid-stroke, red latched locked); the audio click still plays at end of
  travel.
- [x] Trunk lid opens in 3D: `TrunkLid3D` (`src/TrunkLid3D.{h,cpp}`) swings
  the lid about its front-edge hinge while the trunk is open (T key or the UI
  panel). The lid is its own mesh, `ev1_trunk_lid.obj`, split out of the
  chassis visual mesh by `ev1model/split_trunk_lid.py`; open angle and travel
  time are `@design`.

## Floating UI panel
- [ ] **IPC LCD brightness and telltale overrides** — the telltale cells are
  the IPC's own outputs, so a panel override is a second writer on cells the
  IPC owns; settle that with electricsim first. (The console buttons needed
  no such step: they reuse the keyboard's producers, Phase 4 above.)
- [ ] **RSA door-lock feedback and entry-denied LEDs** — no RSA output cell
  carries them yet (entry accepted is the console panel's VALID CODE lamp);
  needs the cells in electricsim first.
- [ ] **IPC WAIT (charge-wait) telltale row** — blocked until electricsim
  adds a separate charge-state signal.

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
