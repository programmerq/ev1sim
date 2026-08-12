# ABS validation — overview

This is the high-level narrative for the EV1 ABS validation suite.
For raw numbers, charts, and per-scenario engineering data, see the
auto-generated [**ABS engineering report**](reports/abs_scenarios.md)
— that's the data artifact you'd flip through to evaluate behavior.
The companion [**tone-ring validation report**](reports/tone_ring_validation.md)
isolates the simavr edge-counting plumbing from the chassis side.

## The four standard scenarios

| Test       | Surface                          | What it validates                              |
|---|---|---|
| `high_mu`  | uniform asphalt (μ ≈ 0.9)        | ABS shouldn't hurt on dry pavement             |
| `low_mu`   | asphalt → packed snow (μ ≈ 0.20), brakes on the snow | ABS should keep wheels rolling on slippery |
| `mu_jump`  | asphalt → ice transition, brakes on asphalt then crosses under braking | algorithm must adapt as grip suddenly drops    |
| `split_mu` | left = asphalt, right = ice, brakes on the split | per-wheel ABS keeps the car going straight    |

Industry calls these "high-mu stop", "low-mu stop", "split-friction
stop", and "mu-jump" / "transition" stop.  μ ("mu") is the friction
coefficient between tire and road.

**The four scenarios that change surface — `low_mu`, `mu_jump`,
`split_mu` and `diagonal_mu` — finish their launch on grip and settle
before they reach the surface under test.**  That is a property of the
level geometry, and it is measured rather than intended: each
transition level's runway is sized to hold the whole pre-brake half of
its scenario, and `scripts/scenario_runway_report.py` re-derives that
budget from a headless run.

When a runway is too short the scenario stops testing what its name
says without failing anything.  `mu_jump` used to cross onto the ice at
full throttle and brake 34 m past the boundary, so no mu-jump happened
at all; `split_mu` used to straddle the seam under power and reach its
brake event already 3.4° off heading — on the one test whose verdict
*is* the yaw.  The budgets live in the `level/*.json` headers, the
geometry is pinned by `tests/test_level_file.cpp`, and the brake
timing that produces the settle is pinned by the `[Runway]` cases in
`tests/test_scenario.cpp` against the measured barrier releases in
`config/scenarios/measured_settle.json`.

Those releases are measurements, so since 2026-08-12 they are **re-derived
rather than trusted**: `scripts/scenario_runway_report.py` runs each scenario
and fails when a derived release or crossing sits more than 0.05 s from the
recorded one.  The C++ cases catch a brake moved backwards and need no Chrono;
the script catches the plant moving forwards under a table that still says what
it always said.  Neither can see the other's failure, which is why both exist.
Re-measure with `--update` and commit what moved.

**The three uniform-surface stops — `high_mu`, `hard_brake` and
`brake_and_steer` — now have that settle too (2026-08-12).**  Their
`set_brake` used to sit behind the `wait_for_speed` barrier, so full
brake was applied on the tick the throttle released: no coast, drivetrain
still loaded, launch slip still in the tyres.  Nothing was mistimed
relative to a transition — they launch and brake on one surface — but the
entry condition was a transient rather than a steady state, which is the
same defect the transition four carried.  The brakes moved to 18.0 / 18.0
/ 15.0 s, past their measured barrier releases of 15.028 / 15.028 /
12.053 s.  Brake values, mu values, the 0.30 s brake-to-steer offset and
the hold durations in the files are unchanged; only the schedule moved.

One precision about "hold durations unchanged": that is true of the
numbers in the scenario files (18 / 18 / 19 s), not of what the car did.
Because the old brake fired late — on the barrier release, not at its own
`at_time_s` — while brake-off stayed at 25.0 s, the *realized* hold was
~10.0 / 10.0 / 12.9 s.  It is now the full 18 / 18 / 19 s.  That changes
no measurement here: the car reaches standstill in 5–7 s, well inside
either hold.

`scripts/scenario_runway_report.py` now covers all seven rather than the
four transition scenarios, and so do the `[Runway]` cases in
`tests/test_scenario.cpp` — the second case there used to assert these
three were *still* broken, and has been replaced by a guard requiring
**every** `config/scenarios/abs_*.json` that gates a brake behind a
barrier to appear in the measured settle table.  A new scenario written
with the same defect now fails on arrival instead of sailing past a
hard-coded list of three.

The report also could not be run the way its own docstring documented: with no
scenario names it exited 2 without simulating anything, because `argparse`
checks a `nargs="*"` list default against `choices` as a single value.  Fixed
2026-08-12.  A re-derivation tool whose default invocation does not run is a
fair part of why nobody re-derived anything.

`abs_hard_brake` also gained a config wrapper (`config/abs_hard_brake.json`).
It had none, so neither `run_abs_compare.sh` nor the runway report could
launch it — a scenario nobody can run is a scenario nobody can measure,
which is part of how its zero settle survived.

### What moved

Measured with `scripts/scenario_runway_report.py`, no BTCM attached (front
braking is straight hydraulic pass-through).  "before" is origin/main — the
old brake schedule *and* the pre-correction coast map, since both change
what the car is doing when the pedal goes down.

| | high_mu | hard_brake | brake_and_steer |
|---|---|---|---|
| barrier releases | 15.028 s | 15.028 s | 12.053 s |
| settle, before → after | 0.00 → **2.97 s** | 0.00 → **2.97 s** | 0.00 → **2.96 s** |
| brake-on speed, before | 67.1 mph | 67.1 mph | 55.9 mph |
| brake-on speed, after | **65.3 mph** | **65.3 mph** | **54.1 mph** |
| stop distance, before | 108.69 m | 108.69 m | 64.91 m |
| stop distance, after | **102.30 m** | **102.29 m** | **66.72 m** |
| vs. ideal at μ 0.9 | 2.14 → 2.12 | 2.14 → 2.12 | 1.83 → **2.01** |

Entry speed drops ~1.8 mph on all three, because the car now coasts ~3 s
before the pedal goes down instead of braking on the tick the throttle
released.

**`brake_and_steer`'s stop got 1.81 m longer, and that is not a
regression** — it is the measurement moving to the thing the scenario
claims to test.  Two real effects: it brakes from 55.9 → 54.1 mph, only a
small v² saving; and it brakes from just above the coast map's 52.1 mph
knee, so it loses the extra (unsourced) coast drag the old map contributed
right there.  Its ratio to an ideal μ-limited stop moves 1.83 → 2.01, into
line with the other two, because the steering input at brake + 0.30 s now
bites a settled car instead of one still shedding launch slip.  A
brake-and-steer number taken during the launch transient was flattering.

`abs_brake_and_steer`'s barrier release is **12.053 s**, not the 12.011 s
that `docs/TODO.md` and the retired `[Runway]` case both carried.  That
figure cited this report script, but the script could not run any of these
three until 2026-08-12 — so it cannot have produced them.  Re-measured
either side of this branch's changes: 12.053 both times.

## How to run

### Automated test sweep (headless)

```sh
# All seven scenarios, BTCM-on vs BTCM-off comparison.  hard_brake and
# diagonal_mu were absent from this loop; hard_brake could not be run at all
# before 2026-08-12 (no config wrapper, and two name lists in
# run_abs_compare.sh that did not know it).
for t in high_mu low_mu mu_jump split_mu diagonal_mu brake_and_steer hard_brake; do
    ./scripts/run_abs_compare.sh "$t"
done

# Then regenerate the engineering report:
./scripts/abs_report.py docs/reports/abs_scenarios.md
```

`run_abs_compare.sh` writes per-scenario outputs to
`/tmp/ev1sim_abs_<test>/`:

  - `summary.txt` — distance, slip stats, ABS event counts
  - `abs_btcm_on.csv` / `abs_btcm_off.csv` — chassis-side data
  - `abs_btcm_on.btcm.csv` — **BTCM-side firmware view** (vehicle-
    speed estimator, per-wheel slip / accel / phase / locked-dwell,
    accelerometer reading) sampled at 50 Hz wall clock.  Toggled by
    `BTCM_CSV_LOG` env var; the script sets it automatically for
    BTCM-on runs.
  - `abs_btcm_*.log` — full controller stdout/stderr per process

### Manual scenario run (with window, for visual debugging)

To watch a scenario with the Chrono visualization open, start the
controllers in separate terminals.  All seven ABS configs are set up
for `realtime: true` so BTCM has wall-clock time to engage; flip
`headless: false` in the config you're running to get a window.

```sh
# Terminal 1 — ev1sim with window
cd ev1sim
# Edit config/abs_<test>.json: set "headless": false
./build/ev1sim --config config/abs_<test>.json

# Terminal 2 — PIM (powertrain control)
cd ../electricsim
./build/ev1/pim/pim_controller

# Terminal 3 — RSA (key/run-mode)
./build/ev1/rsa/rsa_controller

# Terminal 4 — BTCM (brake controller).  BTCM_CSV_LOG optional;
# BTCM_ABS_TRACE=1 enables per-tick algorithm-state log lines.
./build/ev1/btcm/ex_btcm_controller
```

Order matters: ev1sim creates the SHM bus segments, the controllers
attach.  Watch ev1sim's stdout for `connected to main harness bus`
before starting the controllers, and start RSA before BTCM so the
run-mode broadcast is live when BTCM tries to read it.

The `--config <name>` form works too — leave the abs_<test>.json
files alone and pass via env / a copy if you only want headless: false
for a single one-off run.

### Refreshing reports

```sh
# Engineering report (runs all 4 scenarios should be done first):
./scripts/abs_report.py docs/reports/abs_scenarios.md

# Tone-ring validation memo (one-off):
./build/ev1/btcm/test_btcm_tone_ring_validation \
    ../electricsim/build/firmware/btcm_firmware.elf \
    docs/reports/tone_ring_validation.md
```

### Why `realtime: true` matters

ev1sim and the electricsim controllers (BTCM, PIM, RSA) each run on
their own simulated clocks: ev1sim ticks Chrono physics, BTCM
advances the simavr-emulated AVR.  When ev1sim is unpaced
(`realtime: false`) it can run many times faster than BTCM's
simulated AVR, and a 10-second brake event finishes in a wall-
clock fraction of that — long before the firmware has had a chance
to engage ABS.  Setting `realtime: true` paces ev1sim against wall
clock so both sides see the same event durations.

ev1sim emits a startup warning if a scenario has stats logging,
external_sim is enabled, but realtime is false — flag is the
honest answer.

## Architectural notes worth knowing while reading the report

These are the design choices the algorithm makes — read the
engineering report for actual numbers.

### Accelerometer is build-time optional

The original GM EV1 BTCM did **not** have a longitudinal
accelerometer.  Its vehicle-speed estimator used "fastest healthy
wheel × tire radius" with a `max_ref_decel_mps2` clamp.  We default
to that mode (`BTCM_USE_ACCELEROMETER=OFF`) to model the original
hardware accurately.

When the accel path was always-on, an over-aggressive modulator
caused a death spiral on dry pavement: ABS released too much
pressure → chassis decelerated slowly → accelerometer reading was
small → projected `vehicle_speed_mps` stayed high → slip looked
elevated → algorithm dumped more.  Disabling restored sane behavior.
Re-enable the option (`-DBTCM_USE_ACCELEROMETER=ON`) for second-gen
ABS comparison runs.

The host bridge keeps publishing accel onto the chassis bus even
when the firmware's algorithm doesn't consume it, so the signal is
still visible in scan-tool logs and traces.

### Tone-ring chain has documented compensations

`test_btcm_tone_ring_validation` (see report) shows the firmware's
reported wheel rps lands at ~0.61 × the analytical expected value
on sustained drives, even though direct edge-count probes show the
simavr → ISR plumbing is 1:1.  Two pieces account for the gap:

1. **Window-timer drift** (~22 % of the loss).  The firmware's
   tone-ring window check fires at ~64 ms instead of 50 ms because
   main-loop work inflates the dt.  The formula uses the measured
   dt, so this propagates linearly: 50/64 ≈ 0.78.
2. **Sustained-drive edge loss** (~17 % of the loss).  Unexplained;
   probably a simavr scheduling-edge interaction.

`ToneRingGen::kEdgeCalibration = 1.64` on the host compensates so
the firmware's ABS algorithm sees realistic wheel speeds.

### Tone rings are 48-tooth front, 47-tooth rear

Per the EV1 service manual.  Host-side `ToneRingGen` already uses
per-wheel teeth.  Firmware-side `cfg.teeth_per_wheel` is still a
single global value (48), so the rear's reported rps is off by
2 % (47/48) — small but nonzero, listed in the outstanding-issues
section of the engineering report.

### Tire radius

P175/65R14 = 0.2915 m unloaded.  Both ev1sim's chassis tire model
and the firmware's `cfg.tire_radius_m` use this value, so slip-ratio
computations are consistent across the boundary.  Real tires change
radius with pressure / temperature; we don't model that.

### Modulator hydraulic τ

The host's brake modulator (`SimApp::ApplyAbsFrontBrake`) uses a
finite-rate first-order model:

  - APPLY: pressure rises toward MC at τ = 5 ms (caliper fill)
  - DUMP:  pressure decays toward 0 at τ = 60 ms (orifice flow)
  - HOLD:  pressure freezes

Values are first-pass estimates.  Real GM EV1 service-bay numbers
would let us calibrate properly.

## Outstanding issues

Most of these are listed in the engineering report's per-scenario
sections.  At the architectural level:

- **Rear EMB lacks per-wheel modulation.** `split_mu` BTCM-on takes
  ~25 % longer to stop than BTCM-off because the rear motors apply
  symmetric force regardless of which side is on ice.
- **Low-mu / mu-jump still show extended lock.** When all four
  wheels lock together on ice, the no-accelerometer speed estimator
  collapses with them.  Fundamental first-gen ABS limitation.
- **Per-axle teeth count in firmware** (see above).
