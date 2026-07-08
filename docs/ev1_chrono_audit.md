# EV1 Chrono vehicle audit

Cross-reference of every parameter in `data/vehicle/ev1/**/*.json` against
known GM EV1 specifications.  Targets **Generation 2 NiMH (1999)** as the
canonical config — it's the more polished production variant most folks
remember, and the lighter mass + extended range numbers are what the
public datasheets cite.

Symbol legend in the tables:
- ✅ matches real EV1 closely
- ⚠️  approximation — close enough for now, refine with new data
- ❌ wrong — should be fixed
- 🔧 fixed in the audit batch (commit log has the change)

Sources for the real-EV1 numbers in this document:
- GM EV1 product brochure (1999), reproduced widely (e.g. ev1.org archives)
- Wikipedia GM EV1 article (cross-checked figures)
- "Who Killed the Electric Car?" (Paine 2006) — production / range numbers
- General automotive engineering knowledge for the
  derivative-but-undocumented values

The audit was occasioned by the cruise-control demo run — steady-state
cruise hold dropped to ~11.6 m/s when commanding 13.15 m/s.  That bug is
likely in the plant model (this audit), not the controller, so we hold
off cruise gain tuning until these fixes land.

## 1. Vehicle dimensions

| Parameter | Current | Real EV1 | Status |
|---|---|---|---|
| Wheelbase | 2.512 m | 2.514 m (99.2 in) | ✅ within 2 mm |
| Front axle x | +1.346 m (suspension loc) | n/a (derived) | ✅ |
| Rear axle x  | −1.166 m (suspension loc) | n/a (derived) | ✅ |
| Min turning radius | 10.8 m | 11.0 m (curb-to-curb) | ⚠️  close |
| Max steering angle | 13.5° (vehicle); 10° (pinion) | ~13° (matches 11 m TR) | ✅ |
| Length / width / height | n/a (visual mesh) | 4310 / 1765 / 1281 mm | n/a |
| Drag coefficient | n/a (Chrono uses tire/rolling losses, no aero) | 0.19 | ❌ no aero model |
| Frontal area | n/a | 1.89 m² | ❌ no aero model |

**Note on aero**: Chrono's WheeledVehicle doesn't model body aero by default.
The EV1's signature 0.19 Cd should manifest as a `chrono::ChAerodynamicLoad`
or similar applied to the chassis body — currently absent.  Without it,
high-speed deceleration is artificially low.

## 2. Mass and inertia

| Parameter | Current | Real EV1 (NiMH) | Status |
|---|---|---|---|
| Chassis mass | 1217 kg | ~1199 kg (target curb 1281, less ~82 kg accessories) | 🔧 1217→1199 |
| Curb weight (computed) | ~1299 kg | 1281 kg (NiMH) | ⚠️ off by 18 kg |
| Centroidal Frame x | +0.09 m | unknown — likely +0.1 to +0.15 (slight front bias from motor) | ⚠️  |
| Centroidal Frame z | 0.49 m | likely 0.40-0.45 m (T-pack batteries low in floor) | ⚠️ slightly high |
| Ixx (roll) | 130 kg·m² | rg≈0.33 m → plausible | ⚠️ |
| Iyy (pitch) | 600 kg·m² | rg≈0.70 m → plausible | ⚠️ |
| Izz (yaw)  | 680 kg·m² | rg≈0.75 m → plausible | ⚠️ |

**Curb-weight breakdown (Chrono adds these to chassis)**:
- 4× wheel × 8.0 kg = 32 kg
- 4× tire × 7.5 kg = 30 kg
- 4× suspension (spindle 1.0 + upright 1.2 + UCA 0.9 + LCA 1.4) = ~18 kg
- Steering link = 1.6 kg
- Total non-chassis = ~82 kg
- Target chassis = 1281 − 82 = **1199 kg** for Gen 2 NiMH

For PbA Gen 1 (1351 kg curb), chassis would be 1269 kg — choose NiMH as
the canonical config since the demo scenarios are tuned around the more
modern variant.

## 3. Powertrain — motor

| Parameter | Current | Real EV1 (Gen 2 AC induction) | Status |
|---|---|---|---|
| Peak torque | 150 N·m flat 0-6500 RPM | 149 N·m peak | ✅ within 1% |
| Peak power (at 6500 RPM) | 102 kW | 102 kW (137 hp) | ✅ matches |
| Max RPM | 13000 | ~12000-13000 | ✅ |
| Map shape (full throttle) | flat torque to 6500, power-limited beyond | textbook 3-phase induction | ✅ |
| Coast torque (zero throttle) | -5 to -25 N·m | EV1 used aggressive regen (~25 kW) | ⚠️  representative; not commanded regen |
| Motor type | "EngineSimpleMap" (Chrono ICE template) | 3-phase AC induction | ⚠️ template mismatch — works as a static map |

The motor torque/power *map* matches the real EV1 closely.  What's missing:
- Regen integrated with brake commands (today regen lives only in coast
  torque, which fires whenever throttle is released regardless of brake)
- Speed-dependent peak — induction motors have a more complex torque
  envelope at low speeds (limited by inverter current) than a flat 150 N·m
  curve suggests.  Probably negligible for our use cases.

## 4. Drivetrain

| Parameter | Current | Real EV1 | Status |
|---|---|---|---|
| Transmission type | single forward gear, ratio 1.0 | single-speed reduction | ✅ |
| Final drive (Conical Gear) | 0.1 → **10.0:1 reduction** | **10.946:1 reduction** | 🔧 0.1 → 0.0913 |
| Driveline | FWD (`ShaftsDriveline2WD`) | FWD | ✅ |
| Driveshaft inertia | 0.5 kg·m² | unknown — typical | ⚠️  |
| Differential box inertia | 0.6 kg·m² | unknown — typical | ⚠️  |
| Differential lock limit | 100 N·m | open differential, no LSD on EV1 | ✅ (open at this limit) |

The 10.0 vs 10.946 final drive change is small (~9% off) and shifts the
operating RPM at a given speed slightly.  At 13 m/s with current ratio:
motor RPM = 4250.  After fix: 4651 RPM — still well within flat-torque
region, so peak-torque behavior is unchanged.

## 5. Tires

| Parameter | Current | Real EV1 (P175/65R14) | Status |
|---|---|---|---|
| Unloaded radius | 0.2915 m | 0.292 m (computed from spec) | ✅ |
| Width | 0.175 m | 0.175 m (175 mm) | ✅ |
| Rim radius | 0.1778 m | 0.1778 m (14"/2) | ✅ |
| Mass | 7.5 kg | typical | ✅ |
| Coefficient of friction | 0.8 | typical dry asphalt | ✅ |
| Rolling resistance | 0.008 | EV1 spec'd 0.006-0.008 (Michelin Proxima LRR) | ✅ |
| Load index | 82 (475 kg/tire) | 95 (Q-rated) | ⚠️  probably under-spec'd |
| Tire model | TMeasy | n/a | ✅ standard choice |

Tires are the best-modeled part of the vehicle.  The wide margin on
bearing capacity (4750 N vs ~3140 N actual peak load per tire) is
conservative; not worth touching.

### 5.1 Standstill wheel-spin limit cycle → integration step (2026-07-08)

Surfaced by electricsim's VAT `wheel_speed_*.svg` plots: a stopped, unbraked,
undriven wheel reported ±4–5 rad/s of `wheel_omega` (physically impossible),
and the launch phase carried the same near-Nyquist wobble. Root cause is a
**numerical instability of the explicit multibody integration at the 2 ms
step**, not the tire model:

- Below 1 m/s the TMeasy force switches to the Coulomb path (`frblend→0`) and
  the near-zero slip gain `sx = −vsx/(R·|ω| + m_vnum)` (`m_vnum = 0.01`) is very
  high; the low-inertia wheel-spin DOF it drives is stiff, and the 2 ms explicit
  step over/undershoots each tick → a limit cycle.
- Measured standalone (a free wheel at rest, `wheel_omega_rl` rms over t ≥ 1 s):
  step 2 ms → 2.88; **1 ms → 0.094 (31×)**; 0.5 ms → 0.099; 0.25 ms → 0.10 — so
  1 ms is already stable and finer only plateaus.
- `dfx0` is irrelevant here (×0.5 / 0.25 / 0.1 all gave 2.88): the standstill
  force is Coulomb, not the slip curve. Sub-stepping the *tire* is also a no-op —
  this Chrono TMeasy is steady-state (algebraic `Advance()`).

Fix: `simulation.step_size_s` 2 ms → **1 ms** (all configs + the `Config.h`
default). `steps_per_tick = round(tick_dt / step)` auto-doubles, so the
`1/render_fps` co-sim exchange cadence and the stats cadence are unchanged —
only the physics integration is finer. Deterministic (bit-identical run-to-run
preserved); VAT baselines recaptured because every physics value shifts
slightly. Tracked as electricsim `BL-0103`.

## 6. Brakes

| Parameter | Current | Real EV1 | Status |
|---|---|---|---|
| Per-wheel max torque | 1800 N·m | ~700-800 N·m front, ~400-500 rear | 🔧 1800 → 800 |
| Front brake type | "BrakeSimple" | ventilated disc (256 mm) | ⚠️ ok abstraction |
| Rear brake type | "BrakeSimple" (same as front) | drum (200 mm) — much weaker | ⚠️ |

**Math behind the fix**: 1800 N·m × 4 wheels = 7200 N·m total.  At 0.2915 m
tire radius that's 24,700 N braking force = 1.97g of deceleration —
which exceeds the tire's friction-limited deceleration (~0.8g on dry
asphalt).  In practice the wheels lock and ABS modulates, so the
visible deceleration is friction-limited regardless.  But the
unrealistically high commanded torque means even a 30% brake pedal
saturates wheel friction — there's no usable middle range.

Setting per-wheel torque to 800 N·m gives 3200 N·m total ≈ 11,000 N
≈ 0.88g, which still saturates friction at full brake but leaves a
realistic gradient at partial pedal.  Future improvement: split front
800 / rear 500 to match the EV1's disc/drum bias.

## 7. Suspension geometry

| Parameter | Current | Real EV1 | Status |
|---|---|---|---|
| Front template | `DoubleWishbone` | **MacPherson strut** | ❌ wrong template |
| Rear template | `DoubleWishbone` | **trailing arm / twist beam** | ❌ wrong template |
| Front track (inferred) | 1.470 m | 1.496 m (58.9 in) | 🔧 0.735 → 0.748 |
| Rear track (inferred) | 1.244 m | 1.281 m (50.4 in) | 🔧 0.622 → 0.6405 |
| Front camber | -0.10° | typical 0 to -0.5° | ✅ |
| Rear camber | -0.50° | typical -0.5 to -1.0° | ✅ |
| Front toe | -0.05° | typical 0 to ±0.1° | ✅ |
| Rear toe | 0° | typical 0 to +0.1° | ✅ |
| Spring rate (both axles) | 143000 N/m | unknown | ⚠️ identical front/rear is suspicious |
| Damping (front / rear) | 7500 / 7000 N·s/m | unknown | ⚠️ |

**Suspension template mismatch is a known limitation.**  Switching to a
real `MacPhersonStrut` template (front) and a `MultiLink` or trailing-arm
approximation (rear) would be a substantial rebuild — different hard
points, different visualization, different tuning surface.  Deferred until
either (a) a Chrono `MacPhersonStrut` JSON sample is available to start
from, or (b) we have measurements / drawings of EV1's actual suspension
geometry to drive the redesign.

For now the `DoubleWishbone` approximation gives roughly correct
roll / pitch / squat behavior at typical driving inputs — close enough for
the electronics-driven scenarios we're running.

The track-width fixes are clean: just bump the spindle / control-arm Y
coordinates by the half-difference.

## 8. Steering

| Parameter | Current | Real EV1 | Status |
|---|---|---|---|
| Template | RackPinion | rack-and-pinion w/ electric assist | ✅ |
| Pinion radius | 0.3 m | n/a (Chrono effective parameter) | ⚠️ |
| Max pinion angle | 10° | n/a | ⚠️ |
| Steering link length | 0.46 m | unknown | ⚠️ |
| Rack-to-wheel ratio (effective) | 1.35:1 | unknown | ⚠️ |

Steering geometry is consistent end-to-end (13.5° max wheel angle ≈ 11 m
turning radius given the 2.512 m wheelbase).  Internal pinion parameters
are tuning surface, not physical specs.  Leave alone.

## 9. Aerodynamics

**Not modeled.**  See dimensions table.  Adding aero would need a custom
`ChAerodynamicLoad` or chassis-level drag force computation
(`F_drag = 0.5 ρ v² Cd A`).

At 13 m/s (cruise speed in the demo):
- Drag force = 0.5 × 1.225 × 169 × 0.19 × 1.89 = **37 N**
- That's 481 W of drag power at 13 m/s
- For comparison, rolling resistance ≈ 100 N at 13 m/s ≈ 1.3 kW

So drag is a **smaller** loss than rolling resistance at urban speeds.
This explains why the cruise-control demo's steady-state error doesn't
trace back to missing aero — the dominant losses are tire-side, which
TMeasy already models.

## 10. What's missing (open questions)

These probably explain the cruise-hold steady-state error:

1. **Drivetrain efficiency / friction not parameterized.**  The
   `ShaftsDriveline2WD` template applies inertia but the only resistive
   load comes from tire ground forces.  In reality the gearbox + differential
   bleed several kW.  EV1 manuals cite ~92% drivetrain efficiency; that
   missing 8% is currently unaccounted.
2. **Spring/damper rates are guesses.**  Identical front/rear spring
   coefficients (143000 N/m) is implausible — no engineer would design a
   FWD car with a 50/50 spring-rate split given the front-bias mass.
3. **No drivetrain parasitic torque.**  Real motor + inverter has ~5-15%
   loss across the operating range; currently this only shows up in the
   coast-torque map (-5 to -25 N·m) which only fires at zero throttle.

Plan: add a `ChForce` to the chassis representing constant
drivetrain-loss torque, calibrated to a coastdown test from real EV1
data — but this requires real coastdown numbers, which we don't have.
Track this as a future TODO.

## 11. Empirical evidence — coastdown shows 2× excess dissipation

After applying the audit fixes (final drive 10.946, mass 1199, track widths),
the `coastdown` scenario (`config/scenarios/coastdown.json`) produced these
data points after throttle release at t=13.55 s, v=30 m/s:

| t (s) | v (m/s) | decel (m/s²) avg over Δt |
|---|---|---|
| 13.6 | 30.05 |  — |
| 15.2 | 29.23 | 0.48 |
| 20.4 | 27.19 | 0.40 |
| 25.5 | 24.88 | 0.33 |
| 30.6 | 23.86 | 0.30 |
| 43.4 | 19.98 | 0.30 |

Fitting `F = m × decel = F_rr + 0.5·ρ·v²·CdA` to the (30, 20) m/s anchors:

- **Sim F_rr ≈ 198 N** (real EV1 ≈ 100 N) — **~2× too high**
- **Sim CdA ≈ 0.756 m²** (real EV1 = 0.19 × 1.89 ≈ 0.36 m²) — **~2× too high**

So the simulator's total decel force is roughly 2× what published EV1
specs predict — both the static rolling component and the velocity-squared
component.  Likely culprits, in order of suspicion:

1. **TMeasy tire model internal slip dissipation.**  The `Rolling Resistance
   Coefficient: 0.008` only covers static rolling drag; TMeasy also
   dissipates power through longitudinal-slip deformation that scales
   non-linearly with speed.  Tuning the slip-curve parameters (not exposed
   in the current JSON) could halve the v² term.
2. **Driveline shaft inertia acting as effective damper** in the
   `ChShaftsDriveline2WD` solver.  The 0.5 + 0.6 kg·m² shaft inertias
   combined with stiff joints may produce numerical viscosity.
3. **Powertrain coast torque firing during coastdown** — the engine
   `Map Zero Throttle` (-5 to -25 N·m) fires at zero throttle.  At our
   coastdown speeds (rpm range 4000-7000), that's a -10 to -15 N·m
   continuous load.  Through the 10.946:1 reduction that's a tire-side
   force of ~38 to 56 N — 20-30 % of the excess F_rr we measured.

**Concrete next experiment when we want to chase this down**: drop the
zero-throttle map to ~half its current values, re-run coastdown, and
see if F_rr drops to ~150 N.  Anything still excess after that is in
the tire model or the solver.

**For now**: don't chase tire/driveline tuning.  The 2× factor is a
known plant-fidelity limitation captured here.  Cruise-control gain
tuning should still wait until F_rr is closer to spec — otherwise we'd
be tuning around the wrong plant.  See [docs/TODO.md](TODO.md) for the
follow-up task.

## 12. Calibration attempt — what didn't work

After the audit fixes landed, an explicit calibration attempt
(2026-04-30) tried to bring F_rr and CdA closer to spec.  Tooling:
[scripts/fit_coastdown.py](../scripts/fit_coastdown.py) does
weighted-OLS on bin centroids to recover F_rr and CdA from a coastdown
CSV.  Tested values:

| config                                        | F_rr (N) | CdA (m²) |
|---|---|---|
| baseline (engine -5..-25, Crr=0.008)          | 301      | 1.21     |
| engine zeroed (engine 0, Crr=0.008)           | -126*    | 2.01     |
| engine halved (engine -2.5..-12.5, Crr=0.008) | 220      | 1.98     |
| Crr lowered (engine -5..-25, Crr=0.005)       | 185      | 1.75     |
| Crr near-zero (engine -5..-25, Crr=0.001)     | unstable | unstable |

\* negative F_rr is a fit artifact — pure-v² shape doesn't decompose
cleanly.  The "F_avg" at v≈22 m/s is actually a better single-number
summary than the F_rr/CdA decomposition.

| config                                        | F_avg @ 22 m/s | F_avg @ 27 m/s |
|---|---|---|
| baseline                                      | 667 N          | 845 N          |
| engine zeroed                                 | 528 N (-21%)   | 780 N (-8%)    |
| engine halved                                 | 815 N (+22%)   | 1180 N (+40%)  |
| Crr lowered                                   | 723 N          | ~1000 N        |

Engine-zero gave the cleanest reduction (~10-20%).  But "engine halved"
gave WORSE total drag than baseline — the trajectory diverged because
small dynamics changes accumulate over a 30 s coastdown.

**Conclusion**: the simple knobs exposed in the JSON (engine coast map,
tire Crr) can each shave 10-20 % off total drag, but cannot get within
2× of spec.  The remaining ~70-80 % of excess dissipation lives in
TMeasy's internal slip dynamics — `Vehicle Type: "Passenger"` preset
sets longitudinal/lateral slip stiffness defaults that aren't exposed
in the JSON.

**Reverted to baseline config.**  Calibration deferred until either:
(a) Chrono exposes more TMeasy parameters via JSON, (b) we switch to
a different tire model with explicit dissipation knobs (e.g.
Pacejka-style with rolling-resistance map), or (c) we accept the 3×
factor as a known plant limitation and tune the controller around it.

**What the tooling enables for future work**:
- `config/scenarios/coastdown.json` is the standardized validation run.
- `scripts/fit_coastdown.py scenario_coastdown.csv` reports F_rr and CdA.
- Both should converge toward 100 N and 0.36 m² as the plant model improves.

## 13. TMeasy full-parameterization migration (2026-04-30)

The bearing-capacity shortcut in `data/vehicle/ev1/tire/EV1_TMeasyTire.json`
turned out to be **silently overwriting the tire radius**.  Chrono's
`TMeasyTire::Create()` (TMeasyTire.cpp:178) calls
`GuessPassCar70Par(bearing_capacity, m_width, ratio, m_rim_radius, ...)`,
but the function expects `rimDia` (diameter) in the 4th slot, and inside
it sets `m_rim_radius = 0.5 * rimDia` and
`m_unloaded_radius = secth + rimDia / 2.0`.  Net result: a 14" tire
(spec'd 0.2915 m radius) silently became a 7"-equivalent (~0.20 m) at
runtime.

Switched the JSON to the **full parameterization** path so the Design
block radii stay authoritative and every slip parameter is explicit.
Numerical defaults equal what `GuessPassCar70Par(load=2375 N)` would
have computed, so the only change is the radius bug fix.

Coastdown comparison after the migration:

| config              | F_rr (N) | CdA (m²) | F_avg @ 22 m/s | F_avg @ 27 m/s |
|---|---|---|---|---|
| baseline (buggy 7" tire)         | 301 (3.0×) | 1.21 (3.4×) |  667 N | 845 N |
| full-param defaults (true 14")   | 134 (1.34×) | 2.34 (6.5×) |  835 N | 1305 N |
| full-param 0.5× dfx0             | 622 | 0.85 |  902 N | 1006 N |
| full-param 2× dfx0               | -75* | 2.40 |  909 N |  919 N |
| full-param 4× dfx0               | unstable | unstable | — | — |

\* negative F_rr is fit artifact when the fit is dominated by curvature.

**The geometric correction is the bigger story.**  F_rr is now within
35 % of spec instead of 3× — that part of the model was not "broken,"
it was just operating at the wrong scale.  The remaining ~6.5× CdA is
TMeasy slip dynamics scaled to higher tire force at speed, which the
exposed slip-stiffness knob doesn't usefully reduce: lowering it traded
F_rr for CdA, raising it produced numerical instability around 2-4×.

**Locked in**: full-param defaults.  Tire radius is now correct;
slip-stiffness tuning deferred until either (a) we have measured EV1
slip-curve data, (b) we switch to a Pacejka/Magic-Formula tire with
explicit rolling-resistance map, or (c) we accept the high-speed drag
overshoot as a known plant limitation.

## 14. Rear brake actuator + pedal-feel design (proposed)

### Rear brake — self-energizing drum with electric actuator

EV1 service manuals describe the rear brakes as drums driven by an
electric motor at each wheel (kSigRearMotorLR/RR are already published
by BTCM).  No hydraulic line, no shoe-return spring assist — the motor
drives a leadscrew-like mechanism that pushes the shoes outward.

Self-energizing drum behavior summary:

```
T_brake(F_shoe, omega, mu) =
    mu × F_shoe × R_drum × (1 + alpha × sign(omega))
```

where `alpha` is the self-energizing coefficient:
- **Leading shoe**: `alpha ≈ 1-3` (friction torque pulls shoe harder onto drum)
- **Trailing shoe**: `alpha ≈ 0` (friction torque pulls shoe away from drum)
- **Duo-servo / double-leading**: `alpha ≈ 2-3` average

As wheel speed `|omega| → 0`:
- Dynamic friction limit unchanged, but the self-energizing assist
  collapses (no rotation, no friction torque to amplify the shoe force).
- **Required actuator force to hold a given brake torque rises as the
  wheel slows.**  At standstill the EMB needs ~3× the force it needed
  at speed for the same wheel torque.
- This is exactly opposite to a hydraulic system where pedal pressure
  is constant once applied.

**Where it should live**:
- `electricsim/ev1/btcm/`: model the **commanded shoe force** based on
  motor current and travel.  Already publishing `kSigRearMotorLR/RR`;
  needs a force-vs-current curve.  Real EV1 fuse ratings + steady-state
  motor current limits are the spec inputs (manual hints).
- `ev1sim/src/`: model the **physics of the drum** (T_brake formula
  above) and apply per-wheel torque to Chrono.  Reads
  `kSigRearMotorLR/RR` for force command, reads `wheel_omega_rl/rr`
  from physics for the self-energizing factor.  Mirrors the existing
  front-brake bus-mediated path (`ApplyAbsFrontBrake` / `ApplyFrontBrakePerWheel`).

**Estimated parameter ranges (educated guesses)**:
- R_drum ≈ 0.10 m (8" drum)
- mu (drum-on-shoe) ≈ 0.35-0.40 (typical phenolic resin lining)
- alpha ≈ 2.0 (double-leading shoe, conservative)
- F_shoe_max ≈ 4000 N (matches typical caliper clamping force)
- Required motor torque to maintain F_shoe at standstill: ~10-15 N·m
  (back-calculated from fuse-rated current × motor torque constant).

This becomes a calibration target the same way the coastdown is —
add a "rear-brake-only stop" scenario that engages just the EMB
actuators and measures stopping distance vs speed.

### Pedal feel — physical spring, virtual hydraulics

The user's sim rig has a real spring (possibly two-stage) on the brake
pedal.  The pedal **position** is what crosses into the simulator; the
**force feel** is hardware-side.  What ev1sim/electricsim need to do:

1. **Read pedal travel** (already done — `kSigDriverBrakePedalQ8` 6900).
2. **Convert position → master cylinder pressure** with a simplified
   model.  Two-stage suggests two linear segments:
   ```
   pressure(travel_q8) =
       0                                    if travel_q8 < dead_band
       k1 × (travel_q8 - dead_band)         if travel_q8 < transition
       k1 × transition + k2 × (travel_q8 - transition)
                                            otherwise
   ```
   `dead_band` matches the pedal's free-travel before the master
   cylinder cup engages.  `k1` is the soft (initial fluid takeup)
   stage; `k2` is the firmer (full fluid pressure) stage.
3. **Publish master cylinder pressure** as a new chassis-bus signal
   (`kSigChassisBrakeMasterPressureKpa`?).  BTCM consumes this as the
   real brake-effort input — replacing or augmenting the current
   simple `brake_switch` boolean.
4. **ABS isolation feedback** — when BTCM isolates a wheel, the line
   pressure feedback isn't returned to the master cylinder (in real
   cars the pump cycles fluid, kicking back at the pedal).  Active
   force feedback to the rig could simulate this; out of scope for
   this design unless the rig has a force-feedback motor.

**Where it should live**:
- `ev1sim/src/PhysicalWorld.h`: new `BrakePedal` component owning the
  position-to-pressure calibration table.  Publishes pressure as a new
  driver-input bus signal.  Sits alongside `CombinationSwitch`,
  `WiperStalk`, etc.
- `electricsim/ev1/btcm/`: subscribe to the pressure signal, use as the
  primary brake-effort input.  Brake-switch (6904) stays as the ABS
  pump-prime threshold (boolean: any meaningful pedal application).

**Estimated parameter ranges (educated guesses)**:
- `dead_band` ≈ 5-10 % travel (q8 13-25)
- `transition` ≈ 30-40 % travel (q8 76-102)
- `k1` ≈ 100 kPa per 1 % travel (initial takeup)
- `k2` ≈ 250 kPa per 1 % travel (full fluid stage)
- Max pressure at full pedal ≈ 12-15 MPa (typical passenger-car spec)

These are starting points; tune against real EV1 manual data when
available, or to match the user's preferred sim-rig pedal feel.

Both features are deferred — captured in [docs/TODO.md](TODO.md) so
they're not lost when we pivot back to integration work.

## 15. Regen ↔ friction blending — known limitation (S15)

**Status: documented gap, intentionally not modelled.** Flagged by the
regulator-lens safety review; the honest answer here is "we don't have
the data to do this faithfully," so this section records *why* rather
than inventing physics.

What exists today:
- **Friction braking is regen-agnostic.** The front `BrakeSimple` disc
  and the rear `BrakeDrum` EMB ([src/BrakeDrum.h](../src/BrakeDrum.h))
  compute wheel torque from the brake command alone. Nothing subtracts a
  regen contribution from friction, and nothing adds a regen torque to
  the wheels.
- **Regen is a readout, not a force.** [src/MotorCurrent.h](../src/MotorCurrent.h)
  recovers *signed DC-bus current* from the mechanical operating point
  (T·ω) and ev1sim publishes it as the ammeter value `motor_current_a`
  (chassis 4072). It never feeds `DriverCommand` or Chrono wheel torque.
- The coast-torque map (§3, `EV1_EngineSimpleMap.json "Map Zero
  Throttle"`) is residual bearing/windage drag scaled to RPM — *not*
  commanded regen. It fires on throttle release regardless of brake.

Why this blocks the **regen-cutout fail-safe** test: the safety property
is "if regen drops out, the friction system backfills the lost
deceleration so total braking is preserved." In-sim the two systems are
**not coupled** — there is no regen torque on the wheels to cut, and no
blend controller to hand off to friction — so the property is
**untestable as built**. (Contrast S16's BTCM-death property, which *is*
testable because the rear-EMB→friction path is real and modelled.)

What a faithful model would need (and why we're not guessing it):
- A regen-torque-vs-speed envelope for the EV1 induction motor under
  braking (the production car blended up to ~25 kW of regen, but the
  *torque* curve and its low-speed taper are not in the manual corpus).
- The blend-handover law: how the BTCM/PIM split a brake demand between
  regen and friction, and the speed/SOC/temperature conditions that cut
  regen out (cold pack, full SOC, fault). None of this is published.

Fabricating those constants would produce confident-looking but
unsupported numbers — worse than the documented gap. Per the project's
"faithful where the manual is authoritative, documented inference
elsewhere" rule, this stays a gap until either (a) measured/traceable
EV1 regen-blend structure surfaces, or (b) a *structure-only* model is
warranted (regen torque + friction summed to driver demand, with a
friction backfill on cutout) using explicitly `@design`-tagged
engineering choices. The follow-up marker lives in
[docs/TODO.md](TODO.md) under "Bus-mediated physics".

If/when modelled, the natural shape:
- Add a `propulsion_torque` (signed) channel to `DriverCommand`
  (already anticipated in [ARCHITECTURE.md](../ARCHITECTURE.md) "Future
  per-axle brake control"): regen maps to negative propulsion torque.
- A blend controller (EV1: BTCM-side) sums regen + friction to the
  driver's deceleration demand and backfills friction when regen is
  unavailable — at which point the cutout fail-safe becomes a real,
  testable closed-loop property.
