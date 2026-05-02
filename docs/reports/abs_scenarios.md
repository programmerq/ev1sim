# EV1 ABS validation — engineering report

_Generated 2026-05-02 11:49 by `scripts/abs_report.py`._

This report aggregates the four standard ABS validation
scenarios.  Each scenario gets its own section with setup,
numerical result tables, and a stack of charts: vehicle
speed, per-wheel ground speed, slip ratios, ABS phase
timeline, yaw rate, and (for `split_mu`) trajectory.

Source data: `/tmp/ev1sim_abs_<test>/{summary.txt,
abs_btcm_{on,off}.csv}`.  Refresh with
`for t in high_mu low_mu mu_jump split_mu; do ./scripts/run_abs_compare.sh $t; done` and re-run this
script.

## Headline table

| Test | BTCM-on stop | BTCM-off stop | Δ stop dist | ABS events FL/FR | yaw drift (on / off) |
|---|---|---|---:|---:|---|
| high_mu | 150.18 m / 9.92 s | 148.52 m / 9.47 s | +1.7 m | 71 / 76 | +0.00° / +0.07° |
| low_mu | didn't stop, v\_f = 2.91 m/s | didn't stop, v\_f = 1.57 m/s | — | 116 / 107 | +0.13° / +3.54° |
| mu_jump | didn't stop, v\_f = 3.95 m/s | didn't stop, v\_f = 3.65 m/s | — | 204 / 176 | +3.36° / -1.00° |
| split_mu | 74.74 m / 7.14 s | 72.49 m / 6.53 s | +2.2 m | 73 / 78 | -0.72° / +0.09° |
| brake_and_steer | 73.94 m / 5.95 s | 67.05 m / 4.99 s | +6.9 m | 103 / 94 | +82.00° / +27.86° |
| diagonal_mu | 97.12 m / 8.86 s | 77.14 m / 7.55 s | +20.0 m | 132 / 120 | +0.25° / -0.66° |

Δ stop dist > 0 means BTCM-on took *longer* than BTCM-off.
Yaw drift > 0 = car rotated counter-clockwise (toward driver
left).  On `split_mu` the car drifts toward the asphalt side
(positive in our coordinate frame).

## `high_mu`

### Setup

| Field | Value |
|---|---|
| Surface | uniform asphalt (μ ≈ 0.9) |
| Approach | throttle to ≥ 30 m/s, throttle off, full brake |
| Brake fires | scenario t = 6 s |
| Validates | ABS should mostly stay quiet; wheel lock on dry asphalt is rare |

### Result summary (BTCM-on vs BTCM-off)

| Metric | BTCM-on | BTCM-off |
|---|---|---|
| Brake-on at | t = 14.10 s, v = 30.06 m/s | t = 14.10 s, v = 30.06 m/s |
| Stop result | 150.18 m / 9.92 s | 148.52 m / 9.47 s |
| FL events | 71 | 0 |
| FR events | 76 | 0 |
| Yaw drift over brake | +0.00° | +0.07° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.635 | 0.062 | 0.0% |
| FR | 0.553 | 0.063 | 0.0% |
| RL | 0.604 | 0.065 | 0.0% |
| RR | 0.638 | 0.067 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 21.6% | 8.8% | 13.2% | 56.4% | 80 |
| FR | 18.4% | 8.8% | 13.7% | 59.1% | 87 |

### Charts

**[▶ Animated top-down replay (open in browser)](abs_scenarios.charts/high_mu_replay.html)** — watch the car drive through the brake event with per-wheel slip color-coded, surface patches drawn behind, and a scrubbable timeline.  Toggle BTCM-on / BTCM-off in the dropdown to A/B the same instant.

**Vehicle speed**

![speed compare](abs_scenarios.charts/high_mu_speed.svg)

**Effective brake force at each axle.**  Front solid,
rear dashed.  Front line is `applied_front_brake` directly
(the post-ABS-modulation ratio fed to Chrono).  Rear line
is *derived* from `emb_cmd_lr/rr`: max(0, cmd) averaged
across L and R, because the CSV's `applied_rear_brake`
captures the *symmetric* driver-pedal command rather than
the per-wheel override that `ApplyRearEmbBrake` feeds into
Chrono.  BTCM-off rear is forced to 0 here — the EV1's
rear EMB has no hydraulic backup line, so when the
controller is out of the loop the rear free-rolls.

![brake outputs](abs_scenarios.charts/high_mu_brake_outputs.svg)

**Front ABS phase timeline (hydraulic axle).**  Discrete
APPLY/HOLD/DUMP solenoid bands.  See the rear EMB chart
just below for the *equivalent* rear-axle modulation —
same algorithm decisions, different actuator (continuous
motor command in [-1, +1] instead of solenoid phases).

![ABS phase timeline](abs_scenarios.charts/high_mu_phase.svg)

**Rear EMB motor command (electromechanical axle).** Continuous-valued analog of the front Gantt above. `+1` = motor pushing apply, `-1` = motor releasing the shoes, `~0` = hold position.

![rear EMB cmd](abs_scenarios.charts/high_mu_rear_emb.svg)

**Command vs actual actuator state.**  Solid lines are
commands sent to Chrono; dashed lines are the modeled
post-actuator-lag values (caliper hydraulic τ ≈ 50 ms
apply / 80 ms release; rear shoe rate-limited at ~3.33/s).
Useful for understanding why ABS modulation looks blunt
on the speed chart — the actuator lag smooths the rapid
HOLD↔DUMP cycling into a slower-changing pressure curve.

![actuator lag](abs_scenarios.charts/high_mu_actuator_lag.svg)

**Per-wheel ground speed** (chassis line + 4 wheel lines).  Gap = slip.

![per-wheel speeds](abs_scenarios.charts/high_mu_wheel_speed.svg)

**Slip ratios.**  Threshold reference lines at 0.05 and
0.15 mark the algorithm's exit / enter slip thresholds.
Note the chassis-side slip values are *raw tire-contact*
data from Chrono's TMeasy model and look noisier than a
real ABS would see — production ABS heavily filters this
(typically a 10–30 ms low-pass), and our firmware's own
perception of slip (derived from tone-ring counts over
50 ms windows) is naturally smoother.  The peaks here
show what the *tire* is actually doing instant-to-
instant; the firmware's view is a window-average.

![slip ratios](abs_scenarios.charts/high_mu_slip.svg)

**Yaw rate.**  Positive = counter-clockwise rotation.

![yaw rate](abs_scenarios.charts/high_mu_yaw.svg)

**BTCM firmware internal view.**  What the AVR's ABS
algorithm *thinks* is happening, sampled from the
firmware's `abs_controller_t` state via the host-side
`BTCM_CSV_LOG` snapshot.  Black = chassis truth (ev1sim);
blue = firmware's own `vehicle_speed_mps` reference;
colored = firmware's per-wheel ground speed (rps × tire
radius).  Time axis aligned to BTCM's brake-on event so
t=0 here matches t=0 in the chassis charts above.
Divergences between chassis truth and firmware view
show where the ABS algorithm's perception is wrong.

![BTCM internal view](abs_scenarios.charts/high_mu_btcm_view.svg)

**BTCM-side accelerometer reading.**  The host bridge
publishes longitudinal accel onto the chassis bus and
pokes it into `g_host_sensors` even when the firmware's
ABS algorithm doesn't consume it (build-time gated by
`BTCM_USE_ACCELEROMETER`, default OFF to match original
EV1).  Useful for understanding what a second-gen ABS
would see if the firmware was rebuilt with the gate on.

![BTCM accelerometer](abs_scenarios.charts/high_mu_btcm_accel.svg)

## `low_mu`

### Setup

| Field | Value |
|---|---|
| Surface | uniform ice (μ ≈ 0.10) |
| Approach | throttle to ≥ 15 m/s, throttle off, full brake |
| Brake fires | scenario t = 7 s |
| Validates | wheels lock easily; ABS should modulate and keep wheels rolling |

### Result summary (BTCM-on vs BTCM-off)

| Metric | BTCM-on | BTCM-off |
|---|---|---|
| Brake-on at | t = 38.29 s, v = 15.01 m/s | t = 38.29 s, v = 15.01 m/s |
| Stop result | didn't stop, v\_f = 2.91 m/s | didn't stop, v\_f = 1.57 m/s |
| FL events | 116 | 0 |
| FR events | 107 | 0 |
| Yaw drift over brake | +0.13° | +3.54° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 1.000 | 0.465 | 38.3% |
| FR | 1.000 | 0.476 | 39.7% |
| RL | 0.437 | 0.064 | 0.0% |
| RR | 0.418 | 0.062 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 47.8% | 10.3% | 5.4% | 36.5% | 115 |
| FR | 49.9% | 9.8% | 3.8% | 36.5% | 107 |

### Charts

**[▶ Animated top-down replay (open in browser)](abs_scenarios.charts/low_mu_replay.html)** — watch the car drive through the brake event with per-wheel slip color-coded, surface patches drawn behind, and a scrubbable timeline.  Toggle BTCM-on / BTCM-off in the dropdown to A/B the same instant.

**Vehicle speed**

![speed compare](abs_scenarios.charts/low_mu_speed.svg)

**Effective brake force at each axle.**  Front solid,
rear dashed.  Front line is `applied_front_brake` directly
(the post-ABS-modulation ratio fed to Chrono).  Rear line
is *derived* from `emb_cmd_lr/rr`: max(0, cmd) averaged
across L and R, because the CSV's `applied_rear_brake`
captures the *symmetric* driver-pedal command rather than
the per-wheel override that `ApplyRearEmbBrake` feeds into
Chrono.  BTCM-off rear is forced to 0 here — the EV1's
rear EMB has no hydraulic backup line, so when the
controller is out of the loop the rear free-rolls.

![brake outputs](abs_scenarios.charts/low_mu_brake_outputs.svg)

**Front ABS phase timeline (hydraulic axle).**  Discrete
APPLY/HOLD/DUMP solenoid bands.  See the rear EMB chart
just below for the *equivalent* rear-axle modulation —
same algorithm decisions, different actuator (continuous
motor command in [-1, +1] instead of solenoid phases).

![ABS phase timeline](abs_scenarios.charts/low_mu_phase.svg)

**Rear EMB motor command (electromechanical axle).** Continuous-valued analog of the front Gantt above. `+1` = motor pushing apply, `-1` = motor releasing the shoes, `~0` = hold position.

![rear EMB cmd](abs_scenarios.charts/low_mu_rear_emb.svg)

**Command vs actual actuator state.**  Solid lines are
commands sent to Chrono; dashed lines are the modeled
post-actuator-lag values (caliper hydraulic τ ≈ 50 ms
apply / 80 ms release; rear shoe rate-limited at ~3.33/s).
Useful for understanding why ABS modulation looks blunt
on the speed chart — the actuator lag smooths the rapid
HOLD↔DUMP cycling into a slower-changing pressure curve.

![actuator lag](abs_scenarios.charts/low_mu_actuator_lag.svg)

**Per-wheel ground speed** (chassis line + 4 wheel lines).  Gap = slip.

![per-wheel speeds](abs_scenarios.charts/low_mu_wheel_speed.svg)

**Slip ratios.**  Threshold reference lines at 0.05 and
0.15 mark the algorithm's exit / enter slip thresholds.
Note the chassis-side slip values are *raw tire-contact*
data from Chrono's TMeasy model and look noisier than a
real ABS would see — production ABS heavily filters this
(typically a 10–30 ms low-pass), and our firmware's own
perception of slip (derived from tone-ring counts over
50 ms windows) is naturally smoother.  The peaks here
show what the *tire* is actually doing instant-to-
instant; the firmware's view is a window-average.

![slip ratios](abs_scenarios.charts/low_mu_slip.svg)

**Yaw rate.**  Positive = counter-clockwise rotation.

![yaw rate](abs_scenarios.charts/low_mu_yaw.svg)

**BTCM firmware internal view.**  What the AVR's ABS
algorithm *thinks* is happening, sampled from the
firmware's `abs_controller_t` state via the host-side
`BTCM_CSV_LOG` snapshot.  Black = chassis truth (ev1sim);
blue = firmware's own `vehicle_speed_mps` reference;
colored = firmware's per-wheel ground speed (rps × tire
radius).  Time axis aligned to BTCM's brake-on event so
t=0 here matches t=0 in the chassis charts above.
Divergences between chassis truth and firmware view
show where the ABS algorithm's perception is wrong.

![BTCM internal view](abs_scenarios.charts/low_mu_btcm_view.svg)

**BTCM-side accelerometer reading.**  The host bridge
publishes longitudinal accel onto the chassis bus and
pokes it into `g_host_sensors` even when the firmware's
ABS algorithm doesn't consume it (build-time gated by
`BTCM_USE_ACCELEROMETER`, default OFF to match original
EV1).  Useful for understanding what a second-gen ABS
would see if the firmware was rebuilt with the gate on.

![BTCM accelerometer](abs_scenarios.charts/low_mu_btcm_accel.svg)

## `mu_jump`

### Setup

| Field | Value |
|---|---|
| Surface | asphalt for 100 m, then ice (μ jumps from 0.9 → 0.08) |
| Approach | throttle to ≥ 20 m/s on asphalt, throttle off, full brake — typically straddles or has just crossed onto ice when brake hits |
| Brake fires | scenario t = 6 s |
| Validates | algorithm must adapt as friction drops mid-event |

### Result summary (BTCM-on vs BTCM-off)

| Metric | BTCM-on | BTCM-off |
|---|---|---|
| Brake-on at | t = 7.76 s, v = 15.01 m/s | t = 7.76 s, v = 15.01 m/s |
| Stop result | didn't stop, v\_f = 3.95 m/s | didn't stop, v\_f = 3.65 m/s |
| FL events | 204 | 0 |
| FR events | 176 | 0 |
| Yaw drift over brake | +3.36° | -1.00° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 1.000 | 0.436 | 22.2% |
| FR | 1.000 | 0.574 | 24.6% |
| RL | 0.579 | 0.013 | 0.0% |
| RR | 0.573 | 0.016 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 5.2% | 27.0% | 46.5% | 21.3% | 204 |
| FR | 5.7% | 23.9% | 50.6% | 19.8% | 176 |

### Charts

**[▶ Animated top-down replay (open in browser)](abs_scenarios.charts/mu_jump_replay.html)** — watch the car drive through the brake event with per-wheel slip color-coded, surface patches drawn behind, and a scrubbable timeline.  Toggle BTCM-on / BTCM-off in the dropdown to A/B the same instant.

**Vehicle speed**

![speed compare](abs_scenarios.charts/mu_jump_speed.svg)

**Effective brake force at each axle.**  Front solid,
rear dashed.  Front line is `applied_front_brake` directly
(the post-ABS-modulation ratio fed to Chrono).  Rear line
is *derived* from `emb_cmd_lr/rr`: max(0, cmd) averaged
across L and R, because the CSV's `applied_rear_brake`
captures the *symmetric* driver-pedal command rather than
the per-wheel override that `ApplyRearEmbBrake` feeds into
Chrono.  BTCM-off rear is forced to 0 here — the EV1's
rear EMB has no hydraulic backup line, so when the
controller is out of the loop the rear free-rolls.

![brake outputs](abs_scenarios.charts/mu_jump_brake_outputs.svg)

**Front ABS phase timeline (hydraulic axle).**  Discrete
APPLY/HOLD/DUMP solenoid bands.  See the rear EMB chart
just below for the *equivalent* rear-axle modulation —
same algorithm decisions, different actuator (continuous
motor command in [-1, +1] instead of solenoid phases).

![ABS phase timeline](abs_scenarios.charts/mu_jump_phase.svg)

**Rear EMB motor command (electromechanical axle).** Continuous-valued analog of the front Gantt above. `+1` = motor pushing apply, `-1` = motor releasing the shoes, `~0` = hold position.

![rear EMB cmd](abs_scenarios.charts/mu_jump_rear_emb.svg)

**Command vs actual actuator state.**  Solid lines are
commands sent to Chrono; dashed lines are the modeled
post-actuator-lag values (caliper hydraulic τ ≈ 50 ms
apply / 80 ms release; rear shoe rate-limited at ~3.33/s).
Useful for understanding why ABS modulation looks blunt
on the speed chart — the actuator lag smooths the rapid
HOLD↔DUMP cycling into a slower-changing pressure curve.

![actuator lag](abs_scenarios.charts/mu_jump_actuator_lag.svg)

**Per-wheel ground speed** (chassis line + 4 wheel lines).  Gap = slip.

![per-wheel speeds](abs_scenarios.charts/mu_jump_wheel_speed.svg)

**Slip ratios.**  Threshold reference lines at 0.05 and
0.15 mark the algorithm's exit / enter slip thresholds.
Note the chassis-side slip values are *raw tire-contact*
data from Chrono's TMeasy model and look noisier than a
real ABS would see — production ABS heavily filters this
(typically a 10–30 ms low-pass), and our firmware's own
perception of slip (derived from tone-ring counts over
50 ms windows) is naturally smoother.  The peaks here
show what the *tire* is actually doing instant-to-
instant; the firmware's view is a window-average.

![slip ratios](abs_scenarios.charts/mu_jump_slip.svg)

**Yaw rate.**  Positive = counter-clockwise rotation.

![yaw rate](abs_scenarios.charts/mu_jump_yaw.svg)

**BTCM firmware internal view.**  What the AVR's ABS
algorithm *thinks* is happening, sampled from the
firmware's `abs_controller_t` state via the host-side
`BTCM_CSV_LOG` snapshot.  Black = chassis truth (ev1sim);
blue = firmware's own `vehicle_speed_mps` reference;
colored = firmware's per-wheel ground speed (rps × tire
radius).  Time axis aligned to BTCM's brake-on event so
t=0 here matches t=0 in the chassis charts above.
Divergences between chassis truth and firmware view
show where the ABS algorithm's perception is wrong.

![BTCM internal view](abs_scenarios.charts/mu_jump_btcm_view.svg)

**BTCM-side accelerometer reading.**  The host bridge
publishes longitudinal accel onto the chassis bus and
pokes it into `g_host_sensors` even when the firmware's
ABS algorithm doesn't consume it (build-time gated by
`BTCM_USE_ACCELEROMETER`, default OFF to match original
EV1).  Useful for understanding what a second-gen ABS
would see if the firmware was rebuilt with the gate on.

![BTCM accelerometer](abs_scenarios.charts/mu_jump_btcm_accel.svg)

## `split_mu`

### Setup

| Field | Value |
|---|---|
| Surface | left side asphalt (μ = 0.9), right side ice (μ = 0.08) |
| Approach | throttle to ≥ 20 m/s straddling the seam, throttle off, full brake |
| Brake fires | scenario t = 12 s |
| Validates | yaw stability — without per-wheel ABS the asphalt-side wheels brake while ice-side skids, generating a yaw moment |

### Result summary (BTCM-on vs BTCM-off)

| Metric | BTCM-on | BTCM-off |
|---|---|---|
| Brake-on at | t = 12.02 s, v = 20.86 m/s | t = 12.02 s, v = 20.86 m/s |
| Stop result | 74.74 m / 7.14 s | 72.49 m / 6.53 s |
| FL events | 73 | 0 |
| FR events | 78 | 0 |
| Yaw drift over brake | -0.72° | +0.09° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.729 | 0.085 | 0.0% |
| FR | 0.615 | 0.090 | 0.0% |
| RL | 0.562 | 0.090 | 0.0% |
| RR | 0.734 | 0.090 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 31.0% | 9.1% | 33.2% | 26.8% | 161 |
| FR | 34.6% | 7.6% | 33.2% | 24.6% | 171 |

### Charts

**[▶ Animated top-down replay (open in browser)](abs_scenarios.charts/split_mu_replay.html)** — watch the car drive through the brake event with per-wheel slip color-coded, surface patches drawn behind, and a scrubbable timeline.  Toggle BTCM-on / BTCM-off in the dropdown to A/B the same instant.

**Vehicle speed**

![speed compare](abs_scenarios.charts/split_mu_speed.svg)

**Effective brake force at each axle.**  Front solid,
rear dashed.  Front line is `applied_front_brake` directly
(the post-ABS-modulation ratio fed to Chrono).  Rear line
is *derived* from `emb_cmd_lr/rr`: max(0, cmd) averaged
across L and R, because the CSV's `applied_rear_brake`
captures the *symmetric* driver-pedal command rather than
the per-wheel override that `ApplyRearEmbBrake` feeds into
Chrono.  BTCM-off rear is forced to 0 here — the EV1's
rear EMB has no hydraulic backup line, so when the
controller is out of the loop the rear free-rolls.

![brake outputs](abs_scenarios.charts/split_mu_brake_outputs.svg)

**Front ABS phase timeline (hydraulic axle).**  Discrete
APPLY/HOLD/DUMP solenoid bands.  See the rear EMB chart
just below for the *equivalent* rear-axle modulation —
same algorithm decisions, different actuator (continuous
motor command in [-1, +1] instead of solenoid phases).

![ABS phase timeline](abs_scenarios.charts/split_mu_phase.svg)

**Rear EMB motor command (electromechanical axle).** Continuous-valued analog of the front Gantt above. `+1` = motor pushing apply, `-1` = motor releasing the shoes, `~0` = hold position.

![rear EMB cmd](abs_scenarios.charts/split_mu_rear_emb.svg)

**Command vs actual actuator state.**  Solid lines are
commands sent to Chrono; dashed lines are the modeled
post-actuator-lag values (caliper hydraulic τ ≈ 50 ms
apply / 80 ms release; rear shoe rate-limited at ~3.33/s).
Useful for understanding why ABS modulation looks blunt
on the speed chart — the actuator lag smooths the rapid
HOLD↔DUMP cycling into a slower-changing pressure curve.

![actuator lag](abs_scenarios.charts/split_mu_actuator_lag.svg)

**Per-wheel ground speed** (chassis line + 4 wheel lines).  Gap = slip.

![per-wheel speeds](abs_scenarios.charts/split_mu_wheel_speed.svg)

**Slip ratios.**  Threshold reference lines at 0.05 and
0.15 mark the algorithm's exit / enter slip thresholds.
Note the chassis-side slip values are *raw tire-contact*
data from Chrono's TMeasy model and look noisier than a
real ABS would see — production ABS heavily filters this
(typically a 10–30 ms low-pass), and our firmware's own
perception of slip (derived from tone-ring counts over
50 ms windows) is naturally smoother.  The peaks here
show what the *tire* is actually doing instant-to-
instant; the firmware's view is a window-average.

![slip ratios](abs_scenarios.charts/split_mu_slip.svg)

**Yaw rate.**  Positive = counter-clockwise rotation.

![yaw rate](abs_scenarios.charts/split_mu_yaw.svg)

**BTCM firmware internal view.**  What the AVR's ABS
algorithm *thinks* is happening, sampled from the
firmware's `abs_controller_t` state via the host-side
`BTCM_CSV_LOG` snapshot.  Black = chassis truth (ev1sim);
blue = firmware's own `vehicle_speed_mps` reference;
colored = firmware's per-wheel ground speed (rps × tire
radius).  Time axis aligned to BTCM's brake-on event so
t=0 here matches t=0 in the chassis charts above.
Divergences between chassis truth and firmware view
show where the ABS algorithm's perception is wrong.

![BTCM internal view](abs_scenarios.charts/split_mu_btcm_view.svg)

**BTCM-side accelerometer reading.**  The host bridge
publishes longitudinal accel onto the chassis bus and
pokes it into `g_host_sensors` even when the firmware's
ABS algorithm doesn't consume it (build-time gated by
`BTCM_USE_ACCELEROMETER`, default OFF to match original
EV1).  Useful for understanding what a second-gen ABS
would see if the firmware was rebuilt with the gate on.

![BTCM accelerometer](abs_scenarios.charts/split_mu_btcm_accel.svg)

**Trajectory.**  pos_x vs pos_y over the brake event.

![trajectory](abs_scenarios.charts/split_mu_trajectory.svg)

## `brake_and_steer`

### Setup

| Field | Value |
|---|---|
| Surface | uniform asphalt (μ = 0.9) |
| Approach | throttle to ≥ 25 m/s, throttle off, full brake AND 0.4 steering input applied 0.3 s after brake-on |
| Brake fires | scenario t = 6 s, steering t = 6.3 s |
| Validates | steerability under maximum brake — locked fronts can't generate lateral force, so the BTCM-off case should plow straight while BTCM-on follows the steering input.  Headline metric: yaw drift over the brake event and lateral pos_y. |

### Result summary (BTCM-on vs BTCM-off)

| Metric | BTCM-on | BTCM-off |
|---|---|---|
| Brake-on at | t = 11.28 s, v = 25.05 m/s | t = 11.28 s, v = 25.05 m/s |
| Stop result | 73.94 m / 5.95 s | 67.05 m / 4.99 s |
| FL events | 103 | 0 |
| FR events | 94 | 0 |
| Yaw drift over brake | +82.00° | +27.86° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.998 | 0.429 | 10.8% |
| FR | 0.987 | 0.391 | 5.4% |
| RL | 0.998 | 0.061 | 0.5% |
| RR | 0.604 | 0.087 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 29.8% | 11.6% | 27.7% | 30.9% | 133 |
| FR | 12.6% | 9.5% | 22.1% | 55.8% | 99 |

### Charts

**[▶ Animated top-down replay (open in browser)](abs_scenarios.charts/brake_and_steer_replay.html)** — watch the car drive through the brake event with per-wheel slip color-coded, surface patches drawn behind, and a scrubbable timeline.  Toggle BTCM-on / BTCM-off in the dropdown to A/B the same instant.

**Vehicle speed**

![speed compare](abs_scenarios.charts/brake_and_steer_speed.svg)

**Effective brake force at each axle.**  Front solid,
rear dashed.  Front line is `applied_front_brake` directly
(the post-ABS-modulation ratio fed to Chrono).  Rear line
is *derived* from `emb_cmd_lr/rr`: max(0, cmd) averaged
across L and R, because the CSV's `applied_rear_brake`
captures the *symmetric* driver-pedal command rather than
the per-wheel override that `ApplyRearEmbBrake` feeds into
Chrono.  BTCM-off rear is forced to 0 here — the EV1's
rear EMB has no hydraulic backup line, so when the
controller is out of the loop the rear free-rolls.

![brake outputs](abs_scenarios.charts/brake_and_steer_brake_outputs.svg)

**Front ABS phase timeline (hydraulic axle).**  Discrete
APPLY/HOLD/DUMP solenoid bands.  See the rear EMB chart
just below for the *equivalent* rear-axle modulation —
same algorithm decisions, different actuator (continuous
motor command in [-1, +1] instead of solenoid phases).

![ABS phase timeline](abs_scenarios.charts/brake_and_steer_phase.svg)

**Rear EMB motor command (electromechanical axle).** Continuous-valued analog of the front Gantt above. `+1` = motor pushing apply, `-1` = motor releasing the shoes, `~0` = hold position.

![rear EMB cmd](abs_scenarios.charts/brake_and_steer_rear_emb.svg)

**Command vs actual actuator state.**  Solid lines are
commands sent to Chrono; dashed lines are the modeled
post-actuator-lag values (caliper hydraulic τ ≈ 50 ms
apply / 80 ms release; rear shoe rate-limited at ~3.33/s).
Useful for understanding why ABS modulation looks blunt
on the speed chart — the actuator lag smooths the rapid
HOLD↔DUMP cycling into a slower-changing pressure curve.

![actuator lag](abs_scenarios.charts/brake_and_steer_actuator_lag.svg)

**Per-wheel ground speed** (chassis line + 4 wheel lines).  Gap = slip.

![per-wheel speeds](abs_scenarios.charts/brake_and_steer_wheel_speed.svg)

**Slip ratios.**  Threshold reference lines at 0.05 and
0.15 mark the algorithm's exit / enter slip thresholds.
Note the chassis-side slip values are *raw tire-contact*
data from Chrono's TMeasy model and look noisier than a
real ABS would see — production ABS heavily filters this
(typically a 10–30 ms low-pass), and our firmware's own
perception of slip (derived from tone-ring counts over
50 ms windows) is naturally smoother.  The peaks here
show what the *tire* is actually doing instant-to-
instant; the firmware's view is a window-average.

![slip ratios](abs_scenarios.charts/brake_and_steer_slip.svg)

**Yaw rate.**  Positive = counter-clockwise rotation.

![yaw rate](abs_scenarios.charts/brake_and_steer_yaw.svg)

**BTCM firmware internal view.**  What the AVR's ABS
algorithm *thinks* is happening, sampled from the
firmware's `abs_controller_t` state via the host-side
`BTCM_CSV_LOG` snapshot.  Black = chassis truth (ev1sim);
blue = firmware's own `vehicle_speed_mps` reference;
colored = firmware's per-wheel ground speed (rps × tire
radius).  Time axis aligned to BTCM's brake-on event so
t=0 here matches t=0 in the chassis charts above.
Divergences between chassis truth and firmware view
show where the ABS algorithm's perception is wrong.

![BTCM internal view](abs_scenarios.charts/brake_and_steer_btcm_view.svg)

**BTCM-side accelerometer reading.**  The host bridge
publishes longitudinal accel onto the chassis bus and
pokes it into `g_host_sensors` even when the firmware's
ABS algorithm doesn't consume it (build-time gated by
`BTCM_USE_ACCELEROMETER`, default OFF to match original
EV1).  Useful for understanding what a second-gen ABS
would see if the firmware was rebuilt with the gate on.

![BTCM accelerometer](abs_scenarios.charts/brake_and_steer_btcm_accel.svg)

## `diagonal_mu`

### Setup

| Field | Value |
|---|---|
| Surface | alternating-stripe diagonal split-µ (asphalt μ=0.9, ice μ=0.08, wheelbase-period stripes) |
| Approach | throttle to ≥ 18 m/s on the asphalt runway, throttle off, full brake.  Brake fires inside the diagonal zone where one diagonal pair (FL+RR or FR+RL) is on asphalt and the other diagonal pair is on ice; the diagonal flips every wheelbase as the car drives. |
| Brake fires | scenario t = 6 s |
| Validates | per-wheel ABS modulation under continuously-changing per-wheel friction.  No single-axle (front-only or rear-only) algorithm can keep up here; the BTCM has to make distinct decisions for each wheel based on its own sensor data. |

### Result summary (BTCM-on vs BTCM-off)

| Metric | BTCM-on | BTCM-off |
|---|---|---|
| Brake-on at | t = 8.08 s, v = 18.08 m/s | t = 8.08 s, v = 18.08 m/s |
| Stop result | 97.12 m / 8.86 s | 77.14 m / 7.55 s |
| FL events | 132 | 0 |
| FR events | 120 | 0 |
| Yaw drift over brake | +0.25° | -0.66° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.999 | 0.325 | 4.7% |
| FR | 0.999 | 0.330 | 6.5% |
| RL | 0.325 | 0.036 | 0.0% |
| RR | 0.326 | 0.035 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 27.0% | 12.1% | 48.9% | 12.1% | 226 |
| FR | 17.7% | 11.9% | 24.7% | 45.7% | 121 |

### Charts

**[▶ Animated top-down replay (open in browser)](abs_scenarios.charts/diagonal_mu_replay.html)** — watch the car drive through the brake event with per-wheel slip color-coded, surface patches drawn behind, and a scrubbable timeline.  Toggle BTCM-on / BTCM-off in the dropdown to A/B the same instant.

**Vehicle speed**

![speed compare](abs_scenarios.charts/diagonal_mu_speed.svg)

**Effective brake force at each axle.**  Front solid,
rear dashed.  Front line is `applied_front_brake` directly
(the post-ABS-modulation ratio fed to Chrono).  Rear line
is *derived* from `emb_cmd_lr/rr`: max(0, cmd) averaged
across L and R, because the CSV's `applied_rear_brake`
captures the *symmetric* driver-pedal command rather than
the per-wheel override that `ApplyRearEmbBrake` feeds into
Chrono.  BTCM-off rear is forced to 0 here — the EV1's
rear EMB has no hydraulic backup line, so when the
controller is out of the loop the rear free-rolls.

![brake outputs](abs_scenarios.charts/diagonal_mu_brake_outputs.svg)

**Front ABS phase timeline (hydraulic axle).**  Discrete
APPLY/HOLD/DUMP solenoid bands.  See the rear EMB chart
just below for the *equivalent* rear-axle modulation —
same algorithm decisions, different actuator (continuous
motor command in [-1, +1] instead of solenoid phases).

![ABS phase timeline](abs_scenarios.charts/diagonal_mu_phase.svg)

**Rear EMB motor command (electromechanical axle).** Continuous-valued analog of the front Gantt above. `+1` = motor pushing apply, `-1` = motor releasing the shoes, `~0` = hold position.

![rear EMB cmd](abs_scenarios.charts/diagonal_mu_rear_emb.svg)

**Command vs actual actuator state.**  Solid lines are
commands sent to Chrono; dashed lines are the modeled
post-actuator-lag values (caliper hydraulic τ ≈ 50 ms
apply / 80 ms release; rear shoe rate-limited at ~3.33/s).
Useful for understanding why ABS modulation looks blunt
on the speed chart — the actuator lag smooths the rapid
HOLD↔DUMP cycling into a slower-changing pressure curve.

![actuator lag](abs_scenarios.charts/diagonal_mu_actuator_lag.svg)

**Per-wheel ground speed** (chassis line + 4 wheel lines).  Gap = slip.

![per-wheel speeds](abs_scenarios.charts/diagonal_mu_wheel_speed.svg)

**Slip ratios.**  Threshold reference lines at 0.05 and
0.15 mark the algorithm's exit / enter slip thresholds.
Note the chassis-side slip values are *raw tire-contact*
data from Chrono's TMeasy model and look noisier than a
real ABS would see — production ABS heavily filters this
(typically a 10–30 ms low-pass), and our firmware's own
perception of slip (derived from tone-ring counts over
50 ms windows) is naturally smoother.  The peaks here
show what the *tire* is actually doing instant-to-
instant; the firmware's view is a window-average.

![slip ratios](abs_scenarios.charts/diagonal_mu_slip.svg)

**Yaw rate.**  Positive = counter-clockwise rotation.

![yaw rate](abs_scenarios.charts/diagonal_mu_yaw.svg)

**BTCM firmware internal view.**  What the AVR's ABS
algorithm *thinks* is happening, sampled from the
firmware's `abs_controller_t` state via the host-side
`BTCM_CSV_LOG` snapshot.  Black = chassis truth (ev1sim);
blue = firmware's own `vehicle_speed_mps` reference;
colored = firmware's per-wheel ground speed (rps × tire
radius).  Time axis aligned to BTCM's brake-on event so
t=0 here matches t=0 in the chassis charts above.
Divergences between chassis truth and firmware view
show where the ABS algorithm's perception is wrong.

![BTCM internal view](abs_scenarios.charts/diagonal_mu_btcm_view.svg)

**BTCM-side accelerometer reading.**  The host bridge
publishes longitudinal accel onto the chassis bus and
pokes it into `g_host_sensors` even when the firmware's
ABS algorithm doesn't consume it (build-time gated by
`BTCM_USE_ACCELEROMETER`, default OFF to match original
EV1).  Useful for understanding what a second-gen ABS
would see if the firmware was rebuilt with the gate on.

![BTCM accelerometer](abs_scenarios.charts/diagonal_mu_btcm_accel.svg)

