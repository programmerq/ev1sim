# EV1 ABS validation — engineering report

_Generated 2026-05-01 14:05 by `scripts/abs_report.py`._

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
| high_mu | 152.13 m / 9.98 s | 148.52 m / 9.47 s | +3.6 m | 34 / 38 | +1.50° / +0.07° |
| low_mu | didn't stop, v\_f = 31.68 m/s | didn't stop, v\_f = 1.57 m/s | — | 138 / 146 | +1.01° / +3.54° |
| mu_jump | didn't stop, v\_f = 23.04 m/s | didn't stop, v\_f = 18.60 m/s | — | 51 / 44 | -2.29° / +0.24° |
| split_mu | 87.11 m / 7.81 s | 72.49 m / 6.53 s | +14.6 m | 21 / 27 | +0.29° / +0.09° |

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
| Stop result | 152.13 m / 9.98 s | 148.52 m / 9.47 s |
| FL events | 34 | 0 |
| FR events | 38 | 0 |
| Yaw drift over brake | +1.50° | +0.07° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.767 | 0.068 | 0.0% |
| FR | 0.824 | 0.068 | 0.0% |
| RL | 0.941 | 0.073 | 0.0% |
| RR | 0.645 | 0.070 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 74.6% | 10.8% | 7.9% | 6.7% | 37 |
| FR | 71.9% | 11.7% | 7.6% | 8.8% | 41 |

### Charts

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
| Stop result | didn't stop, v\_f = 31.68 m/s | didn't stop, v\_f = 1.57 m/s |
| FL events | 138 | 0 |
| FR events | 146 | 0 |
| Yaw drift over brake | +1.01° | +3.54° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 1.000 | 0.502 | 44.0% |
| FR | 1.000 | 0.537 | 47.5% |
| RL | 0.028 | 0.013 | 0.0% |
| RR | 0.028 | 0.012 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 16.6% | 26.8% | 53.9% | 2.7% | 137 |
| FR | 17.2% | 28.3% | 51.8% | 2.7% | 145 |

### Charts

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
| Brake-on at | t = 19.95 s, v = 20.01 m/s | t = 19.95 s, v = 20.01 m/s |
| Stop result | didn't stop, v\_f = 23.04 m/s | didn't stop, v\_f = 18.60 m/s |
| FL events | 51 | 0 |
| FR events | 44 | 0 |
| Yaw drift over brake | -2.29° | +0.24° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 1.000 | 0.804 | 58.9% |
| FR | 1.000 | 0.426 | 30.9% |
| RL | 0.996 | 0.064 | 0.6% |
| RR | 0.994 | 0.068 | 0.6% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 16.4% | 32.1% | 38.4% | 13.2% | 50 |
| FR | 16.4% | 27.7% | 42.8% | 13.2% | 43 |

### Charts

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
| Stop result | 87.11 m / 7.81 s | 72.49 m / 6.53 s |
| FL events | 21 | 0 |
| FR events | 27 | 0 |
| Yaw drift over brake | +0.29° | +0.09° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.570 | 0.084 | 0.0% |
| FR | 0.689 | 0.085 | 0.0% |
| RL | 0.753 | 0.094 | 0.0% |
| RR | 0.811 | 0.090 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 58.5% | 3.4% | 27.5% | 10.6% | 39 |
| FR | 61.9% | 4.2% | 27.8% | 6.1% | 47 |

### Charts

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

**Trajectory.**  pos_x vs pos_y over the brake event.

![trajectory](abs_scenarios.charts/split_mu_trajectory.svg)

