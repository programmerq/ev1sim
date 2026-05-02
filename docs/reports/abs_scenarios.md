# EV1 ABS validation — engineering report

_Generated 2026-05-01 23:50 by `scripts/abs_report.py`._

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
| high_mu | 150.30 m / 9.98 s | 148.52 m / 9.47 s | +1.8 m | 69 / 70 | +0.01° / +0.07° |
| low_mu | didn't stop, v\_f = 3.11 m/s | didn't stop, v\_f = 1.57 m/s | — | 142 / 130 | -0.53° / +3.54° |
| mu_jump | didn't stop, v\_f = 4.19 m/s | didn't stop, v\_f = 3.65 m/s | — | 239 / 266 | +1.24° / -1.00° |
| split_mu | 74.20 m / 6.98 s | 72.49 m / 6.53 s | +1.7 m | 71 / 66 | +0.09° / +0.09° |

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
| Stop result | 150.30 m / 9.98 s | 148.52 m / 9.47 s |
| FL events | 69 | 0 |
| FR events | 70 | 0 |
| Yaw drift over brake | +0.01° | +0.07° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.709 | 0.066 | 0.0% |
| FR | 0.664 | 0.067 | 0.0% |
| RL | 0.666 | 0.069 | 0.0% |
| RR | 0.571 | 0.066 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 18.4% | 8.8% | 12.0% | 60.8% | 75 |
| FR | 19.9% | 8.8% | 12.9% | 58.5% | 85 |

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
| Stop result | didn't stop, v\_f = 3.11 m/s | didn't stop, v\_f = 1.57 m/s |
| FL events | 142 | 0 |
| FR events | 130 | 0 |
| Yaw drift over brake | -0.53° | +3.54° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 1.000 | 0.466 | 38.5% |
| FR | 1.000 | 0.474 | 39.2% |
| RL | 0.391 | 0.060 | 0.0% |
| RR | 0.407 | 0.060 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 47.2% | 11.9% | 5.9% | 35.0% | 141 |
| FR | 50.5% | 10.9% | 4.8% | 33.8% | 130 |

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
| Stop result | didn't stop, v\_f = 4.19 m/s | didn't stop, v\_f = 3.65 m/s |
| FL events | 239 | 0 |
| FR events | 266 | 0 |
| Yaw drift over brake | +1.24° | -1.00° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 1.000 | 0.520 | 10.1% |
| FR | 1.000 | 0.344 | 3.3% |
| RL | 0.562 | 0.015 | 0.0% |
| RR | 0.559 | 0.014 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 6.1% | 23.5% | 60.2% | 10.2% | 239 |
| FR | 5.2% | 28.0% | 60.2% | 6.7% | 266 |

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
| Stop result | 74.20 m / 6.98 s | 72.49 m / 6.53 s |
| FL events | 71 | 0 |
| FR events | 66 | 0 |
| Yaw drift over brake | +0.09° | +0.09° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.696 | 0.074 | 0.0% |
| FR | 0.509 | 0.078 | 0.0% |
| RL | 0.556 | 0.087 | 0.0% |
| RR | 0.809 | 0.090 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 31.2% | 7.1% | 34.9% | 26.8% | 151 |
| FR | 33.4% | 7.4% | 31.9% | 27.3% | 147 |

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

