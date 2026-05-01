# EV1 ABS validation — engineering report

_Generated 2026-05-01 13:24 by `scripts/abs_report.py`._

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
| high_mu | 152.03 m / 9.98 s | 148.52 m / 9.47 s | +3.5 m | 35 / 35 | +3.22° / +0.07° |
| low_mu | didn't stop, v\_f = 31.64 m/s | didn't stop, v\_f = 1.57 m/s | — | 151 / 143 | -3.67° / +3.54° |
| mu_jump | didn't stop, v\_f = 23.54 m/s | didn't stop, v\_f = 18.60 m/s | — | 38 / 55 | -0.84° / +0.24° |
| split_mu | 91.96 m / 7.87 s | 72.49 m / 6.53 s | +19.5 m | 17 / 21 | -0.11° / +0.09° |

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
| Stop result | 152.03 m / 9.98 s | 148.52 m / 9.47 s |
| FL events | 35 | 0 |
| FR events | 35 | 0 |
| Yaw drift over brake | +3.22° | +0.07° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.562 | 0.065 | 0.0% |
| FR | 0.597 | 0.059 | 0.0% |
| RL | 0.594 | 0.065 | 0.0% |
| RR | 0.658 | 0.067 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 67.0% | 12.6% | 7.3% | 13.2% | 38 |
| FR | 74.3% | 11.4% | 7.9% | 6.4% | 37 |

### Charts

![speed compare](abs_scenarios.charts/high_mu_speed.svg)

![per-wheel speeds](abs_scenarios.charts/high_mu_wheel_speed.svg)

![slip ratios](abs_scenarios.charts/high_mu_slip.svg)

![ABS phase timeline](abs_scenarios.charts/high_mu_phase.svg)

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
| Stop result | didn't stop, v\_f = 31.64 m/s | didn't stop, v\_f = 1.57 m/s |
| FL events | 151 | 0 |
| FR events | 143 | 0 |
| Yaw drift over brake | -3.67° | +3.54° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 1.000 | 0.549 | 50.6% |
| FR | 1.000 | 0.471 | 44.0% |
| RL | 0.028 | 0.013 | 0.0% |
| RR | 0.028 | 0.012 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 15.7% | 30.0% | 50.7% | 3.6% | 149 |
| FR | 16.4% | 27.7% | 52.2% | 3.6% | 141 |

### Charts

![speed compare](abs_scenarios.charts/low_mu_speed.svg)

![per-wheel speeds](abs_scenarios.charts/low_mu_wheel_speed.svg)

![slip ratios](abs_scenarios.charts/low_mu_slip.svg)

![ABS phase timeline](abs_scenarios.charts/low_mu_phase.svg)

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
| Stop result | didn't stop, v\_f = 23.54 m/s | didn't stop, v\_f = 18.60 m/s |
| FL events | 38 | 0 |
| FR events | 55 | 0 |
| Yaw drift over brake | -0.84° | +0.24° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 1.000 | 0.367 | 17.8% |
| FR | 1.000 | 0.948 | 80.3% |
| RL | 0.992 | 0.065 | 0.6% |
| RR | 0.993 | 0.066 | 0.6% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 11.3% | 23.3% | 57.2% | 8.2% | 37 |
| FR | 11.9% | 33.3% | 46.5% | 8.2% | 54 |

### Charts

![speed compare](abs_scenarios.charts/mu_jump_speed.svg)

![per-wheel speeds](abs_scenarios.charts/mu_jump_wheel_speed.svg)

![slip ratios](abs_scenarios.charts/mu_jump_slip.svg)

![ABS phase timeline](abs_scenarios.charts/mu_jump_phase.svg)

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
| Stop result | 91.96 m / 7.87 s | 72.49 m / 6.53 s |
| FL events | 17 | 0 |
| FR events | 21 | 0 |
| Yaw drift over brake | -0.11° | +0.09° |

### Per-wheel slip statistics (BTCM-on)

| Wheel | peak | mean | time-locked |
|---|---:|---:|---:|
| FL | 0.650 | 0.064 | 0.0% |
| FR | 0.521 | 0.070 | 0.0% |
| RL | 0.692 | 0.083 | 0.0% |
| RR | 0.607 | 0.080 | 0.0% |

### Front ABS phase distribution during brake event (BTCM-on)

| Wheel | APPLY | HOLD | DUMP | stale | phase transitions |
|---|---:|---:|---:|---:|---:|
| FL | 49.4% | 2.2% | 33.9% | 14.5% | 36 |
| FR | 41.8% | 4.9% | 31.4% | 21.9% | 39 |

### Charts

![speed compare](abs_scenarios.charts/split_mu_speed.svg)

![per-wheel speeds](abs_scenarios.charts/split_mu_wheel_speed.svg)

![slip ratios](abs_scenarios.charts/split_mu_slip.svg)

![ABS phase timeline](abs_scenarios.charts/split_mu_phase.svg)

![yaw rate](abs_scenarios.charts/split_mu_yaw.svg)

![trajectory](abs_scenarios.charts/split_mu_trajectory.svg)

