# EV1 ABS scenario validation — comparison report

_Generated 2026-05-01 09:19 by `scripts/abs_report.py`._

Source: `/tmp/ev1sim_abs_<test>/summary.txt` for each of the four
standard validation scenarios.  Re-run with
`for t in high_mu low_mu mu_jump split_mu; do ./scripts/run_abs_compare.sh $t; done`
then re-run this script.

## Headline: stopping distance, BTCM-on vs BTCM-off

| Test | BTCM-on stop | BTCM-off stop | Δ |
|---|---|---|---|
| high_mu | 152.0 m / 9.98 s | 148.5 m / 9.47 s | +3.5 m |
| low_mu | didn't stop, v\_f = 31.64 m/s | didn't stop, v\_f = 1.57 m/s | — |
| mu_jump | didn't stop, v\_f = 23.54 m/s | didn't stop, v\_f = 18.60 m/s | — |
| split_mu | 92.0 m / 7.87 s | 72.5 m / 6.53 s | +19.5 m |

## ABS engagement (front phase transitions per wheel)

| Test | BTCM-on FL/FR | BTCM-off FL/FR |
|---|---:|---:|
| high_mu | 35 / 35 | 0 / 0 |
| low_mu | 151 / 143 | 0 / 0 |
| mu_jump | 38 / 55 | 0 / 0 |
| split_mu | 17 / 21 | 0 / 0 |

## Per-wheel time-locked percentage during brake event

| Test | BTCM-on | BTCM-off |
|---|---|---|
| high_mu | FL 0% / FR 0% / RL 0% / RR 0% | FL 0% / FR 0% / RL 0% / RR 0% |
| low_mu | FL 51% / FR 44% / RL 0% / RR 0% | FL 36% / FR 41% / RL 0% / RR 0% |
| mu_jump | FL 18% / FR 80% / RL 1% / RR 1% | FL 21% / FR 24% / RL 0% / RR 0% |
| split_mu | FL 0% / FR 0% / RL 0% / RR 0% | FL 0% / FR 0% / RL 0% / RR 0% |

## Per-wheel peak slip during brake event

| Test | BTCM-on (FL / FR / RL / RR) | BTCM-off (FL / FR / RL / RR) |
|---|---|---|
| high_mu | 0.56 / 0.60 / 0.59 / 0.66 | 0.66 / 0.63 / 0.61 / 0.61 |
| low_mu | 1.00 / 1.00 / 0.03 / 0.03 | 1.00 / 1.00 / 0.55 / 0.54 |
| mu_jump | 1.00 / 1.00 / 0.99 / 0.99 | 1.00 / 1.00 / 0.01 / 0.02 |
| split_mu | 0.65 / 0.52 / 0.69 / 0.61 | 0.45 / 0.62 / 0.74 / 0.64 |

## How to interpret

- **Δ stop distance** in the headline table is BTCM-on minus
  BTCM-off.  Positive = ABS made it worse, negative = ABS helped.
  An ideal ABS lands close to zero on dry pavement (where wheel
  lock isn't a real risk) and strongly negative on slippery
  surfaces (where lock-prevention pays off).
- **Phase transitions** = number of times the front-wheel ABS
  state machine moved between APPLY/HOLD/DUMP during the brake
  event.  Zero on dry pavement is correct; double-digit counts
  on slippery surfaces show the algorithm is engaging.
- **Time-locked** = % of brake event the wheel was at near-zero
  rotation.  ABS is supposed to keep this at 0 % (or close).
  High locked % despite high engagement count means the algorithm
  is firing but not effectively modulating.
- **Peak slip** = highest instantaneous slip ratio seen.  Real
  ABS targets the peak-grip slip range (~0.10–0.15).  Slip near
  1.0 means the wheel was fully locked.  Slip near 0 means it was
  rolling cleanly.

