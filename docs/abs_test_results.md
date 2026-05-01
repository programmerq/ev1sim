# ABS validation test results — first pass

Four standard ABS validation tests, BTCM-on vs BTCM-off comparison.
Run with `./scripts/run_abs_compare.sh <test>` from the ev1sim repo
root; outputs land in `/tmp/ev1sim_abs_<test>/`.

## Quick reference — what each test checks

| Test            | Surface                           | Validates                                       |
|---|---|---|
| `high_mu`       | uniform asphalt (μ=0.9)           | does ABS at least not hurt on dry roads?        |
| `low_mu`        | uniform ice (μ=0.10)              | does ABS keep wheels at peak slip on slick?     |
| `mu_jump`       | asphalt → ice transition          | does ABS adapt when grip suddenly drops?        |
| `split_mu`      | left=asphalt, right=ice           | does per-wheel ABS keep the car straight?       |

In real automotive validation these go by names like "high-mu stop",
"split-friction stop", and "mu-jump stop".  The terminology is
self-explanatory: μ ("mu") is the friction coefficient between tire
and road, "split" means the left and right wheels see different
surfaces, and "jump" means the surface changes mid-stop.

## Headline results (post-SignalDefine fix, 2026-04-30)

| Test     | BTCM-on stop      | BTCM-off stop    | Front locked time | ABS phase changes |
|---|---|---|---|---|
| high_mu  | 148.55 m / 9.50 s | 148.52 m / 9.47 s | 0 % / 0 %          | 0 |
| low_mu   | did NOT stop in 60 s, vf=1.16 m/s | vf=1.57 m/s | 37-41 % / 37-41 % | 0 |
| mu_jump  | did NOT stop in 30 s, vf=18.16 m/s | vf=18.60 m/s | 21-24 % / 21-24 % | 0 |
| split_mu | 94.00 m / 7.81 s  | 77.68 m / 6.78 s | 0 % / 0 %          | 0 |

**ABS phase transitions during the brake event: zero across all four
tests.**  The BTCM AVR firmware's ABS state machine settles into
`APPLY` (no modulation) at startup and never transitions to `HOLD` /
`DUMP` even when wheels are clearly locked (37-41 % time-locked on
ice).  This is the dominant outstanding finding.

## What the data shows about the ABS chain

- **Hardware-modeling chain works correctly.**  Rear EMB peak slip
  is consistently ~0.7 with BTCM-on vs ~0.05 with BTCM-off — proves
  the BTCM rear-motor commands are reaching ev1sim and the
  `BrakeDrum` self-energizing model is producing torque per-wheel.
- **Front ABS modulator wiring works correctly.**  When the firmware
  briefly toggles solenoid pins at startup (POST self-test), the
  phase transitions appear in `abs_phase_fl/fr` columns of the CSV
  with the correct freshness behavior.
- **The AVR firmware ABS algorithm doesn't engage.**  Zero phase
  transitions during any brake event, even when wheels lock for
  20-41 % of the time on ice.  Either the firmware needs more
  inputs than it currently has (run_1, accel, etc.), or the slip
  detection threshold is set so high it never trips, or the
  tone-ring pulse rate isn't reaching the AVR fast enough for the
  algorithm to compute wheel speed deltas at the relevant cadence.

## Anti-result on split-mu (worth noting)

On `split_mu`, BTCM-**on** stops in 94 m and BTCM-**off** stops in
77 m — i.e. enabling the brake controller makes the car *worse*.
This isn't an ABS regression; it's a known-correct simulation
artifact:

- BTCM-on commands the rear EMB at full force on **both** sides.
  The right rear is on ice (μ=0.08) and skids freely without
  contributing braking, but its drag still upsets the chassis.
- BTCM-off doesn't command the rear at all.  Both rear wheels
  free-roll, and only the front (asphalt-side dominant) decelerates
  the car.

A working per-wheel ABS would modulate the ice-side rear down to
peak-slip and let the asphalt-side rear use its full friction.
Until the AVR firmware ABS engages, the BTCM-on case can't be
better than the BTCM-off case on asymmetric surfaces — and is
sometimes worse.

## Repro

```sh
# Run any one test:
./scripts/run_abs_compare.sh high_mu
./scripts/run_abs_compare.sh low_mu
./scripts/run_abs_compare.sh mu_jump
./scripts/run_abs_compare.sh split_mu

# All tests (writes /tmp/ev1sim_abs_<test>/summary.txt for each):
for t in high_mu low_mu mu_jump split_mu; do
    ./scripts/run_abs_compare.sh "$t"
done
```

Each comparison prints two ASCII overlays: vehicle speed
(BTCM-on vs BTCM-off) and chassis vs per-wheel "ground-equivalent
speed" (the gap is slip).  The BTCM-on plot has an annotation lane
showing front-left ABS phase: `_` for APPLY, `H` for HOLD,
`D` for DUMP (currently all `_` because the firmware doesn't
modulate).

## Next concrete step

A focused firmware diagnostic on `electricsim/ev1/btcm/firmware/`:
trace what the ABS state machine actually evaluates each loop, and
why it never leaves APPLY.  Likely candidates: missing `run_1`
input, slip threshold mis-tuned for our wheel-pulse rate, or the
algorithm waiting on a peer signal (peer-comm UART frame from RSA
or PIM) that the simulator doesn't drive.
