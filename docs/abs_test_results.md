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

## Headline results (post AbsPhaseFront freshness fix, 2026-04-30)

| Test     | BTCM-on stop      | BTCM-off stop    | Front locked time | ABS phase changes |
|---|---|---|---|---|
| high_mu  | 148.52 m / 9.47 s  | 148.52 m / 9.47 s | 0 % / 0 %          | 0 / 0 |
| low_mu   | did NOT stop in 60 s, vf≈22.85 m/s | vf=1.57 m/s | 39-46 % / 36-41 % | 79 / 79 |
| mu_jump  | did NOT stop in 30 s, vf=18.16 m/s | vf=18.60 m/s | 20-24 % / 20-24 % | 0 / 0 |
| split_mu | 92.92 m / 7.87 s  | 77.68 m / 6.78 s | 0 % / 0 %          | 0 / 0 |

**Mixed picture:** the bus-side reporting bug is fixed (`low_mu` now
shows 79 phase events per front wheel during the brake), but two
scenarios still report zero engagement despite obvious wheel lock.
Two distinct issues are at play.

### Fixed: AbsPhaseFront bus-side freshness (consumer side)

The `low_mu` run was producing ~130 firmware ABS phase events but the
ev1sim CSV showed 0.  Root cause: solenoid signals publish *on
change*, but the firmware's ABS algorithm enters HOLD↔DUMP cycling
during a long lock event — and *both* phases hold `iso=1`.  Only the
dump pin toggles, so `kSigSolFL_ISO` was published once at HOLD entry
and never refreshed for the remainder of the brake event.
`ExternalSimConnector::GetAbsPhaseFront`'s freshness check required
*both* `ts_iso` and `ts_dmp` ages within the 200 ms window, which
went stale ~200 ms after the first HOLD entry.  Fix: OR the ages
instead of AND.  As long as either pin updated recently, the producer
is alive and the last-known values of both pins are valid.

### Outstanding: firmware ABS algorithm doesn't engage on mu-jump
and split-mu

`mu_jump` has fronts 20-24 % time-locked and `split_mu` has rears
exhibiting 0.7+ peak slip on the ice side, yet the firmware logs
*zero* phase events during either brake (only the boot POST + a
release-edge artifact).  The firmware genuinely doesn't see these as
slip events worth modulating.

Likely candidates for the firmware tuning gap, ranked by suspicion:

1. **Reference speed estimator collapses with the wheels.**  When all
   four wheels lock together (mu_jump second half), the
   `max_ref_decel_mps2 = 11.0` clamp prevents instant collapse but
   only buys ~2 s before slip estimates approach zero.
2. **Tone-ring sample window vs. locked-wheel detection.**  The
   firmware integrates wheel pulses over a 50 ms window.  For a
   fully-locked wheel emitting zero pulses, the firmware reads
   `wheel_rps = 0` correctly — but the *deceleration estimate* (used
   for early HOLD via `decel_threshold_rps2`) requires several
   consecutive samples before `wheel_accel_rps2` exceeds threshold,
   by which time slip has already saturated.
3. **Asymmetric grip on split-mu doesn't lock either side enough.**
   Ice-side wheels skid freely (slip ≈ 0.7) but never time-lock
   because they're free-rolling, not stopped — and the
   `slip_threshold_enter = 0.15` *should* trigger HOLD here, so this
   one is the most surprising and worth focused diagnosis.

Next focused batch on the firmware: trace what `vehicle_speed_mps`
and per-wheel `slip_ratio` look like *inside* the ATmega328P during
a `split_mu` brake event.  Add a debug log (perhaps gated on a
`-DABS_TRACE` flag) emitting the algorithm's view of slip + accel
each tick.

## What the data shows about the ABS chain

- **Hardware-modeling chain works correctly.**  Rear EMB peak slip
  is consistently ~0.7 with BTCM-on vs ~0.05 with BTCM-off (visible
  on `mu_jump` rear data: 0.99 BTCM-on vs 0.014 BTCM-off) — proves
  the BTCM rear-motor commands are reaching ev1sim and the
  `BrakeDrum` self-energizing model is producing torque per-wheel.
- **Front ABS modulator wiring works correctly end-to-end.**  Now
  visible in the `low_mu` annotation lane: `DDDDDDDDDD____` showing
  the ~16 s sustained DUMP that the firmware actually commands.
- **The firmware-side algorithm tuning still has gaps.**  Wheels
  visibly lock on `mu_jump` and `split_mu` without provoking
  modulation.  See the breakdown in the section above.

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
`D` for DUMP.  The `low_mu` run shows sustained `DDDDDD...` as
the firmware aggressively dumps front pressure on ice.

## Next concrete step

A focused diagnostic on the firmware-side ABS algorithm, since the
bus-side reporting is now trustworthy.  Specifically: instrument
`abs_algo.c` (gated on a build-time flag so we don't bloat the
production firmware) to emit per-tick records of
`vehicle_speed_mps`, per-wheel `wheel_speed_rps`,
`wheel_accel_rps2`, and the resulting `slip` value during a
`split_mu` run.  Compare against ev1sim's per-wheel slip ratios in
the CSV — that gap is the firmware's perception error, and it
should point at the specific tuning parameter that's wrong.
