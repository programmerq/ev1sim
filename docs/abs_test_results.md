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

## Headline results (post tone-ring sign + interleave fixes, 2026-04-30)

| Test     | BTCM-on stop      | BTCM-off stop    | Front locked time | ABS phase changes |
|---|---|---|---|---|
| high_mu  | did NOT stop, vf≈18 m/s            | 148.52 m / 9.47 s | 0 % / 0 %         | 86 / 88 |
| low_mu   | did NOT stop in 60 s, vf≈30.8 m/s  | vf=1.57 m/s       | 49-55 % / 36-41 % | 154 / 126 |
| mu_jump  | did NOT stop in 30 s, vf≈23.3 m/s  | vf=18.60 m/s      | 56 % / 17 % FL/FR | 53 / 32 |
| split_mu | 93.6 m / 7.6 s (when ABS quiet); doesn't stop when ABS engages | 77.7 m / 6.78 s | 0 % / 0 %         | 0 / 96 (flaky) |

The firmware ABS algorithm now engages on three of four scenarios.
The fourth (`split_mu`) is sensitive to startup timing — see below.
Engagement is the *good* news; the *bad* news is that the algorithm
over-dumps and makes braking worse than the no-ABS baseline on every
test where it engages.

### Fixed: bus-side ABS phase freshness (consumer)

`GetAbsPhaseFront` ANDed iso/dmp ages against the freshness window.
HOLD↔DUMP cycling only toggles dmp; iso stays at 1, so its timestamp
ages out and the entire brake event reads stale.  Switched to OR.

### Fixed: ev1sim → BTCM tone-ring chain

Two upstream bugs prevented the firmware from seeing wheel rotation:

  - **Sign of omega.**  Chrono's `GetSpindleOmega` is signed per the
    spindle's body frame, so forward-rolling wheels often arrive as
    negative numbers.  The host's tone-gen clamps phase ≥ 0 — negative
    omega advances phase to 0 and emits no edges.  Real tone-ring
    sensors are direction-agnostic; the controller now takes
    `std::abs(omega)` at the bus boundary.
  - **Edge coalescing in the host loop.**  The host-side
    `ToneRingGen::step()` emits N level toggles per call when
    `omega × dt > kHalfTooth`.  All toggles ran between AVR cycles, so
    the AVR core only saw the final pin level — usually right back at
    the starting value, or one transition instead of N.  At highway
    omega the firmware undercounted by ~3 ×.  Fixed by splitting the
    20 000-cycle AVR step into 100 sub-steps of 200 cycles each and
    advancing the tone gen between sub-steps.

These two together restored 4-wheel speed visibility to the firmware
and unblocked the slip / decel branches of the ABS state machine.

### Outstanding: ABS over-dumps (firmware tuning)

Once engaged, the algorithm releases too much pressure.  On
`high_mu`, BTCM-on holds front slip at 0.03 — essentially free-rolling
— and never accumulates enough deceleration to stop within scenario
time.  On `low_mu`, `mu_jump`, and `split_mu` it does the same and
ends up worse than the no-ABS case.  Likely candidates: the
`dump_duration_ms` is too long for the simulated valve dynamics; the
exit-DUMP slip threshold (0.05) is too low (real ABS targets ~0.10);
and the rear axle's longer DUMP (18 ms) compounds the issue on the
self-energizing drum.

### Outstanding: split_mu engagement is flaky

In repeated runs `split_mu` reports either 0 / 0 phase events (and
stops normally) or ~96 / 96 phase events (and over-dumps).  Root
cause: `kSigDriverBrakePedalQ8` and `kSigChassisBrakeMasterPressureKpa`
are both publish-on-change in ev1sim — if BTCM connects to the bus
*after* the brake transition fires, it never sees the change and
runs as if brake is never pressed.  The scenario's brake event was
moved out to `t=12s` (was `t=7s`) to give BTCM more startup time, but
the underlying issue is producer-side: ev1sim should publish
brake-state heartbeats so late-joining consumers can pick up the
current value.

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
