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
| `low_mu`   | uniform ice (μ ≈ 0.10)           | ABS should keep wheels rolling on slippery     |
| `mu_jump`  | asphalt → ice transition         | algorithm must adapt as grip suddenly drops    |
| `split_mu` | left = asphalt, right = ice      | per-wheel ABS keeps the car going straight    |

Industry calls these "high-mu stop", "low-mu stop", "split-friction
stop", and "mu-jump" / "transition" stop.  μ ("mu") is the friction
coefficient between tire and road.

## How to run

```sh
# All four:
for t in high_mu low_mu mu_jump split_mu; do
    ./scripts/run_abs_compare.sh "$t"
done

# Then regenerate the engineering report:
./scripts/abs_report.py docs/reports/abs_scenarios.md

# Optional: refresh the tone-ring validation memo too:
./build/ev1/btcm/test_btcm_tone_ring_validation \
    ../electricsim/build/firmware/btcm_firmware.elf \
    docs/reports/tone_ring_validation.md
```

`run_abs_compare.sh` writes
`/tmp/ev1sim_abs_<test>/{summary.txt,abs_btcm_on.csv,abs_btcm_off.csv,*.log}`.
The report script reads those and emits markdown + SVG charts under
`docs/reports/`.

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
