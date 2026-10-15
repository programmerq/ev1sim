# EV1 vehicle-dynamics notes (Round 4)

Physics-fidelity work on the Chrono::Vehicle model: **brake bias**, **body
aerodynamics**, and a **suspension-geometry audit**.  The Chrono app itself is
local-only (Chrono isn't in CI), so each item ships an executable, Chrono-free
regression guard that runs in the normal unit-test suite:

| Area | Guard |
|------|-------|
| Brake bias  | [`tests/test_brake_bias.cpp`](../tests/test_brake_bias.cpp) |
| Aerodynamics | [`tests/test_aerodynamics.cpp`](../tests/test_aerodynamics.cpp) + [`src/Aerodynamics.h`](../src/Aerodynamics.h) |
| Suspension  | [`tests/test_suspension_geometry.cpp`](../tests/test_suspension_geometry.cpp) |

---

## Brake bias — front disc / rear drum

The real EV1 runs **front vented discs + rear drums**.  Previously all four
wheels shared one `EV1_BrakeSimple.json` at 800 N·m/wheel (3200 N·m total,
sized for ~0.88 g friction-limited peak decel).  Round 4 splits that into two
per-axle files and **biases the split front while preserving the total**:

| File | Wheel | Pair | Share |
|------|------:|-----:|------:|
| [`EV1_BrakeSimple_Front.json`](../data/vehicle/ev1/brake/EV1_BrakeSimple_Front.json) | **1120 N·m** | 2240 | 70 % |
| [`EV1_BrakeSimple_Rear.json`](../data/vehicle/ev1/brake/EV1_BrakeSimple_Rear.json)  | **480 N·m**  | 960  | 30 % |
| **Total** | | **3200 N·m** | 100 % |

### Why 70/30

The EV1's static weight is ~**50/50** (CG at chassis x≈0.09 m, almost mid-
wheelbase of 2.512 m — the T-shaped battery pack sits low and central).  But a
~0.8 g stop transfers load **forward**, so the front axle carries far more than
half the braking load at the limit.  A 70/30 front bias matches that dynamic
load distribution, keeps the rears away from premature lock-up, and respects the
front disc's larger thermal capacity.  The total is held at 3200 N·m so the
already-calibrated peak deceleration is unchanged — Round 4 redistributes
braking, it does not add or remove it.

### Coupling to the rear EMB

The rear brake is electromechanical: `SimApp::ApplyRearEmbBrake` turns a BTCM
motor command into a shoe force, runs it through the self-energizing
[`BrakeDrum`](../src/BrakeDrum.h) model, and divides the resulting physical
torque by `SimApp::kRearBrakeMaxTorqueNm` to get the [0,1] ratio Chrono's brake
takes.  **That constant must equal the rear JSON's `Maximum Torque`** or the
applied torque is wrong, so both are pinned at **480 N·m** and the test asserts
it.  The drum peaks at ~456 N·m at speed
(μ0.38 × 4000 N × 0.10 m × (1 + α2.0)), so a 480 N·m allocation lets the EMB
nearly saturate its share without clipping.

---

## Body aerodynamics

The EV1's signature **Cd 0.19** — the lowest of any mass-produced car of its
era — was previously absent from the model (lumped into tire dissipation).
Round 4 applies it explicitly to the chassis.

[`src/Aerodynamics.h`](../src/Aerodynamics.h) is the Chrono-free source of truth
for the constants and the drag law `F = 0.5·ρ·Cd·A·v²`:

| Constant | Value | Note |
|----------|------:|------|
| Cd | 0.19 | GM-published |
| Frontal area A | 1.89 m² | ~20.3 sq ft (approximate) |
| Air density ρ | 1.225 kg/m³ | ISA sea level, 15 °C |
| ⇒ CdA | 0.359 m² | |

Reference points the unit test pins: at **100 km/h** drag ≈ **170 N**
(~4.7 kW to overcome).  `ev1sim::BuildEV1Vehicle` (src/EV1Vehicle.cpp) feeds these same constants
into Chrono's `ChChassis::SetAerodynamicDrag(Cd, area, air_density)`, which
applies the identical formula at the chassis COM each `Synchronize`.

> **Calibration follow-up.**  Because drag was implicitly baked into the tire
> rolling/slip dissipation before, the tire model may now slightly over-damp at
> speed.  Re-check top speed and coastdown on the local Chrono build and trim
> tire rolling resistance if needed — drag and rolling losses should be
> accounted separately, not double-counted.

---

## Suspension geometry: what is printed and what is estimated

Both suspension files are built on the ev1 repo's suspension baseline
(`geometry/suspension/ev1_suspension_hardpoints.csv`, ev1 commit `a6ab714`,
PR #144).  Each file carries a `//PROVENANCE` block, the baseline points it
uses with their tier and source (`//hardpoints_ev1_mm`), and the one frame map
that turns them into Chrono coordinates (`//frame`).
`tests/test_suspension_geometry.cpp` recomputes every Chrono coordinate from
that table, so the table and the numbers cannot drift apart.

**Printed in the EV1 manuals:**

| Value | Front | Rear | Printed source |
|---|---|---|---|
| Layout | short/long arm, coil-over on a yoke to the lower arm | beam axle on two leading, two trailing links and a track bar | chassis p. 251 / p. 271 |
| Track | 1470 mm (57.9 in) | 1244 mm (49.0 in) | chassis p. 62, "Exterior Dimensions" |
| Camber | −0.10° | −0.50° | chassis p. 61, "ALIGNMENT SPECIFICATIONS" |
| Toe (each wheel) | −0.05° | 0.00° | chassis p. 61 |
| Body-side points | damper top | spring seat, lower link bracket, upper link bracket | body repair manual p. 155 / p. 158 |

**Estimated:** every other hardpoint (ball joints, arm pivots, tie rods, the
yoke bolt, the rear link axle ends, dampers and track bar), the spring rates
(the baseline's 1.35 Hz front / 1.5 Hz rear ride targets) and all masses.  The
spring free lengths are derived from the chassis load, and the dampers for a
0.30 ride damping ratio.

### What the test checks

* **Every coordinate is the table through the frame map**, within 1 mm, and on
  the front every `Location` key is mapped; body COMs sit at the centroid of
  the points that hold them.
* **Layout:** the front upper arm is shorter than the lower in front view and
  the coil-over lands on the lower arm; the rear axle is one rigid body, the
  upper links lead, the lower links trail and the track bar crosses the car;
  the upper links are one spring-damper on the centreline with both links'
  rates.
* **Each spring carries the chassis at the design position:** free length =
  installed length + static load / rate, with the load from the chassis file's
  mass and COM (through the geometry's motion ratio on the front).  A mass or
  COM edit that forgets the springs fails here.
* **The `//PROVENANCE` block must still be there** and name the layout page and
  the baseline commit.
* Track equals the spindle position, the rear track is narrower than the front,
  and masses, inertias, rates and damping are positive.
