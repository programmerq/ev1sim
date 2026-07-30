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
(~4.7 kW to overcome).  `VehicleWorld::CreateEV1` feeds these same constants
into Chrono's `ChChassis::SetAerodynamicDrag(Cd, area, air_density)`, which
applies the identical formula at the chassis COM each `Synchronize`.

> **Calibration follow-up.**  Because drag was implicitly baked into the tire
> rolling/slip dissipation before, the tire model may now slightly over-damp at
> speed.  Re-check top speed and coastdown on the local Chrono build and trim
> tire rolling resistance if needed — drag and rolling losses should be
> accounted separately, not double-counted.

---

## Suspension geometry: what is measured and what is borrowed

Three numbers per axle come off the EV1.  Every coordinate comes off a Chrono
sample car.  Both suspension JSONs carry that split in a `//PROVENANCE` block at
the top, and `tests/test_suspension_geometry.cpp` checks the split rather than
describing it.

**Measured — EV1 chassis manual, pinned to the printed values:**

| Value | Front | Rear | Printed source |
|---|---|---|---|
| Track | 1470 mm (57.9 in) | 1244 mm (49.0 in) | chassis p. 62, "Exterior Dimensions" |
| Camber | −0.10° | −0.50° | chassis p. 61, "ALIGNMENT SPECIFICATIONS" |
| Toe (each wheel) | −0.05° | 0.00° | chassis p. 61 |

The track figures set the spindle half-tracks: 0.735 m front, 0.622 m rear.

**Borrowed — every hardpoint coordinate** is Chrono's stock
`sedan/suspension/Sedan_DoubleWishbone.json`, rescaled to the printed track:
lateral × 0.921168 (= 1470/1595.8) front and × 0.779546 (= 1244/1595.8) rear,
longitudinal by roughly the same, vertical fitted to the ev1model wheelwells.
Masses, inertias and spring/damper rates are the sedan's, scaled down and
rounded; the collision radii and axle inertia are the sedan's verbatim.  No EV1
suspension hardpoint is published in any manual this program holds, so there is
nothing to replace them with — the point is that they are labelled.

**Wrong on the rear axle.**  The front `DoubleWishbone` is the right layout —
chassis p. 251 describes a short/long arm independent front suspension with
upper and lower control arms and two tie rods.  The rear is not: chassis p. 271
describes a one-piece beam axle on five links (two upper leading, two lower
trailing, and a track bar).  There are no rear control arms, uprights or tie
rods on an EV1, so those hardpoints describe no part.  Replacement is deferred —
see [TODO.md](TODO.md).

### What the test checks

* **Track, camber and toe** are pinned to the printed page-62 and page-61
  values, so an edit that reverts them fails.
* **Every lateral hardpoint** must equal the sedan sample's value times the
  scale the file declares in `//hardpoint_scale_y`, within 4 mm.  This is the
  check that catches a partial rescale: an earlier commit moved the declared
  front track by bumping four outboard Y coordinates 13 mm and leaving ten,
  which left the spindle claiming one track and the arms encoding another.
* **The `//PROVENANCE` block must still be there** and still name the sedan
  sample.  Strip it and the coordinates read as EV1 measurements again.
* **Masses & inertias** — every spindle/upright/control-arm mass and every
  principal moment of inertia is asserted strictly positive (catches a dropped
  or zeroed field).
* **Springs & dampers** — spring rate 143 000 N/m and free length (0.48 m front
  / 0.42 m rear), damping 7500 N·s/m front / 7000 rear: all asserted positive.
  These give a firm-but-plausible ride frequency for a low, ~1281 kg car; they
  are tuned, not sourced.
* **Control-arm orientation** — each arm's outboard (upright) hardpoint sits
  further from centerline than both its inboard (chassis) mounts.  A sign flip
  here would mean an arm pointing the wrong way; the test guards against it on
  both the upper and lower arms, both axles.
