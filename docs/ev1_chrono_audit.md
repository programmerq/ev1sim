# EV1 Chrono vehicle audit

Cross-reference of every parameter in `data/vehicle/ev1/**/*.json` against
known GM EV1 specifications.  Targets **Generation 2 NiMH (1999)** as the
canonical config — it's the more polished production variant most folks
remember, and the lighter mass + extended range numbers are what the
public datasheets cite.

Symbol legend in the tables:
- ✅ matches real EV1 closely
- ⚠️  approximation — close enough for now, refine with new data
- ❌ wrong — should be fixed
- 🔧 fixed in the audit batch (commit log has the change)

Sources for the real-EV1 numbers in this document:
- **EV1 service manuals, transcribed in the `ev1-manual-redux` repo** — the
  primary source, and the one section 7 should have used from the start.  Cite
  the printed page (e.g. "chassis p. 62"), not "known specifications."
- GM EV1 product brochure (1999), reproduced widely (e.g. ev1.org archives)
- Wikipedia GM EV1 article (cross-checked figures)
- "Who Killed the Electric Car?" (Paine 2006) — production / range numbers
- General automotive engineering knowledge for the
  derivative-but-undocumented values

The audit was occasioned by the cruise-control demo run — steady-state
cruise hold dropped to ~11.6 m/s when commanding 13.15 m/s.  That bug is
likely in the plant model (this audit), not the controller, so we hold
off cruise gain tuning until these fixes land.

## 1. Vehicle dimensions

| Parameter | Current | Real EV1 | Status |
|---|---|---|---|
| Wheelbase | 2.512 m | 2.514 m (99.2 in) | ✅ within 2 mm |
| Front axle x | +1.346 m (suspension loc) | n/a (derived) | ✅ |
| Rear axle x  | −1.166 m (suspension loc) | n/a (derived) | ✅ |
| Min turning radius | 10.8 m | 11.0 m (curb-to-curb) | ⚠️  close |
| Max steering angle | 13.5° (vehicle); 10° (pinion) | ~13° (matches 11 m TR) | ✅ |
| Length / width / height | n/a (visual mesh) | 4310 / 1765 / 1281 mm | n/a |
| Drag coefficient | n/a (Chrono uses tire/rolling losses, no aero) | 0.19 | ❌ no aero model |
| Frontal area | n/a | 1.89 m² | ❌ no aero model |

**Note on aero**: Chrono's WheeledVehicle doesn't model body aero by default.
The EV1's signature 0.19 Cd should manifest as a `chrono::ChAerodynamicLoad`
or similar applied to the chassis body — currently absent.  Without it,
high-speed deceleration is artificially low.

## 2. Mass and inertia

| Parameter | Current | Real EV1 (NiMH) | Status |
|---|---|---|---|
| Chassis mass | 1217 kg | ~1199 kg (target curb 1281, less ~82 kg accessories) | 🔧 1217→1199 |
| Curb weight (computed) | ~1299 kg | 1281 kg (NiMH) | ⚠️ off by 18 kg |
| Centroidal Frame x | +0.09 m | unknown — likely +0.1 to +0.15 (slight front bias from motor) | ⚠️  |
| Centroidal Frame z | 0.49 m | likely 0.40-0.45 m (T-pack batteries low in floor) | ⚠️ slightly high |
| Ixx (roll) | 130 kg·m² | rg≈0.33 m → plausible | ⚠️ |
| Iyy (pitch) | 600 kg·m² | rg≈0.70 m → plausible | ⚠️ |
| Izz (yaw)  | 680 kg·m² | rg≈0.75 m → plausible | ⚠️ |

**Curb-weight breakdown (Chrono adds these to chassis)**:
- 4× wheel × 8.0 kg = 32 kg
- 4× tire × 7.5 kg = 30 kg
- 4× suspension (spindle 1.0 + upright 1.2 + UCA 0.9 + LCA 1.4) = ~18 kg
- Steering link = 1.6 kg
- Total non-chassis = ~82 kg
- Target chassis = 1281 − 82 = **1199 kg** for Gen 2 NiMH

For PbA Gen 1 (1351 kg curb), chassis would be 1269 kg — choose NiMH as
the canonical config since the demo scenarios are tuned around the more
modern variant.

## 3. Powertrain — motor

| Parameter | Current | Real EV1 (Gen 2 AC induction) | Status |
|---|---|---|---|
| Peak torque | **141 N·m at 7000 RPM**, held flat below it | **141 N·m at 7000 RPM** (prop p250) | ✅ the rating is sourced; the flat plateau below it is ⚠️ @inferred |
| Peak power | **103.4 kW at 7000 RPM** | **103 kW (138 Hp) at 7000 RPM** (prop p250) | ✅ sourced — the map's value is the stated torque × the stated speed |
| Drive speed ceiling | **16000 RPM** | propulsion disabled above **16000 RPM** (prop p328, DTC 007) | ✅ sourced — but see the labelling note |
| Map shape, 0-16000 RPM | flat torque to 7000, constant power beyond | textbook 3-phase induction | ✅ corner sourced; the shape either side of it is @inferred |
| Map shape, 13000-16000 RPM | constant power to 15500, ramp to zero at 16000 | not stated anywhere | ⚠️ @inferred — continuation of the region below, plus a chosen ramp width |
| Coast torque (zero throttle) | **-5 to -12.5 N·m**, constant power above 8350 RPM, zero at 16000 | zero-pedal negative torque *is* commanded regen (prop p57), bounded at 365 V × 30 A = **10 950 W** (prop p60, p209) | ✅ bounded by the print since 2026-08-12 — see §3.1; the sub-8000 RPM ramp is still ⚠️ @inferred |
| Motor type | "EngineSimpleMap" (Chrono ICE template) | 3-phase AC induction | ⚠️ template mismatch — works as a static map |

**The corner point (2026-08-11).**  The first three rows of that table used to
read 150 N·m flat to 6500 RPM on a ~102 kW envelope, from secondary web
figures, with the primary document disagreeing in the next column.  It now
reads off the print.

@source:manual propulsion p250, "DRIVE MOTOR PERFORMANCE": *"The drive motor has
a peak torque rating 141 N·m (104 ft−lb) at 7000 RPM.  The peak power rating is
103 kw (138 Hp) at 7000 RPM."*  Read off the `.jpg` scan; the page's own folio
reads `250 PROPULSION`.

Those two sentences are one operating point, not two competing ones, and the
page is arithmetically self-consistent about it:

| printed | check |
|---|---|
| 141 N·m at 7000 RPM | 141 × 7000 × 2π/60 = **103.36 kW** |
| 103 kW at 7000 RPM | 103 kW ÷ 745.7 = **138.1 hp** ✓ printed 138 Hp |
| 104 ft−lb | 141 ÷ 1.35582 = **104.0 ft−lb** ✓ |

Peak torque and peak power coinciding at one speed *is* the corner of an
induction drive's envelope — torque-limited below, power-limited above.  The
map now carries 141.0 N·m at every full-throttle point at or below 7000 RPM and
141 × 7000 ÷ RPM above it.  Peak torque falls 6.0 %; base speed rises 7.7 %.

**Exactly one full-throttle point in that table is sourced: 7000 RPM, 141.0
N·m.**  p250 prints a *rating*, not a curve — it says nothing about torque at
1000 or 5000 RPM.  What the coincidence of the two ratings buys is the *shape*
to anchor on that point, and the shape is the textbook one: a real induction
machine rolls off near zero speed against its inverter current limit, and its
field-weakening region is not an exact hyperbola.  Neither is modelled, and
`EV1_EngineSimpleMap.json` labels the plateau and the hyperbola `@inferred`
accordingly.

What that did to the drive, by road speed (10.946:1, 0.2915 m tyre):

| shaft RPM | ≈ mph | torque before | torque after | Δ |
|---|---|---|---|---|
| 0–6500 | 0–40.5 | 150.0 | 141.0 | **−6.0 %** |
| 7000 | 43.7 | 143.0 | 141.0 | −1.4 % |
| 7500–15500 | 46.8–96.7 | 130.0 → 62.8 | 131.6 → 63.7 | **+1.3 to +1.4 %** |

The envelope *gains* torque above ~47 mph, because the constant-power region
now holds the printed 103.4 kW instead of the old ~102 kW.  The old map's 7000
RPM point was also 104.8 kW — 2.8 % above its own envelope, the rounding
artefact of a corner sitting at 6500 — so the pre-change map's true peak power
was neither of the two numbers anyone had written down.

Measured, full throttle on dry asphalt (µ 0.90), from `abs_high_mu_stop`, no
BTCM attached:

| | before | after | Δ |
|---|---|---|---|
| 0–30 mph | 6.768 s | 7.083 s | +0.315 s (+4.7 %) |
| 0–60 mph | 12.676 s | 13.062 s | +0.386 s (+3.0 %) |
| 0–67.1 mph (30 m/s) | 14.664 s | 15.022 s | +0.358 s (+2.4 %) |

The percentage shrinks with speed because the loss is confined to the
constant-torque region; above ~47 mph the car is pulling slightly *harder* than
before.

Two things that do **not** move, measured rather than assumed:

* **Coastdown.**  The coast map was not edited *by this corner-point change* and
  `coastdown` still releases at 30 m/s, so speed as a function of
  time-since-release is the same curve: worst |Δv| = **0.0028 m/s over 74 s of
  coast** (0.010 % at 27.8 m/s), which is the 0.8 mm/s difference in speed at
  the instant the barrier released, propagated.  Only the time to *reach*
  release speed moves, by the same ~0.36 s.
  **Superseded 2026-08-12 as to the coast map:** §3.1 did edit it, above
  8350 RPM, and the clause that used to close this bullet — "the drag
  calibration in section 11 is unaffected" — no longer holds.  §11.1 has the
  measured effect (CdA 0.880 → 0.571 m²).  Everything above remains true of the
  p250 corner-point change this section is about.
* **The drive ceiling.**  `wheelspin_ceiling_probe` on dry asphalt, 60 s of full
  throttle: terminal speed **43.423 → 43.428 m/s (97.13 → 97.14 mph)**.  Up
  there the envelope is on the shared 15 500 → 16 000 RPM ramp to zero, so where
  it crosses road load barely moves.

**The speed ceiling (2026-08-11).**  The map used to end at 13 000 RPM, and this
table used to call that "✅".  It was neither correct nor a ceiling.

It was not correct because 13 000 RPM is not a property of the motor — it is the
~80 mph **software** top-speed calibration restated in RPM (80 mph ÷ 0.2915 m
tire × 10.946 = 12 824 RPM).  @source:manual propulsion p250: "The PIM will
limit vehicle speed in the forward direction to 129 km/h (80 mph)", enforced by
decreasing the torque current.  That is the propulsion controller's calibration,
not the drive's capability, and encoding it as physics made a software choice
unmodifiable.  The drive's actual ceiling is **16 000 RPM** — @source:manual
propulsion p328 (DTC 007): a speed/direction pulse rate above 34 000 Hz
"corresponds to a shaft speed of 16,000 RPM", at which "the SERVICE NOW telltale
is illuminated, the DTC is stored and propulsion is disabled".  That is ~100 mph
at the road: above the software cap, not significantly beyond it.

It was not a ceiling because Chrono's clamp on `Maximal Engine Speed RPM` bounds
the torque **lookup**, not the shaft (`ChEngineSimpleMap.cpp:39`), and
`ChFunctionInterp` holds the endpoint value outside the table
(`ChFunctionInterp.cpp:49-52`, extrapolation off by default).  So the map's last
torque was delivered at every higher speed forever: 74.9 N·m at any RPM, which
at the 45 500 RPM implied by the reported observation is **357 kW from a 103 kW
drive**.  An unloaded wheel therefore accelerated without bound.  Reproduced on
ice at full throttle: at 40 s the shaft was at **48 145 RPM** (front wheels
451.6 and 469.6 rad/s — the faster one making **306 mph** of tread speed, 378 kW
delivered) and still climbing linearly, with the car itself doing 34.7 mph.

Wheel speeds are quoted per wheel and shaft speed from the **average** of the
two, because the differential is open and the wheels straddle the shaft: taking
either wheel alone misstates the shaft by several percent in each direction.
`scripts/motor_ceiling_report.py` does that conversion and prints both.

The fix extends the map to 16 000 RPM along the same constant-power hyperbola
and pins the last point of **both** maps at **0 N·m**, so the endpoint hold
delivers nothing above the ceiling at any pedal position.  Every pre-existing
map point is preserved verbatim.  Guarded by
[`tests/test_motor_speed_ceiling.cpp`](../tests/test_motor_speed_ceiling.cpp).

**What "16 000 RPM" is, precisely.**  The DTC is titled *"VEHICLE SPEED INPUT
PULSE RATE TOO HIGH"*, the threshold is a **pulse rate** (34 000 Hz), and the
reaction is a **PCM software action**.  16 000 RPM is GM restating that rate as
shaft speed.  The manual states no rotor-burst speed, no bearing speed and no
inverter frequency limit anywhere, so this is **not** a mechanical redline and
nothing here calls it one — it is the speed above which the propulsion system
stops producing torque, which is exactly what a torque map can represent.
Note also that the same p250 says the PIM decreases torque current *"to limit
drive motor shaft speed"*: shaft-speed limiting is a PIM function on the real
car.  This map is not that limiter; it is the envelope the limiter acts inside.

**What moves, measured — not "nothing".**  Preserving the points means
trajectories are identical *while the shaft is below 13 000 RPM*, which is
**78.7 mph** — not "in normal driving".  Because ev1sim has no top-speed
limiter at all, any run holding full throttle long enough crosses it.  Measured,
full throttle held 120 s on dry asphalt (µ 0.90):

| | before | after |
|---|---|---|
| speed at 120 s | 103.46 m/s = **231.4 mph** | 43.42 m/s = **97.1 mph** |
| shaft at 120 s | 38 760 RPM, still accelerating | 15 880 RPM, settled |
| first divergence | — | t = 17.15 s, at **78.7 mph** / 13 064 RPM |

A 231 mph EV1 was the defect, so the direction is right — but it is a change,
and the scenarios were enumerated rather than assumed.  All ten in
`config/scenarios/` are bounded below 78.7 mph by their own throttle-release
events: the seven ABS scenarios plus `cruise_demo_electronics` and `coastdown`
release at or below 30 m/s (67.1 mph, 10 760 RPM), and `accel_brake_local`
holds 0.7 throttle for 7.5 s and never approaches it.

**Wheel speed is bounded only through the differential.**  The shaft is capped,
but the open differential still lets one front wheel run above the mean while
the other runs below.  On µ 0.02 a single wheel reaches ~204 rad/s (59 m/s of
tread) with the shaft pinned at 15 881 RPM.  That is what an open diff does, and
it is a far cry from the unbounded 451 rad/s — but "the wheel cannot spin fast"
would be the wrong claim; the correct one is that the *drive* can no longer feed
it.

**Why nothing else stopped it.**  The runaway needed only the missing ceiling,
but it is worth recording what else was checked and found not to be the cause:

* **No current or power limit exists in the torque path.**  ev1sim's plant has
  no inverter current limit, no pack power limit and no torque-command clamp —
  the engine map is a static torque lookup with nothing downstream of it.
  Limits *do* exist in the sibling PIM (`ev1/pim/pim_drive_plant.h`: a 400 A bus
  ceiling `@source:manual p.56`, a 150 N·m torque clamp, and a 13 000 RPM rail),
  but `pim_drive_plant` is a **controller-side plant that integrates its own
  motor speed and never feeds Chrono**.  It is a parallel model, not a limiter
  in this torque path, so its limits could not have bound the wheel even had
  the controller been running.
* **The controller was not running.**  `config/scenarios/abs_low_mu_stop.json`
  sets `driver_mode: "local"`, and `SimApp::ApplyElectronicsThrottle` returns
  immediately unless the mode is `"electronics"`, so PIM's throttle path — and
  its traction-control derate — was bypassed entirely.
* **The tyre is not a contributor.**  TMeasy behaves correctly: at µ 0.10 with
  ~4 kN on the wheel it resists roughly 0.10 × 4 kN × 0.2915 m ≈ 117 N·m, while
  the held 74.9 N·m through the 10.946:1 reduction delivers roughly 410 N·m
  *per front wheel*.  The tyre saturates at the friction limit, which is what it
  should do; it simply cannot remove drive torque.  Nothing in the slip-force
  computation keeps torque applied — the torque was applied by the engine map.

**So: which speed does the limiter watch — motor or road?**  In the model, the
question does not arise, because **there is no limiter of any kind in the torque
path.**  Stating that as the three-way answer it is:
* *No limiter at all* — **true**, for the plant that actually integrates the
  wheel.
* *A limiter that watches road speed, which wheelspin escapes* — **does not
  apply.**  The only road-speed-referenced limits in the program are PIM's
  cruise ceilings (`PIM_CRUISE_MAX_SETPOINT_MPS` 35.76 m/s and
  `PIM_CRUISE_OVERSPEED_MPS` 38.0 m/s), which act only in cruise and are not a
  top-speed limiter.
* *A limiter that watches motor speed but is bypassed or mis-scaled* — **does
  not apply either.**  The motor-speed-referenced limits that exist (PIM's
  DTC 007 at 16 000 RPM, the drive-plant's 13 000 RPM rail) sit outside the
  torque path — and DTC 007 additionally could not fire, because ev1sim was
  publishing the *clamped* speed on 4070.

On the **real car** the answer is motor shaft speed, and it is sourced: the
speed/direction sensor is "attached to the end of the rotor shaft" and the PCM
calculates shaft speed from it (prop p326); the PCM "uses and monitors the motor
shaft speed signal … including vehicle speed signal generation, torque control,
and cruise control" (prop p336).  True road speed lives in the BTCM and reaches
the PCM as a torque *retard request*, not a speed feed; wheel speed arrives only
as a dead-sensor plausibility check (DTC 078).  This matters for the future
PIM-side limiter: a motor-referenced limiter reads the **spinning** wheels
during wheelspin, so it engages sooner rather than being escaped.

**Corner-point disagreement (closed 2026-08-11).**  The map's 150 N·m /
6500 RPM corner came from secondary web figures and has been replaced by the
141 N·m / 7000 RPM / 103 kW corner the primary service manual states on
propulsion p250 — see "The corner point" in section 3 above for the citation,
the arithmetic that shows the page is self-consistent, and the measured
before/after.  It is a ~6 % peak-torque change and a ~7.7 % base-speed change,
so unlike the ceiling fix it does move every acceleration trajectory, and it
landed before the next VAT baseline recapture rather than after.

### 3.1 The coast map against the printed regeneration limit (2026-08-12)

PR #54 filed an unresolved exceedance: the coast map reached ~34 kW where the
manual bounds regeneration at ~11 kW.  It declined to act on it, on the
grounds that the map was mechanical drag and "mechanical drag is not bounded
by an electrical limit."  That escape hatch does not survive reading p57.

**What the print says.** Quoted from the `.jpg` scans, not the OCR sidecars.
The p57 sidecar drops three fragments: "With the PRND in D (drive) and" and
"*like an automatic transmission*" from the coast-freely sentence, and
"*accelerator position*," from the list of PCM inputs.  Those matter for
precision — the PRND-in-D qualifier is what says *when* coast down applies, and
accelerator position is one of the three inputs the curve is a function of.

**They do not change this section's conclusion, and an earlier draft of this
paragraph claimed they did.**  The load-bearing sentence — "The coast down
feature uses a calibrated amount of regenerative braking to gradually slow down
the vehicle" — is present verbatim in the sidecar.  §3.1 would have survived
reading the sidecar alone.  The `.jpg` gives the accurate transcription and is
where the quotes below come from; it did not rescue the argument, and saying it
did was the same hedge-loses-its-qualifier failure the drops themselves are an
example of.  The pages:

- **p57, "COAST DOWN FUNCTION"** (folio `PROPULSION 57`): "With the PRND in D
  (drive) and the accelerator pedal released, the EV1 will coast freely like an
  automatic transmission in neutral.  The coast down feature uses a calibrated
  amount of regenerative braking to gradually slow down the vehicle. ... The
  PCM controls coast down by providing a negative torque current as a function
  of drive motor shaft speed/direction sensor rate, accelerator position, and
  coast down drag."
- **p60, "REGENERATION"** (folio `60 PROPULSION`) and **p209, "REGENERATIVE
  BRAKING"** (folio `PROPULSION 209`), the same sentence verbatim on both:
  "The maximum allowed regeneration is 365 volts DC and 30 amps DC."

p57 is what makes the limit apply.  Negative torque commanded as a function of
shaft speed with the pedal released is exactly a zero-throttle torque-vs-RPM
table, so the slot holds **commanded regen** — not the bearing friction and
windage the JSON header had claimed since it was written.  Regen is bounded, so
the map is bounded.

Being precise about how far p57 goes: it describes **two** states, a base one
where the car "will coast freely like an automatic transmission in neutral",
and the coast-down feature that adds calibrated regen on top — and p204 shows
coast down is a driver-selectable button, so both are real operating modes.
p57 does not say which one a single-curve model should represent.  This map is
the **coast-down-enabled** case, which is a modelling choice, recorded again in
the conditions table below.  What p57 does settle is that the enabled case is
commanded regen rather than mechanical drag, and that is the claim §3.1 needs:
the curve was sitting at 5.5× a limit that its own header had argued did not
apply to it.

One consequence of the re-labelling that the model does not distinguish:
Chrono blends the two maps by throttle, so this curve also contributes at
*partial* throttle, where the real PCM would be summing pedal torque against a
retard request rather than applying a coast-down calibration.  Unmodelled, and
unchanged by this correction.

**What the number means, precisely.**  365 × 30 = 10 950 W, and it is
**pack-side DC**: both pages place the sentence immediately after describing
the motor and PIM acting "as a generator feeding current back to the battery
pack for storage."  The map is **shaft** torque, on the other side of the
inverter.

Capping shaft power at the printed pack number is **sufficient** to guarantee
the print is never violated: pack power is at most the electromagnetic part of
shaft power times an efficiency η ≤ 1, and both are at most total shaft power,
so shaft ≤ 10 950 W ⟹ pack ≤ 10 950 W whatever η is, with no efficiency
invented.

It is **not necessary**, and an earlier draft of this section called it "the
one choice that cannot violate the print" and "the tightest reading the page
supports", which overstates it in two ways.  The print bounds only what reaches
the pack.  This map is *total* zero-pedal shaft torque, and its mechanical
component — bearings, windage — never reaches the pack at all, so the page on
its own permits total shaft power above 10 950 W by however much mechanical
drag contributes.  Splitting the two needs a drag curve the manual does not
print, so the cap here binds the sum.

Note which way that errs.  Because the cap binds the sum, the model gives the
car *less* coast drag than a real one carrying both regen and mechanical
losses; §11.1 measures exactly that — total drag force fell and the car coasts
further.  So this is conservative about **regen**, not about drag, and it is
the reason the correction makes the sim coast further rather than less far.

It is a ceiling, not a continuous rating: no duty cycle, no time limit and no
temperature condition is printed with it, and p60 uses the same "maximum"
idiom for the APM (22 amps) and PSCM (five amps) feeds.  No page states a peak
above it.

**Conditions recorded rather than clamped.**  The map does not represent
everything the pages condition the limit on:

| condition | page | modelled? |
|---|---|---|
| Coast down is a driver-selectable button; default with RSA data lost is "Coast down request = No Coast Down" | p204, DTC 028 | no — the map is the coast-down-**enabled** case |
| Regen amount is a function of PSOC; inhibited outright at PSOC 100 % | p209 | no — no pack model, so this is the **maximum**-regen case, which is what the ceiling bounds |
| Regen disabled below ~10 km/h (6 mph) | p209 | not as a cutoff; the −5 N·m breakpoint happens to sit at 6.2 mph |
| Shaft-to-pack efficiency η | — | not stated anywhere; see above |
| Mechanical drag, which genuinely is *not* electrically bounded | — | not separated out; the cap binds the sum, which is the conservative order |

**What changed.**  Nothing at or below 8000 RPM (49.9 mph).  The old curve
crossed the ceiling at 8349 RPM (52.1 mph) and then ran away; above a knee at
8350 RPM the curve is now constant power at the printed 10 950 W.

An earlier draft of this section, and of the JSON header, said that left "the
whole region the 2026-04-30 coastdown calibration fitted" untouched.  **That is
false.**  Coastdown runs 30 → 10 m/s, i.e. **3586–10 758 RPM**, so 34 % of its
fitted span sits above the knee and moved.  §11.1 measures the consequence —
the fit's CdA moves 8× — instead of asserting there wasn't one.  What is true
is the narrower claim: no *breakpoint* at or below 8000 RPM changed value.

| RPM | mph | old N·m | old kW | new N·m | new kW |
|---|---|---|---|---|---|
| 8000 | 49.9 | −12.0 | 10.1 | −12.0 | 10.1 |
| 8350 | 52.1 | −12.5 | 10.9 | −12.5 | 10.9 |
| 10000 | 62.4 | −15.0 | **15.7** | −10.4 | 10.9 |
| 12000 | 74.9 | −20.0 | **25.1** | −8.7 | 10.9 |
| 13000 | 81.1 | −25.0 | **34.0** | −8.0 | 10.9 |
| 15500 | 96.7 | −37.0 | **60.1** | −6.7 | 10.9 |

The old peak was 5.5× the printed ceiling, from a drive whose peak *motoring*
power is 103 kW.  Torque now falls with speed above the knee, which is the
correct shape for a regen-limited drive — the old monotonic ramp was wrong in
shape as well as in magnitude.

Values are rounded to 0.1 N·m **toward zero**, and chosen as the largest 0.1
N·m step at each breakpoint that keeps the *piecewise-linear* map under the
ceiling.  1/ω is convex, so the chord Chrono actually evaluates between two
exact constant-power points sits above the curve: a breakpoint-only check
passes a map that exceeds the cap by 9 W at 11 437 RPM, which is why 11000
reads −9.4 and not the exact −9.5.  Dense max over the interpolant: 10 947 W.

`tests/test_motor_speed_ceiling.cpp` pins this with the two printed numbers
(365 V, 30 A) rather than a derived wattage, sweeps the interpolant at 10 RPM,
and separately requires the curve to *track* the ceiling above the knee —
without that second half an all-zero coast map would pass perfectly.  (10 RPM,
not 1: the chord excess spans a whole 1000 RPM segment so it cannot hide
between samples, and the lookup helper carries a `REQUIRE` per call, which at
1 RPM put 16 000 assertions into the suite total from this one case.)

What's also still missing:
- Regen integrated with brake commands (today regen lives only in coast
  torque, which fires whenever throttle is released regardless of brake)
- Speed-dependent peak — induction motors have a more complex torque
  envelope at low speeds (limited by inverter current) than the flat 141 N·m
  plateau suggests.  p250 states the *rating*, not the curve below it, so this
  stays @inferred whatever the plateau's height.  Probably negligible for our
  use cases.
- The ~80 mph software cap itself.  ev1sim has no top-speed limiter at all;
  the cap belongs on the propulsion-controller side (electricsim PIM), which
  is where the manual puts it.

## 4. Drivetrain

| Parameter | Current | Real EV1 | Status |
|---|---|---|---|
| Transmission type | single forward gear, ratio 1.0 | single-speed reduction | ✅ |
| Final drive (Conical Gear) | 0.1 → **10.0:1 reduction** | **10.946:1 reduction** | 🔧 0.1 → 0.0913 |
| Driveline | FWD (`ShaftsDriveline2WD`) | FWD | ✅ |
| Driveshaft inertia | 0.5 kg·m² | unknown — typical | ⚠️  |
| Differential box inertia | 0.6 kg·m² | unknown — typical | ⚠️  |
| Differential lock limit | 100 N·m | open differential, no LSD on EV1 | ✅ (open at this limit) |

The 10.0 vs 10.946 final drive change is small (~9% off) and shifts the
operating RPM at a given speed slightly.  At 13 m/s with current ratio:
motor RPM = 4250.  After fix: 4651 RPM — still well within flat-torque
region, so peak-torque behavior is unchanged.

## 5. Tires

| Parameter | Current | Real EV1 (P175/65R14) | Status |
|---|---|---|---|
| Unloaded radius | 0.2915 m | 0.292 m (computed from spec) | ✅ |
| Width | 0.175 m | 0.175 m (175 mm) | ✅ |
| Rim radius | 0.1778 m | 0.1778 m (14"/2) | ✅ |
| Mass | 7.5 kg | typical | ✅ |
| Coefficient of friction | 0.8 | typical dry asphalt | ✅ |
| Rolling resistance | 0.008 | EV1 spec'd 0.006-0.008 (Michelin Proxima LRR) | ✅ |
| Load index | 82 (475 kg/tire) | 95 (Q-rated) | ⚠️  probably under-spec'd |
| Tire model | TMeasy | n/a | ✅ standard choice |

Tires are the best-modeled part of the vehicle.  The wide margin on
bearing capacity (4750 N vs ~3140 N actual peak load per tire) is
conservative; not worth touching.

### 5.1 Standstill wheel-spin limit cycle → integration step (2026-07-08)

Surfaced by electricsim's VAT `wheel_speed_*.svg` plots: a stopped, unbraked,
undriven wheel reported ±4–5 rad/s of `wheel_omega` (physically impossible),
and the launch phase carried the same near-Nyquist wobble. Root cause is a
**numerical instability of the explicit multibody integration at the 2 ms
step**, not the tire model:

- Below 1 m/s the TMeasy force switches to the Coulomb path (`frblend→0`) and
  the near-zero slip gain `sx = −vsx/(R·|ω| + m_vnum)` (`m_vnum = 0.01`) is very
  high; the low-inertia wheel-spin DOF it drives is stiff, and the 2 ms explicit
  step over/undershoots each tick → a limit cycle.
- Measured standalone (a free wheel at rest, `wheel_omega_rl` rms over t ≥ 1 s):
  step 2 ms → 2.88; **1 ms → 0.094 (31×)**; 0.5 ms → 0.099; 0.25 ms → 0.10 — so
  1 ms is already stable and finer only plateaus.
- `dfx0` is irrelevant here (×0.5 / 0.25 / 0.1 all gave 2.88): the standstill
  force is Coulomb, not the slip curve. Sub-stepping the *tire* is also a no-op —
  this Chrono TMeasy is steady-state (algebraic `Advance()`).

Fix: `simulation.step_size_s` 2 ms → **1 ms** (all configs + the `Config.h`
default). `steps_per_tick = round(tick_dt / step)` auto-doubles, so the
`1/render_fps` co-sim exchange cadence and the stats cadence are unchanged —
only the physics integration is finer. Deterministic (bit-identical run-to-run
preserved); VAT baselines recaptured because every physics value shifts
slightly. Tracked as electricsim `BL-0181`.

## 6. Brakes

| Parameter | Current | Real EV1 | Status |
|---|---|---|---|
| Per-wheel max torque | 1800 N·m | ~700-800 N·m front, ~400-500 rear | 🔧 1800 → 800 |
| Front brake type | "BrakeSimple" | ventilated disc (256 mm) | ⚠️ ok abstraction |
| Rear brake type | "BrakeSimple" (same as front) | drum (200 mm) — much weaker | ⚠️ |

**Math behind the fix**: 1800 N·m × 4 wheels = 7200 N·m total.  At 0.2915 m
tire radius that's 24,700 N braking force = 1.97g of deceleration —
which exceeds the tire's friction-limited deceleration (~0.8g on dry
asphalt).  In practice the wheels lock and ABS modulates, so the
visible deceleration is friction-limited regardless.  But the
unrealistically high commanded torque means even a 30% brake pedal
saturates wheel friction — there's no usable middle range.

Setting per-wheel torque to 800 N·m gives 3200 N·m total ≈ 11,000 N
≈ 0.88g, which still saturates friction at full brake but leaves a
realistic gradient at partial pedal.  Future improvement: split front
800 / rear 500 to match the EV1's disc/drum bias.

## 7. Suspension geometry

> **Corrected 2026-07-29.**  This section previously fixed the track widths to
> 1.496 m / 1.281 m and called the front template wrong.  Both were mistakes;
> the table below is re-sourced against the chassis manual and the correction
> is written up under the table.

| Parameter | In the JSON | Printed source | Status |
|---|---|---|---|
| Front template | `DoubleWishbone` | short/long arm (SLA), chassis p. 251 | ✅ right layout |
| Rear template | `DoubleWishbone` | beam axle on five links, chassis p. 271 | ❌ wrong layout |
| Front track | 1.470 m (spindle Y 0.735) | 1470 mm (57.9 in), chassis p. 62 | ✅ |
| Rear track | 1.244 m (spindle Y 0.622) | 1244 mm (49.0 in), chassis p. 62 | ✅ |
| Front camber | -0.10° | -0.10° nominal, chassis p. 61 | ✅ |
| Rear camber | -0.50° | -0.50° nominal, chassis p. 61 | ✅ |
| Front toe | -0.05° | -0.05° per wheel, chassis p. 61 | ✅ |
| Rear toe | 0° | 0.00° per wheel, chassis p. 61 | ✅ |
| All hardpoint coordinates | rescaled Chrono `sedan` sample | nothing published | ⚠️ template geometry, labelled as such in the files |
| Spring rate (both axles) | 143000 N/m | nothing published | ⚠️ |
| Damping (front / rear) | 7500 / 7000 N·s/m | nothing published | ⚠️ |

### What the printed source says, and what this audit got wrong

**Track.**  EV1 chassis manual, printed page 62, "Exterior Dimensions": *Track
Front 1470 mm (57.9 in)*, *Track Rear 1244 mm (49.0 in)*.  GM's own EV1 product
site gives the same pair.  The 1.496 m / 1.281 m figures this audit introduced
have no source: **1281 mm is the Overall Height from the same table, two rows
above Track Rear.**  The files shipped with 1470/1244, this audit replaced them,
and a later regression test pinned the replacements — which is what made a
transcription slip look like a deliberate choice.  Restored to the printed
values, with the test now pinning those.

**Front layout.**  Chassis manual page 251: *"The short/long arm (SLA)
independent front suspension used on the EV1 … a steering knuckle, wheel hub
assembly, coil over shock assembly, upper and lower ball joint and upper and
lower control arm … steered by two front tie rods."*  SLA is a double wishbone,
so the front template is the right kinematic layout.  This audit's "MacPherson
strut" was wrong.  Unmodeled on that axle: the coil-over-shock yoke linkage and
the stabilizer bar.

**Rear layout.**  Chassis manual page 271: *"a rear axle assembly with aluminum
end castings and a tubular center section … two coil springs, two twin tube
shock absorbers and five connecting links … two upper suspension leading links,
two lower suspension trailing links, and a track bar."*  A one-piece beam axle
on five links — not a twist beam, and not independent.  The `DoubleWishbone`
template models nothing that exists back there: no control arms, no uprights, no
tie rods, and a track bar (Panhard rod) with no counterpart in the template.
Chrono's JSON factory does offer closer templates — `SolidAxle` carries the same
link set but is a steered axle, and `GenericWheeledSuspension` could express the
layout exactly — but neither can be filled in without hardpoints, so the file
stays a stand-in and says so in its own header.

**Hardpoints.**  Every coordinate in both files is Chrono's stock
`sedan/suspension/Sedan_DoubleWishbone.json` rescaled: lateral coordinates by
1470/1595.8 = 0.921168 front and 1244/1595.8 = 0.779546 rear, longitudinal by
about the same, vertical fitted to the ev1model wheelwells.  No EV1 suspension
hardpoint is published anywhere this program holds — a search of both service
manuals and all four parts-catalog sections produced a wrench size, a tap size
and hub runout limits.  So there is nothing to replace them with, and the
deliverable is that they are labelled instead of implied.
`tests/test_suspension_geometry.cpp` checks each lateral coordinate against the
sedan value times the scale the file declares, so a future track edit either
moves all of them or fails.

**Alignment.**  Camber and toe are the one part of these files that is measured.
This audit validated them against "typical" ranges; page 61 prints all four
nominals and all four match.  They are now pinned to the printed values.

## 8. Steering

| Parameter | Current | Real EV1 | Status |
|---|---|---|---|
| Template | RackPinion | rack-and-pinion w/ electric assist | ✅ |
| Pinion radius | 0.3 m | n/a (Chrono effective parameter) | ⚠️ |
| Max pinion angle | 10° | n/a | ⚠️ |
| Steering link length | 0.46 m | unknown | ⚠️ |
| Rack-to-wheel ratio (effective) | 1.35:1 | unknown | ⚠️ |

Steering geometry is consistent end-to-end (13.5° max wheel angle ≈ 11 m
turning radius given the 2.512 m wheelbase).  Internal pinion parameters
are tuning surface, not physical specs.  Leave alone.

## 9. Aerodynamics

**Not modeled.**  See dimensions table.  Adding aero would need a custom
`ChAerodynamicLoad` or chassis-level drag force computation
(`F_drag = 0.5 ρ v² Cd A`).

At 13 m/s (cruise speed in the demo):
- Drag force = 0.5 × 1.225 × 169 × 0.19 × 1.89 = **37 N**
- That's 481 W of drag power at 13 m/s
- For comparison, rolling resistance ≈ 100 N at 13 m/s ≈ 1.3 kW

So drag is a **smaller** loss than rolling resistance at urban speeds.
This explains why the cruise-control demo's steady-state error doesn't
trace back to missing aero — the dominant losses are tire-side, which
TMeasy already models.

## 10. What's missing (open questions)

These probably explain the cruise-hold steady-state error:

1. **Drivetrain efficiency / friction not parameterized.**  The
   `ShaftsDriveline2WD` template applies inertia but the only resistive
   load comes from tire ground forces.  In reality the gearbox + differential
   bleed several kW.  EV1 manuals cite ~92% drivetrain efficiency; that
   missing 8% is currently unaccounted.
2. **Spring/damper rates are guesses.**  Identical front/rear spring
   coefficients (143000 N/m) is implausible — no engineer would design a
   FWD car with a 50/50 spring-rate split given the front-bias mass.
3. **No drivetrain parasitic torque.**  Real motor + inverter has ~5-15%
   loss across the operating range; currently this only shows up in the
   coast-torque map, which only fires at zero throttle.  (That map read
   -5 to -25 N·m when this was written; since 2026-08-12 it is -5 to -12.5
   N·m — see §3.1.)

Plan: add a `ChForce` to the chassis representing constant
drivetrain-loss torque, calibrated to a coastdown test from real EV1
data — but this requires real coastdown numbers, which we don't have.
Track this as a future TODO.

## 11. Empirical evidence — coastdown shows 2× excess dissipation

After applying the audit fixes (final drive 10.946, mass 1199, track widths),
the `coastdown` scenario (`config/scenarios/coastdown.json`) produced these
data points after throttle release at t=13.55 s, v=30 m/s:

| t (s) | v (m/s) | decel (m/s²) avg over Δt |
|---|---|---|
| 13.6 | 30.05 |  — |
| 15.2 | 29.23 | 0.48 |
| 20.4 | 27.19 | 0.40 |
| 25.5 | 24.88 | 0.33 |
| 30.6 | 23.86 | 0.30 |
| 43.4 | 19.98 | 0.30 |

Fitting `F = m × decel = F_rr + 0.5·ρ·v²·CdA` to the (30, 20) m/s anchors:

- **Sim F_rr ≈ 198 N** (real EV1 ≈ 100 N) — **~2× too high**
- **Sim CdA ≈ 0.756 m²** (real EV1 = 0.19 × 1.89 ≈ 0.36 m²) — **~2× too high**

So the simulator's total decel force is roughly 2× what published EV1
specs predict — both the static rolling component and the velocity-squared
component.  Likely culprits, in order of suspicion:

1. **TMeasy tire model internal slip dissipation.**  The `Rolling Resistance
   Coefficient: 0.008` only covers static rolling drag; TMeasy also
   dissipates power through longitudinal-slip deformation that scales
   non-linearly with speed.  Tuning the slip-curve parameters (not exposed
   in the current JSON) could halve the v² term.
2. **Driveline shaft inertia acting as effective damper** in the
   `ChShaftsDriveline2WD` solver.  The 0.5 + 0.6 kg·m² shaft inertias
   combined with stiff joints may produce numerical viscosity.
3. **Powertrain coast torque firing during coastdown** — the engine
   `Map Zero Throttle` fires at zero throttle.  As written this said the map
   was "-5 to -25 N·m", that coastdown covered "rpm range 4000-7000", and that
   the load there was "-10 to -15 N·m".  All three were wrong: coastdown spans
   3586-10 758 RPM and the map reached -16.9 N·m over it.  §11.1 has the
   corrected figures and the result of actually running the experiment below;
   the shipped map is now -5 to -12.5 N·m.

**Concrete next experiment when we want to chase this down**: drop the
zero-throttle map to ~half its current values, re-run coastdown, and
see if F_rr drops to ~150 N.  Anything still excess after that is in
the tire model or the solver.

**For now**: don't chase tire/driveline tuning.  The 2× factor is a
known plant-fidelity limitation captured here.  Cruise-control gain
tuning should still wait until F_rr is closer to spec — otherwise we'd
be tuning around the wrong plant.  See [docs/TODO.md](TODO.md) for the
follow-up task.

### 11.1 That experiment has now been run (2026-08-12)

Suspicion 3 above is the one §3.1 acted on, and two of its numbers were wrong
in the direction that made it look small.  Coastdown runs from 30 m/s down, so
it spans **3586–10 758 RPM**, not "4000-7000"; the old map's load over that
span reached −16.9 N·m, not "−10 to −15".

The coastdown was re-run either side of the §3.1 correction — same binary, same
scenario, only `Map Zero Throttle` differing, and differing only above 8350 RPM
(52.1 mph).  Both runs are identical until throttle release, as they must be:
the launch is at full throttle, where Chrono's blend `T_zero·(1−throttle) +
T_full·throttle` multiplies the coast map out entirely.  Measured speed at
release differs by **0.003 mph**.

**Read the earlier version of this section with suspicion.** It reported
F_rr 250.5 → 438.1 N and CdA 0.747 → 0.090 m², and concluded the residual drag
had become "speed-independent" and therefore "not aero-shaped".  Those figures
came from `scripts/fit_coastdown.py`, which capped its window at 30 s after
release (`MAX_DURATION_AFTER_RELEASE_S`).  The corrected car coasts further, so
the cap truncated the two runs at different speeds — 18.26 m/s (40.8 mph)
against 19.24 m/s (43.0 mph) — and the low-speed bins were being compared over
different windows.  The 8× CdA collapse was substantially that artefact.

The retraction is reproducible, and was re-run on 2026-08-12 before the tool
was changed: the pre-change `fit_coastdown.py`, on the two CSVs from
`config/coastdown.json`, prints F_rr 251.0 → 438.2 N and CdA 0.748 →
0.091 m².  That is the artefact, to three figures, from a committed run
condition — so the fix below is aimed at a defect that has been watched
happening rather than at a description of one.

#### The numbers below are the ones the shipped tool produces (2026-08-12, second pass)

An earlier version of this table read F_rr 193.4 → 239.6 N and CdA 0.880 →
0.571 m².  Those came from a **one-off unweighted fit written for that
session** and never committed — `fit_coastdown.py` still had its 30 s cap at
the time, so it could not have produced them.  A number in an audit that the
repo's own tool cannot reproduce is the defect this section exists to fix,
one level up.  Re-derived here with the shipped tool over a committed run
condition, and the difference is the estimator, not the plant: the shipped fit
weights each speed bin by its sample count (inverse-variance for a bin mean),
the ad-hoc one did not.  Same CSVs, ±4 %.

Both runs come from `config/coastdown.json` — also new on 2026-08-12, because
until then no config wrapper existed and the run condition behind every number
in §11 and §11.1 was unrecorded.  Reproduce with:

```sh
./build/ev1sim --config config/coastdown.json        # keep scenario_coastdown.csv
git stash / swap the coast map, re-run, keep the second CSV
scripts/fit_coastdown.py old.csv new.csv             # whole coast
scripts/fit_coastdown.py --v-max 23.28 old.csv new.csv   # the control
```

| fit window | | old coast map | new coast map |
|---|---|---|---|
| whole coast (10.01–29.78 m/s) | F_rr | 188.9 ± 17.6 N (1.89× spec) | 231.7 ± 48.2 N (2.32× spec) |
| whole coast | CdA | 0.914 ± 0.069 m² (2.54× spec) | **0.621 ± 0.181 m² (1.72× spec)** |
| below the knee only (v ≤ 23.28 m/s) | F_rr | 165.8 ± 8.8 N | 166.1 ± 9.2 N |
| below the knee only | CdA | 1.075 ± 0.051 m² | 1.072 ± 0.053 m² |

The ± is the scatter of the speed bins about the fitted line (n−2 degrees of
freedom) — model error, not sampling error, since each bin averages hundreds of
samples.  It is what says whether a two-parameter law describes the window at
all, and it is the number the retracted claim never carried: the CdA that was
reported as 0.090 m², "a quarter of spec", is **0.088 ± 0.145 m² over the
window it was fitted on**.  Consistent with no aero term at all, which is not a
measurement of one.  `fit_coastdown.py` now says so and exits non-zero.

**The bottom two rows are the control, and they are the reason to believe the
top two.**  Below the knee the two maps are byte-identical, and the fit agrees
to 0.3 N and 0.003 m² — well inside the ± above. Mean deceleration per band,
computed off the raw CSVs with no window cap, says the same thing more
directly:

| band | old | new |
|---|---|---|
| 25–30 m/s (56–67 mph) | 0.4610 m/s² | **0.3724 m/s²** |
| 20–25 m/s (45–56 mph) | 0.3788 m/s² | 0.3713 m/s² |
| 15–20 m/s (34–45 mph) | 0.2880 m/s² | 0.2881 m/s² |
| 10–15 m/s (22–34 mph) | 0.2076 m/s² | 0.2076 m/s² |

The two bands below the knee are identical to four decimals; 20–25 straddles it
and moves 2 %; 25–30 sits wholly above it and moves 19 %.  That is exactly the
footprint the §3.1 edit should have, and nothing else moved.  This table needs
no fit at all, which is why it is the check on the one above.

**What this does and does not say.**  The old coast curve's torque rose with
speed, and a two-parameter `F_rr + CdA·v²` fit has nowhere to put a rising
term but CdA — so it was inflating the apparent aero coefficient, and removing
it cuts CdA by 32 % (2.54× → 1.72× spec).  §11's headline was therefore
*partly* an artefact of an unsourced curve.  It was not wholly one: CdA is
still 1.7× spec and F_rr is still 2.3×, deceleration still rises with speed
across the range (0.208 → 0.288 → 0.372 m/s²), and the excess is still there.
An earlier draft of this section claimed the residual was "not aero-shaped" and
pointed the drag work away from the aero term; that was wrong, and it was wrong
because it rested on the window-truncated fit.

Coasting from 30.07 m/s (67.3 mph), the corrected car holds more speed while
above the knee, converging again below 52 mph where the maps agree:

| s after release | old mph | new mph | Δ mph |
|---|---|---|---|
| 0 | 67.3 | 67.3 | +0.00 |
| 5 | 62.2 | 63.6 | +1.39 |
| 10 | 57.2 | 59.5 | +2.24 |
| 15 | 52.6 | 55.3 | **+2.64** |
| 20 | 48.4 | 51.0 | +2.62 |
| 30 | 40.9 | 43.0 | +2.19 |
| 50 | 29.6 | 31.1 | +1.44 |
| 70 | 20.6 | 22.0 | +1.38 |

Re-fitting the drag calibration is still open and is still the item in
`docs/TODO.md` — but it should be fitted against a coast map bounded by the
print, over the speed range both runs reach rather than a fixed slice of time,
and with the sub-knee control above as the check that the fit is measuring the
plant rather than the window.  `fit_coastdown.py` now enforces the first two of
those: it takes both CSVs at once, states the window it fitted, and refuses
when there isn't a shared one.


## 12. Calibration attempt — what didn't work

After the audit fixes landed, an explicit calibration attempt
(2026-04-30) tried to bring F_rr and CdA closer to spec.  Tooling:
[scripts/fit_coastdown.py](../scripts/fit_coastdown.py) does
weighted-OLS on bin centroids to recover F_rr and CdA from a coastdown
CSV.  Tested values:

| config                                        | F_rr (N) | CdA (m²) |
|---|---|---|
| baseline (engine -5..-25, Crr=0.008)          | 301      | 1.21     |
| engine zeroed (engine 0, Crr=0.008)           | -126*    | 2.01     |
| engine halved (engine -2.5..-12.5, Crr=0.008) | 220      | 1.98     |
| Crr lowered (engine -5..-25, Crr=0.005)       | 185      | 1.75     |
| Crr near-zero (engine -5..-25, Crr=0.001)     | unstable | unstable |

\* negative F_rr is a fit artifact — pure-v² shape doesn't decompose
cleanly.  The "F_avg" at v≈22 m/s is actually a better single-number
summary than the F_rr/CdA decomposition.

| config                                        | F_avg @ 22 m/s | F_avg @ 27 m/s |
|---|---|---|
| baseline                                      | 667 N          | 845 N          |
| engine zeroed                                 | 528 N (-21%)   | 780 N (-8%)    |
| engine halved                                 | 815 N (+22%)   | 1180 N (+40%)  |
| Crr lowered                                   | 723 N          | ~1000 N        |

Engine-zero gave the cleanest reduction (~10-20%).  But "engine halved"
gave WORSE total drag than baseline — the trajectory diverged because
small dynamics changes accumulate over a 30 s coastdown.

**Conclusion**: the simple knobs exposed in the JSON (engine coast map,
tire Crr) can each shave 10-20 % off total drag, but cannot get within
2× of spec.  The remaining ~70-80 % of excess dissipation lives in
TMeasy's internal slip dynamics — `Vehicle Type: "Passenger"` preset
sets longitudinal/lateral slip stiffness defaults that aren't exposed
in the JSON.

**Reverted to baseline config.**  Calibration deferred until either:
(a) Chrono exposes more TMeasy parameters via JSON, (b) we switch to
a different tire model with explicit dissipation knobs (e.g.
Pacejka-style with rolling-resistance map), or (c) we accept the 3×
factor as a known plant limitation and tune the controller around it.

**What the tooling enables for future work**:
- `./build/ev1sim --config config/coastdown.json` is the standardized
  validation run — one command, committed terrain, no external sim.
- `scripts/fit_coastdown.py scenario_coastdown.csv` reports F_rr and CdA with
  the speed window it fitted and a spread on each.  Comparing a before and an
  after means passing **both** CSVs in one invocation, not eyeballing two runs.
- Both should converge toward 100 N and 0.36 m² as the plant model improves.

The table above is from 2026-04-30 and predates all of that: its numbers were
measured on an unrecorded terrain with the old time-capped fit, so treat them
as the shape of the result (which knob moves which way) and not as figures to
compare against anything measured since.

## 13. TMeasy full-parameterization migration (2026-04-30)

The bearing-capacity shortcut in `data/vehicle/ev1/tire/EV1_TMeasyTire.json`
turned out to be **silently overwriting the tire radius**.  Chrono's
`TMeasyTire::Create()` (TMeasyTire.cpp:178) calls
`GuessPassCar70Par(bearing_capacity, m_width, ratio, m_rim_radius, ...)`,
but the function expects `rimDia` (diameter) in the 4th slot, and inside
it sets `m_rim_radius = 0.5 * rimDia` and
`m_unloaded_radius = secth + rimDia / 2.0`.  Net result: a 14" tire
(spec'd 0.2915 m radius) silently became a 7"-equivalent (~0.20 m) at
runtime.

Switched the JSON to the **full parameterization** path so the Design
block radii stay authoritative and every slip parameter is explicit.
Numerical defaults equal what `GuessPassCar70Par(load=2375 N)` would
have computed, so the only change is the radius bug fix.

Coastdown comparison after the migration:

| config              | F_rr (N) | CdA (m²) | F_avg @ 22 m/s | F_avg @ 27 m/s |
|---|---|---|---|---|
| baseline (buggy 7" tire)         | 301 (3.0×) | 1.21 (3.4×) |  667 N | 845 N |
| full-param defaults (true 14")   | 134 (1.34×) | 2.34 (6.5×) |  835 N | 1305 N |
| full-param 0.5× dfx0             | 622 | 0.85 |  902 N | 1006 N |
| full-param 2× dfx0               | -75* | 2.40 |  909 N |  919 N |
| full-param 4× dfx0               | unstable | unstable | — | — |

\* negative F_rr is fit artifact when the fit is dominated by curvature.

**The geometric correction is the bigger story.**  F_rr is now within
35 % of spec instead of 3× — that part of the model was not "broken,"
it was just operating at the wrong scale.  The remaining ~6.5× CdA is
TMeasy slip dynamics scaled to higher tire force at speed, which the
exposed slip-stiffness knob doesn't usefully reduce: lowering it traded
F_rr for CdA, raising it produced numerical instability around 2-4×.

**Locked in**: full-param defaults.  Tire radius is now correct;
slip-stiffness tuning deferred until either (a) we have measured EV1
slip-curve data, (b) we switch to a Pacejka/Magic-Formula tire with
explicit rolling-resistance map, or (c) we accept the high-speed drag
overshoot as a known plant limitation.

## 14. Rear brake actuator + pedal-feel design (proposed)

### Rear brake — self-energizing drum with electric actuator

EV1 service manuals describe the rear brakes as drums driven by an
electric motor at each wheel (kSigRearMotorLR/RR are already published
by BTCM).  No hydraulic line, no shoe-return spring assist — the motor
drives a leadscrew-like mechanism that pushes the shoes outward.

Self-energizing drum behavior summary:

```
T_brake(F_shoe, omega, mu) =
    mu × F_shoe × R_drum × (1 + alpha × sign(omega))
```

where `alpha` is the self-energizing coefficient:
- **Leading shoe**: `alpha ≈ 1-3` (friction torque pulls shoe harder onto drum)
- **Trailing shoe**: `alpha ≈ 0` (friction torque pulls shoe away from drum)
- **Duo-servo / double-leading**: `alpha ≈ 2-3` average

As wheel speed `|omega| → 0`:
- Dynamic friction limit unchanged, but the self-energizing assist
  collapses (no rotation, no friction torque to amplify the shoe force).
- **Required actuator force to hold a given brake torque rises as the
  wheel slows.**  At standstill the EMB needs ~3× the force it needed
  at speed for the same wheel torque.
- This is exactly opposite to a hydraulic system where pedal pressure
  is constant once applied.

**Where it should live**:
- `electricsim/ev1/btcm/`: model the **commanded shoe force** based on
  motor current and travel.  Already publishing `kSigRearMotorLR/RR`;
  needs a force-vs-current curve.  Real EV1 fuse ratings + steady-state
  motor current limits are the spec inputs (manual hints).
- `ev1sim/src/`: model the **physics of the drum** (T_brake formula
  above) and apply per-wheel torque to Chrono.  Reads
  `kSigRearMotorLR/RR` for force command, reads `wheel_omega_rl/rr`
  from physics for the self-energizing factor.  Mirrors the existing
  front-brake bus-mediated path (`ApplyAbsFrontBrake` / `ApplyFrontBrakePerWheel`).

**Estimated parameter ranges (educated guesses)**:
- R_drum ≈ 0.10 m (8" drum)
- mu (drum-on-shoe) ≈ 0.35-0.40 (typical phenolic resin lining)
- alpha ≈ 2.0 (double-leading shoe, conservative)
- F_shoe_max ≈ 4000 N (matches typical caliper clamping force)
- Required motor torque to maintain F_shoe at standstill: ~10-15 N·m
  (back-calculated from fuse-rated current × motor torque constant).

This becomes a calibration target the same way the coastdown is —
add a "rear-brake-only stop" scenario that engages just the EMB
actuators and measures stopping distance vs speed.

### Pedal feel — physical spring, virtual hydraulics

The user's sim rig has a real spring (possibly two-stage) on the brake
pedal.  The pedal **position** is what crosses into the simulator; the
**force feel** is hardware-side.  What ev1sim/electricsim need to do:

1. **Read pedal travel** (already done — `kSigDriverBrakePedalQ8` 6900).
2. **Convert position → master cylinder pressure** with a simplified
   model.  Two-stage suggests two linear segments:
   ```
   pressure(travel_q8) =
       0                                    if travel_q8 < dead_band
       k1 × (travel_q8 - dead_band)         if travel_q8 < transition
       k1 × transition + k2 × (travel_q8 - transition)
                                            otherwise
   ```
   `dead_band` matches the pedal's free-travel before the master
   cylinder cup engages.  `k1` is the soft (initial fluid takeup)
   stage; `k2` is the firmer (full fluid pressure) stage.
3. **Publish master cylinder pressure** as a new chassis-bus signal
   (`kSigChassisBrakeMasterPressureKpa`?).  BTCM consumes this as the
   real brake-effort input — replacing or augmenting the current
   simple `brake_switch` boolean.
4. **ABS isolation feedback** — when BTCM isolates a wheel, the line
   pressure feedback isn't returned to the master cylinder (in real
   cars the pump cycles fluid, kicking back at the pedal).  Active
   force feedback to the rig could simulate this; out of scope for
   this design unless the rig has a force-feedback motor.

**Where it should live**:
- `ev1sim/src/PhysicalWorld.h`: new `BrakePedal` component owning the
  position-to-pressure calibration table.  Publishes pressure as a new
  driver-input bus signal.  Sits alongside `CombinationSwitch`,
  `WiperStalk`, etc.
- `electricsim/ev1/btcm/`: subscribe to the pressure signal, use as the
  primary brake-effort input.  Brake-switch (6904) stays as the ABS
  pump-prime threshold (boolean: any meaningful pedal application).

**Estimated parameter ranges (educated guesses)**:
- `dead_band` ≈ 5-10 % travel (q8 13-25)
- `transition` ≈ 30-40 % travel (q8 76-102)
- `k1` ≈ 100 kPa per 1 % travel (initial takeup)
- `k2` ≈ 250 kPa per 1 % travel (full fluid stage)
- Max pressure at full pedal ≈ 12-15 MPa (typical passenger-car spec)

These are starting points; tune against real EV1 manual data when
available, or to match the user's preferred sim-rig pedal feel.

Both features are deferred — captured in [docs/TODO.md](TODO.md) so
they're not lost when we pivot back to integration work.

## 15. Regen ↔ friction blending — known limitation (S15)

**Status: documented gap, intentionally not modelled.** Flagged by the
regulator-lens safety review; the honest answer here is "we don't have
the data to do this faithfully," so this section records *why* rather
than inventing physics.

What exists today:
- **Friction braking is regen-agnostic.** The front `BrakeSimple` disc
  and the rear `BrakeDrum` EMB ([src/BrakeDrum.h](../src/BrakeDrum.h))
  compute wheel torque from the brake command alone. Nothing subtracts a
  regen contribution from friction, and nothing adds a regen torque to
  the wheels.
- **Regen is a readout, not a force.** [src/MotorCurrent.h](../src/MotorCurrent.h)
  recovers *signed DC-bus current* from the mechanical operating point
  (T·ω) and ev1sim publishes it as the ammeter value `motor_current_a`
  (chassis 4072). It never feeds `DriverCommand` or Chrono wheel torque.
- The coast-torque map (§3, `EV1_EngineSimpleMap.json "Map Zero
  Throttle"`) is residual bearing/windage drag scaled to RPM — *not*
  commanded regen. It fires on throttle release regardless of brake.

Why this blocks the **regen-cutout fail-safe** test: the safety property
is "if regen drops out, the friction system backfills the lost
deceleration so total braking is preserved." In-sim the two systems are
**not coupled** — there is no regen torque on the wheels to cut, and no
blend controller to hand off to friction — so the property is
**untestable as built**. (Contrast S16's BTCM-death property, which *is*
testable because the rear-EMB→friction path is real and modelled.)

What a faithful model would need (and why we're not guessing it):
- A regen-torque-vs-speed envelope for the EV1 induction motor under
  braking (the production car blended up to ~25 kW of regen, but the
  *torque* curve and its low-speed taper are not in the manual corpus).
- The blend-handover law: how the BTCM/PIM split a brake demand between
  regen and friction, and the speed/SOC/temperature conditions that cut
  regen out (cold pack, full SOC, fault). None of this is published.

Fabricating those constants would produce confident-looking but
unsupported numbers — worse than the documented gap. Per the project's
"faithful where the manual is authoritative, documented inference
elsewhere" rule, this stays a gap until either (a) measured/traceable
EV1 regen-blend structure surfaces, or (b) a *structure-only* model is
warranted (regen torque + friction summed to driver demand, with a
friction backfill on cutout) using explicitly `@design`-tagged
engineering choices. The follow-up marker lives in
[docs/TODO.md](TODO.md) under "Bus-mediated physics".

If/when modelled, the natural shape:
- Add a `propulsion_torque` (signed) channel to `DriverCommand`
  (already anticipated in [ARCHITECTURE.md](../ARCHITECTURE.md) "Future
  per-axle brake control"): regen maps to negative propulsion torque.
- A blend controller (EV1: BTCM-side) sums regen + friction to the
  driver's deceleration demand and backfills friction when regen is
  unavailable — at which point the cutout fail-safe becomes a real,
  testable closed-loop property.
