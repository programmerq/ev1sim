# ev1sim — TODO

Catch-all for physical-world / render / sim follow-ups deferred from focused
work.  When you finish an item, delete it from here.

## Tire model

- [ ] **Slip-dynamics drag calibration (deferred).**  CdA still 6.5× spec
  in coastdown after the geometry fix.  The remaining excess lives in
  TMeasy's internal slip-curve shape; the exposed `Initial Slopes
  dFx/dsx` knob doesn't usefully reduce it (lowering trades F_rr for
  CdA, raising past 2× hits numerical instability).  Defer until we
  have measured EV1 slip-curve data or migrate to a Pacejka tire model.
  (The `Body aerodynamics` follow-up — "trim tire rolling/slip dissipation
  now that drag is no longer lumped into it" — is this same item.)
  **Updated 2026-08-12:** re-fit this against the corrected coast map, not the
  old one, and run it as `./build/ev1sim --config config/coastdown.json`.
  The pre-2026-08-12 coast curve's torque rose with speed, and a two-parameter
  `F_rr + CdA·v²` fit has nowhere to put a rising term but CdA — so §11's "2×
  on both terms" was partly an artefact of an unsourced curve.  Bounding the
  coast map moves **CdA 0.914 → 0.621 m²** (2.54× → 1.72× spec) and **F_rr
  188.9 → 231.7 N**.  Below the knee, where the two maps are identical, both
  fits agree to 0.3 N and 0.003 m² — use that sub-knee control
  (`--v-max 23.28`) as the check that any future fit is measuring the plant and
  not the window.  The excess is reduced, **not** explained away: CdA is still
  1.7× spec, and an earlier version of this entry wrongly concluded the residual
  was "not aero-shaped" on the strength of a time-truncated fit.
  `scripts/fit_coastdown.py` no longer has that 30 s cap: pass it both CSVs and
  it fits them over the speed range they share, names it, and refuses when
  there isn't one.  Its numbers also now carry a spread, and a window that
  cannot separate F_rr from CdA says so instead of printing a confident value —
  which is what makes "CdA is 1.7× spec" a claim worth acting on and the
  retracted "0.09 m², a quarter of spec" one that never was (it was
  0.088 ± 0.145 m²).

## Brakes

- [ ] **Front calipers keep a pressure model separate from the BTCM's.**
  `SimApp.cpp` (the ABS phase block that ends in `ApplyFrontBrakePerWheel`)
  runs its own fill and dump time constants off the BTCM's valve commands and
  ignores the per-wheel cylinder pressure the BTCM publishes (4153/4154). So a
  front isolation valve that sticks against its command, a drifting pressure
  sensor, or the master-cylinder fallback through a valve that is stuck closed
  cannot change how the car stops. Coupling the brake to the BTCM's pressure
  stopped every anti-lock case from cycling, so the two models need
  reconciling first: electricsim `BL-2026-11-27-front-caliper-two-models`
  (electricsim #939) and `BL-2026-11-26-brake-front-hydraulic-plant`
  (electricsim #938).

## Future: physical sim rig

- [ ] Adapt the user's physical driving sim rig with a lookalike RSA, IPC
  display and chime/horn sound effects.  Real switches already reach the sim:
  a USB button box's buttons bind to momentary actions or to maintained
  switches (seat belts, charge coupler, A/C, defrost, windows, HVAC fan and
  mode rotaries) through `input.rig` (`src/RigSwitches.h`), and a button bound
  to `park_brake_press` is one push of the park brake button.  Still open: the
  lookalike RSA and IPC hardware, and switches that must reach the bus as
  contact states rather than PhysicalWorld inputs.

## Vendored stub hygiene
