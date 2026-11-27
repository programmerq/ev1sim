# ev1sim — TODO

Catch-all for physical-world / render / sim follow-ups deferred from focused
work.  When you finish an item, delete it from here.

## Tire model

- [ ] **Slip-dynamics drag calibration (deferred).**  Coastdown CdA is
  **1.7× spec**: 0.621 ± 0.181 m² against the printed 0.19 × 1.89 = 0.36 m²
  (chassis p. 62), with F_rr about 2.3× the rolling-resistance estimate
  (`docs/ev1_chrono_audit.md` §11.1).  That fit predates the driveshaft inertia
  at 0.03 kg·m² (0.5 sat in every deceleration it used), the printed 1350 kg
  and the printed tyre load index; re-measure
  with `./build/ev1sim --config config/coastdown.json` and
  `scripts/fit_coastdown.py`.  The excess lives in TMeasy's internal slip-curve
  shape; the exposed `Initial Slopes dFx/dsx` knob doesn't usefully reduce it
  (lowering trades F_rr for CdA, raising past 2× hits numerical instability).
  Defer until we have measured EV1 slip-curve data or migrate to a Pacejka
  tire model.  The sub-knee control (`--v-max 23.28`) is the check that a fit
  is measuring the plant and not the window, and the script reports a spread
  and refuses a window that cannot separate F_rr from CdA.  (The `Body
  aerodynamics` follow-up, "trim tire rolling/slip dissipation now that drag
  is no longer lumped into it", is this same item.)

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
