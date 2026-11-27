# ev1sim — TODO

Catch-all for physical-world / render / sim follow-ups deferred from focused
work.  When you finish an item, delete it from here.

## Tire model

- [ ] **Slip-dynamics drag calibration (deferred).**  Coastdown CdA is
  **2.7× spec**: 0.99 ± 0.28 m² against the printed 0.19 × 1.89 = 0.36 m²
  (chassis p. 62), with F_rr 278 ± 71 N, 2.6× the rolling-resistance
  estimate, over 10.0-29.6 m/s on the 1350 kg car (`./build/ev1sim --config
  config/coastdown.json`, then `scripts/fit_coastdown.py`).  The 1281 kg car
  fits the same within the spread (0.98 m² at its own mass), so the mass and
  tyre values do not move it.  The 0.621 m² (1.7×) in
  `docs/ev1_chrono_audit.md` §11.1 is an older plant's fit and no longer
  reproduces.
  The excess lives in TMeasy's internal slip-curve
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
