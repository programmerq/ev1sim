# ev1sim — TODO

Catch-all for physical-world / render / sim follow-ups deferred from focused
work.  When you finish an item, delete it from here.

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
