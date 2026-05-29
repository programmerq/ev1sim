# electricsim CI stub

A **minimal, self-contained** stand-in for electricsim's `src/io/` layer, used
only by CI's `integrated-build` job. Pointing `-DELECTRICSIM_DIR=` at this tree
makes ev1sim's CMake build the `electricsim_connector` shim and compile
`ExternalSimConnector.cpp` with `EV1SIM_HAVE_EXTERNAL_SIM=1` — **without a real
electricsim checkout or any repo secret**.

## What it does / doesn't catch

- ✅ **Catches** compile/link breakage of the `EV1SIM_HAVE_EXTERNAL_SIM`-guarded
  transport/publish path — the class of bug the stub build `#if`-excludes (e.g.
  the dangling `ext_horn_*` reference that broke the integrated build).
- ⚠️ **Does not** verify ev1sim's chassis IDs are in sync with the *real*
  electricsim: `ev1_chassis_signals.hpp` here is pinned to ev1sim's own values,
  so the drift-guard `static_assert`s pass tautologically. Live drift detection
  needs the App-token integrated build against the real repo.

## Surface

Only what the connector uses: `protocol.hpp` (frames/deltas/enums),
`shm_transport.hpp` (a header-only no-op transport), and
`ev1_chassis_signals.hpp` (canonical IDs). The four `src/io/*.cpp` are
intentionally near-empty — they exist only because ev1sim's CMake builds the
connector lib from those four sources and checks each is present.

## Maintenance

If you add a chassis ID to the drift guard (a new `EV1SIM_CHASSIS_ID_MATCHES`
line in `ExternalSimConnector.cpp`), regenerate `ev1_chassis_signals.hpp` so the
new `kSigChassis*` constant exists here, or `integrated-build` will fail to
compile. If electricsim's protocol/transport API changes, mirror the minimal
change in `protocol.hpp` / `shm_transport.hpp`.
