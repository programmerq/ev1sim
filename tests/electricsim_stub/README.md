# electricsim CI stub

A **self-contained** stand-in for electricsim's `src/io/` wire-truth substrate,
used only by CI's `integrated-build` job. Pointing `-DELECTRICSIM_DIR=` at this
tree makes ev1sim's CMake build the `electricsim_wire_substrate` lib and compile
`ExternalSimConnector.cpp` + `WireTruthChassis.cpp` with
`EV1SIM_HAVE_EXTERNAL_SIM=1` / `EV1SIM_HAVE_WIRE_TRUTH=1` — **without a real
electricsim checkout or any repo secret**.

## What it does / doesn't catch

- ✅ **Catches** compile/link breakage of the `EV1SIM_HAVE_EXTERNAL_SIM`-guarded
  WireTable publish/consume path + the real `WireTruthChassis` implementation —
  the code the plain stub build `#if`-excludes.
- ⚠️ **Does not** verify ev1sim's chassis IDs are in sync with the *live*
  electricsim, nor that the topology hash matches a running fleet:
  `ev1_chassis_signals.hpp` / `topology_generated.h` here are vendored copies, so
  the drift-guard `static_assert`s pass against the vendored constants. Live
  drift detection needs the App-token integrated build against the real repo.

## Surface

The wire-truth substrate the connector now depends on (electricsim deleted the
legacy SharedMemoryTransport message queue — `protocol` / `shm_transport` /
`signal_catalog` / `bridge_adapter` — in the "rip out the message-queue layer"
change; those stubs are gone here too):

- `wire_table.{cpp,hpp}` — the shared-memory `WireTable` (the substrate).
- `topology/env_open.{cpp,hpp}` — `try_open_from_env()` (the attach idiom).
- `topology/topology_generated.h` — generated `kWire*` ids, `kTopologyHash`,
  `declare_all()`.
- `ev1_chassis_signals.hpp` — canonical chassis IDs for the connector's
  drift-guard `static_assert`s.
- `gm8192/gm8192_frame.{c,h}`, `gm8192/gm8192_rx_framer.{cpp,hpp}`,
  `uart/uart_rx.{cpp,hpp}`, `uart/uart_tx.{cpp,hpp}` — the GM-8192 frame snoop
  `WireTruthChassis` uses to decode module TX frames off the bus (IPC-cluster
  Phase 1, `docs/ipc_rsa_display_plan.md`). They depend only on
  `wire_table.{cpp,hpp}` + each other.

These are **vendored copies** of electricsim's files (self-contained — they pull
in only standard headers and each other). ev1sim's CMake stale-check probes
exactly this set, so the tree must stay complete.

## Maintenance

These are copies of electricsim `src/io/` files; refresh them from the
electricsim tree when:

- you add a chassis ID to the drift guard (a new `EV1SIM_CHASSIS_ID_MATCHES`
  line in `ExternalSimConnector.cpp`) — re-copy `ev1_chassis_signals.hpp` so the
  new `kSigChassis*` constant exists here;
- the connector starts reading/writing a new `kWire*` cell — re-copy
  `topology_generated.h` so the constant resolves;
- electricsim's `WireTable` / `env_open` API changes — re-copy the matching file;
- `WireTruthChassis` snoops a new module's GM-8192 TX frame, or the GM-8192
  frame / UART state machines change — re-copy `gm8192/` + `uart/`.

Copy verbatim from `${ELECTRICSIM_DIR}/src/io/` (do not hand-edit) so the stub
stays a faithful, compileable subset.
