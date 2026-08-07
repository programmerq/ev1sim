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
- ⚠️ **Does not** verify the topology hash matches a *running* fleet — that needs
  a live co-sim, not a build. What it no longer fails to catch is the copy going
  stale; see below.

## The drift guard, and why the old one could not work

The `static_assert` drift guards in `ExternalSimConnector.cpp` compare
`ev1_chassis_signals.hpp` / `topology_generated.h` constants against ev1sim's own.
Both sides of that comparison are *vendored copies in this directory*, so the
assertions compare the copy to itself and pass by construction. They were green
throughout the period when **5 of the 15 vendored files had silently diverged from
their originals — including the substrate itself, `wire_table.{cpp,hpp}`.** A check
that cannot fail is not a check.

`check_stub_sync.sh` is the one that can. It has two legs:

- **Integrity** (always, no external-sim tree needed): every vendored file's
  sha256 matches `sync_manifest.txt`, and the manifest and the tree list exactly
  the same files. This catches a hand-edited stub — which this README forbids but
  nothing previously enforced — and it runs in CI, where no external-sim checkout
  exists.
- **Upstream sync** (when an external-sim tree is supplied): every vendored file
  is byte-identical to its original. This catches the copy being stale.

Neither leg alone is enough. Integrity would happily certify a tree that is
internally consistent and a year behind; upstream sync cannot run in CI. The
verdict is three-valued — `IN_SYNC` / `DRIFTED` / `UPSTREAM_UNAVAILABLE` — and with
`--require-upstream` the third is a failure, so "no external-sim tree was found"
can never be reported as success.

**The guard's own ability to fail is tested.** `selftest_check_stub_sync.sh` runs
it against deliberately corrupted copies (one flipped byte, a deleted file, an
unlisted extra file, a truncated manifest, an upstream that moved, a missing
upstream) and requires the verdict to change in each case. Both scripts are
registered as CTests under the `guard` label, so:

```sh
ctest -L guard        # the checks, and the proof they respond to their input
```

## Refreshing the vendored copies

Copy verbatim from `${ELECTRICSIM_DIR}/src/` (do not hand-edit), then re-record the
manifest **in the same commit as the copy**:

```sh
tests/electricsim_stub/check_stub_sync.sh --update --upstream <external-sim tree>
```

`--update` requires `--upstream` on purpose: the manifest records what the copy is
in sync *with*, so recording it without checking upstream would bless whatever
happens to be on disk. Never run `--update` on its own to clear a red `stub_sync` —
that records the drift as intended rather than fixing it, and leaves the next
reader unable to tell the difference.

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
- `topology/shm_test_helpers.hpp` — pulled in by `wire_table.cpp` on POSIX.
- `../net_host/conductor_publisher.hpp` — the publish edge for **conductor** cells.
  Conductor energisation is an output of the external sim's solver, so those cells
  carry a distinct type with no write overload and this header is the only place
  one can be set. ev1sim never publishes a conductor in production (it consumes
  them); its *tests* stand in for the solver and use this same edge rather than
  casting the type away. It sits outside `src/io/` because it depends on the
  substrate rather than being part of it, so the substrate lib takes both
  `${ELECTRICSIM_DIR}/src/io` and `${ELECTRICSIM_DIR}/src` as include roots.

These are **vendored copies** of electricsim's files (self-contained — they pull
in only standard headers and each other). ev1sim's CMake stale-check probes
exactly this set, so the tree must stay complete, and `check_stub_sync.sh` fails if
the tree and `sync_manifest.txt` disagree in either direction.

## Maintenance

**Refresh the whole set, not the one file that broke your build.** The old advice
here was a list of "re-copy X when Y changes", and following it is how the tree
ended up with 5 stale files: each refresh took the file that was failing to compile
and left its neighbours behind. `check_stub_sync.sh` now checks all of them, so a
partial refresh fails immediately rather than years later.

The procedure, in one commit:

```sh
cp -r <external-sim tree>/src/io/…                tests/electricsim_stub/src/io/
cp    <external-sim tree>/src/net_host/conductor_publisher.hpp \
                                                  tests/electricsim_stub/src/net_host/
tests/electricsim_stub/check_stub_sync.sh --update --upstream <external-sim tree>
ctest -L guard
```

Copy verbatim (do not hand-edit) so the stub stays a faithful, compileable subset —
the integrity leg treats any local edit as drift, which is the point.

A topology change on the producer side is the case that matters most: it moves
`kTopologyHash`, and a stub that still carries the old value builds an ev1sim that
**refuses to attach to the live fleet**. That refusal used to be silent; it is now
a loud, fatal diagnostic (see `AttachOutcome` in `src/WireTruthChassis.h`), but the
fix is still to refresh this tree and rebuild — not to relax the hash gate.
