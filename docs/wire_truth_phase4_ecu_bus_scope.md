# Wire-truth Phase 4 — retire ev1sim's ECU-bus SharedMemoryTransport

Written 2026-06-17. Follows Phase 3 (chassis ring drop, commit `a502071`).

## Why

electricsim's `claude/wire-truth-retire-legacy` (PR #171) commit `19f17aa`
("rip out the message-queue layer; every wire is shared memory") deletes the
entire host-side `SharedMemoryTransport` message queue — `protocol`,
`shm_transport`, `signal_catalog`, `bridge_adapter`, `chassis_bridge` are gone,
and the two-ring split (`electricsim_ev1_bus` + chassis) collapses into ONE
WireTable opened via `try_open_from_env`. No compat shim; the substrate is
mandatory.

ev1sim compiles 4 of those deleted sources (`signal_catalog.cpp`, `protocol.cpp`,
`shm_transport.cpp`, `bridge_adapter.cpp`) for its `main_transport` (the
`electricsim_ev1_bus` ECU endpoint). Against electricsim head, ev1sim's CMake
stale-check (`_ev1sim_check_electricsim_tree`, requires those 4) fails, so the
connector **silently compiles as a no-op stub** — all external-sim integration
(chassis included) disabled. ev1sim still builds; the regression is invisible to
electricsim CI (it doesn't build ev1sim).

One shared WireTable is exactly what ev1sim already attaches, so Phase 4 finishes
what Phase 3 started: get ev1sim entirely off the legacy transport.

## Current state (pre-Phase-4)

`ExternalSimConnector` holds two `SharedMemoryTransport`s:
- `transport` (chassis, `electricsim_chassis_bus`) — **vestigial** since Phase 3
  (never created; publishes guarded off; the WireTable overlay is authoritative).
- `main_transport` (`electricsim_ev1_bus`) — **live**: publishes driver-input
  GM-8192 frames and polls ECU telemetry frames (`ExternalSimConnector.cpp`
  ~3202-3361 consume, ~3363+ publish).

Both depend on `SharedMemoryTransport` / `protocol.hpp` / `shm_transport.hpp`,
all deleted upstream.

## Target

Pure WireTable. ev1sim already opens the unified region via
`WireTruthChassis::OpenFromEnv` (→ `try_open_from_env`). Phase 4:

1. **CMake**: drop the 4 deleted connector sources from `_electricsim_sources`;
   ev1sim now needs only the substrate lib (`wire_table.cpp` + `topology/env_open.cpp`).
   Update the stale-check so a transport-free electricsim tree is *valid*, not "stale."
2. **Connector**: delete both `SharedMemoryTransport` members, the
   `#include "protocol.hpp"` / `"shm_transport.hpp"`, and the GM-8192 frame
   publish/poll paths.
3. **Publish side**: ev1sim already mirrors driver inputs to the WireTable
   (`WireTruthChassis` ProducerRegistry). Confirm 1:1 coverage vs the old
   `main_transport` frame-publish, then delete the frame-publish.
4. **Consume side**: re-point the `main_transport` poll onto WireTable reads (map below).

## Consume migration map

ev1sim's `main_transport` poll (`ExternalSimConnector.cpp` ~3276-3359) consumes:

| ev1sim read | signal | wire path at electricsim head | status |
|---|---|---|---|
| RSA run-mode | 5711 | `RSA_RUN1_OUT` / `RSA_RUN2_OUT` (+ RHJB RUN1/RUN2 already read) | ✅ migratable (enum→bool remap) |
| AD main contactor | 5224 | `APM_HV_CONTACTOR_CLOSED` | ✅ migratable |
| AD state enum | `kSigAdStateEnum` | `AD_STATE_*` (confirm w/ electricsim) | ✅ likely |
| BTCM canonical frame | 5050 | `GM8192_BTCM_TX` + `gm8192_rx_framer` | ✅ migratable (involved) |
| Front ABS solenoids FL/FR ISO/DMP | `kSigSol*` | `CHASSIS_BTCM_ISO_CLOSE_*` / `DUMP_OPEN_*` (already read) | ✅ redundant w/ chassis |
| Rear EMB motors LR/RR | `kSigRearMotor*` | `CHASSIS_BTCM_EMB_MOTOR_CMD_*` (already read) | ✅ redundant w/ chassis |
| **PIM cruise active** | 5360 | — none — | ❌ GAP |
| **PIM cruise setpoint** | 5361 | — none — | ❌ GAP |
| **AD precharge relay** | 5225 | — none — | ❌ GAP |

Freshness: the main-bus consume keyed off frame `monotonic_time_ns` + windows
(e.g. `kAbsFreshnessWindow`); the WireTable `written()` / generation contract
replaces that.

## Gaps (blocked on electricsim — requested on PR #171)

- **PIM cruise state** (5360/5361) — PIM publishes no cruise-state cell post-rip-out.
- **AD precharge relay** (5225) + precharge sequence — no cell.

Until those land, ev1sim's cruise readout + precharge-sequence logging stay dark.
Consumers are `written()`-gated, so they degrade gracefully (like the aux-battery
producer gap), not crash.

## Sequencing

ev1sim builds (stub fallback) against head regardless, so there's no hard
deadline:
1. **(this branch)** Rip out `SharedMemoryTransport`; migrate the ✅ reads; drop
   the redundant frame-publish; fix the CMake stale-check. Connector compiles
   against head as a real WireTable client again.
2. **(electricsim)** declare the 3 gap cells (PR #171 ask).
3. **(this branch)** wire the gap reads once the cells exist.

## Preserved / out of scope

GM-8192 serial/UART framing (`gm8192/*`, `uart/*`) survives upstream and is how
the BTCM frame reaches the wire (`GM8192_BTCM_TX`). ev1sim keeps `gm8192_rx_framer`
to decode it. The chassis WireTable overlay (`WireTruthChassis`, Phase 3) is
unchanged.
