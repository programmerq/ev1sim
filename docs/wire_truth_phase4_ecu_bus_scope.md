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

Each read the old `main_transport` poll carried, and where it comes from now:

| ev1sim read | was | wire path | status |
|---|---|---|---|
| RSA run-mode | 5711 | `RSA_RUN1_OUT` / `RSA_RUN2_OUT` (+ RHJB RUN1/RUN2) | ✅ done |
| AD main contactor | 5224 | `APM_HV_CONTACTOR_CLOSED` | ✅ done |
| AD state enum | `kSigAdStateEnum` | `AD_STATE_A/B/C` + `AD_POWER_SUPPLY` | ✅ done |
| BTCM liveness and frame data | 5050 | `GM8192_BTCM_TX` → `gm8192_rx_framer` → `BusSnoop` $91 decode | ✅ done |
| Front ABS solenoids FL/FR | 5010-5013 | `CHASSIS_BTCM_ISO_CLOSE_*` + `BTCM_FRONT_MOD_*` motor leads | ✅ done (chassis cells only) |
| Rear EMB motors LR/RR | 5014/5015 | `BTCM_REAR_EMB_*` motor leads | ✅ done (chassis cells only) |
| PIM cruise active / setpoint | 5360/5361 | `PIM_CRUISE_ACTIVE` / `PIM_CRUISE_SETPOINT_MPS` | ✅ done |
| AD precharge relay | 5225 | `HV_ES_PRECHARGE_CONTACTOR` | ✅ done |

Freshness: each read is `written()`-gated. BTCM liveness is a valid $91 BTCM
Data Response (framed, SumCheck good, N = 11) decoded within the window
(`ExternalSimConnector::BtcmAlive`); the actuator cells are commanded state and
stay authoritative while the BTCM is alive.

Every row is migrated; the connector holds no `SharedMemoryTransport` and no
legacy signal-id path for the BTCM.

## Preserved / out of scope

GM-8192 serial/UART framing (`gm8192/*`, `uart/*`) survives upstream and is how
the BTCM frame reaches the wire (`GM8192_BTCM_TX`). ev1sim keeps `gm8192_rx_framer`
to decode it. The chassis WireTable overlay (`WireTruthChassis`, Phase 3) is
unchanged.
