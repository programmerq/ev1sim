# Wire-truth migration scope — ev1sim's side of the chassis substrate

**Status:** 2026-06-15. **ev1sim's migration is mechanism-complete in both
directions.** Landed: Phase 1 (scope, this doc); the **producer dual-write** (a
table-driven `mirror_signal` writes all **84** ev1sim-produced cells onto the
wire alongside their ring publish); and the **consumer overlay** (a 66-cell
table-driven `apply_consumer_overlay` reads each ev1sim-consumed cell from the
wire `written()`-gated and routes it through the same `DebugInject*` dispatch
the ring uses). Because the consumer overlay already covers **all 66** cells,
**each electricsim producer batch lights up on ev1sim automatically** — no
further ev1sim code per batch. **All ev1sim-consumer batches are now landed by
electricsim and verified live on ev1sim:** B (horn), A (bulbs), J (wiper/washer),
K (HVAC), L (RSA), N (BTCM brake), I (PIM motor), C (RHJB sub-modules), and M
(IPC telltales + trip + dim). ev1sim builds against electricsim `df153a1` (hash
`0x27469F03`), **487/487 green**, with `[e2e]` tests driving the connector's real
overlay sinks out to the getters for a representative cell of each batch. Cross-
repo protocol: electricsim `notes/wire_truth_cross_repo_handshake.md`.

**Cutover Phase 1 COMPLETE — both directions now run on the wire.** electricsim
flipped every in-repo consumer of an ev1sim-*produced* cell to **wire-
authoritative** (reads the wire as the source of truth, not the ring): PIM
(`caa5e57`, 12 cells), IPC+APM (`9b5552a`, 5), LHJB+RHJB (`9899cd8`, 17), and
RSA+BTCM (`17fc22c`/`cba3f4d`, "Phase 1 COMPLETE", 31). Combined with ev1sim's
consumer overlay (which already reads every electricsim-produced cell off the
wire), the WireTable is now the authoritative path in **both** directions for
all chassis/driver-input cells. Every one of those reads is fed by ev1sim's
producer dual-write — all the cells are in ev1sim's producer registry, so **no
ev1sim change was needed for any module's flip** (verified 487/487 green against
each push; electricsim's parallel "disabled-fallback retirement sweep" deleted
internal fallback code/tests only — no contract constant or topology cell, hash
still `0x27469F03`). ev1sim continues to dual-write the ring during the soak.

Remaining on ev1sim's side: only the deferred **visual** LCD render (3D/2D
panel, ~weeks) and the eventual final **ring cutover** (delete ev1sim's legacy
ring publish/poll once every cell is wire-authoritative on both ends and soaked).

**What this is.** electricsim is moving the chassis bus off the legacy ring
(`SharedMemoryTransport "electricsim_chassis_bus"`, `DeltaRecord` publish/poll
of `kSig*` IDs) onto a shared-memory **WireTable**: a directory of named, typed
cells indexed by `WireId`, latest-value-wins, lock-free per cell, gated by a
`topology_hash` so peers built against different topologies refuse to attach.
ev1sim is a cross-repo peer on that bus. This doc enumerates every chassis cell
ev1sim touches, how it touches it today, and the per-group plan to move each one
onto the wire.

**Authoritative sources (read these, don't re-derive):**
- electricsim `docs/3d_sim_contract.md` — the cross-repo wire contract (signal
  IDs, encodings, cadence). **Encodings are frozen** (`EV1_CHASSIS_CONTRACT_VERSION
  1.9.0`); this migration does not change them.
- electricsim `config/topology.yaml` + generated `src/io/topology/topology_generated.h`
  — the 234 declared cells, their `WireId`s, types, `producer:`/`consumers:`,
  `legacy_signal_id`, `default`/`init_policy`.
- electricsim `notes/phase_c_chassis_migration_scope.md` — electricsim's own
  side of this migration (per-module producer/consumer batches, the §9 strategic
  addendum). ev1sim's plan is the mirror of it.
- electricsim `src/io/wire_table.hpp` (the API) and `src/io/topology/env_open.hpp`
  (`try_open_from_env` — the attach idiom ev1sim reuses).
- ev1sim `src/ExternalSimConnector.cpp` — ev1sim's bus endpoint (the code being
  migrated).

---

## 1. Headline — ev1sim touches **150** chassis/driver-input cells

Derived by joining `topology.yaml` (`producer:`/`consumers:`/`legacy_signal_id`)
with `topology_generated.h` (`WireId`s), restricted to the chassis (4000–4197)
and driver-input (6900–6999) ranges, classified from ev1sim's point of view per
the topology convention (a cell ev1sim **produces** omits `producer:`; a cell
ev1sim **solely consumes** has `consumers: []`):

| | count |
|---|---|
| chassis + driver-input cells declared in topology | 166 |
| **ev1sim-touched** | **150** |
| — ev1sim **produces** (no `producer:` AND no `driver:`) | 84 |
| — ev1sim **consumes** (explicit `consumers: []`) | 63 |
| — ev1sim consumes **+ an in-repo IPC telltale** (bulbs 4002/4012/4013) | 3 |
| inter-ECU only (NOT ev1sim) | 16 |

**This revises electricsim's "~170" estimate down to 150.** The larger numbers
in electricsim's doc (163 / 177 / 181 / 185) count inter-ECU cells, the four
already-migrated HV/charge-wake cells, and the 16 deprecated 6900-range aliases
— none of which are ev1sim's to move. Type spread of the 150: 94 `bit`, 35
`float32`, 18 `byte`, 1 `uint32`, 1 `uint16`, 1 `uint64` (the sim-time clock).

**Classification rule (a sharp edge worth stating).** A cell is *ev1sim-produced*
only when it has **neither** `producer:` **nor** `driver:` — `topology.yaml` uses
both forms, and the four earliest-migrated cells (HV bus 4155/4156/4157,
charge-wake 4187) carry a `driver:` (`hv_bus_host` / `lhjb_ecu`), so they are
**electricsim**-produced, not ev1sim's. A cell is *ev1sim-consumed* only with an
**explicit `consumers: []`** (the convention's "only ev1sim reads it"); an
*absent* consumers field (as those 4 driver cells have) means inter-ECU, not
ev1sim. The 16 excluded cells = electricsim's 12 "pure-inter-ECU" triage
(`phase_c_chassis_migration_scope.md` §9.1) + those 4 already-migrated
`driver:` cells — the cells with NO ev1sim end.

---

## 2. The `topology_generated.h` sharing decision — **Strategy C-variant** (consume from `ELECTRICSIM_DIR`)

**Decision: ev1sim compiles the substrate directly from the electricsim tree it
already links** — no vendored copy, no submodule, no second YAML. This is a
variant of the task's option C (consume the substrate from electricsim) realised
through the mechanism ev1sim *already* uses for the connector.

Why: ev1sim's CMake already discovers a sibling/`-DELECTRICSIM_DIR` electricsim
checkout and compiles four of its `src/io/*.cpp` into the `electricsim_connector`
lib (with a stale-path guard and a stub fallback for CI). The wire substrate is
the same shape, so this PR adds an `electricsim_wire_substrate` static lib that
compiles `${ELECTRICSIM_DIR}/src/io/wire_table.cpp` +
`src/io/topology/env_open.cpp` and exposes `src/io` as an include root
(`topology_generated.h` comes along).

Why not the alternatives:
- **A (vendored copy of the header):** the topology hash would silently drift
  from the `wire_table.cpp` ev1sim links the moment electricsim regenerates it —
  exactly the failure the hash gate exists to catch, reintroduced as a build-time
  footgun.
- **B (dual-generated from a copied YAML):** forces ev1sim to carry a copy of
  `topology.yaml` + run `gen_topology_header.py`; same drift surface as A plus a
  toolchain dependency, for no benefit while electricsim is a sibling checkout.
- **C-submodule/FetchContent:** heaviest integration; unnecessary given the
  connector already resolves a sibling tree.

**Drift safety (the whole point):** because ev1sim compiles
`topology_generated.h` *and* links the `wire_table.cpp` from the **same**
`ELECTRICSIM_DIR` tree, the compiled-in `kTopologyHash` is always exactly what
that `wire_table.cpp` enforces. ev1sim cannot attach with a stale hash — a
mismatched checkout fails the attach gate loudly at runtime
(`WireTable::attach: topology hash mismatch …`) rather than corrupting reads.

**Sync procedure (write this down):**
1. electricsim changes `config/topology.yaml` → regenerates
   `topology_generated.h` (its own `gen_topology_header.py --check` CI gate
   enforces this) → the new `kTopologyHash` is committed on the electricsim
   branch ev1sim points at.
2. ev1sim needs **no source change** — its next build against that
   `ELECTRICSIM_DIR` recompiles the header + relinks `wire_table.cpp`, picking up
   the new hash and any new/renamed `kWire…` constants automatically.
3. If a `kWire…` constant ev1sim references by name is **renamed/removed**
   upstream, ev1sim fails to *compile* (caught immediately, like the 4139 case in
   §8) — not silently at runtime. Fix the reference, or file a cross-repo note if
   it's an upstream over-removal.
4. CI's no-electricsim integrated build uses `tests/electricsim_stub`, which has
   no `wire_table.cpp`; there `EV1SIM_HAVE_WIRE_TRUTH` is off and
   `WireTruthChassis` compiles as a disabled no-op, so the gate still runs.

---

## 3. How ev1sim touches the chassis bus **today** (the code being migrated)

`src/ExternalSimConnector.cpp` (~3.8k lines) is ev1sim's sole bus endpoint. It is
NOT a per-signal `publish_frame`/`poll_frame` fan-out — it batches:

- **Producer side (ev1sim → ECU):** state is staged via `SetXxx()` setters
  (`SetPanelSensor`, `SetTurnHazSwOutputs`, `SetVehicleState`, the driver-input
  setters, …) and the per-tick `Tick()` builds `DeltaRecord` batches from
  on-change diffs, then calls `transport->publish_frame(batch)` (the ~15
  `publish_frame` sites are batch flushes for the chassis segment + the main
  segment + heartbeats/presence/signal-define, **not** one per signal). Endpoint
  metadata (id, qualified name, `input_to_sim`) comes from builder lists like the
  `out[i++] = {…}` tables (~L768, ~L1019).
- **Consumer side (ECU → ev1sim):** the `Tick()` drain loop (one `poll_frame`
  per segment, 2 total) reads every `DeltaRecord`, looks it up via
  `FindEndpoint(signal_id)` gated on `ep->input_to_sim`, decodes by type
  (float32 LE / uint32 LE / uint8 / bool), and dispatches into `m_state` fields
  (`horn_low`, `bulb[]`, telltale flags, …) surfaced by `GetXxx()` getters the
  renderer reads.

**Migration shape per direction (mechanical, contract-frozen):**
- *Producer cell:* keep the ring publish, **add** a wire write next to the
  on-change diff (dual-write). Purely additive — in-repo consumers keep reading
  the ring until electricsim flips them, so there is no electricsim dependency to
  start the producer side.
- *Consumer cell:* **overlay** a written()-gated wire read after the drain
  (exactly the horn proof-of-life in §7): wire value wins when its producer has
  written it, else keep the ring value. Dormant-but-correct until electricsim's
  producer moves onto the wire.

---

## 4. Per-signal scope (generated from `topology.yaml` × `topology_generated.h`)

Grouped by functional area. `#` is the numeric `WireId` (order-assigned; ev1sim
never hardcodes it — code resolves the `kWire…` symbol). `dir` is ev1sim's role.

#### Group A — Bulb feeds (ECU->ev1sim) — 17 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4000 | BULB_FEED_LINE_LBL | 73 | bit | ECU->ev1sim (consume) |
| 4001 | BULB_FEED_LINE_RBL | 74 | bit | ECU->ev1sim (consume) |
| 4002 | BULB_FEED_LINE_LHBH | 75 | bit | ECU->ev1sim (+IPC) |
| 4003 | BULB_FEED_LINE_RHBH | 76 | bit | ECU->ev1sim (consume) |
| 4004 | BULB_FEED_LINE_LLBH | 77 | bit | ECU->ev1sim (consume) |
| 4005 | BULB_FEED_LINE_RLBH | 78 | bit | ECU->ev1sim (consume) |
| 4006 | BULB_FEED_LINE_LRSM | 79 | bit | ECU->ev1sim (consume) |
| 4007 | BULB_FEED_LINE_RRSM | 80 | bit | ECU->ev1sim (consume) |
| 4008 | BULB_FEED_LINE_LFML | 81 | bit | ECU->ev1sim (consume) |
| 4009 | BULB_FEED_LINE_RFML | 82 | bit | ECU->ev1sim (consume) |
| 4010 | BULB_FEED_LINE_LFTS | 83 | bit | ECU->ev1sim (consume) |
| 4011 | BULB_FEED_LINE_RFTS | 84 | bit | ECU->ev1sim (consume) |
| 4012 | BULB_FEED_LINE_LRTS | 85 | bit | ECU->ev1sim (+IPC) |
| 4013 | BULB_FEED_LINE_RRTS | 86 | bit | ECU->ev1sim (+IPC) |
| 4014 | BULB_FEED_LINE_LRSL | 87 | bit | ECU->ev1sim (consume) |
| 4015 | BULB_FEED_LINE_CHMSL | 88 | bit | ECU->ev1sim (consume) |
| 4016 | BULB_FEED_LINE_RRSL | 89 | bit | ECU->ev1sim (consume) |

#### Group B — Horn drives (ECU->ev1sim) — 2 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4020 | HORN_DRIVE_LINE_LOW | 90 | bit | ECU->ev1sim (consume) |
| 4021 | HORN_DRIVE_LINE_HIGH | 91 | bit | ECU->ev1sim (consume) |

#### Group C — Panel-ajar discretes (ev1sim->ECU) — 4 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4030 | PANEL_AJAR_HOOD | 92 | bit | ev1sim->ECU (produce) |
| 4031 | PANEL_AJAR_TRUNK | 93 | bit | ev1sim->ECU (produce) |
| 4032 | PANEL_AJAR_DOOR_LEFT | 94 | bit | ev1sim->ECU (produce) |
| 4033 | PANEL_AJAR_DOOR_RIGHT | 95 | bit | ev1sim->ECU (produce) |

#### Group D — Driver stalk/switch outputs (ev1sim->ECU) — 22 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4040 | COMB_SW_LOW_BEAM_OUT | 96 | bit | ev1sim->ECU (produce) |
| 4041 | COMB_SW_FLASH_TO_PASS_OUT | 97 | bit | ev1sim->ECU (produce) |
| 4042 | COMB_SW_PARK_HEADLAMP_OUT | 98 | bit | ev1sim->ECU (produce) |
| 4043 | TURN_HAZ_SW_RIGHT_TURN_OUT | 99 | bit | ev1sim->ECU (produce) |
| 4044 | TURN_HAZ_SW_LEFT_TURN_OUT | 100 | bit | ev1sim->ECU (produce) |
| 4045 | TURN_HAZ_SW_HAZARD_OUT | 101 | bit | ev1sim->ECU (produce) |
| 4046 | TURN_HAZ_SW_HORN_OUT | 102 | bit | ev1sim->ECU (produce) |
| 4047 | CRUISE_SW_SET_COAST_OUT | 103 | bit | ev1sim->ECU (produce) |
| 4048 | CRUISE_SW_RESUME_ACCEL_OUT | 104 | bit | ev1sim->ECU (produce) |
| 4049 | CRUISE_SW_ON_OFF_OUT | 105 | bit | ev1sim->ECU (produce) |
| 4050 | PRND_SELECTOR_A | 106 | bit | ev1sim->ECU (produce) |
| 4051 | PRND_SELECTOR_B | 107 | bit | ev1sim->ECU (produce) |
| 4052 | PRND_SELECTOR_C | 108 | bit | ev1sim->ECU (produce) |
| 4053 | PRND_SELECTOR_D | 109 | bit | ev1sim->ECU (produce) |
| 4054 | WIPER_SW_DELAY_OUT | 110 | bit | ev1sim->ECU (produce) |
| 4055 | WIPER_SW_REQUEST_OUT | 111 | bit | ev1sim->ECU (produce) |
| 4056 | WIPER_SW_HI_OUT | 112 | bit | ev1sim->ECU (produce) |
| 4057 | WIPER_SW_WASHER_SWITCH_OUT | 113 | bit | ev1sim->ECU (produce) |
| 4170 | DOOR_LOCK_SW_LH_LOCK_OUT | 186 | bit | ev1sim->ECU (produce) |
| 4171 | DOOR_LOCK_SW_LH_UNLOCK_OUT | 187 | bit | ev1sim->ECU (produce) |
| 4172 | DOOR_LOCK_SW_RH_LOCK_OUT | 188 | bit | ev1sim->ECU (produce) |
| 4173 | DOOR_LOCK_SW_RH_UNLOCK_OUT | 189 | bit | ev1sim->ECU (produce) |

#### Group F — Motor / brake-master / sim-time (mixed) — 5 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4070 | CHASSIS_MOTOR_RPM | 115 | float32 | ev1sim->ECU (produce) |
| 4071 | CHASSIS_MOTOR_TORQUE_NM | 116 | float32 | ev1sim->ECU (produce) |
| 4072 | CHASSIS_MOTOR_CURRENT_A | 117 | float32 | ECU->ev1sim (consume) |
| 4074 | CHASSIS_BRAKE_MASTER_PRESSURE_KPA | 119 | float32 | ev1sim->ECU (produce) |
| 4075 | CHASSIS_SIM_TIME_NS | 120 | uint64 | ev1sim->ECU (produce) |

#### Group G — ECU actuator/cmd outputs ev1sim renders (ECU->ev1sim) — 9 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4080 | CHASSIS_WIPER_MOTOR_COMMAND | 121 | byte | ECU->ev1sim (consume) |
| 4081 | CHASSIS_WASHER_PUMP_COMMAND | 122 | bit | ECU->ev1sim (consume) |
| 4082 | CHASSIS_HVAC_BLOWER_LEVEL | 123 | byte | ECU->ev1sim (consume) |
| 4083 | CHASSIS_DEFROST_GRID_ACTIVE | 124 | bit | ECU->ev1sim (consume) |
| 4084 | CHASSIS_DOOR_LOCK_CMD_DRIVER | 125 | byte | ECU->ev1sim (consume) |
| 4085 | CHASSIS_DOOR_LOCK_CMD_PASSENGER | 126 | byte | ECU->ev1sim (consume) |
| 4086 | CHASSIS_POWER_WINDOW_MOTOR_DRIVER | 127 | byte | ECU->ev1sim (consume) |
| 4087 | CHASSIS_POWER_WINDOW_MOTOR_PASSENGER | 128 | byte | ECU->ev1sim (consume) |
| 4088 | CHASSIS_RSA_SHIFT_BLOCKED | 129 | bit | ECU->ev1sim (consume) |

#### Group H — Ambient sensors (ev1sim->ECU) — 2 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4090 | CHASSIS_AMBIENT_TEMP_C | 130 | float32 | ev1sim->ECU (produce) |
| 4091 | CHASSIS_AMBIENT_HUMIDITY_PCT | 131 | float32 | ev1sim->ECU (produce) |

#### Group I — Vehicle dynamics (ev1sim->ECU) — 18 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4100 | CHASSIS_SPEED_MPS | 132 | float32 | ev1sim->ECU (produce) |
| 4101 | CHASSIS_ACCEL_LONG | 133 | float32 | ev1sim->ECU (produce) |
| 4102 | CHASSIS_ACCEL_LAT | 134 | float32 | ev1sim->ECU (produce) |
| 4103 | CHASSIS_YAW_RATE | 135 | float32 | ev1sim->ECU (produce) |
| 4104 | CHASSIS_APPLIED_THROTTLE | 136 | float32 | ev1sim->ECU (produce) |
| 4105 | CHASSIS_APPLIED_FRONT_BRAKE | 137 | float32 | ev1sim->ECU (produce) |
| 4106 | CHASSIS_APPLIED_REAR_BRAKE | 138 | float32 | ev1sim->ECU (produce) |
| 4107 | CHASSIS_FRONT_BRAKE_PRESSURE | 139 | float32 | ev1sim->ECU (produce) |
| 4108 | CHASSIS_REAR_BRAKE_POSITION | 140 | float32 | ev1sim->ECU (produce) |
| 4109 | CHASSIS_STEERING_TORQUE | 141 | float32 | ev1sim->ECU (produce) |
| 4110 | CHASSIS_WHEEL_OMEGA_FL | 142 | float32 | ev1sim->ECU (produce) |
| 4111 | CHASSIS_WHEEL_OMEGA_FR | 143 | float32 | ev1sim->ECU (produce) |
| 4112 | CHASSIS_WHEEL_OMEGA_RL | 144 | float32 | ev1sim->ECU (produce) |
| 4113 | CHASSIS_WHEEL_OMEGA_RR | 145 | float32 | ev1sim->ECU (produce) |
| 4120 | CHASSIS_SLIP_RATIO_FL | 146 | float32 | ev1sim->ECU (produce) |
| 4121 | CHASSIS_SLIP_RATIO_FR | 147 | float32 | ev1sim->ECU (produce) |
| 4122 | CHASSIS_SLIP_RATIO_RL | 148 | float32 | ev1sim->ECU (produce) |
| 4123 | CHASSIS_SLIP_RATIO_RR | 149 | float32 | ev1sim->ECU (produce) |

#### Group J — IPC telltales / LCD-derived (ECU->ev1sim) — 27 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4130 | CHASSIS_IPC_SEATBELT_TELLTALE_DRIVER | 150 | bit | ECU->ev1sim (consume) |
| 4131 | CHASSIS_IPC_SEATBELT_TELLTALE_PASSENGER | 151 | bit | ECU->ev1sim (consume) |
| 4132 | CHASSIS_IPC_TRIP_DISTANCE_M | 152 | float32 | ECU->ev1sim (consume) |
| 4134 | CHASSIS_IPC_BRAKE_TELLTALE | 153 | bit | ECU->ev1sim (consume) |
| 4135 | CHASSIS_IPC_PARK_BRAKE_TELLTALE | 154 | bit | ECU->ev1sim (consume) |
| 4136 | CHASSIS_IPC_ANTILOCK_TELLTALE | 155 | bit | ECU->ev1sim (consume) |
| 4138 | CHASSIS_IPC_AIR_BAG_TELLTALE | 157 | bit | ECU->ev1sim (consume) |
| 4140 | CHASSIS_IPC_SERVICE_NOW_TELLTALE | 158 | bit | ECU->ev1sim (consume) |
| 4141 | CHASSIS_IPC_CHECK_MESSAGES_TELLTALE | 159 | bit | ECU->ev1sim (consume) |
| 4142 | CHASSIS_IPC_TEMP_TELLTALE | 160 | bit | ECU->ev1sim (consume) |
| 4143 | CHASSIS_IPC_BATTERY_LIFE_TELLTALE | 161 | bit | ECU->ev1sim (consume) |
| 4144 | CHASSIS_IPC_REDUCED_PERF_TELLTALE | 162 | bit | ECU->ev1sim (consume) |
| 4145 | CHASSIS_IPC_CHECK_TIRE_PRESS_TELLTALE | 163 | bit | ECU->ev1sim (consume) |
| 4147 | CHASSIS_BTCM_ISO_CLOSE_FL | 171 | bit | ECU->ev1sim (consume) |
| 4148 | CHASSIS_BTCM_ISO_CLOSE_FR | 172 | bit | ECU->ev1sim (consume) |
| 4149 | CHASSIS_BTCM_DUMP_OPEN_FL | 173 | bit | ECU->ev1sim (consume) |
| 4150 | CHASSIS_BTCM_DUMP_OPEN_FR | 174 | bit | ECU->ev1sim (consume) |
| 4151 | CHASSIS_BTCM_EMB_MOTOR_CMD_LR | 175 | float32 | ECU->ev1sim (consume) |
| 4152 | CHASSIS_BTCM_EMB_MOTOR_CMD_RR | 176 | float32 | ECU->ev1sim (consume) |
| 4153 | CHASSIS_BTCM_CYL_PRESSURE_FL_K_PA | 177 | float32 | ECU->ev1sim (consume) |
| 4154 | CHASSIS_BTCM_CYL_PRESSURE_FR_K_PA | 178 | float32 | ECU->ev1sim (consume) |
| 4158 | CHASSIS_IPC_LEFT_TURN_TELLTALE | 164 | bit | ECU->ev1sim (consume) |
| 4159 | CHASSIS_IPC_RIGHT_TURN_TELLTALE | 165 | bit | ECU->ev1sim (consume) |
| 4160 | CHASSIS_IPC_HIGH_BEAM_TELLTALE | 166 | bit | ECU->ev1sim (consume) |
| 4161 | CHASSIS_IPC_PARK_LAMP_TELLTALE | 167 | bit | ECU->ev1sim (consume) |
| 4162 | CHASSIS_IPC_DOOR_AJAR_TELLTALE | 168 | bit | ECU->ev1sim (consume) |
| 4163 | CHASSIS_IPC_DIM_DUTY_PCT | 169 | byte | ECU->ev1sim (consume) |

#### Group K — Door-lock state / road grade / pitch (ev1sim->ECU) — 5 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4165 | CHASSIS_DOOR_LOCK_STATE_DRIVER | 181 | bit | ev1sim->ECU (produce) |
| 4166 | CHASSIS_DOOR_LOCK_STATE_PASSENGER | 182 | bit | ev1sim->ECU (produce) |
| 4167 | CHASSIS_DOOR_LOCK_STATE_TRUNK | 183 | bit | ev1sim->ECU (produce) |
| 4168 | CHASSIS_ROAD_GRADE_PCT | 184 | float32 | ev1sim->ECU (produce) |
| 4169 | CHASSIS_PITCH_DEG | 185 | float32 | ev1sim->ECU (produce) |

#### Group L — RHJB sub-modules + inter-ECU (ECU->ev1sim) — 10 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 4180 | CHASSIS_RHJB_PMM_RUN1_BUS | 190 | bit | ECU->ev1sim (consume) |
| 4181 | CHASSIS_RHJB_PMM_RUN2_BUS | 191 | bit | ECU->ev1sim (consume) |
| 4182 | CHASSIS_RHJB_DLM_LH_LOCK | 192 | bit | ECU->ev1sim (consume) |
| 4183 | CHASSIS_RHJB_DLM_LH_UNLOCK | 193 | bit | ECU->ev1sim (consume) |
| 4184 | CHASSIS_RHJB_DLM_RH_LOCK | 194 | bit | ECU->ev1sim (consume) |
| 4185 | CHASSIS_RHJB_DLM_RH_UNLOCK | 195 | bit | ECU->ev1sim (consume) |
| 4186 | CHASSIS_RHJB_DILM_LEVEL | 196 | byte | ECU->ev1sim (consume) |
| 4192 | CHASSIS_AUX_BATTERY_TERMINAL_MV | 201 | uint32 | ECU->ev1sim (consume) |
| 4193 | CHASSIS_AUX_BATTERY_PRESENT | 202 | bit | ECU->ev1sim (consume) |
| 4194 | CHASSIS_AUX_BATTERY_SOC_PCT | 203 | byte | ECU->ev1sim (consume) |

#### Group M — Driver inputs (main segment) (ev1sim->ECU) — 29 cells

| sig | WireId `kWire…` | # | type | dir |
|--|--|--|--|--|
| 6900 | DRIVER_BRAKE_PEDAL_Q8 | 206 | byte | ev1sim->ECU (produce) |
| 6901 | DRIVER_STEERING_DEG_Q8 | 207 | uint16 | ev1sim->ECU (produce) |
| 6902 | DRIVER_GEAR_SELECTOR | 208 | byte | ev1sim->ECU (produce) |
| 6903 | DRIVER_THROTTLE_Q8 | 209 | byte | ev1sim->ECU (produce) |
| 6904 | DRIVER_BRAKE_SWITCH | 210 | bit | ev1sim->ECU (produce) |
| 6910 | EXT_WHEEL_SPEED_FL | 213 | float32 | ev1sim->ECU (produce) |
| 6911 | EXT_WHEEL_SPEED_FR | 214 | float32 | ev1sim->ECU (produce) |
| 6912 | EXT_WHEEL_SPEED_RL | 215 | float32 | ev1sim->ECU (produce) |
| 6913 | EXT_WHEEL_SPEED_RR | 216 | float32 | ev1sim->ECU (produce) |
| 6952 | DRIVER_IPC_TRIP_RESET_BUTTON | 217 | bit | ev1sim->ECU (produce) |
| 6964 | DRIVER_SEATBELT_BUCKLED | 211 | bit | ev1sim->ECU (produce) |
| 6965 | DRIVER_SEATBELT_BUCKLED_PASSENGER | 212 | bit | ev1sim->ECU (produce) |
| 6971 | DRIVER_RSA_MODE_BUTTON | 218 | byte | ev1sim->ECU (produce) |
| 6975 | DRIVER_RSA_KEYPAD_BUTTON1 | 219 | bit | ev1sim->ECU (produce) |
| 6976 | DRIVER_RSA_KEYPAD_BUTTON2 | 220 | bit | ev1sim->ECU (produce) |
| 6977 | DRIVER_RSA_KEYPAD_BUTTON3 | 221 | bit | ev1sim->ECU (produce) |
| 6978 | DRIVER_RSA_KEYPAD_BUTTON4 | 222 | bit | ev1sim->ECU (produce) |
| 6979 | DRIVER_RSA_KEYPAD_BUTTON5 | 223 | bit | ev1sim->ECU (produce) |
| 6980 | DRIVER_POWER_WINDOW_DRIVER_UP | 224 | bit | ev1sim->ECU (produce) |
| 6981 | DRIVER_POWER_WINDOW_DRIVER_DOWN | 225 | bit | ev1sim->ECU (produce) |
| 6982 | DRIVER_POWER_WINDOW_PASSENGER_UP | 226 | bit | ev1sim->ECU (produce) |
| 6983 | DRIVER_POWER_WINDOW_PASSENGER_DOWN | 227 | bit | ev1sim->ECU (produce) |
| 6985 | DRIVER_RSA_EXTERIOR_KEYPAD1 | 228 | byte | ev1sim->ECU (produce) |
| 6986 | DRIVER_RSA_EXTERIOR_KEYPAD2 | 229 | byte | ev1sim->ECU (produce) |
| 6987 | DRIVER_RSA_EXTERIOR_KEYPAD3 | 230 | byte | ev1sim->ECU (produce) |
| 6988 | DRIVER_RSA_EXTERIOR_KEYPAD4 | 231 | byte | ev1sim->ECU (produce) |
| 6989 | DRIVER_RSA_EXTERIOR_KEYPAD5 | 232 | byte | ev1sim->ECU (produce) |
| 6990 | DRIVER_DOOR_HANDLE_ATTEMPT_DRIVER | 233 | bit | ev1sim->ECU (produce) |
| 6991 | DRIVER_DOOR_HANDLE_ATTEMPT_PASSENGER | 234 | bit | ev1sim->ECU (produce) |

---

## 5. Per-group execution plan + effort (ev1sim side)

ev1sim's per-cell work is mechanical and contract-frozen: **producer** cells get
a dual-write (ring + wire) next to their on-change diff; **consumer** cells get a
written()-gated wire overlay after the drain (the §7 horn pattern). The seam
(`src/WireTruthChassis.{h,cpp}`) and the attach/test scaffolding already exist,
so every group below is "add cells + a test", not new infrastructure.

The **gating asymmetry** (electricsim `phase_c_chassis_migration_scope.md` §9):
- *Producer* groups have **no electricsim dependency** — ev1sim writing the wire
  is purely additive; in-repo consumers keep reading the ring until electricsim
  flips them. These can land immediately and are the recommended next work.
- *Consumer* groups are **dormant until electricsim's producer writes the wire**
  (its Phase C-b). ev1sim can wire them now (written()-gated, so a no-op until
  then), but the live payoff is gated cross-repo. Wiring them early is still
  valuable: it's correct, tested, and "lights up" automatically when C-b lands.

| group | cells | ev1sim role | electricsim dep? | effort | notes |
|---|---|---|---|---|---|
| **B Horn** | 2 | consume | C-b Batch B | **DONE** (§7) | proof-of-life |
| C Panel-ajar | 4 | produce | none | ~0.5 h | dual-write; RHJB DLM consumes 4032/33 |
| D Driver stalk/switch | 22 | produce | none | ~2 h | combo/turn-haz/cruise/PRND/wiper/door-lock-sw |
| H Ambient | 2 | produce | none | ~0.5 h | no in-repo consumer yet (declare+write) |
| I Vehicle dynamics | 18 | produce | none | ~2.5 h | all float32; BTCM/PIM/IPC consume |
| K Door-state/grade/pitch | 5 | produce | none | ~1 h | mostly future consumers |
| M Driver inputs (main seg) | 29 | produce | none | ~3 h | RSA keypad/window heavy; main-segment WireBank |
| F Motor/brake/sim-time | 5 | mixed | partial | ~1.5 h | 4072 consume (PIM); 4075 is uint64 sim-time |
| A Bulb feeds | 17 | consume | C-b Batch A | ~1.5 h | dormant until LHJB→wire; 3 also feed IPC |
| G ECU actuator outputs | 9 | consume | C-b (rhjb/htcm/rsa) | ~1.5 h | wiper/hvac/doorlock/window render |
| J IPC telltales / LCD | 27 | consume | C-b Batch M | auto | in scope; overlay already reads them. Only the LCD *visual* render is deferred ~weeks (§8) |
| L RHJB sub-modules | 10 | consume | C-b Batch C | ~2 h | PMM/DLM/DILM render |

**Update (2026-06-15): BOTH sides are mechanism-complete — the effort column
above is now historical.** The table-driven `mirror_signal` dual-write covers
all **84** producer cells, and `apply_consumer_overlay` covers all **66**
consumer cells (every row in the table above). So there is **no per-batch ev1sim
code left** for any group: as electricsim moves each producer onto the wire,
ev1sim's overlay reads it automatically. The horn (electricsim Batch B) is the
first live cell and is verified end-to-end (see §7).

**Remaining ev1sim effort is now just per-batch verification (~0.5 h each)** as
electricsim lands its producer batches, plus the eventual ring cutover (delete
ev1sim's legacy ring publish/poll once every cell is wire-authoritative and
soaked — a coordinated late step, not yet). Group J / Batch M (IPC telltales +
LCD) IS in scope and migrates like any other batch — ev1sim's overlay already
reads those cells; only the separate ev1sim *visual* work (rendering the LCD
segments to a 3D/2D panel) is deferred ~weeks (§8).

---

## 6. Risks / things that don't fit the simple ring→wire swap

1. **Consumer payoff is gated on electricsim (the big one).** Every consumer
   group is a dormant no-op until electricsim's matching C-b producer batch
   lands. ev1sim can't "finish" bulbs/horn/telltales/actuators alone. Track each
   consumer group against its electricsim batch (§5 column).
2. **The horn producer isn't on the wire yet.** The proof-of-life (§7) is
   verified by an in-process round-trip; in the *live* fleet it's a no-op until
   electricsim Batch B (LHJB→wire) lands. This is expected, not a defect.
3. **Multi-producer / bidirectional cells.** `kSigChargerCouplerPresent` (4060)
   is produced by BOTH ev1sim (operator UI) and electricsim's charger demo, so
   it classifies as inter-ECU here and is *excluded* from the clean 150 — it
   needs an explicit multi-writer policy (electricsim §6.2) before ev1sim writes
   it. The door-lock cmd (4084/85, ECU→ev1sim) vs door-lock *state* (4165/66,
   ev1sim→ECU) pair moves in lock-step across two IDs with an ordering
   assumption (cmd write should precede state write) — keep them in one batch.
4. **sim-time clock (4075) is `uint64`.** Only ev1sim produces it; every run-mode
   controller consumes it via `SimClock`. It's a `WireType::kUint64` cell (the
   one non-standard type). `WireTruthChassis` currently exposes only `bit`
   accessors — extend it with `write_uint64`/`read_uint64` for this cell. Treat
   4075 as its own mini-batch given the fan-out.
5. **Cadence/heartbeat semantics differ.** The ring relies on on-change +
   periodic heartbeat re-publish (e.g. the 200 ms brake heartbeat, the 1 Hz
   latched-button re-assert) so late joiners converge. The WireTable is
   latest-value-wins with a generation counter — a late joiner reads the current
   value immediately, so the heartbeat re-publish becomes redundant for wire
   cells. Don't delete the ring heartbeats while dual-writing (ring consumers
   still need them); revisit only at ring-retirement.
6. **Two WireBank instances.** Driver inputs (6900-range, Group M) ride the
   **main** harness instance; the 4xxx chassis cells ride the **chassis**
   instance (electricsim topology header notes this). They share one
   `topology_hash`, but confirm ev1sim attaches the correct segment(s) for the
   cells it touches — Group M may need a second attach. `try_open_from_env`
   handles one named segment; a second `WireTruthChassis` (or a multi-segment
   variant) may be needed for the main-segment driver-input cells.
7. **Render-thread attach latency.** `OpenFromEnv` → `try_open_from_env` can poll
   up to `ELECTRICSIM_WIRES_ATTACH_TIMEOUT_MS` (default 2 s) for the segment.
   ev1sim attaches it once, lazily, after the ring connects (so the producer is
   up); standalone ev1sim (env unset) returns instantly. If a 2 s one-time stall
   is unacceptable on the render thread, lower the timeout for ev1sim.
8. **bit-vs-byte-coerced dispatch (found + fixed during Batch K/L verification).**
   The overlay routes a cell by its WIRE type, but the connector's two ring-decode
   tables cover different signal_ids: `DebugInjectDelta` handles the "true bit"
   signals (horn, bulbs, washer, solenoids), while a set of *bit*-typed cells the
   ring delivered as 1-byte deltas — HVAC defrost (4083), RSA shift-blocked
   (4088), and the IPC telltales (4130-4145, Batch M) — are served only by
   `DebugInjectU8`. So a `bit` wire routed to `DebugInjectDelta` alone silently
   dropped. Fixed by having the connector's `on_bit` sink dispatch to BOTH
   (`DebugInjectDelta` + `DebugInjectU8`); the two tables partition the ids (no
   shared id sets a different field; neither has a default), so the non-owning
   call is a safe no-op. This already covers the Batch M IPC telltales (in scope;
   ~weeks out) so they route correctly when electricsim migrates them. Verified
   by the `[e2e]` Batch A/J/K/L test.

---

## 7. Proof-of-life (this session) — the horn drive lines (Group B)

Implemented and verified end-to-end **without touching electricsim**:

- **Build seam:** `electricsim_wire_substrate` static lib (CMake) compiles
  `wire_table.cpp` + `topology/env_open.cpp` from `ELECTRICSIM_DIR`; gated by
  `EV1SIM_HAVE_WIRE_TRUTH`, with a no-op stub when absent (CI-safe).
- **Attach seam:** `src/WireTruthChassis.{h,cpp}` — a PIMPL wrapper (header is
  electricsim-free) that attaches the shared `WireTable` via
  `topology::try_open_from_env("attacher")` (same env contract as every
  electricsim controller: `ELECTRICSIM_WIRES_NAME`/`_ROLE`, hash =
  `kTopologyHash`). Typed reads honour `written()`: `std::nullopt` ⇒ fall back to
  the ring.
- **Consumer migration:** `ExternalSimConnector::Tick()` overlays
  `horn_low_drive()`/`horn_high_drive()` (cells `kWireHORN_DRIVE_LINE_LOW`/`_HIGH`
  = chassis 4020/4021) onto `m_state->horn_low/high` after the ring drain — wire
  wins when written, else ring value retained.
- **Round-trip test:** `tests/test_wire_truth_chassis.cpp` plays both cross-repo
  roles in-process against the **real** generated topology: creates the table
  (creator path, `declare_all` + the canonical hash), writes the horn cells,
  attaches through `WireTruthChassis`, asserts the values match; asserts a
  never-written cell reads `nullopt` (fallback); asserts the producer-write
  direction; asserts a **topology-hash mismatch refuses to attach**
  (`segment=0x27469F02 vs expected=0x27469F03`) and a missing segment returns
  nullptr.

**Verification:** `ev1sim_tests` builds and passes **475/475** against the real
electricsim substrate (`ELECTRICSIM_DIR=…/electricsim`, branch
`claude/wire-truth-retire-legacy`), and **470/470** in the no-substrate stub
build (wire-truth compiled out). Topology hash matched exactly (no drift).

A *live* fleet round-trip (electricsim driver writes 4020 → ev1sim renders horn)
additionally requires electricsim Batch B (LHJB→wire); see §8.

---

## 8. Cross-repo follow-ups (NOT ev1sim blockers — note, don't execute here)

1. **electricsim Batch B (LHJB horn → wire)** lights up the §7 path live. Until
   then ev1sim's horn overlay is dormant-correct. (electricsim
   `phase_c_chassis_migration_scope.md` Batch B, ~0.5 h there.)
2. **4139 / `kSigChassisBpmPackVoltageMv` — verify the upstream removal.**
   electricsim *dropped* 4139 ("retire-not-migrate", "no ev1sim contract use"),
   which broke ev1sim's build against the wire-truth branch (ev1sim subscribed to
   it for the floating-UI pack-voltage readout). This session dropped only the
   cross-repo drift-guard line (build unblocked; ev1sim's plumbing left intact
   but inert). **Open question for electricsim:** was 4139 over-removed (dropped
   rather than migrated to a wire cell)? Confirm the intent.
3. **Battery-voltage display direction (future, do not design now).** Per
   maintainer guidance, ev1sim should *not* carry its own pack-voltage signal at
   all: that display value moves to on-module signals (the **IPC LCD**). This
   is about the *pack-voltage readout*, NOT the telltale cells.

   **Clarification (2026-06-16): Group J / Batch M IS in scope for the wire
   migration.** The IPC telltale + LCD segments exist and migrate onto the wire
   like any other batch; ev1sim's consumer overlay already reads those cells
   (they are in the 66-cell registry, and the `on_bit` byte-coerced-bool fix in
   §6.8 routes them correctly). What is **deferred** (~weeks out) is purely the
   ev1sim-side **visual** work — rendering the LCD segments to a 3D model / 2D
   panel as a "device." That render-design effort is independent of the wire
   plumbing, which is already done on ev1sim's side.
4. **Driver-input main-segment attach (Group M).** Confirm whether the 6900-range
   cells need a second `WireBank`/segment attach on ev1sim's side (§6.6).

---

## 9. Suggested sequencing

1. **Producer groups first** (no cross-repo dep): C → H → K → D → I → M, then the
   F producer cells. ~84 cells, ~11 h, each a dual-write + test. Immediately
   useful: electricsim consumers can flip to the wire as they're ready.
2. **sim-time (4075)** as its own step once `WireTruthChassis` grows uint64
   accessors (§6.4).
3. **Consumer groups** (A, G, L, B-already-done) as electricsim's C-b producer
   batches land — wire them written()-gated ahead of time so they light up
   automatically.
4. **Group J / Batch M (IPC telltales + LCD)** migrates like any other batch
   (in scope); ev1sim's overlay already consumes those cells. Only the ev1sim
   *visual* LCD render (3D/2D panel) is deferred ~weeks (§8.3).
5. **Multi-producer cells** (4060 coupler; door-lock cmd/state pair) last, with
   an explicit policy.
6. Ring publishes stay live throughout (ev1sim is the consumer that keeps the
   ring populated for any remaining ring-only peers); ring retirement is a later,
   coordinated step gated on ev1sim fully moving onto the substrate.
