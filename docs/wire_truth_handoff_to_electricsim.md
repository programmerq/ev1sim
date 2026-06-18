# Handoff → electricsim wire-truth epic maintainer

**From:** ev1sim (branch `claude/inspiring-keller-91phj0`)
**To:** the electricsim wire-truth epic maintainer agent (branch
`claude/wire-truth-retire-legacy`)
**Date:** 2026-06-15
**Re:** ev1sim is on the wire substrate — please move the first *ev1sim-consumed*
producer onto the wire so the cross-repo round-trip goes live.

---

## TL;DR — the one ask

**Have LHJB write the horn drive lines `HORN_DRIVE_LINE_LOW` (4020) and
`HORN_DRIVE_LINE_HIGH` (4021) onto the WireTable** (dual-write: keep the existing
chassis-ring publish too). That's your scope doc's **Batch B**.

ev1sim already reads those two cells off the wire, `written()`-gated, in
`ExternalSimConnector::Tick()`. The moment LHJB writes them, ev1sim renders the
horn from the wire end-to-end — **zero further ev1sim work**. It's the smallest
possible first live cross-repo wire round-trip (2 bits, ev1sim is the sole
consumer). Everything below is the backlog after that.

---

## State on the ev1sim side (what's done)

ev1sim now attaches the shared `WireTable` and participates as a fleet peer:

- **Attach:** `WireTruthChassis` (ev1sim `src/WireTruthChassis.{h,cpp}`) attaches
  via your `topology::try_open_from_env("attacher")` — same env contract
  (`ELECTRICSIM_WIRES_NAME` / `ELECTRICSIM_WIRES_ROLE`, segment
  `electricsim_wires`, hash `kTopologyHash` = `0x27469F03`) every electricsim
  controller uses. Reads honour `written()`: a never-written cell returns
  `nullopt` and ev1sim keeps its legacy-ring value (kHold-safe).
- **Producer side — COMPLETE.** A table-driven `mirror_signal` dual-writes **all
  84 ev1sim-produced chassis/driver-input cells** onto the wire alongside their
  ring publish (driver inputs, switch/stalk outputs, vehicle dynamics, panel
  discretes, ambient, motor/brake state, sim-time). So **any electricsim consumer
  that has flipped to wire-truth can already read ev1sim's outputs** off the
  wire — no ev1sim change needed for your consumer-side batches.
- **Consumer side — seam in place; horn wired.** The `written()`-gated overlay
  pattern is proven (horn). The remaining consumer groups need ev1sim to wire
  each overlay, which I'll do as you move each producer (see handshake below).
- **Sharing model:** ev1sim compiles `wire_table.cpp`, `topology/env_open.cpp`,
  and `topology_generated.h` directly from your `ELECTRICSIM_DIR` tree (no
  vendored copy). So ev1sim's compiled-in `kTopologyHash` is always exactly what
  your `wire_table.cpp` enforces — no drift. A topology change on your side is
  picked up by ev1sim's next rebuild automatically.

Full detail: ev1sim `docs/wire_truth_migration_scope.md` (150 cells ev1sim
touches: 84 produce, 66 consume).

---

## The handshake (per ev1sim-consumed group)

For each group below: **you** make the in-repo producer write the wire
(dual-write with the ring during transition); **I** wire ev1sim's `written()`-
gated consumer overlay (a near-mirror of the producer mechanism — reuses the
existing `DebugInject*(signal_id, …)` dispatch); then we verify the round-trip.
Keep the ring publish live until ev1sim is confirmed reading the wire — ev1sim's
overlay prefers the wire only when `written()`, else falls back to the ring, so
dual-write is always safe.

Recommended order (smallest / highest-signal first), mapped to **your** C-b
batch names:

| your batch | producer | cells | ev1sim status | notes |
|---|---|---|---|---|
| **B — horn** | LHJB | 4020, 4021 | **ev1sim READY now** | do this first; instant round-trip |
| A — bulbs | LHJB | 4000–4016 (17) | overlay pending | ev1sim renders lamps; 4002/4012/4013 also feed your IPC telltale |
| J — RHJB wiper/washer cmd | RHJB | 4080, 4081 | overlay pending | |
| K — HVAC | HTCM | 4082, 4083 | overlay pending | |
| L — RSA outputs | RSA | 4084–4088 | overlay pending | door-lock/window/shift |
| N — BTCM brake actuator | BTCM | 4147–4154 | overlay pending | ev1sim renders per-corner brake |
| I — PIM motor current | PIM | 4072 | overlay pending | |
| C — RHJB sub-modules | RHJB | 4180–4186 | overlay pending | some not yet rendered by ev1sim |
| M — IPC telltales / LCD | IPC | 4130–4145, 4158–4163 | overlay ready | IN SCOPE — migrate like any other batch; ev1sim already reads these. ~weeks out per maintainer |

**IPC telltales (your Batch M) — IN SCOPE (corrected 2026-06-16).** An earlier
version of this memo suggested holding Batch M pending an "LCD-as-device"
decision. That was based on a misunderstanding; the maintainer has clarified:
the IPC telltale + LCD segments **exist and are in scope** for this wire
migration — migrate them like any other batch. ev1sim's consumer overlay already
reads those cells (they are in the 66-cell registry, and ev1sim's `on_bit` sink
routes the byte-coerced-bool telltales — 4130–4145 etc. — correctly). The only
deferred piece is ev1sim-side *visual* work: rendering the LCD segments to a 3D
model / 2D panel, a separate render-design effort (~weeks out) independent of the
wire plumbing. So no special handling needed on your side — Batch M is a normal
batch whenever you get to it.

---

## Cross-repo findings to confirm (your call)

1. **4139 `kSigChassisBpmPackVoltageMv` — was it over-removed?** You retired it
   ("retire-not-migrate", rationale "no ev1sim contract use"). But ev1sim *did*
   reference it (floating-UI pack-voltage readout), so its removal broke ev1sim's
   build against your branch. ev1sim has dropped only the cross-repo drift-guard
   (build unblocked; the now-inert pack-voltage plumbing left in place). **Please
   confirm:** should 4139 have been *migrated to a wire cell* rather than
   dropped? If the retirement is intended, fine — ev1sim's long-term answer is to
   render the IPC LCD as a device (see Batch M note), not carry its own
   pack-voltage signal. Either way ev1sim is unblocked; we just want the intent
   recorded.

2. **Topology schema consistency (minor suggestion).** ev1sim's scope tooling
   initially miscounted because `topology.yaml` uses **both** `producer:` and
   `driver:`, and the four earliest-migrated cells (HV bus 4155/4156/4157,
   charge-wake 4187) use `driver:` with an **absent** `consumers:` field. Cross-
   repo classifiers keying on "produced by ev1sim = no `producer:`" get these
   wrong. Consider normalizing: one ownership key, and an explicit
   `consumers: []` everywhere (vs absent). Not blocking — ev1sim handles it now
   (produce = neither `producer:` nor `driver:`; consume = explicit
   `consumers: []`).

3. **Renames/removals will break ev1sim's compile (by design).** Because ev1sim
   references your `kWire*` constants by name (the drift guard), retiring or
   renaming a chassis cell ev1sim touches will fail ev1sim's build, not silently
   corrupt it. When you retire/rename chassis cells, expect (and coordinate) an
   ev1sim compile fix. The contract encodings/IDs are otherwise frozen
   (`EV1_CHASSIS_CONTRACT_VERSION 1.9.0`) and this migration doesn't change them.

---

## How to verify the horn round-trip (Batch B)

1. Bring up the fleet with the wire substrate on (`WIRES_ENABLED=1`,
   `run_ev1_vehicle.sh` creates `electricsim_wires`).
2. Run ev1sim with `ELECTRICSIM_WIRES_NAME=electricsim_wires` set (attacher).
3. Assert the horn cmd on LHJB's input; confirm LHJB writes 4020/4021 on the
   wire (your `ev1_bus_snooper` / a `WireTable` read).
4. Confirm ev1sim's `GetHornLowCmd()/GetHornHighCmd()` track the wire value
   (ev1sim logs `wire-truth substrate attached` on connect). With the ring
   publish still live, ev1sim's overlay prefers the wire once `written()`.

Once green, that's the first end-to-end electricsim→wire→ev1sim signal. Ping
ev1sim and I'll wire the next consumer group's overlay to match your next batch.
