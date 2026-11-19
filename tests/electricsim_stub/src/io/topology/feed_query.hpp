// src/io/topology/feed_query.hpp — THE one implementation of "am I powered".
//
// Plan section 7.4: *"the count of distinct IMPLEMENTATIONS of 'am I powered' goes to 1.
// The count of FEED QUERIES goes to the number of tiers the module physically has. The
// shared header's signature is `energised(feed_node)`, called once per tier."*
//
// This is that header. It is four lines of code and a page of reasons, because the four
// lines are not the hard part — every one of the nine implementations LHJB carried before
// pass 1 was also four lines, and they disagreed with each other about the only question
// that matters.
//
// ── THE ONLY QUESTION THAT MATTERS: WHAT DOES UNWRITTEN MEAN? ──────────────────────────
//
// It means DARK. Not "powered", not "hold the last value", not "assume the producer means
// well". A conductor cell is published by the solver and by nothing else, so an unwritten
// conductor cell means THE SOLVER HAS NOT SPOKEN — no graph was loaded, no solve ran, or
// the observation failed validation and the host refused to publish. None of those is
// evidence that a wire is hot.
//
// The deleted `src/models/battery/switched_feed_gate.hpp:106` answered that question
// `return true;`, and `power_gate.hpp:80` answered it the same way, and
// `ev1/lhjb/controller.cpp:672` answered it `? latch : true`. Plan section 1 is the census
// of how many places in this tree answered "absent means powered". This function is the
// one place the answer now lives, and it is the other one.
//
// ── WHY THERE IS NO DEFAULT PARAMETER ──────────────────────────────────────────────────
//
// There is no `energised(t, feed, bool fallback)` overload and there must never be one.
// A fallback parameter moves the decision back to the call site, which is exactly the
// arrangement that produced nine implementations in one module. Plan section 4.1 says the
// same thing about `read_bit_or_default(ConductorId, ...)`: *"not 'for symmetry', not
// 'for the tests', not 'just for the transition'."*
//
// ── WHY IT TAKES A ConductorId AND NOT A WireId ────────────────────────────────────────
//
// Because then it cannot be pointed at a cell some module writes. A feed query answers
// "what did the solver compute", and the type is what guarantees that the thing on the
// other end is a solver-published cell (plan section 4.1's type-split). Passing a plain
// WireId here does not compile.

#pragma once

#include <cstddef>
#include <cstdint>

#include "../wire_table.hpp"

namespace electricsim {
namespace io {

// Is this feed node energised, as computed by the solver this tick?
//
// FALSE when: the cell is not declared or is the wrong type; the solver has never
// published it; or the solver published it dark. Those are three different situations and
// they collapse to one answer on purpose — a module has nothing useful to do differently
// in any of them, and giving it a way to tell them apart is giving it a way to special-case
// the first two. `energised_detail()` below exists for DIAGNOSTICS, where telling them
// apart is the whole point.
inline bool energised(const WireTable& table, ConductorId feed) {
  WireTable::Sample<bool> s;
  if (!table.read_bit_sample(feed, &s)) return false;
  if (!s.written()) return false;
  return s.value;
}

// The diagnostic form. Not for control flow: nothing that decides whether a lamp lights
// may branch on `never_published`, because "the solver has not spoken" is not a licence
// to proceed. It exists so a log line can say WHY a module is dark, which is the
// difference between a fleet you can debug and one that is simply off.
struct FeedReading {
  bool energised = false;
  bool never_published = false;  // the solver has not published this cell at all
  bool undeclared = false;       // the cell is not in this segment's topology
};

inline FeedReading energised_detail(const WireTable& table, ConductorId feed) {
  FeedReading r;
  WireTable::Sample<bool> s;
  if (!table.read_bit_sample(feed, &s)) {
    r.undeclared = true;
    return r;
  }
  if (!s.written()) {
    r.never_published = true;
    return r;
  }
  r.energised = s.value;
  return r;
}

// What signal does this line carry, as computed by the solver this tick? A PWM line's
// duty, an analog sensor line's position, a dimmer line's level, in permille.
//
// 0 when the solver published the line dark, AND when it has not published the line at
// all or the cell is undeclared: the same collapse as energised(), for the same reason. A
// receiver on a line nobody has shown to be live sees no signal, which is what a real
// input stage sees on a dead wire.
inline std::uint16_t line_level(const WireTable& table, ConductorId line) {
  WireTable::Sample<std::uint16_t> s;
  if (!table.read_uint16_sample(line, &s)) return 0;
  if (!s.written()) return 0;
  return s.value;
}

// ── WHY THERE IS NO `rail_millivolts()` HERE ──────────────────────────────────────────
//
// The battery-model pass made CHASSIS_AUX_BATTERY_TERMINAL_MV a uint32 CONDUCTOR cell,
// which raised the obvious question: does the mV answer need its own one-implementation
// helper beside energised()?
//
// It does not, and the reason is worth writing down because the symmetry is tempting.
// energised() exists to collapse a THREE-valued read into the ONE answer a control path
// may act on, so that no call site can invent a fourth. The mV read has no such
// collapse to perform: a brown-out comparator legitimately needs the number AND its
// written-ness, and `WireTable::read_uint32_sample(ConductorId, Sample*)` already hands
// back exactly those two things and nothing else. A wrapper would add no rule.
//
// MEASURED 2026-08-08 — and the measurement came back the OPPOSITE of what the
// battery-model pass first wrote here, so it is recorded the way it actually reads. The
// four brown-out comparators over the aux rail DO NOT AGREE:
//
//   ev1/lhjb/controller.cpp:668       `mv_ok && pr_ok && both .written()`   — AND
//   ev1/rhjb/controller.cpp:678       `mv_ok && pr_ok`                      — AND
//   ev1/rsa/rsa_aux_rail_gate.hpp:79  `if (have_mv || have_pres)`           — OR, then
//                                      substitutes mv = 0 / present = false for whichever
//                                      cell is missing
//   ev1/apm/apm_power_gate.hpp:82     same OR-with-substitution
//
// All four do agree that an entirely unwritten rail is DARK, which is why the pass left
// every one of them alone and why the defect was never that the consumers were too
// strict. But on a rail where ONE cell is published and the other is not, the AND pair
// hold their latch and the OR pair go dark.
//
// THAT CASE USED TO BE UNREACHABLE AND THIS PASS MADE IT REACHABLE. Before, neither cell
// was ever written by anybody, so the two shapes were indistinguishable. Now the solver
// publishes the presence bit from the node's tag but REFUSES to publish the millivolt
// cell when the node's potential is undefined — a floating rail — so "one published, one
// not" is a state the fleet can actually be in. Filed as
// BL-2026-08-08-aux-rail-comparators-disagree-on-half-published. It is not fixed
// here because RSA's and APM's gates belong to their own module passes, and rewriting two
// modules' power gates inside the change that first makes the difference observable is
// how a pass stops being reviewable.
//
// THIS PARAGRAPH ENFORCES NOTHING and must not be read as if it did — a count in a
// comment is not a check. What would enforce it is an audit that enumerates the readers
// of these two cells and fails when they stop agreeing; that is named in the item above.
//
// ══════════════════════════════════════════════════════════════════════════════════════
// THE POWER-TRANSITION OBSERVER
// notes/circuit_truth_power_transitions.md Part 1, §1.2-§1.6.
// ══════════════════════════════════════════════════════════════════════════════════════
//
// §1.2, the whole design in one paragraph: a module's host derives its own power
// transitions, from its own retained previous sample of ONE conductor cell, using ONE
// shared implementation that lives beside energised(). The solver gains nothing and stays
// a pure function of (topology, published inputs, stored state). There is no new
// transport: the solver publishes conductor cells and hosts read them, exactly as before.
//
// WHY IT IS HERE AND NOT THIRTEEN TIMES IN ev1/. Measured 2026-08-05: the trigger already
// exists, hand-rolled, in thirteen `*_key_off()` call sites — `ev1/bpm/controller.cpp:481`
// and `ev1/htcm/controller.cpp:441` are the same eight lines. They DISAGREE about the only
// question that matters: BPM's is_powered() returns true when the cell has never been
// written, HTCM's leaves now_powered false. That is not thirteen edge detectors, it is
// thirteen answers to "what does unwritten mean" wearing an edge detector as a hat — the
// same shape as the nine implementations of "am I powered" that this header collapsed.
//
// ── THE ONE NARROW EXEMPTION TO THIS HEADER'S OWN RULE, STATED SO IT IS NOT SMUGGLED ──
//
// energised_detail() says above: "Not for control flow: nothing that decides whether a
// lamp lights may branch on never_published." The observer below branches on
// never-published AND on staleness. The exemption is exactly this wide:
//
//     THESE STATES MAY GATE EVENT EMISSION. THEY MAY NEVER GATE LEVEL.
//
// A module in kUnattached or kStale answers energised() by the existing rule — unwritten
// is dark; a stalled producer's cell keeps its last published value — and runs its outputs
// accordingly. It simply does not claim an edge happened. "The solver has not spoken"
// remains no licence to proceed; it becomes a licence not to write a fault code into
// EEPROM.
//
// The exemption is CHECKABLE, not promised: PowerObservation::level is computed from the
// retained live state and never from the state name, so
// `obs.level == energised(table, cell)` on every step, in every state, including kStale
// and kUnattached. src/io/tests/test_power_transition.cpp asserts that equality on every
// row of every leg. If a future edit ever gates level on state, that assertion is what
// goes red.

// §1.3 — the four INPUT states. Four, not two, and each extra one is here because of a
// specific way this program has been wrong before:
//   * kUnattached — "the solver has not spoken yet" must never be mistaken for "the
//     vehicle went dark". A module that boots before the solver and then sees a dark cell
//     has not completed a power cycle; emitting a fall there would advance BPM's
//     100-power-transfer-cycle auto-clear for a cycle that never happened, i.e. advance
//     NON-VOLATILE bookkeeping from a startup race.
//   * kStale — "the solver died" must never be mistaken for it either. solver_host.hpp
//     already guarantees the producer half (it does not publish when the observation fails
//     validation, so the cell keeps its value and its write_gen stops advancing, which a
//     consumer can SEE); this is the consumer half of the same promise. It is finding R-8
//     faced from the other side of the segment: the answer is not to invent a heartbeat,
//     it is to refuse to manufacture an event from an absence.
enum class FeedState : std::uint8_t {
  kUnattached = 0,  // generation == 0: the solver has never published this cell
  kHot = 1,         // published, value true
  kDark = 2,        // published, value false
  kStale = 3,       // generation unchanged for kStaleAfterUnchangedSamples observations
};

// §1.3 — the three OUTPUTS. There is no kUnattachedFall and no kStaleFall; that is the
// design, not an omission.
enum class PowerEvent : std::uint8_t { kNone = 0, kRise = 1, kFall = 2 };

// N = 2, per §1.3: the solver runs at barrier cadence and a module host may read between
// two of its publishes, so ONE unchanged generation is the expected one-tick lag of plan
// section 4.6. TWO is a solver that did not run, or that refused to publish.
inline constexpr std::uint8_t kStaleAfterUnchangedSamples = 2;

// One cell, read ONCE (§1.4 step 1). A host that reads the same cell twice in one pass can
// see two different values: the seqlock is per cell, the publish loop is not atomic across
// cells, and there is no snapshot. Passing a FeedSample around instead of a ConductorId is
// what makes "sample once" the shape of the API rather than a rule in a comment.
struct FeedSample {
  bool          declared = false;    // false: undeclared id, or wrong cell type
  bool          value = false;
  std::uint32_t generation = 0;      // 0 == never published (WireTable::Sample::written())
};

inline FeedSample sample_feed(const WireTable& table, ConductorId cell) {
  FeedSample f;
  WireTable::Sample<bool> s;
  if (!table.read_bit_sample(cell, &s)) return f;  // undeclared: treated as unattached
  f.declared = true;
  f.value = s.value;
  f.generation = s.generation;
  return f;
}

// The host's retained state for ONE cell. The host owns it; the observer is a pure
// function of (this, the new sample), which is guarantee G5: replaying the same sequence
// of (value, generation) pairs from the same initial state yields the same events. No
// cross-tick state was added to the solver, so the replay property of requirement
// section 7 survives untouched.
struct PowerTransitionState {
  std::uint32_t last_generation = 0;
  std::uint8_t  unchanged = 0;
  // The last LIVE state. NEVER kStale — that is the invariant which makes the kStale rows
  // of §1.3's table ("kRise iff prev-live state was kDark") a lookup rather than a second
  // history.
  FeedState     live = FeedState::kUnattached;
};

struct PowerObservation {
  FeedState  state = FeedState::kUnattached;
  PowerEvent event = PowerEvent::kNone;
  // What energised() answers this pass. Computed from the retained LIVE state, never from
  // `state` — see the exemption above.
  bool       level = false;
  // DIAGNOSTIC ONLY, never gates anything: the generation went backwards or returned to 0,
  // which §1.3's table marks "(impossible)" — a re-created segment, or a wrap. No event is
  // emitted for it, because inventing one is exactly what G2 forbids.
  bool       generation_regressed = false;
};

// §1.3's table, as the one expression it collapses to once kStale is defined as "the live
// state does not move":
//
//   | from \ to   | kHot                     | kDark                    | kStale          |
//   |-------------|--------------------------|--------------------------|-----------------|
//   | kUnattached | kRise                    | (none)                   | (impossible)    |
//   | kHot        | (none)                   | kFall                    | (none), holds   |
//   | kDark       | kRise                    | (none)                   | (none), holds   |
//   | kStale      | kRise iff prev-live kDark| kFall iff prev-live kHot | (none)          |
//
// kUnattached -> kDark emits NOTHING and kUnattached -> kHot DOES emit kRise: booting
// powered IS the rising edge for the module's per-cycle state, and it preserves exactly
// what `ev1/bpm/controller.cpp:148-151` seeds today ("Seed FALSE so the first iteration
// (default-powered) is a false->true edge that keys the supervisor on"). Booting DARK is
// not a power cycle and never was.
inline PowerObservation step_power_transition(const FeedSample& s,
                                              PowerTransitionState* st) {
  PowerObservation o;
  const FeedState prev_live = st->live;

  if (!s.declared || s.generation == 0) {
    // kUnattached. Deliberately checked BEFORE staleness: a generation that has never
    // moved off zero is "the solver has not spoken", which is a different sentence from
    // "the solver stopped speaking", and only one of the two is tellable from a frozen
    // counter.
    o.generation_regressed = (st->last_generation != 0);
    st->last_generation = 0;
    st->unchanged = 0;
    st->live = FeedState::kUnattached;
    o.state = FeedState::kUnattached;
    o.level = false;   // unwritten is DARK — feed_query's answer, unchanged
    return o;
  }

  if (s.generation == st->last_generation) {
    if (st->unchanged < 0xFFu) ++st->unchanged;
  } else {
    o.generation_regressed = (s.generation < st->last_generation);
    st->unchanged = 0;
  }
  st->last_generation = s.generation;

  if (st->unchanged >= kStaleAfterUnchangedSamples) {
    // kStale. No event, and the LEVEL HOLDS: a solver that stops publishing must not look
    // like a vehicle that went dark. The live state is deliberately not written, so the
    // next live sample is compared against the last state the solver actually asserted.
    o.state = FeedState::kStale;
    o.level = (prev_live == FeedState::kHot);
    return o;
  }

  const FeedState now = s.value ? FeedState::kHot : FeedState::kDark;
  st->live = now;
  o.state = now;
  o.level = (now == FeedState::kHot);
  if (now == FeedState::kHot && prev_live != FeedState::kHot) {
    o.event = PowerEvent::kRise;
  } else if (now == FeedState::kDark && prev_live == FeedState::kHot) {
    o.event = PowerEvent::kFall;
  }
  return o;
}

// ── §1.6 — THE SPLIT: A SUPPLY FALL IS NOT A KEY CYCLE ────────────────────────────────
//
// `bpm_supervisor_key_off()` does TWO jobs on one edge:
//
//   Job A — close the POWER-TRANSFER cycle. Non-volatile: 100 consecutive fault-free
//           transfer cycles erase DTC history. Its unit is a VEHICLE cycle.
//   Job B — clear PER-CYCLE state. Volatile; its unit is a POWER cycle, and it "must not
//           carry into the next key-on or DTC 294/295 can fire one event early".
//
// Bound to one edge these were indistinguishable, because the 840 module-B+ is always
// hot off the 842 bus (`w_840_feed`) and RUN 1 follows `sw_pmm_run1_stage`, so THE TWO
// FALL AT DIFFERENT TIMES — and the single binding is wrong in BOTH directions:
//
//   * module-B+ falls only when the battery goes, never at a key-off. Binding the
//     vehicle cycle to it would let a plug-in charge session or a parked wake count as
//     nothing, and a battery swap count as a key cycle advancing the 100-PT-cycle
//     auto-clear — erasing DTC history on a counter the manual ties to power transfers.
//   * module-B+ does NOT fall at key-off, so per-cycle state that "must not carry into
//     the next key-on" carries straight across a key-off/key-on pair with the PMM awake.
//
// So: the SUPPLY's kFall triggers Job B only. Job A moves to the RUN-discrete's own
// falling edge — a separate event, below — behind G4's >= 2-barrier-tick debounce.
//
// THIS IS ONE FACT READ TWICE, NOT A SECOND OBSERVER. step_run_cycle() CALLS
// step_power_transition(); there is no copy of the state table here.
enum class RunCycleEvent : std::uint8_t { kNone = 0, kRunCycleEnd = 1 };

// G4's floor, and the reason the number is 2 rather than 1: the solver publishes cells
// sequentially and a host can read across the seam, so a transient disagreement between
// two cells is observable even on a tick where the solver computed both the same way.
// Every DTC predicate in this fleet already carries a debounce (DTC 281's is
// BPM_SUPERVISOR_RUN1_DEBOUNCE_MS); no BOOKKEEPING trigger did, and Job A is bookkeeping.
inline constexpr std::uint8_t kRunCycleFallDebounceSamples = 2;

struct RunCycleState {
  PowerTransitionState feed;
  // Consecutive BARRIER TICKS — new generations, not host passes — on which the discrete
  // has been dark since the fall. The distinction is the whole debounce: a host that reads
  // twice between two of the solver's publishes has made ONE observation, and counting it
  // twice would let a module close a vehicle's power-transfer cycle by polling faster.
  // §1.4's G4 says "a debounce of >= 2 barrier ticks", and a barrier tick is a publish.
  std::uint8_t         dark_ticks = 0;
  bool                 fall_pending = false;
};

struct RunCycleObservation {
  PowerObservation feed;
  RunCycleEvent    event = RunCycleEvent::kNone;
};

inline RunCycleObservation step_run_cycle(const FeedSample& s, RunCycleState* st) {
  RunCycleObservation o;
  o.feed = step_power_transition(s, &st->feed);

  // A stalled solver neither confirms nor denies a pending fall: the debounce is SUSPENDED,
  // not reset and not advanced. Advancing it would let a dead solver close a vehicle's
  // power-transfer cycle (G2); resetting it would let a one-tick stall erase a real
  // key-off.
  if (o.feed.state == FeedState::kStale) return o;

  if (o.feed.event == PowerEvent::kFall) {
    st->fall_pending = true;
    st->dark_ticks = 0;
  } else if (o.feed.event == PowerEvent::kRise ||
             o.feed.state == FeedState::kUnattached) {
    // The discrete came back, or the segment went away. Either way the fall was not a run
    // cycle ending, and re-arming here is what keeps G1 (exactly once) true.
    st->fall_pending = false;
    st->dark_ticks = 0;
  }

  // `feed.unchanged == 0` means step_power_transition() saw a NEW generation this pass,
  // i.e. the solver actually published. Re-reading the same publish does not advance the
  // debounce — see RunCycleState::dark_ticks. This is the line the §1.8 receipt's
  // "stale/pending" row exists to hold down: without it, a solver that died one tick after
  // RUN 1 fell would have its silence counted as the confirming tick, and Job A — a
  // NON-VOLATILE counter the manual ties to power transfers — would advance on an absence.
  if (st->fall_pending && o.feed.state == FeedState::kDark && st->feed.unchanged == 0) {
    if (st->dark_ticks < 0xFFu) ++st->dark_ticks;
    if (st->dark_ticks >= kRunCycleFallDebounceSamples) {
      o.event = RunCycleEvent::kRunCycleEnd;
      st->fall_pending = false;
      st->dark_ticks = 0;
    }
  }
  return o;
}

// ── §1.4 — ORDERING, AS A FUNCTION BODY RATHER THAN A CONVENTION ──────────────────────
//
// Within one pass of a module host, in this order, no exceptions:
//   1. sample every feed cell ONCE, into locals;
//   2. compute the state and event for each cell against the retained previous state;
//   3. deliver kRunCycleEnd handlers (Job A), then kFall handlers (Job B), BOTH before
//      any level-driven logic this pass;
//   4. deliver kRise handlers;
//   5. run level logic, gated on the post-event level.
//
// Step 3 before step 5 is guarantee G3, and it is load-bearing: a `*_key_off()` body must
// execute with the per-cycle state the cycle actually accumulated, not with state some
// level handler has already begun overwriting for the next one. A host that called a
// per-cell helper once per cell would deliver cell 1's LEVEL before cell 2's FALL, so the
// ordering has to be owned by a function that sees every cell — which is why this takes
// the whole declared set and not one cell.
//
// G3a — WITHIN THE FALL CLASS, JOB A RUNS BEFORE JOB B. §1.4 as first written ordered
// falls before rises before level and said nothing about two FALL-CLASS handlers'
// relative order, so this function shipped delivering the supply fall (Job B) first —
// plan section 20's finding V4-4, which named BPM's pass as the owner because BPM's is
// the pass that first binds both handlers. The order is now stated and enforced here:
//
//     JOB A READS PER-CYCLE STATE THAT JOB B IS ENTITLED TO DESTROY.
//
// Job A (`bpm_supervisor_run_cycle_end`) closes the power-transfer cycle by consuming
// `pt_cycle_had_transfer` and the active-DTC set; Job B (`bpm_supervisor_key_off`) clears
// per-cycle state wholesale. `bpm_supervisor_key_off()`'s own comment has documented this
// order since long before the split — "Feed it to the registry's 100-PT-cycle auto-clear
// BEFORE clearing per-cycle state" — and the split is what made it possible to get wrong.
// It was not wrong in effect at the commit that introduced it, because the one field Job A
// reads happens to be cleared at key-ON rather than by Job B's block; V4-4's point is that
// that is an accident of where a flag resets and not a property anything stated. It is
// stated now, and src/io/tests/test_power_transition.cpp's "both fall on one solve" leg
// fails if it is ever reversed.
//
// This generalises past BPM, which is why it lives here rather than in the caller: a
// run-cycle end is bookkeeping that READS the cycle just completed, and a supply fall is
// the MCU stopping, which DISCARDS it. Reads before discards, in every module.
//
// G6 — NO PUBLISH ON THE WAY DOWN. A falling edge does not license a final write of the
// module's own output cells. `ev1/htcm/controller.cpp:447-449` does exactly that today
// ("One final publish with supply_present=false so the *_FEED cells de-energize in shm");
// after HTCM's pass those cells are the solver's and that write does not compile. The
// replacement is not a different write: a module with a dark feed writes every contact
// CAUSE false, every tick, ON THE LEVEL — not on the edge. The graph turns "my drivers are
// off" into "that wire is dark". The edge is for bookkeeping only. That is why the Handler
// contract below has no "on the way down, publish this" hook and never will.
//
// Handler must provide (delivery order within one pass: on_run_cycle_end, on_supply_fall,
// on_supply_rise, on_level — G3 and G3a above):
//   void on_run_cycle_end(std::size_t index);  // Job A — close the power-transfer cycle
//   void on_supply_fall();                     // Job B — clear per-cycle state
//   void on_supply_rise();                     // key-on
//   void on_level(bool supply_level, const RunCycleObservation* discretes, std::size_t n);
//
// `discretes`/`discrete_states`/`discrete_obs` are parallel arrays of length `n`, owned by
// the caller, in the order config/nets/<module>.yaml declares them (§1.5). `n == 0` is
// legal and means the module declared no discretes.
template <typename Handler>
inline PowerObservation run_module_power_pass(const WireTable& table,
                                              ConductorId supply,
                                              PowerTransitionState* supply_state,
                                              const ConductorId* discretes,
                                              RunCycleState* discrete_states,
                                              RunCycleObservation* discrete_obs,
                                              std::size_t n,
                                              Handler& h) {
  // 1 + 2. Every cell sampled exactly once, then stepped. No cell is read twice, and no
  // handler runs until every cell has been read — a handler that re-read a cell could see
  // a value from the other side of the publish loop's seam (G4).
  const PowerObservation supply_obs =
      step_power_transition(sample_feed(table, supply), supply_state);
  for (std::size_t i = 0; i < n; ++i) {
    discrete_obs[i] = step_run_cycle(sample_feed(table, discretes[i]), &discrete_states[i]);
  }

  // 3. FALLS FIRST — every fall, across every cell, before any level logic (G3), and
  // within the fall class every JOB A before any JOB B (G3a). Reads before discards.
  for (std::size_t i = 0; i < n; ++i) {
    if (discrete_obs[i].event == RunCycleEvent::kRunCycleEnd) h.on_run_cycle_end(i);
  }
  if (supply_obs.event == PowerEvent::kFall) h.on_supply_fall();

  // 4. Then rises.
  if (supply_obs.event == PowerEvent::kRise) h.on_supply_rise();

  // 5. Then the level, which is the only thing that may drive an output.
  h.on_level(supply_obs.level, discrete_obs, n);
  return supply_obs;
}

}  // namespace io
}  // namespace electricsim
