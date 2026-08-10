/*
 * WireTable — shared-memory primitive for per-wire state cells.
 *
 * Each "wire" is a named WireId (uint32_t). Each cell holds a typed
 * value (bit, byte, uint16, uint32, float32) at a fixed offset within
 * a shared-memory segment. Writes and reads use std::atomic with
 * acquire/release ordering; on platforms where all scalar widths up
 * to 32 bits are lock-free (x86_64, ARM64) this is wait-free for both
 * producer and consumer. Wider cell types (future bytes[N]) will use
 * a per-cell seqlock; the directory already reserves space for one.
 *
 * This is the parallel substrate alongside SharedMemoryTransport (the
 * existing ring buffer). See docs/wire_truth_substrate.md for design
 * context — wire cells represent wire state directly (latest-value-
 * wins, no transport, no broadcast) instead of a queue of messages.
 * Step 1 of epic/wire-truth-substrate ships the primitive only; named
 * WireIds and the topology loader come in a follow-up sub-PR.
 *
 * Failure model: WireTable uses static factory functions that return
 * nullptr on failure (with a stderr diagnostic). This avoids the
 * "constructor silently degrades" pattern in SharedMemoryTransport
 * which makes initialization failures hard to detect at call sites.
 *
 * @design 2026-06-08 claude — epic/wire-truth-substrate, step 1.
 */

#ifndef ELECTRICSIM_SRC_IO_WIRE_TABLE_HPP_
#define ELECTRICSIM_SRC_IO_WIRE_TABLE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

// The ONE holder of the conductor publish capability — a "one friend" rule: if a
// caller needs two, the fix is to fix the split, not grow the friend list.
// Forward-declared here so SolverToken can name it.
//
// WHY IT IS NOT src/net/solver/publish.cpp: src/net/ compiles STANDALONE against
// nothing but the C++ standard library — no embedded-hardware dependencies — and
// tests/circuit_truth/net_core_build.sh proves that by GLOBBING every .cpp under
// src/net/ and compiling it with `-I src/net` and no other include path. A
// publish.cpp under src/net/solver/ would have to include src/io/wire_table.hpp and
// would break that build — the one suite that must stay green at every commit. The
// publish edge therefore lives one directory out, in src/net_host/, which hosts the
// solver's I/O-facing integration points. Still exactly one place; still nothing
// else may name SolverToken.
namespace electricsim::net_host {
class ConductorPublisher;
}  // namespace electricsim::net_host

namespace electricsim::io {

using WireId = std::uint32_t;

// ─── Cell classes ───────────────────────────────────────────────────────
//
// Every cell in config/topology.yaml carries a `class:` key. It is REQUIRED and
// validated: scripts/gen_topology_header.py fails the build on a cell with no
// `class:` and on an unrecognised value ("absent means CHECKED" — a silently
// unclassified cell would be "absent means powered" rebuilt inside the fix).
//
//   kConductor          a physical conductor. Its energisation is an OUTPUT of the
//                       solver's provenance computation, never something a module
//                       asserts about itself. Emitted as ConductorId — a strong type
//                       with NO write_bit() overload, so an illegitimate conductor
//                       write is a compile error rather than an audit finding.
//   kElementState       a module-decided element INPUT (contact `closed`, converter
//                       `enable`, transducer mechanical input). Modules write CAUSES;
//                       the solver reads them. Emitted as ElementStateId.
//   kSemantic           an ordinary cell (commands, telemetry, counters). Plain WireId,
//                       unchanged write surface.
//   kUnclassifiedLegacy the one-time pin on the cells that predate the classification.
//                       Held by a DECREASING ratchet in the generator that fails on
//                       `!=`, never `<=` — a `<=` check would let a module bank headroom
//                       for two free writes later.
enum class CellClass : std::uint8_t {
  kUnknown = 0,
  kConductor = 1,
  kElementState = 2,
  kSemantic = 3,
  kUnclassifiedLegacy = 4,
};

// ConductorId — the opaque handle for a `class: conductor` cell.
//
// A scoped enum over WireId: zero runtime cost, no implicit conversion in either
// direction. The generated header emits `inline constexpr ConductorId kWireX{123U};`
// in place of the old `inline constexpr WireId kWireX = 123U;`, and from that moment
// there is NO path from a module TU to a value-write of that cell — with or without an
// `@wire` comment, on one line or on five. That last clause is the measurement that
// motivated the type: 4 of LHJB's 19 conductor writes are multi-line calls a
// line-oriented grep misses, and all four are conductor writes.
enum class ConductorId : WireId {};

// ElementStateId — the opaque handle for a `class: element_state` cell.
//
// The transport SURVIVES the migration; only the semantics change. These are the cells a
// module writes to say "my contact is closed" / "my converter is enabled" — a CAUSE. The
// solver reads them, runs the graph, and publishes the CONSEQUENCE to conductor cells.
// Distinct from ConductorId because the write direction is the opposite one: modules may
// write these and the solver may not.
enum class ElementStateId : WireId {};

// SolverToken — proof that a value is the output of a provenance computation.
//
// The token is NOT "you are on the allow-list of things that may
// produce voltage" — that older allow-list model is deliberately gone. It is: THIS VALUE IS THE
// OUTPUT OF A PROVENANCE COMPUTATION, and it is constructible only where that
// computation happens. A future agent tempted to friend a second TU is being tempted to
// publish energisation that no provenance path backs.
//
// Non-copyable and non-movable on purpose: a token that can be copied can be stashed in
// a global and handed to anyone, which turns the capability into a formality.
class SolverToken {
 public:
  SolverToken(const SolverToken&) = delete;
  SolverToken& operator=(const SolverToken&) = delete;
  SolverToken(SolverToken&&) = delete;
  SolverToken& operator=(SolverToken&&) = delete;

 private:
  // NOT `= default`. Under C++17 a `= default` first declaration is user-DECLARED but
  // not user-PROVIDED, which leaves SolverToken an AGGREGATE (it has no base classes,
  // no virtuals, and every member -- there are none -- would be public). Aggregate
  // initialisation (`io::SolverToken tok{};`) does not call a constructor at all, so
  // the private-access check below never runs and any TU can mint a token. A
  // user-PROVIDED body (this one) removes the type from aggregate-hood, so `tok{}`
  // goes through overload resolution like any other call and the private access
  // check fires. Verified three-valued in src/net_host/tests/type_split_compile_receipt.sh
  // leg 3b (circuit-truth session 2 phase 4, finding V4-1).
  SolverToken() noexcept {}
  friend class ::electricsim::net_host::ConductorPublisher;
};

enum class WireType : std::uint8_t {
  kBit = 1,      // 1-bit semantic, 1-byte storage (0 or 1).
  kByte = 2,     // 1-byte storage.
  kUint16 = 3,   // 2-byte storage.
  kUint32 = 4,   // 4-byte storage.
  kFloat32 = 5,  // 4-byte storage (IEEE 754 bit-pattern).

  // kBitStream — a bounded single-producer / multiple-independent-reader
  // bit FIFO. Unlike the scalar types above (one 32-bit atomic, latest-
  // value-wins), a bit-stream cell PRESERVES the bits a producer appends
  // in a bounded ring so a coarse-tick consumer can replay every bit it
  // missed between its own ticks. This is the GM-8192 migration's answer
  // to the bit-cadence problem (a 122 µs bit period vs. 5–50 ms host
  // ticks): the producer appends a whole frame's bits in one tick; each
  // reader drains the bits appended since its own private cursor. The
  // cell occupies more than one 64-byte slot and uses a per-cell seqlock
  // (the "future bytes[N]" seam this header reserved at lines 9-10).
  // @design 2026-06-13 claude — round-19, sub-PR 4-C½;
  //   notes/manual_supplements.yaml#2026-06-13-gm8192-bit-ring-cell.
  kBitStream = 6,

  // kUint64 — 8-byte unsigned scalar. Added for the chassis sim-time
  // clock (kSigChassisSimTimeNs, 4075): a nanosecond counter overflows
  // a uint32 every ~4.29 s, so the physics↔electrical boundary clock
  // needs 64 bits (~584 years at ns resolution). Stored in a dedicated
  // 64-bit atomic word in the Cell (the scalar `value` word stays 32-bit
  // so the existing types are byte-for-byte unchanged); the same
  // gen-guarded seqlock provides the written()/freshness contract.
  // @design 2026-06-15 claude — Phase C-a (chassis migration);
  //   notes/phase_c_chassis_migration_scope.md.
  kUint64 = 7,
};

struct WireTableOptions {
  // POSIX shared-memory segment name (without leading "/"). The
  // implementation prepends "/" automatically on POSIX. Keep to ~28
  // characters total; macOS PSHMNAMLEN is 31.
  std::string name{"electricsim_wires"};

  // Maximum directory entries (cells). Sets the upper bound on
  // declare() calls. Fixed at creation time and immutable.
  std::size_t max_cells{4096};

  // Cell-area capacity in bytes. Each declared cell consumes one
  // 64-byte slot (cache line) for false-sharing avoidance.
  std::size_t cell_area_bytes{256 * 1024};

  // Topology hash that attachers must match. Zero disables the check
  // (intended for tests and the not-yet-wired Step 1 primitive).
  // Step 1b (topology loader) populates this from the canonical YAML.
  std::uint32_t topology_hash{0};
};

class WireTable {
 public:
  // Create + initialize a new segment. Returns nullptr if the segment
  // already exists, ftruncate fails, or mmap fails. Logs the failure
  // reason to stderr in all cases. init_complete is published as 1
  // before this function returns, so attachers can connect
  // immediately — declare() calls AFTER this point race against the
  // late-declare scan and may produce spurious "wire not found"
  // results if an attacher's read arrives before declare's release
  // store; for the "declare every wire at startup" pattern, use the
  // overload below.
  static std::unique_ptr<WireTable> create(const WireTableOptions& opts);

  // Create + run a declare callback BEFORE publishing init_complete.
  // This closes the race window where an attacher could observe
  // init_complete=1 while the directory was still empty. The
  // callback is invoked with the freshly-mapped table while
  // init_complete is still 0 (attachers in attach() are still
  // polling); when it returns true, init_complete=1 is published and
  // the table is returned. If the callback returns false, the
  // segment is torn down and nullptr is returned (caller's
  // declarations failed — e.g., directory full).
  //
  // Use this overload when the full topology is known at creation
  // time, as in env_open's creator path:
  //
  //   auto t = WireTable::create(opts, [](WireTable& tab) {
  //     return topology::declare_all(tab);
  //   });
  //
  // (Bugbot review of epic PR #85: init_complete-before-declares
  // race that causes spurious ParityCheckedWireBit mismatches during
  // producer startup.)
  static std::unique_ptr<WireTable> create(
      const WireTableOptions& opts,
      const std::function<bool(WireTable&)>& declare_fn);

  // Attach to an existing segment. Returns nullptr if the segment is
  // missing, the magic header is wrong, or topology_hash mismatches a
  // non-zero opts.topology_hash. Logs the failure reason to stderr.
  static std::unique_ptr<WireTable> attach(const WireTableOptions& opts);

  // Adopt an existing, structurally-valid segment AS its owner. Same
  // mapping + validation as attach(), but the returned handle takes
  // lifecycle ownership: it behaves like a creator (unlinks the
  // segment on destruction). Returns nullptr on the same conditions as
  // attach() (missing / bad magic / version or topology-hash mismatch
  // / corrupt header / timed-out init).
  //
  // The use case is orphan recovery in env_open's creator path: a
  // previous driver process died without unlinking, leaving a VALID
  // segment with live readers still mapped to it. Re-creating (unlink
  // + shm_open O_EXCL) would mint a NEW inode and orphan those readers
  // — shm_unlink only drops the name, existing mappings persist on the
  // old inode (Bugbot review of epic PR #85: "consumer stuck on
  // unlinked segment"). adopt() reuses the SAME inode, so readers keep
  // working and the new driver writes where they read.
  //
  // adopt() does NOT re-declare the topology — the adopted segment
  // already carries it (its topology_hash matched, which is exactly
  // how we know the directory is the one we expect). declare() on the
  // result is still permitted (is_creator is true) for adding new
  // cells, but declaring an already-present id is a no-op-false.
  static std::unique_ptr<WireTable> adopt(const WireTableOptions& opts);

  ~WireTable();

  WireTable(const WireTable&) = delete;
  WireTable& operator=(const WireTable&) = delete;

  // Declare a cell (creator only, before any read/write traffic). The
  // declaration is durable in the segment — subsequent attachers see
  // it via the directory. Returns true on success, false if the id is
  // already declared, the directory is full, the cell-area is
  // exhausted, or this table is an attacher.
  bool declare(WireId id, WireType type);

  // Typed read accessors. Return false if the id is undeclared or the
  // declared type does not match the access function.
  bool read_bit(WireId id, bool* out) const;
  bool read_byte(WireId id, std::uint8_t* out) const;
  bool read_uint16(WireId id, std::uint16_t* out) const;
  bool read_uint32(WireId id, std::uint32_t* out) const;
  bool read_uint64(WireId id, std::uint64_t* out) const;
  bool read_float32(WireId id, float* out) const;

  // WireSample — {value, write_generation} pair returned by
  // read_*_sample. `generation == 0` means the producer has never
  // written this cell, the substrate-side fact that lets consumers
  // distinguish a never-written cell from a producer that wrote 0
  // (the "Pattern A" failure class from the round-3 reviews). Every
  // write_*() bumps generation by 1, so callers that remember the
  // previous generation can also detect "changed since I looked"
  // and "still advancing" (producer alive vs. stalled).
  // See docs/wire_truth_written_ness.md.
  //
  // @design 2026-06-11 claude — round-4, step 4a.
  template <typename T>
  struct Sample {
    T             value{};
    std::uint32_t generation{0};
    bool written() const noexcept { return generation != 0; }
  };

  // Typed read accessors that also report write-generation. Return
  // false only on undeclared id / type mismatch (same contract as
  // read_*). On success, out->written() distinguishes "never written"
  // (gen == 0) from "written at least once" (gen > 0).
  bool read_bit_sample(WireId id, Sample<bool>* out) const;
  bool read_byte_sample(WireId id, Sample<std::uint8_t>* out) const;
  bool read_uint16_sample(WireId id, Sample<std::uint16_t>* out) const;
  bool read_uint32_sample(WireId id, Sample<std::uint32_t>* out) const;
  bool read_uint64_sample(WireId id, Sample<std::uint64_t>* out) const;
  bool read_float32_sample(WireId id, Sample<float>* out) const;

  // ─── Conductor cells ──────────────────────────────────────────────────
  //
  // publish_conductor is the ONLY write path to a `class: conductor` cell, and it
  // demands a SolverToken, which is constructible in exactly one place (see the
  // SolverToken declaration above). There is deliberately no ConductorId overload of
  // write_bit(): a module that wants to assert its own feed is hot has no expression
  // for it that compiles.
  //
  // Reads are NOT the problem and stay working — but only the read that CARRIES
  // WRITTEN-NESS. read_bit(ConductorId, bool*) does not exist either, because a plain
  // read collapses "de-energised" and "nobody has solved this yet" into one `false`,
  // which is the same defect as a defaulting read wearing a smaller hat.
  //
  // And there is NO read_bit_or_default(ConductorId, ...) — not "for symmetry", not
  // "for the tests", not "just for the transition". Deliberately restated at each
  // call site: a defaulting overload would rebuild the consumer-side fallback on
  // the exact cells the type-split exists to protect, in a header nobody reviews
  // twice. The absence is enforced by the absence of an implicit conversion: passing
  // a ConductorId to the WireId overload does not compile.
  bool publish_conductor(ConductorId id, bool energised, const SolverToken& token);
  bool read_bit_sample(ConductorId id, Sample<bool>* out) const;

  // THE MILLIVOLT HALF OF THE SAME ANSWER — finding R-2's closure.
  //
  // Until the battery-model pass, publish_conductor() published a bool and NOTHING
  // ELSE, so — recorded twice in scripts/gen_topology_header.py's movement log, once
  // in pass 3 and again in pass 4 — *"a uint32 cell cannot be a conductor at all"*.
  // That sentence is a statement about this API, not about the vehicle. Every analog
  // supply rail in the fleet (the 12 V aux post, the HV DC link, the APM's 12 V
  // output) was therefore forced OUT of the conductor class and into
  // `unclassified-legacy`, where any process may write it — which is how
  // CHASSIS_AUX_BATTERY_TERMINAL_MV ended up owned by a host process that no VAT
  // fleet spawns, and how the whole 12 V distribution graph came to have no producer.
  //
  // A node's potential is an OUTPUT of the same provenance computation that decides
  // whether it is energised at all (src/net/net_types.hpp NodeObservation::voltage_mv,
  // valid IFF voltage_defined()). It is the same answer, carried to more decimal
  // places. So it gets the same token, the same sole-writer guarantee, and the same
  // refusal to default.
  //
  // WHY THERE IS NO `read_uint32(ConductorId, uint32*)`. Same reason as the bit form:
  // a plain read collapses "this rail is at 0 mV" (bonded to chassis) and "nobody has
  // solved this yet" into one zero. The mV case is WORSE than the bit case, because 0
  // mV is a physically meaningful reading that a grounded node genuinely has —
  // is_grounded_zero() exists precisely to keep it distinct from floating. Only the
  // sample form, which carries written-ness, exists.
  bool publish_conductor_mv(ConductorId id, std::uint32_t millivolts,
                            const SolverToken& token);
  bool read_uint32_sample(ConductorId id, Sample<std::uint32_t>* out) const;

  // ─── Element-state cells ────────────────────────────────────────────────
  //
  // The module-decided half of the element model: contact `closed`, converter/source
  // `enable`, transducer mechanical input. Modules WRITE these (they are causes) and
  // the solver READS them. Same segment, same seqlock, same write_gen — zero lines
  // of new IPC.
  //
  // Same no-defaulting-read rule as ConductorId, for the same reason: an element whose
  // `closed` is unwritten is TOPOLOGICALLY ABSENT, which a defaulting
  // read would silently turn into "closed" or "open" — a claim the graph never made.
  bool write_element_state(ElementStateId id, bool closed);
  bool read_bit_sample(ElementStateId id, Sample<bool>* out) const;

  // The ANALOG cause. A source element's terminal state is a number, not a bit: an
  // SLA cell at 40% state-of-charge sits at a different open-circuit voltage than the
  // same cell at 100%, and that difference is the whole reason a chemistry model
  // exists. This is the transport for it.
  //
  // Same three-valued contract as the bit form and for the same reason: UNWRITTEN IS
  // NOT ZERO VOLTS. A source whose mV cause has never been written is not a flat
  // battery — it is a battery no chemistry model is watching, and the solver falls
  // back to the element's AUTHORED open_circuit_mv (a reviewed number in
  // config/nets/<module>.yaml, not a runtime guess). Writing zero here IS the claim
  // "this battery is flat", and it darkens the car. The two must never be the same
  // input, which is why the read is sample-only.
  bool write_element_state_mv(ElementStateId id, std::uint32_t millivolts);
  bool read_uint32_sample(ElementStateId id, Sample<std::uint32_t>* out) const;

  // Typed write accessors. Return false if the id is undeclared or
  // the declared type does not match. Any attached process may write,
  // but the wire-truth model expects exactly one declared driver per
  // cell (enforced by the topology layer in Step 1b, not here).
  bool write_bit(WireId id, bool value);
  bool write_byte(WireId id, std::uint8_t value);
  bool write_uint16(WireId id, std::uint16_t value);
  bool write_uint32(WireId id, std::uint32_t value);
  bool write_uint64(WireId id, std::uint64_t value);
  bool write_float32(WireId id, float value);

  // ─── bit-stream (kBitStream) accessors ──────────────────────────────
  //
  // A bit-stream cell is a single-producer / multiple-independent-reader
  // bit FIFO of fixed capacity (kBitStreamCapacityBits). It dissolves the
  // GM-8192 bit-cadence problem: the producer appends a whole frame's bits
  // in one coarse tick; each reader keeps its OWN cursor (in process-local
  // memory — there is NO per-reader state in shm) and drains the bits
  // appended since that cursor on its own coarse tick.
  //
  // Capacity of a bit-stream cell's ring, in bits. Sized to cover the
  // slowest fleet reader's tick interval with ≥2× margin: RSA at 50 ms is
  // the slowest reader; at 8192 baud that is ~410 bits per interval, so
  // 1024 bits (~125 ms of bus traffic) is ~2.5× margin. A reader that
  // falls more than this far behind detects the overrun (below) and
  // re-syncs framing on the next Start edge rather than silently decoding
  // corrupted bits. @design 2026-06-13 claude — round-19, sub-PR 4-C½.
  static constexpr std::size_t kBitStreamCapacityBits = 1024;

  // Append `n` bits to the bit-stream cell `id` (producer side; one
  // declared driver per cell). Returns false on undeclared id / type
  // mismatch / null table. Bits are taken from `bits[0..n)`; bits[k] != 0
  // means logic-1. Appending more than kBitStreamCapacityBits in a single
  // tick is permitted but only the most-recent kBitStreamCapacityBits are
  // retained for readers — the surplus is unobservable, which a reader
  // detects as an overrun. Bumps the cell's write_gen (so has_cell /
  // write_gen / for_each_unwritten treat it like any other written cell).
  bool append_bits(WireId id, const bool* bits, std::size_t n);

  // Append a single bit (convenience; same contract as append_bits with
  // n == 1).
  bool append_bit(WireId id, bool value);

  // Read every bit appended to bit-stream cell `id` since absolute index
  // *cursor, up to `max` bits, into out[0..*got). Advances *cursor by the
  // number of bits consumed. Returns false on undeclared id / type
  // mismatch / null table (and leaves *got = 0, *cursor unchanged).
  //
  // Overrun: if the reader fell behind by more than the ring capacity
  // (total_bits - *cursor > kBitStreamCapacityBits), the oldest bits are
  // gone. The reader cannot replay them, so this call clamps *cursor up to
  // (total_bits - kBitStreamCapacityBits) — the oldest bit still in the
  // ring — sets *overran = true, and returns the recoverable bits from
  // there. The caller must treat *overran as "lost framing; re-sync on the
  // next Start edge". When no bits were lost, *overran is set false.
  // `overran` may be null if the caller does not care.
  //
  // A fresh reader starts with *cursor == 0; the first read replays from
  // the oldest bit still in the ring (with *overran reflecting whether any
  // pre-cursor history was already evicted).
  bool read_bits_since(WireId id, std::uint64_t* cursor, bool* out,
                       std::size_t max, std::size_t* got,
                       bool* overran = nullptr) const;

  // Total bits ever appended to bit-stream cell `id` (monotonic). Returns
  // false on undeclared id / type mismatch / null table. Lets a reader ask
  // "how far behind am I" without draining. Tear-free via the seqlock.
  bool bit_stream_total(WireId id, std::uint64_t* out_total) const;

  // Introspection (tests, diagnostics, topology-hash verification by
  // external tools, late-attacher cell-existence checks).
  std::uint32_t topology_hash() const noexcept;
  std::size_t cell_count() const noexcept;
  bool is_creator() const noexcept;

  // has_cell may insert into the process-local id_to_index cache on
  // a late-declare miss, so it can theoretically allocate and throw
  // std::bad_alloc. NOT noexcept. (Bugbot PR #86 fourth-round low
  // severity: noexcept-then-allocate was the implicit-terminate
  // footgun.)
  bool has_cell(WireId id) const;

  // Type-agnostic write-generation accessor (round-4 step 4e). Returns
  // false on undeclared id (same shape as has_cell with a re-scan-on-
  // miss); on success, *out_gen is the cell's current write_gen
  // (0 == never written, >0 == that many writes have been published).
  // Used by the substrate-side "never written after init" diagnostic
  // emitted by the codegen — the generic form of every bespoke
  // missing-producer guard the round-3 reviews layered onto specific
  // sites (RUN1_FLEET_PRODUCERS in scripts/run_ev1_vehicle.sh, the
  // RUN1 / aux-rail consumer gates from 4c / 4d). Lets a caller ask
  // "is this cell live yet?" without knowing or caring about its
  // type. See docs/wire_truth_written_ness.md.
  //
  // Same lazy-cache-update path as has_cell, so NOT noexcept.
  bool write_gen(WireId id, std::uint32_t* out_gen) const;

  // ── Co-sim tick barrier (primitive-4 B) ──────────────────────────
  //
  // A deterministic per-tick lockstep between the ev1sim PUBLISHER (leader) and
  // the electricsim controller CONSUMERS, so the co-sim produces identical
  // outcomes across runs and host speeds. Without it, consumers free-run over
  // the latest-value-wins cells and skip a run-to-run-variable subset of
  // CHASSIS_SIM_TIME_NS (4075) samples → non-deterministic ABS modulation.
  //
  // State lives in Header::reserved[] (ABI-invisible: not in kTopologyHash, no
  // kFormatVersion bump). The barrier is INERT unless a leader calls
  // barrier_arm(): a fresh segment's reserved[] is zero (enabled == 0), and a
  // pre-barrier binary never touches these bytes. Consumers must additionally
  // gate participation on SimClock::is_sim_time_master() so a standalone
  // controller (no 4075 publisher) never engages — keeping every existing test
  // byte-identical.
  //
  // Protocol (leader holds the first tick until all `consumer_count` consumers
  // have acked it, so every consumer starts from the SAME tick → deterministic;
  // membership needs no shm registry — the leader is told the count out-of-band
  // via EV1SIM_FLEET_N):
  //   leader:   barrier_arm() once; then per physics tick, AFTER publishing all
  //             cells (4075 last): barrier_publish_tick(); barrier_await_acks(N).
  //   consumer: per loop pass when barrier_active() && is_sim_time_master():
  //             process the tick it just read, barrier_ack(); subsequent passes
  //             barrier_await_tick(&prev_gen) to BLOCK for the next tick.
  //
  // Wake is ADVISORY (Linux futex on the gen words; portable spin-poll fallback
  // elsewhere): the shm counters are source-of-truth, so a lost/absent wake only
  // costs latency. Every wait is bounded by timeout_ms; on a crashed peer the
  // wait returns false (never wedges) so the caller can abort the run.

  // Leader: arm the barrier (idempotent). After this, consumers that are
  // sim-time masters will block-and-ack.
  void barrier_arm();
  // Leader: open a new tick — reset the ack counter, bump the publish
  // generation, wake blocked consumers. Call AFTER publishing all cells.
  void barrier_publish_tick();
  // Leader: block until `consumer_count` consumers have acked the current tick,
  // or `timeout_ms` elapses. Returns true if all acked, false on timeout (a
  // consumer crashed/never-joined — the caller should abort the run).
  bool barrier_await_acks(std::uint32_t consumer_count, int timeout_ms);

  // Consumer: is a leader driving the barrier?
  bool barrier_active() const;
  // Consumer: block until the publish generation advances past *prev_gen (a new
  // tick), updating *prev_gen, or until timeout_ms. Returns true on a new tick,
  // false on timeout (leader gone). A fresh consumer passes *prev_gen == 0.
  bool barrier_await_tick(std::uint32_t* prev_gen, int timeout_ms) const;
  // Consumer: signal that this consumer has finished processing the current
  // tick (bumps the ack counter, wakes the leader).
  void barrier_ack();
  // Current publish generation (the tick the leader has opened). Consumers use
  // it to seed *prev_gen when joining mid-stream. 0 before the first tick.
  std::uint32_t barrier_publish_gen() const;

  // ── TEST-ONLY fault-injection hooks ──────────────────────────────
  //
  // Force bit-stream cell `id`'s seqlock counter to a given (raw) value,
  // bypassing the append protocol. Used by the substrate regression tests
  // to simulate a producer that crashed mid-append (leaving `seq` ODD), so
  // the bounded-retry (S1) and adopt-normalization (S2) fixes can be proven
  // without an inherently racy real mid-append kill. Returns false on
  // undeclared id / type mismatch. NOT for production use — it deliberately
  // corrupts the seqlock invariant. (Added 2026-06-13 for the adversarial
  // substrate review's S1/S2 regression coverage.)
  bool debug_set_bit_stream_seq(WireId id, std::uint64_t raw_seq);

  // Read bit-stream cell `id`'s raw seqlock counter (test introspection,
  // pairs with debug_set_bit_stream_seq). Returns false on undeclared id /
  // type mismatch.
  bool debug_bit_stream_seq(WireId id, std::uint64_t* out_seq) const;

 private:
  WireTable();
  // Shared body for attach()/adopt(). take_ownership controls whether
  // the resulting handle unlinks the segment on destruction.
  static std::unique_ptr<WireTable> attach_impl(const WireTableOptions& opts,
                                                bool take_ownership);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace electricsim::io

#endif  // ELECTRICSIM_SRC_IO_WIRE_TABLE_HPP_
