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

namespace electricsim::io {

using WireId = std::uint32_t;

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
