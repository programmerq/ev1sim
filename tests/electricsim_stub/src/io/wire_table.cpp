/*
 * WireTable implementation. See wire_table.hpp for design notes.
 *
 * Segment layout:
 *   Offset 0:    Header (64 bytes — magic, version, hash, sizes, atomic cell_count)
 *   Offset 64:   Directory (max_cells × 16-byte CellDescriptor)
 *   Offset N:    Cell area (aligned up to 64; each cell occupies a 64-aligned
 *                block — one 64-byte slot for the scalar types, several for
 *                a kBitStream cell)
 *
 * Each SCALAR cell is a single `std::atomic<uint32_t>`. Bit/byte cells use
 * the low 8 bits; uint16 uses the low 16; uint32 and float32 use all 32.
 * 32-bit aligned atomic load/store is wait-free and torn-read-free on
 * x86_64 / ARM64 — no per-cell seqlock is needed for these widths.
 *
 * A kBitStream cell (round-19, sub-PR 4-C½) is the realization of the
 * "future bytes[N]" seam this file's predecessor reserved: it is WIDER
 * than one atomic word (a bit ring + counters), so its multi-word reads
 * are made tear-free with a per-cell SEQLOCK rather than a single atomic
 * load. The cell occupies a multi-slot block; CellDescriptor::offset
 * carries its (variable) start so cell_at() resolves it the same way it
 * resolves a scalar cell. See BitStreamCell below for the layout and the
 * seqlock protocol.
 *
 * Failure model: factories return nullptr after a stderr diagnostic.
 * Constructor itself can't fail (private no-op); all real init happens
 * in create()/attach().
 *
 * @design 2026-06-08 claude — epic/wire-truth-substrate, step 1.
 */

#include "wire_table.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <new>
#include <string>
#include <thread>
#include <unordered_map>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <fcntl.h>
#include "topology/shm_test_helpers.hpp"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#if defined(__linux__)
#include <climits>        // INT_MAX (FUTEX_WAKE count)
#include <ctime>          // struct timespec (FUTEX_WAIT timeout)
#include <linux/futex.h>  // FUTEX_WAIT / FUTEX_WAKE
#include <sys/syscall.h>  // SYS_futex
#endif
#endif

namespace electricsim::io {

namespace {

// Magic identifies the segment as a WireTable segment (distinct from
// SharedMemoryTransport's ring header).
constexpr char kMagic[8] = {'W', 'I', 'R', 'E', 'T', 'B', 'L', '\0'};
// kFormatVersion 2 (2026-06-11, round-4 step 4a): Cell gained a second
// atomic word (write_gen) in the previously-padding bytes 4..7, see
// docs/wire_truth_written_ness.md. The layout is byte-identical for a
// v1 reader against a v2 segment for the value half, but a v1 reader
// would see write_gen as garbage padding and a v2 reader against a v1
// segment would see write_gen as the cell's leftover zero-fill (never
// bumped). Either direction is a real correctness regression, so the
// version check in attach() refuses both. Belt-and-braces, the YAML
// format_version was bumped in lockstep so kTopologyHash also moves.
constexpr std::uint32_t kFormatVersion = 2;
constexpr std::size_t kCellSize = 64;
constexpr std::size_t kHeaderSize = 64;

struct CellDescriptor {
  std::uint32_t id;
  std::uint32_t offset;        // bytes from start of cell area
  std::uint8_t  type;          // WireType
  std::uint8_t  reserved[3];
  std::uint32_t cell_seq;      // reserved for future multi-word cells
};
static_assert(sizeof(CellDescriptor) == 16, "CellDescriptor must be 16 bytes");

struct Header {
  // init_complete is written LAST by the creator (release ordering) and
  // read FIRST by attachers (acquire ordering, with bounded retry).
  // The release/acquire pair makes every other header field — magic,
  // version, sizes — visible to an attacher that observes
  // init_complete == 1. A freshly mmap'd (zero-filled) segment has
  // init_complete == 0, meaning "creator hasn't published yet"; an
  // attacher arriving in that window polls instead of failing.
  // (Bugbot review on PR #86 caught the race: previously, the magic
  // memcpy and other header writes weren't release-synchronized with
  // any acquire load, so an attacher could read magic before version
  // / cell_area_offset / cell_area_bytes were visible.)
  std::atomic<std::uint32_t> init_complete;
  std::uint32_t  version;
  char           magic[8];
  std::uint32_t  topology_hash;
  std::uint32_t  max_cells;
  std::uint32_t  cell_area_offset;   // bytes from start of segment
  std::uint32_t  cell_area_bytes;
  std::atomic<std::uint32_t> cell_count;
  std::uint8_t   reserved[28];
};
static_assert(sizeof(Header) == kHeaderSize, "Header must be 64 bytes");

// Cell layout — 64 bytes / one cache line per cell:
//
//   bytes 0..3    value      — the wire value (low bits per type)
//   bytes 4..7    write_gen  — monotonic write count. 0 = never written;
//                              every write_*() bumps it by exactly 1. The
//                              substrate-side ground truth that dissolves
//                              the "absent producer reads as zero" failure
//                              class (round-3 reviews; round-4 design memo
//                              docs/wire_truth_written_ness.md).
//   bytes 8..63   padding    — false-sharing isolation. Reserved for the
//                              multi-word (bytes[N]) seqlock sketched for a
//                              future per-cell or per-group wake mechanism;
//                              the seqlock's sequence counter unifies with
//                              write_gen when that arrives.
//
// Writer ordering — value first (release), then write_gen bump (also
// release, via bump_write_gen_saturating). A reader who observes
// write_gen >= K (acquire) is guaranteed by the release/acquire pair
// to also observe the matching value store, because the value store
// is sequenced-before the gen bump in the writer. write_gen
// saturates at UINT32_MAX rather than wrapping to 0 — see
// bump_write_gen_saturating below for the rationale.
struct alignas(64) Cell {
  std::atomic<std::uint32_t> value;      // scalar types <= 32 bits
  std::atomic<std::uint32_t> write_gen;
  std::atomic<std::uint64_t> value64;    // kUint64 storage (Phase C-a)
};
static_assert(sizeof(Cell) == 64, "Cell must be 64 bytes");
// value64 lives in the previously-unused tail of the 64-byte slot, so
// adding it changes neither the slot stride nor the layout of the 32-bit
// `value` word — every pre-existing scalar type is byte-for-byte
// identical in the segment. kUint64 must be a lock-free 64-bit atomic
// for cross-process sharing (a libatomic spinlock would be process-local
// and silently break the shared cell); assert it at compile time.
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "kUint64 cells require a lock-free 64-bit atomic");

// ── Co-sim tick barrier state (primitive-4 B) ───────────────────────────────
// Overlaid on Header::reserved[28] (byte offset 36). Three lock-free 32-bit
// atomics = 12 bytes of 28; ABI-invisible — reserved[] is not in kTopologyHash,
// no kFormatVersion bump, and the creator zero-fills it so a fresh or
// pre-barrier segment reads as a fully-inert (enabled == 0) BarrierState.
// @design 2026-06-28 claude — MEMBERSHIP/LIVENESS trade-off. An earlier sketch
// had 5 atomics including an `epoch` and an
// `expected` consumer count for self-registration. The shipped struct drops
// those two: consumer count is passed out-of-band (EV1SIM_FLEET_N, set by
// vat/fleet.py only for an all-converted fleet) and liveness is a bounded
// timeout, not self-registration. Consequence + accepted deviation from
// this codebase's usual self-healing "next locker recovers" robust-mutex
// posture: if a consumer
// CRASHES mid-tick the leader can't reach ack_count and eats the full
// barrier_await_acks timeout that tick (it never wedges *forever* — that
// posture is still honored — but it isn't instant-self-healing either).
// Acceptable because the barrier
// is opt-in (armed only for ev1sim-led, all-converted fleets) so it never
// touches the general substrate, and the timeout is generous by design (live-
// but-slow peers are the normal case). Hardening path if it bites: have the
// leader treat a consumer PID-exit (vat already monitors fleet PIDs) as an
// immediate abort instead of waiting the timeout. Tracked alongside the proposal.
struct BarrierState {
  std::atomic<std::uint32_t> enabled;      // 0 = inert; 1 = leader armed
  std::atomic<std::uint32_t> publish_gen;  // per-tick generation (consumer-wait word)
  std::atomic<std::uint32_t> ack_count;    // consumers that acked the tick (leader-wait word)
};
static_assert(sizeof(BarrierState) <= sizeof(Header::reserved),
              "BarrierState must fit in Header::reserved[]");
static_assert(std::atomic<std::uint32_t>::is_always_lock_free,
              "barrier counters require lock-free 32-bit atomics");

// reserved[] starts at offset 36 (4-byte aligned: every prior header field is a
// 4- or 8-byte scalar/atomic), and each BarrierState member is a 4-byte atomic,
// so all three are naturally aligned. The creator memsets reserved to 0 before
// publishing init_complete, so this overlay reads as zero (inert) until armed.
inline BarrierState* barrier_of(Header* h) {
  return reinterpret_cast<BarrierState*>(h->reserved);
}
inline const BarrierState* barrier_of(const Header* h) {
  return reinterpret_cast<const BarrierState*>(h->reserved);
}

// ── Advisory cross-process wake for the barrier ─────────────────────────────
// Linux: futex(FUTEX_WAIT/WAKE) on the shared 32-bit word — cross-process
// correct because the (non-PRIVATE) futex keys on the underlying physical page
// of a MAP_SHARED mapping, not the per-process virtual address. Elsewhere: a
// portable short-sleep poll. Both are ADVISORY: the atomic is the source of
// truth and every wait re-checks under a bounded slice, so a lost/absent wake
// only costs latency, never correctness (docs/wire_truth_substrate.md).
constexpr int kBarrierWaitSliceMs = 25;  // max latency a missed wake can add

inline void barrier_futex_wake(std::atomic<std::uint32_t>* word) {
#if defined(__linux__)
  ::syscall(SYS_futex, reinterpret_cast<std::uint32_t*>(word),
            FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
#else
  (void)word;  // poll fallback: the waiter re-checks on its own slice.
#endif
}

// Block while *word == expected, up to a single ~kBarrierWaitSliceMs slice.
// Returns immediately if the value already differs. Returns regardless after
// the slice (the caller's loop re-checks the atomic and the overall deadline).
inline void barrier_futex_wait_slice(std::atomic<std::uint32_t>* word,
                                     std::uint32_t expected) {
#if defined(__linux__)
  struct timespec ts;
  ts.tv_sec = 0;
  ts.tv_nsec = static_cast<long>(kBarrierWaitSliceMs) * 1000000L;
  ::syscall(SYS_futex, reinterpret_cast<std::uint32_t*>(word),
            FUTEX_WAIT, expected, &ts, nullptr, 0);
#else
  (void)word;
  (void)expected;
  std::this_thread::sleep_for(std::chrono::milliseconds(kBarrierWaitSliceMs));
#endif
}

// Wait until `word` != `expected`, up to timeout_ms of WALL time (the wait
// duration does not affect any sim-time computation — it only governs how long
// we block — so using steady_clock here does not reintroduce nondeterminism).
// Returns true if the value changed, false on timeout.
inline bool barrier_wait_word_changed(std::atomic<std::uint32_t>* word,
                                      std::uint32_t expected, int timeout_ms) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  for (;;) {
    if (word->load(std::memory_order_acquire) != expected) return true;
    if (std::chrono::steady_clock::now() >= deadline) {
      return word->load(std::memory_order_acquire) != expected;
    }
    barrier_futex_wait_slice(word, expected);
  }
}

// {raw, generation} snapshot of a Cell. Internal — the typed
// read_*_sample accessors decode raw to the cell's type and copy
// generation through.
struct RawSample {
  std::uint32_t raw;
  std::uint32_t generation;
};

// Seqlock-style snapshot: gen-first, value, gen-again. When the two
// gen loads match, the {value, gen} pair is exactly at that
// generation — no writer interleaved. If the gens differ, retry
// once; if a writer is STILL interleaving on the retry (very
// high-frequency producer), accept the latest gen with a fresh value
// load. In that fallback the value is still a real prior value and
// the gen is the latest seen — the pair is "value >= gen" rather
// than "value == gen", which is fine for our use cases (a consumer
// only needs `gen != 0` to dissolve Pattern A; freshness diagnostics
// tolerate a slight overestimate of gen).
//
// Why gen-first instead of value-first: with value-first, on a
// strong-acquire reader the gen.load(acquire) synchronizes-with the
// writer's release on gen, but the earlier value.load is NOT
// constrained by that synchronization — the reader could observe
// {value=initial, gen=1} (gen advanced, but value load happened
// before the writer's value store was visible). gen-first inverts
// the dependency so the synchronization established by the gen
// acquire bounds the subsequent value load too.
//
// @design 2026-06-11 claude — round-4, step 4a.
RawSample read_raw_sample(const Cell* cell) {
  for (int attempt = 0; attempt < 2; ++attempt) {
    const std::uint32_t g_before =
        cell->write_gen.load(std::memory_order_acquire);
    const std::uint32_t raw =
        cell->value.load(std::memory_order_acquire);
    const std::uint32_t g_after =
        cell->write_gen.load(std::memory_order_acquire);
    if (g_before == g_after) {
      return {raw, g_after};
    }
  }
  const std::uint32_t g =
      cell->write_gen.load(std::memory_order_acquire);
  const std::uint32_t raw =
      cell->value.load(std::memory_order_acquire);
  return {raw, g};
}

// {raw64, generation} snapshot — the kUint64 analogue of RawSample.
struct RawSample64 {
  std::uint64_t raw;
  std::uint32_t generation;
};

// Same gen-first seqlock as read_raw_sample, reading the 64-bit
// value64 word instead of the 32-bit value word. The 64-bit load is a
// single lock-free atomic (asserted at the Cell definition), so there
// is no intra-word tearing to guard against; the gen-before == gen-after
// check still guards against a writer interleaving between the value
// store and the gen bump (the same Pattern-A / freshness contract the
// 32-bit path provides).
RawSample64 read_raw_sample64(const Cell* cell) {
  for (int attempt = 0; attempt < 2; ++attempt) {
    const std::uint32_t g_before =
        cell->write_gen.load(std::memory_order_acquire);
    const std::uint64_t raw =
        cell->value64.load(std::memory_order_acquire);
    const std::uint32_t g_after =
        cell->write_gen.load(std::memory_order_acquire);
    if (g_before == g_after) {
      return {raw, g_after};
    }
  }
  const std::uint32_t g =
      cell->write_gen.load(std::memory_order_acquire);
  const std::uint64_t raw =
      cell->value64.load(std::memory_order_acquire);
  return {raw, g};
}

// Saturated increment of Cell::write_gen. Bumps by 1 unless the
// counter is already at UINT32_MAX, in which case it stays there. The
// motivation is correctness, not performance: a plain fetch_add wraps
// modulo 2^32, so a cell written UINT32_MAX times would roll back to
// gen == 0 and silently re-appear never-written — Pattern A
// re-emerging at the substrate level after a long uptime. At today's
// fleet rates (10-100 Hz per wire) the wrap is 1-13 years away, but
// the parent memo's step-4 GM-8192 bit migration targets 8192 Hz
// toggling per serial segment, which crosses UINT32_MAX in ~6 days
// continuous — well inside a long-running simulation lifetime.
// Saturate, don't wrap.
//
// The CAS-loop form is race-free under any writer-count contract; the
// topology declares ONE driver per cell, so the CAS succeeds on the
// first try in steady state and the overhead is one extra atomic load
// vs the plain fetch_add. Once saturated, subsequent writes leave gen
// at UINT32_MAX; the value still publishes correctly because the
// value.store(release) in the caller synchronizes-with a reader's
// value.load(acquire) directly (same-atomic release/acquire pair).
// The only feature lost post-saturation is gen-delta-as-liveness
// detection, which is acceptable: a consumer that has seen 2^32 gen
// bumps is no longer concerned about producer liveness.
//
// @design 2026-06-11 claude — round-4 self-review fix.
void bump_write_gen_saturating(Cell& cell) {
  std::uint32_t old = cell.write_gen.load(std::memory_order_relaxed);
  for (;;) {
    if (old == UINT32_MAX) return;  // already saturated; leave it
    if (cell.write_gen.compare_exchange_weak(
            old, old + 1u,
            std::memory_order_release,
            std::memory_order_relaxed)) {
      return;
    }
    // CAS failed — `old` now holds the actual current value; retry.
  }
}

// ─── bit-stream cell (kBitStream) ──────────────────────────────────────
//
// A single-producer / multiple-independent-reader bit FIFO living in shm.
// Layout (all in the cell block, 64-aligned start via CellDescriptor::
// offset, same as a scalar cell):
//
//   seq          — seqlock counter. EVEN = stable, ODD = a writer is
//                  mid-append. Readers retry while it is odd or changes
//                  across the read. Single writer (the declared driver),
//                  so the protocol is the classic single-writer seqlock.
//   total_bits   — monotonic count of bits EVER appended. ring[i % CAP]
//                  holds the bit at absolute index i; a reader replays
//                  absolute indices [cursor, total_bits).
//   write_gen    — mirror of the scalar Cell::write_gen so has_cell /
//                  write_gen() / for_each_unwritten treat a bit-stream
//                  cell uniformly (0 = never appended). Bumped (saturating)
//                  inside the seqlock-protected append.
//   ring[]       — CAP bits packed 8-per-byte. ring bit for absolute index
//                  i is byte (i/8 % CAP_BYTES), bit (i & 7).
//
// CAP = WireTable::kBitStreamCapacityBits (1024). The block is sized to
// the next 64-byte multiple so cells stay cache-line aligned and the
// false-sharing isolation of the scalar cells is preserved.
//
// SEQLOCK PROTOCOL (the highest-risk part — get the ordering right):
//
//   Writer (append):
//     0. s = seq.load(relaxed); if (s odd) ++s          // S2 normalize:
//        an adopted orphan (producer crashed mid-append) can leave seq ODD;
//        round up to the next even before resuming, or the odd/even meaning
//        inverts permanently for that cell.
//     1. seq.store(s+1, release)                        // → ODD: "writing"
//     2. write ring bytes (RELAXED ATOMIC) + total_bits (relaxed); bump gen.
//     3. seq.store(s+2, release)                        // → EVEN: "stable"
//
//   Publication is carried SOLELY by the step-3 even-store (release), paired
//   with the reader's s2 acquire load + recheck. There is deliberately NO
//   standalone release fence after the odd-store — a release fence there
//   establishes no usable synchronizes-with edge (N1, review 2026-06-13);
//   the even-store is what every reader synchronizes-with.
//
//   B1: ring bytes are std::atomic<uint8_t> accessed memory_order_relaxed.
//   The seqlock throws away torn snapshots, but a seqlock over *plain*
//   (non-atomic) data is still a data race / UB in ISO C++ (Boehm P1478) —
//   relaxed atomics make the per-byte races benign while the seq release/
//   acquire pair carries the ordering. No extra cost on the fast path.
//
//   Reader (read_bits_since / total):
//     for (bounded retries) {                   // S1: NOT unbounded
//       s1 = seq.load(acquire);                 // if odd, writer mid-append
//       if (s1 & 1) continue;                   // spin/retry on odd (capped)
//       <read total_bits + ring slice, relaxed atomic>
//       atomic_thread_fence(acquire);           // order the reads before s2
//       s2 = seq.load(acquire);
//       if (s1 == s2) break;                    // consistent snapshot
//     }                                         // a writer interleaved → retry
//
//   S1 (review 2026-06-13): the odd-spin and the s1!=s2 retry are BOUNDED by
//   kSeqlockMaxRetries. If a single producer is killed between its odd-store
//   and even-store, seq stays ODD forever; an unbounded reader spin would
//   hang the fleet tick loop (UartRx::tick) for every consumer. On exceeding
//   the cap the reader treats it as "no new bits this tick" (got=0, ok=true,
//   cursor unchanged) and returns rather than blocking — mirroring the
//   self-healing "next locker recovers" contract used by this codebase's
//   robust-mutex transport. The normal
//   uncontended / single-in-flight-append path returns on the first or
//   second iteration, so the cap never bites in practice.
//
//   Why this is correct cross-process: seq is a process-shared atomic in
//   the mapped segment. The writer's even-store (release) synchronizes-with
//   the reader's s2 acquire load; when s1 == s2 (and even), every ring/total
//   store the writer made happened-before the reader's reads (the acquire
//   fence orders the relaxed reads before the s2 load). A torn read (writer
//   interleaving) is caught by s1 != s2 and retried (bounded).
//
// @design 2026-06-13 claude — round-19, sub-PR 4-C½;
//   notes/manual_supplements.yaml#2026-06-13-gm8192-bit-ring-cell.
constexpr std::size_t kBitStreamCapBits  = WireTable::kBitStreamCapacityBits;
constexpr std::size_t kBitStreamCapBytes = (kBitStreamCapBits + 7) / 8;  // 128
static_assert(kBitStreamCapBits % 8 == 0,
              "bit-stream capacity must be a whole number of bytes");

struct BitStreamCell {
  std::atomic<std::uint64_t> seq;         // even=stable, odd=writing
  std::atomic<std::uint64_t> total_bits;  // monotonic bits-ever-appended
  std::atomic<std::uint32_t> write_gen;   // 0 = never appended
  std::uint32_t              pad;         // align ring to 8
  // B1 (adversarial substrate review 2026-06-13): the ring payload is
  // accessed concurrently by the writer (relaxed RMW) and cross-process
  // readers (relaxed load) while the seqlock discards torn snapshots. A
  // seqlock over *plain* data is data-racy / UB under the ISO C++ memory
  // model even when torn results are thrown away (Boehm P1478). Making the
  // bytes std::atomic with memory_order_relaxed makes the per-byte races
  // benign-by-definition; the seq release/acquire pair still carries all the
  // happens-before ordering. std::atomic<std::uint8_t> is lock-free and
  // address-free on every target (asserted below), so this is safe in
  // cross-process shm and keeps each cell exactly one byte — the segment
  // layout / cell-block size is unchanged.
  std::atomic<std::uint8_t>  ring[kBitStreamCapBytes];
};

// B1: cross-process atomics must be lock-free (no hidden lock table that a
// second process can't see) AND a single byte each (so the ring layout and
// kBitStreamCellBytes are byte-identical to the prior plain-uint8_t array).
static_assert(std::atomic<std::uint8_t>::is_always_lock_free,
              "ring atomics must be lock-free for cross-process shm");
static_assert(sizeof(std::atomic<std::uint8_t>) == 1,
              "atomic ring byte must stay one byte (segment layout invariant)");
static_assert(alignof(std::atomic<std::uint8_t>) == 1,
              "atomic ring byte must keep 1-byte alignment (no layout shift)");

// S1 (adversarial substrate review 2026-06-13): the seqlock reader's
// odd-spin / s1!=s2 retry is BOUNDED by this cap. A single producer killed
// between its odd-store and even-store leaves seq permanently ODD; an
// unbounded reader spin would wedge the fleet tick loop for every consumer
// (UartRx::tick runs from that loop). On exceeding the cap a reader gives up
// for this tick — "no new bits" — instead of blocking. The cap is generous
// (a healthy append toggles seq twice and completes in nanoseconds, so a
// genuine in-flight append is observed-stable within one or two iterations);
// it exists only to break the dead-producer livelock, not to time-slice a
// live writer. Process-local (not stored in the shared segment).
constexpr unsigned kSeqlockMaxRetries = 4096;

// The block a bit-stream cell occupies, rounded up to a whole number of
// 64-byte slots so every cell start stays cache-line aligned.
constexpr std::size_t kBitStreamCellBytes =
    (sizeof(BitStreamCell) + 63) & ~static_cast<std::size_t>(63);
static_assert(kBitStreamCellBytes % kCellSize == 0,
              "bit-stream cell block must be a whole number of 64-byte slots");

// Bytes a cell of the given type occupies in the cell area. Scalar types
// occupy exactly one 64-byte slot (unchanged from the original uniform
// layout); kBitStream occupies its multi-slot block.
std::size_t cell_block_bytes(std::uint8_t type) {
  return type == static_cast<std::uint8_t>(WireType::kBitStream)
             ? kBitStreamCellBytes
             : kCellSize;
}

// Read `bit` (0/1) at absolute index `i` from a ring of CAP bits.
// B1: the byte load is a relaxed atomic; the seqlock acquire-fence + s2
// recheck (in read_bits_since) carries the happens-before ordering.
inline bool ring_get_bit(const std::atomic<std::uint8_t>* ring,
                         std::uint64_t i) {
  const std::uint64_t pos = i % kBitStreamCapBits;
  return (ring[pos >> 3].load(std::memory_order_relaxed) >> (pos & 7u)) & 1u;
}

// Set `bit` (0/1) at absolute index `i` in a ring of CAP bits.
// B1: relaxed-atomic byte load/store; the single declared producer is the
// only writer, so this stays a plain (uncontended) RMW on the write side.
inline void ring_set_bit(std::atomic<std::uint8_t>* ring, std::uint64_t i,
                         bool value) {
  const std::uint64_t pos = i % kBitStreamCapBits;
  const std::uint8_t mask = static_cast<std::uint8_t>(1u << (pos & 7u));
  std::uint8_t byte = ring[pos >> 3].load(std::memory_order_relaxed);
  if (value) {
    byte = static_cast<std::uint8_t>(byte | mask);
  } else {
    byte = static_cast<std::uint8_t>(byte & ~mask);
  }
  ring[pos >> 3].store(byte, std::memory_order_relaxed);
}

constexpr std::size_t kDirectoryOffset = kHeaderSize;

std::size_t cell_area_offset(std::size_t max_cells) {
  const std::size_t after_dir =
      kDirectoryOffset + max_cells * sizeof(CellDescriptor);
  // Align up to 64 bytes so cells start on a cache-line boundary.
  return (after_dir + 63) & ~static_cast<std::size_t>(63);
}

std::size_t total_segment_bytes(std::size_t max_cells,
                                std::size_t cell_area_bytes) {
  return cell_area_offset(max_cells) + cell_area_bytes;
}

// Normalize "foo" → "/foo"; accept already-leading-"/" names too.
std::string posix_name(const std::string& name) {
  if (!name.empty() && name[0] == '/') return name;
  return "/" + name;
}

#if defined(_WIN32)
// Win32 shared-memory object name. The kernel resolves names in the
// per-session `Local\` namespace by default; strip any leading
// POSIX-style slash so the same configured `name` works on both
// platforms (an attacher passing `/esct_foo` on Windows would otherwise
// create a name in a `\` subnamespace that the producer's
// `CreateFileMappingA("esct_foo")` can't resolve).
std::string win32_name(const std::string& name) {
  if (!name.empty() && name[0] == '/') return name.substr(1);
  return name;
}
#endif

}  // namespace

struct WireTable::Impl {
  bool is_creator{false};
  std::string segment_name;       // includes leading "/" on POSIX; bare name on Win32
  std::size_t mapping_size{0};
  void* mapped{nullptr};

#if defined(_WIN32)
  HANDLE hMapping{nullptr};       // CreateFileMappingA / OpenFileMappingA handle
#else
  int fd{-1};
#endif

  Header* header{nullptr};
  CellDescriptor* directory{nullptr};
  std::uint8_t* cell_area{nullptr};

  // Process-local cache. Populated on attach (snapshot scan) and
  // updated on declare() / on lookup-miss re-scan. `mutable` because
  // const read paths (read_*, has_cell) may insert on a successful
  // late-declare lookup. Process-private; not shared via the segment.
  // For single-thread-per-process access this is fine; the design memo
  // notes that multi-threaded access within one process is future work
  // and would need a small mutex around updates.
  mutable std::unordered_map<WireId, std::uint32_t> id_to_index;

  ~Impl() { cleanup(); }

  void cleanup() {
    if (mapped != nullptr && mapping_size > 0) {
#if defined(_WIN32)
      ::UnmapViewOfFile(mapped);
#else
      ::munmap(mapped, mapping_size);
#endif
    }
    mapped = nullptr;
#if defined(_WIN32)
    if (hMapping != nullptr) {
      ::CloseHandle(hMapping);
      hMapping = nullptr;
    }
    // Win32 has no shm_unlink: when the last handle to a paging-file-
    // backed mapping is closed, the kernel reclaims the name. The
    // `is_creator` flag + segment_name distinction is therefore moot on
    // Windows; cleanup is implicit. (POSIX path keeps the explicit
    // unlink because POSIX shm names outlive their last fd.)
#else
    if (fd != -1) {
      ::close(fd);
      fd = -1;
    }
    if (is_creator && !segment_name.empty()) {
      electricsim::io::shm_unlink_quiet(segment_name);
    }
#endif
  }

  // Look up the directory index for an id with type-tag verification.
  // On cache miss, re-scan the directory in case new cells were
  // declared after this table was attached. (Bugbot PR #86 second-pass
  // medium severity: attach()'s snapshot of id_to_index was static,
  // so a creator that called declare() AFTER an attacher joined left
  // those cells invisible to the attacher even though cell_count
  // reflected the shared total.)
  bool lookup(WireId id, WireType expected_type,
              std::uint32_t* out_index) const {
    auto it = id_to_index.find(id);
    if (it != id_to_index.end()) {
      const std::uint32_t idx = it->second;
      if (directory[idx].type != static_cast<std::uint8_t>(expected_type)) {
        return false;
      }
      *out_index = idx;
      return true;
    }
    // Cache miss — scan the live directory under acquire ordering on
    // cell_count. Re-load cell_count after each pass and continue
    // scanning any newly-published entries until the count is stable;
    // a single snapshot would skip declares that publish between our
    // initial load and our return (Bugbot PR #86 third-round medium
    // severity).
    //
    // Cap the scanned count at max_cells (immutable, validated at
    // attach time). A misbehaving or hostile creator could store an
    // inflated cell_count into shared memory; without the cap, this
    // function would index past the directory's actual bounds.
    // (Bugbot PR #86 fourth-round medium severity.)
    const std::uint32_t max_cells = header->max_cells;
    std::uint32_t scanned = 0;
    while (true) {
      std::uint32_t count =
          header->cell_count.load(std::memory_order_acquire);
      if (count > max_cells) count = max_cells;
      if (count == scanned) break;
      for (std::uint32_t i = scanned; i < count; ++i) {
        if (directory[i].id == id) {
          if (directory[i].type != static_cast<std::uint8_t>(expected_type)) {
            return false;
          }
          id_to_index[id] = i;  // cache for next time
          *out_index = i;
          return true;
        }
      }
      scanned = count;
    }
    return false;
  }

  // Like lookup() but without type checking; used by has_cell().
  bool resolve_index(WireId id, std::uint32_t* out_index) const {
    auto it = id_to_index.find(id);
    if (it != id_to_index.end()) {
      *out_index = it->second;
      return true;
    }
    // Same retry-until-stable scan as lookup(); see comment there.
    // Cap by max_cells for the same OOB-protection reason.
    const std::uint32_t max_cells = header->max_cells;
    std::uint32_t scanned = 0;
    while (true) {
      std::uint32_t count =
          header->cell_count.load(std::memory_order_acquire);
      if (count > max_cells) count = max_cells;
      if (count == scanned) break;
      for (std::uint32_t i = scanned; i < count; ++i) {
        if (directory[i].id == id) {
          id_to_index[id] = i;
          *out_index = i;
          return true;
        }
      }
      scanned = count;
    }
    return false;
  }

  Cell* cell_at(std::uint32_t idx) const {
    return reinterpret_cast<Cell*>(cell_area + directory[idx].offset);
  }

  BitStreamCell* bit_stream_at(std::uint32_t idx) const {
    return reinterpret_cast<BitStreamCell*>(cell_area + directory[idx].offset);
  }

  // Running byte offset of the NEXT cell to be declared (creator only).
  // Variable-stride bump allocator: scalar cells advance it by one 64-byte
  // slot, a kBitStream cell by its multi-slot block. For a scalar-only
  // topology this reproduces the original `current * kCellSize` layout
  // byte-for-byte (offsets 0, 64, 128, ...). Process-local; declare() is
  // creator-only + single-threaded so no synchronization is needed.
  std::uint32_t next_cell_offset{0};
};

WireTable::WireTable() : impl_(std::make_unique<Impl>()) {}

WireTable::~WireTable() = default;

// ─── factories ─────────────────────────────────────────────────────

std::unique_ptr<WireTable> WireTable::create(const WireTableOptions& opts) {
  return create(opts, nullptr);
}

std::unique_ptr<WireTable> WireTable::create(
    const WireTableOptions& opts,
    const std::function<bool(WireTable&)>& declare_fn) {
  // Reject zero-sized options at the create() seam, matching the
  // checks attach() performs against the published header. Without
  // this, create()-then-attach() would succeed on the creator and
  // fail on every attacher — quiet, asymmetric "validity" that's
  // hard to debug. (Bugbot PR #86 third-round low severity.)
  if (opts.max_cells == 0) {
    std::fprintf(stderr,
                 "WireTable::create: max_cells must be > 0 "
                 "(otherwise no cells can be declared)\n");
    return nullptr;
  }
  if (opts.cell_area_bytes < kCellSize) {
    std::fprintf(stderr,
                 "WireTable::create: cell_area_bytes (%zu) must be at "
                 "least one cell (%zu)\n",
                 opts.cell_area_bytes, kCellSize);
    return nullptr;
  }
  // The header stores max_cells / cell_area_offset / cell_area_bytes as
  // uint32_t. On a 64-bit host, size_t options larger than UINT32_MAX
  // would truncate silently when written into the header — e.g.
  // max_cells == 2^32 truncates to 0, passing the ==0 guard above yet
  // storing 0 in shared memory, so create() succeeds while declare()
  // always fails and attach() rejects the segment. Reject anything that
  // would not round-trip through the 32-bit fields, including the
  // post-alignment cell_area_offset. (Bugbot finding on epic PR #85.)
  if (opts.max_cells > UINT32_MAX) {
    std::fprintf(stderr,
                 "WireTable::create: max_cells (%zu) exceeds uint32 header "
                 "field limit (%u)\n",
                 opts.max_cells, UINT32_MAX);
    return nullptr;
  }
  if (opts.cell_area_bytes > UINT32_MAX) {
    std::fprintf(stderr,
                 "WireTable::create: cell_area_bytes (%zu) exceeds uint32 "
                 "header field limit (%u)\n",
                 opts.cell_area_bytes, UINT32_MAX);
    return nullptr;
  }
  if (cell_area_offset(opts.max_cells) > UINT32_MAX) {
    std::fprintf(stderr,
                 "WireTable::create: directory for max_cells (%zu) pushes "
                 "cell_area_offset past the uint32 header field limit\n",
                 opts.max_cells);
    return nullptr;
  }

  auto table = std::unique_ptr<WireTable>(new WireTable());
  Impl* impl = table->impl_.get();
  impl->is_creator = true;
  impl->mapping_size = total_segment_bytes(opts.max_cells, opts.cell_area_bytes);

#if defined(_WIN32)
  // Win32 backend: CreateFileMappingA with INVALID_HANDLE_VALUE backs
  // the mapping in the paging file (matches POSIX shm_open semantics —
  // ephemeral, per-boot, no file-system footprint). The size is encoded
  // in the high/low DWORD pair on the create call (POSIX path uses a
  // separate ftruncate). ERROR_ALREADY_EXISTS after a non-null return
  // means another live process opened the same name first; refuse to
  // adopt under the O_EXCL invariant the POSIX path also enforces.
  impl->segment_name = win32_name(opts.name);
  const std::uint64_t sz = impl->mapping_size;
  impl->hMapping = ::CreateFileMappingA(
      INVALID_HANDLE_VALUE,
      nullptr,
      PAGE_READWRITE,
      static_cast<DWORD>((sz >> 32) & 0xFFFFFFFFULL),
      static_cast<DWORD>(sz & 0xFFFFFFFFULL),
      impl->segment_name.c_str());
  if (impl->hMapping == nullptr) {
    std::fprintf(stderr,
                 "WireTable::create: CreateFileMappingA(%s) failed: error %lu\n",
                 impl->segment_name.c_str(),
                 static_cast<unsigned long>(::GetLastError()));
    return nullptr;
  }
  if (::GetLastError() == ERROR_ALREADY_EXISTS) {
    std::fprintf(stderr,
                 "WireTable::create: segment %s already exists; refusing to "
                 "adopt under O_EXCL semantics\n",
                 impl->segment_name.c_str());
    ::CloseHandle(impl->hMapping);
    impl->hMapping = nullptr;
    return nullptr;
  }
  impl->mapped = ::MapViewOfFile(impl->hMapping, FILE_MAP_ALL_ACCESS, 0, 0,
                                 static_cast<SIZE_T>(impl->mapping_size));
  if (impl->mapped == nullptr) {
    std::fprintf(stderr,
                 "WireTable::create: MapViewOfFile(%s) failed: error %lu\n",
                 impl->segment_name.c_str(),
                 static_cast<unsigned long>(::GetLastError()));
    return nullptr;
  }
  // Win32 maps a freshly-created paging-file-backed mapping zero-filled,
  // matching POSIX `mmap(shm_open + ftruncate)`.
#else
  impl->segment_name = posix_name(opts.name);
  impl->fd = ::shm_open(impl->segment_name.c_str(),
                        O_RDWR | O_CREAT | O_EXCL, 0666);
  if (impl->fd < 0) {
    std::fprintf(stderr,
                 "WireTable::create: shm_open(%s) failed: %s\n",
                 impl->segment_name.c_str(), std::strerror(errno));
    impl->segment_name.clear();  // don't shm_unlink someone else's segment
    return nullptr;
  }
  if (::ftruncate(impl->fd, static_cast<off_t>(impl->mapping_size)) != 0) {
    std::fprintf(stderr,
                 "WireTable::create: ftruncate(%s, %zu) failed: %s\n",
                 impl->segment_name.c_str(), impl->mapping_size,
                 std::strerror(errno));
    return nullptr;  // dtor will shm_unlink (we did create it)
  }
  impl->mapped = ::mmap(nullptr, impl->mapping_size,
                        PROT_READ | PROT_WRITE, MAP_SHARED, impl->fd, 0);
  if (impl->mapped == MAP_FAILED) {
    impl->mapped = nullptr;
    std::fprintf(stderr,
                 "WireTable::create: mmap(%s) failed: %s\n",
                 impl->segment_name.c_str(), std::strerror(errno));
    return nullptr;
  }
#endif

  // Initialize header. mmap of a freshly-truncated segment is
  // zero-filled, so init_complete starts at 0 ("creator still
  // populating"). Write every other field with plain stores; the
  // single release store of init_complete=1 at the very end
  // synchronizes-with any attacher that observes it via acquire,
  // making all the prior stores visible.
  impl->header = static_cast<Header*>(impl->mapped);
  impl->header->version = kFormatVersion;
  impl->header->topology_hash = opts.topology_hash;
  impl->header->max_cells = static_cast<std::uint32_t>(opts.max_cells);
  impl->header->cell_area_offset =
      static_cast<std::uint32_t>(cell_area_offset(opts.max_cells));
  impl->header->cell_area_bytes =
      static_cast<std::uint32_t>(opts.cell_area_bytes);
  impl->header->cell_count.store(0, std::memory_order_relaxed);
  std::memset(impl->header->reserved, 0, sizeof(impl->header->reserved));
  std::memcpy(impl->header->magic, kMagic, sizeof(kMagic));

  // Set up directory + cell_area pointers BEFORE running the optional
  // declare callback (declare() reads these). Attachers in attach()
  // are still polling init_complete; they won't see any of this until
  // we publish init_complete=1 below.
  impl->directory = reinterpret_cast<CellDescriptor*>(
      static_cast<std::uint8_t*>(impl->mapped) + kDirectoryOffset);
  impl->cell_area = static_cast<std::uint8_t*>(impl->mapped)
                  + impl->header->cell_area_offset;

  // Run the caller's declarations while init_complete is still 0.
  // This closes the race window where an attacher could observe
  // init_complete=1 with cell_count=0 and report spurious "wire not
  // found" results for cells the caller was about to declare.
  // (Bugbot review of epic PR #85.)
  if (declare_fn) {
    if (!declare_fn(*table)) {
      std::fprintf(stderr,
                   "WireTable::create: declare callback failed for segment %s\n",
                   impl->segment_name.c_str());
      // dtor will munmap + shm_unlink since is_creator=true and
      // segment_name is still set.
      return nullptr;
    }
  }

  // Publication barrier. Any later store (e.g. declare()) uses its own
  // release/acquire on cell_count and is independent of this fence.
  impl->header->init_complete.store(1, std::memory_order_release);
  return table;
}

std::unique_ptr<WireTable> WireTable::attach(const WireTableOptions& opts) {
  return attach_impl(opts, /*take_ownership=*/false);
}

std::unique_ptr<WireTable> WireTable::adopt(const WireTableOptions& opts) {
  return attach_impl(opts, /*take_ownership=*/true);
}

std::unique_ptr<WireTable> WireTable::attach_impl(const WireTableOptions& opts,
                                                 bool take_ownership) {
  auto table = std::unique_ptr<WireTable>(new WireTable());
  Impl* impl = table->impl_.get();
  // Keep is_creator=false through mapping + validation so that ANY
  // early-return failure path destroys the handle WITHOUT unlinking
  // the segment — we must never unlink a segment we failed to validate
  // (it may belong to a healthy peer, or be a live creator mid-init).
  // Ownership is conferred only on the success path below, just before
  // we return.
  impl->is_creator = false;

#if defined(_WIN32)
  // Win32 backend: OpenFileMappingA returns a handle to the existing
  // named mapping (NULL if missing — matches POSIX shm_open's ENOENT).
  // MapViewOfFile(... 0) maps the entire mapping; VirtualQuery on the
  // result reports the actual region size (POSIX uses fstat for the
  // same purpose).
  impl->segment_name = win32_name(opts.name);
  impl->hMapping = ::OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE,
                                      impl->segment_name.c_str());
  if (impl->hMapping == nullptr) {
    std::fprintf(stderr,
                 "WireTable::attach: OpenFileMappingA(%s) failed: error %lu\n",
                 impl->segment_name.c_str(),
                 static_cast<unsigned long>(::GetLastError()));
    return nullptr;
  }
  impl->mapped = ::MapViewOfFile(impl->hMapping, FILE_MAP_ALL_ACCESS, 0, 0, 0);
  if (impl->mapped == nullptr) {
    std::fprintf(stderr,
                 "WireTable::attach: MapViewOfFile(%s) failed: error %lu\n",
                 impl->segment_name.c_str(),
                 static_cast<unsigned long>(::GetLastError()));
    return nullptr;
  }
  MEMORY_BASIC_INFORMATION mbi{};
  if (::VirtualQuery(impl->mapped, &mbi, sizeof(mbi)) != sizeof(mbi)) {
    std::fprintf(stderr,
                 "WireTable::attach: VirtualQuery on mapping %s failed: "
                 "error %lu\n",
                 impl->segment_name.c_str(),
                 static_cast<unsigned long>(::GetLastError()));
    return nullptr;
  }
  impl->mapping_size = static_cast<std::size_t>(mbi.RegionSize);
  if (impl->mapping_size < kHeaderSize) {
    std::fprintf(stderr,
                 "WireTable::attach: segment %s too small (%zu bytes)\n",
                 impl->segment_name.c_str(), impl->mapping_size);
    return nullptr;
  }
#else
  impl->segment_name = posix_name(opts.name);
  impl->fd = ::shm_open(impl->segment_name.c_str(), O_RDWR, 0);
  if (impl->fd < 0) {
    std::fprintf(stderr,
                 "WireTable::attach: shm_open(%s) failed: %s\n",
                 impl->segment_name.c_str(), std::strerror(errno));
    return nullptr;
  }
  struct stat st{};
  if (::fstat(impl->fd, &st) != 0) {
    std::fprintf(stderr,
                 "WireTable::attach: fstat(%s) failed: %s\n",
                 impl->segment_name.c_str(), std::strerror(errno));
    return nullptr;
  }
  impl->mapping_size = static_cast<std::size_t>(st.st_size);
  if (impl->mapping_size < kHeaderSize) {
    std::fprintf(stderr,
                 "WireTable::attach: segment %s too small (%zu bytes)\n",
                 impl->segment_name.c_str(), impl->mapping_size);
    return nullptr;
  }
  impl->mapped = ::mmap(nullptr, impl->mapping_size,
                        PROT_READ | PROT_WRITE, MAP_SHARED, impl->fd, 0);
  if (impl->mapped == MAP_FAILED) {
    impl->mapped = nullptr;
    std::fprintf(stderr,
                 "WireTable::attach: mmap(%s) failed: %s\n",
                 impl->segment_name.c_str(), std::strerror(errno));
    return nullptr;
  }
#endif

  impl->header = static_cast<Header*>(impl->mapped);

  // The creator may be mid-initialization (shm_open / ftruncate done
  // but header not yet populated → init_complete is the zero-fill
  // value). Poll with acquire ordering until the creator publishes,
  // bounded by a generous timeout to fail loudly on a stuck creator.
  // (Bugbot PR #86 medium severity: previously, attach() returned
  // nullptr immediately on a zero header, breaking parallel attach
  // during creator setup.)
  {
    using namespace std::chrono;
    constexpr auto kPollInterval = microseconds(100);
    constexpr auto kAttachTimeout = seconds(5);
    const auto deadline = steady_clock::now() + kAttachTimeout;
    while (impl->header->init_complete.load(std::memory_order_acquire) != 1u) {
      if (steady_clock::now() >= deadline) {
        std::fprintf(stderr,
                     "WireTable::attach: timed out waiting for creator init "
                     "on segment %s (init_complete still 0 after %lds)\n",
                     impl->segment_name.c_str(),
                     static_cast<long>(kAttachTimeout.count()));
        return nullptr;
      }
      std::this_thread::sleep_for(kPollInterval);
    }
  }

  // init_complete == 1 with acquire synchronizes-with the creator's
  // release store; magic, version, sizes, etc. are now safe to read.
  if (std::memcmp(impl->header->magic, kMagic, sizeof(kMagic)) != 0) {
    std::fprintf(stderr,
                 "WireTable::attach: bad magic in segment %s\n",
                 impl->segment_name.c_str());
    return nullptr;
  }
  if (impl->header->version != kFormatVersion) {
    std::fprintf(stderr,
                 "WireTable::attach: version mismatch (segment=%u, expected=%u)\n",
                 impl->header->version, kFormatVersion);
    return nullptr;
  }
  if (opts.topology_hash != 0 &&
      impl->header->topology_hash != opts.topology_hash) {
    std::fprintf(stderr,
                 "WireTable::attach: topology hash mismatch "
                 "(segment=0x%08X, expected=0x%08X)\n",
                 impl->header->topology_hash, opts.topology_hash);
    return nullptr;
  }

  // Validate every header size against the mmap'd region before
  // computing any pointer or scanning the directory. Inflated or
  // corrupt values from a misbehaving / hostile producer must not be
  // able to make us read past the mapping. (Bugbot PR #86 medium
  // severity: cell_count was loaded and iterated without verifying it
  // fit in the segment, and the directory / cell-area sizes were
  // never checked.)
  const std::uint32_t max_cells = impl->header->max_cells;
  const std::uint32_t cell_area_off = impl->header->cell_area_offset;
  const std::uint32_t cell_area_sz  = impl->header->cell_area_bytes;
  if (max_cells == 0u) {
    std::fprintf(stderr,
                 "WireTable::attach: header max_cells is 0 in segment %s\n",
                 impl->segment_name.c_str());
    return nullptr;
  }
  const std::uint64_t dir_end =
      static_cast<std::uint64_t>(kDirectoryOffset)
    + static_cast<std::uint64_t>(max_cells) * sizeof(CellDescriptor);
  if (dir_end > impl->mapping_size) {
    std::fprintf(stderr,
                 "WireTable::attach: directory end (%llu) exceeds mapping (%zu) "
                 "in segment %s\n",
                 static_cast<unsigned long long>(dir_end),
                 impl->mapping_size, impl->segment_name.c_str());
    return nullptr;
  }
  if (cell_area_off < dir_end) {
    std::fprintf(stderr,
                 "WireTable::attach: cell_area_offset (%u) overlaps directory "
                 "(ends at %llu) in segment %s\n",
                 cell_area_off, static_cast<unsigned long long>(dir_end),
                 impl->segment_name.c_str());
    return nullptr;
  }
  const std::uint64_t cells_end =
      static_cast<std::uint64_t>(cell_area_off) + cell_area_sz;
  if (cells_end > impl->mapping_size) {
    std::fprintf(stderr,
                 "WireTable::attach: cell area end (%llu) exceeds mapping (%zu) "
                 "in segment %s\n",
                 static_cast<unsigned long long>(cells_end),
                 impl->mapping_size, impl->segment_name.c_str());
    return nullptr;
  }

  impl->directory = reinterpret_cast<CellDescriptor*>(
      static_cast<std::uint8_t*>(impl->mapped) + kDirectoryOffset);
  impl->cell_area = static_cast<std::uint8_t*>(impl->mapped) + cell_area_off;

  // Snapshot cell_count under acquire (pairs with declare()'s release
  // store) and validate it against the directory's capacity before
  // iterating. Each directory entry's per-cell offset must also fit
  // inside the cell area or cell_at() would read past the mapping.
  const std::uint32_t count =
      impl->header->cell_count.load(std::memory_order_acquire);
  if (count > max_cells) {
    std::fprintf(stderr,
                 "WireTable::attach: cell_count (%u) > max_cells (%u) "
                 "in segment %s — corrupt header\n",
                 count, max_cells, impl->segment_name.c_str());
    return nullptr;
  }
  impl->id_to_index.reserve(count);
  std::uint64_t high_water = 0;  // highest block-end seen, for adopt's allocator
  for (std::uint32_t i = 0; i < count; ++i) {
    const std::uint32_t entry_off = impl->directory[i].offset;
    // Each entry's full block (a kBitStream cell is wider than one slot)
    // must fit inside the cell area or cell_at()/bit_stream_at() would read
    // past the mapping. Round-19: the per-cell size is type-dependent.
    const std::size_t block = cell_block_bytes(impl->directory[i].type);
    const std::uint64_t entry_end =
        static_cast<std::uint64_t>(entry_off) + block;
    if (entry_end > cell_area_sz) {
      std::fprintf(stderr,
                   "WireTable::attach: directory entry %u has offset %u "
                   "+ block %zu outside cell area (size=%u) in segment %s "
                   "— corrupt\n",
                   i, entry_off, block, cell_area_sz,
                   impl->segment_name.c_str());
      return nullptr;
    }
    if (entry_end > high_water) high_water = entry_end;
    impl->id_to_index[impl->directory[i].id] = i;
  }
  // adopt() may declare() further cells; seed the bump allocator past the
  // existing cells so a new cell never overlaps an adopted one. (A plain
  // attach() can't declare() — is_creator stays false — so the value is
  // harmless there.)
  impl->next_cell_offset = static_cast<std::uint32_t>(high_water);
  // Validation passed — confer ownership now (adopt path) so the dtor
  // unlinks on clean shutdown. attach path leaves this false.
  impl->is_creator = take_ownership;
  return table;
}

// ─── declare ───────────────────────────────────────────────────────

bool WireTable::declare(WireId id, WireType type) {
  if (!impl_->is_creator || impl_->header == nullptr) return false;
  if (impl_->id_to_index.find(id) != impl_->id_to_index.end()) return false;

  const std::uint32_t current =
      impl_->header->cell_count.load(std::memory_order_relaxed);
  if (current >= impl_->header->max_cells) return false;

  // Variable-stride bump allocator (round-19): a scalar cell takes one
  // 64-byte slot, a kBitStream cell its multi-slot block. next_cell_offset
  // tracks where the NEXT cell lands; for a scalar-only topology this is
  // exactly `current * kCellSize` (the original uniform layout). The bit-
  // stream block is a whole number of 64-byte slots, so every cell start
  // stays cache-line aligned regardless of declaration order.
  const std::size_t block = cell_block_bytes(static_cast<std::uint8_t>(type));
  const std::size_t start = impl_->next_cell_offset;
  const std::size_t end_bytes = start + block;
  if (end_bytes > impl_->header->cell_area_bytes) return false;
  if (end_bytes > UINT32_MAX) return false;  // offset field is uint32

  CellDescriptor& desc = impl_->directory[current];
  desc.id = id;
  desc.offset = static_cast<std::uint32_t>(start);
  desc.type = static_cast<std::uint8_t>(type);
  desc.reserved[0] = 0;
  desc.reserved[1] = 0;
  desc.reserved[2] = 0;
  desc.cell_seq = 0;

  if (type == WireType::kBitStream) {
    BitStreamCell* bs =
        reinterpret_cast<BitStreamCell*>(impl_->cell_area + desc.offset);
    bs->seq.store(0, std::memory_order_relaxed);         // even = stable
    bs->total_bits.store(0, std::memory_order_relaxed);  // nothing appended
    bs->write_gen.store(0, std::memory_order_relaxed);   // never-written
    bs->pad = 0;
    // B1: ring bytes are now std::atomic — zero them via relaxed stores
    // rather than memset (object-representation memset over an atomic array
    // is not guaranteed well-defined). declare() runs single-threaded before
    // the cell is published, so plain relaxed stores are sufficient.
    for (std::size_t b = 0; b < kBitStreamCapBytes; ++b) {
      bs->ring[b].store(0, std::memory_order_relaxed);
    }
  } else {
    Cell* cell = reinterpret_cast<Cell*>(impl_->cell_area + desc.offset);
    cell->value.store(0, std::memory_order_relaxed);
    // write_gen starts at 0 — the never-written marker. write_*() bumps
    // it on every value publish (see write_bit / write_byte / etc).
    cell->write_gen.store(0, std::memory_order_relaxed);
  }

  impl_->next_cell_offset = static_cast<std::uint32_t>(end_bytes);

  // Publish: attachers that observe (current+1) see a fully-written
  // descriptor for slot `current`.
  impl_->header->cell_count.store(current + 1, std::memory_order_release);
  impl_->id_to_index[id] = current;
  return true;
}

// ─── read accessors ───────────────────────────────────────────────

bool WireTable::read_bit(WireId id, bool* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kBit, &idx)) return false;
  const std::uint32_t v =
      impl_->cell_at(idx)->value.load(std::memory_order_acquire);
  *out = (v & 1u) != 0;
  return true;
}

bool WireTable::read_byte(WireId id, std::uint8_t* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kByte, &idx)) return false;
  const std::uint32_t v =
      impl_->cell_at(idx)->value.load(std::memory_order_acquire);
  *out = static_cast<std::uint8_t>(v & 0xFFu);
  return true;
}

bool WireTable::read_uint16(WireId id, std::uint16_t* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kUint16, &idx)) return false;
  const std::uint32_t v =
      impl_->cell_at(idx)->value.load(std::memory_order_acquire);
  *out = static_cast<std::uint16_t>(v & 0xFFFFu);
  return true;
}

bool WireTable::read_uint32(WireId id, std::uint32_t* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kUint32, &idx)) return false;
  *out = impl_->cell_at(idx)->value.load(std::memory_order_acquire);
  return true;
}

bool WireTable::read_uint64(WireId id, std::uint64_t* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kUint64, &idx)) return false;
  *out = impl_->cell_at(idx)->value64.load(std::memory_order_acquire);
  return true;
}

bool WireTable::read_float32(WireId id, float* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kFloat32, &idx)) return false;
  const std::uint32_t bits =
      impl_->cell_at(idx)->value.load(std::memory_order_acquire);
  std::memcpy(out, &bits, sizeof(float));
  return true;
}

// ─── write accessors ──────────────────────────────────────────────
//
// Each write_* publishes the value first (release) and then bumps
// write_gen via bump_write_gen_saturating (CAS-loop with release
// ordering on success). The two stores are sequenced-before in the
// writer; release ordering on the gen bump prevents the value store
// from being reordered past it; so a reader who observes write_gen
// >= K via an acquire load is guaranteed to also observe the matching
// value store. The first write to a never-written cell (write_gen ==
// 0) takes it to 1, satisfying the written() predicate that
// downstream consumers gate on. write_gen saturates at UINT32_MAX
// rather than wrapping to 0 — see bump_write_gen_saturating for why.

bool WireTable::write_bit(WireId id, bool value) {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kBit, &idx)) return false;
  Cell* cell = impl_->cell_at(idx);
  cell->value.store(value ? 1u : 0u, std::memory_order_release);
  bump_write_gen_saturating(*cell);
  return true;
}

// ─── Conductor / element-state cells ───────────────────────────────────────
//
// Both delegate to the WireId path: the transport is unchanged — same segment,
// same WireTable, same seqlock, same write_gen. What changed is who
// can NAME the cell, and that is decided at compile time by the argument type, not
// here at runtime. These four bodies are the only place in the tree that converts a
// ConductorId or an ElementStateId back to a WireId.

bool WireTable::publish_conductor(ConductorId id, bool energised,
                                  const SolverToken& token) {
  // `token` carries no data. Its whole job is to be unconstructible outside
  // ConductorPublisher, so reaching this line at all is proof the value came out of a
  // provenance computation.
  (void)token;
  return write_bit(static_cast<WireId>(id), energised);
}

bool WireTable::read_bit_sample(ConductorId id, Sample<bool>* out) const {
  return read_bit_sample(static_cast<WireId>(id), out);
}

bool WireTable::publish_conductor_mv(ConductorId id, std::uint32_t millivolts,
                                     const SolverToken& token) {
  // Same token, same argument as the bit form: a node's potential is an output of the
  // same provenance computation that decided whether it is energised at all.
  (void)token;
  return write_uint32(static_cast<WireId>(id), millivolts);
}

bool WireTable::read_uint32_sample(ConductorId id, Sample<std::uint32_t>* out) const {
  return read_uint32_sample(static_cast<WireId>(id), out);
}

bool WireTable::write_element_state(ElementStateId id, bool closed) {
  return write_bit(static_cast<WireId>(id), closed);
}

bool WireTable::read_bit_sample(ElementStateId id, Sample<bool>* out) const {
  return read_bit_sample(static_cast<WireId>(id), out);
}

bool WireTable::write_element_state_mv(ElementStateId id, std::uint32_t millivolts) {
  return write_uint32(static_cast<WireId>(id), millivolts);
}

bool WireTable::read_uint32_sample(ElementStateId id, Sample<std::uint32_t>* out) const {
  return read_uint32_sample(static_cast<WireId>(id), out);
}

bool WireTable::write_byte(WireId id, std::uint8_t value) {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kByte, &idx)) return false;
  Cell* cell = impl_->cell_at(idx);
  cell->value.store(value, std::memory_order_release);
  bump_write_gen_saturating(*cell);
  return true;
}

bool WireTable::write_uint16(WireId id, std::uint16_t value) {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kUint16, &idx)) return false;
  Cell* cell = impl_->cell_at(idx);
  cell->value.store(value, std::memory_order_release);
  bump_write_gen_saturating(*cell);
  return true;
}

bool WireTable::write_uint32(WireId id, std::uint32_t value) {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kUint32, &idx)) return false;
  Cell* cell = impl_->cell_at(idx);
  cell->value.store(value, std::memory_order_release);
  bump_write_gen_saturating(*cell);
  return true;
}

bool WireTable::write_uint64(WireId id, std::uint64_t value) {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kUint64, &idx)) return false;
  Cell* cell = impl_->cell_at(idx);
  cell->value64.store(value, std::memory_order_release);
  bump_write_gen_saturating(*cell);
  return true;
}

bool WireTable::write_float32(WireId id, float value) {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kFloat32, &idx)) return false;
  std::uint32_t bits;
  std::memcpy(&bits, &value, sizeof(float));
  Cell* cell = impl_->cell_at(idx);
  cell->value.store(bits, std::memory_order_release);
  bump_write_gen_saturating(*cell);
  return true;
}

// ─── bit-stream (kBitStream) accessors ─────────────────────────────
//
// Single-producer / multiple-independent-reader bit FIFO. The producer
// (one declared driver per cell) appends bits under the seqlock; readers
// drain bits-since-cursor under the seqlock's retry loop. See BitStreamCell
// above for the full protocol and ordering rationale.

bool WireTable::append_bit(WireId id, bool value) {
  return append_bits(id, &value, 1);
}

bool WireTable::append_bits(WireId id, const bool* bits, std::size_t n) {
  if (n == 0) return true;  // nothing to do; still a valid no-op append
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kBitStream, &idx)) return false;
  BitStreamCell* bs = impl_->bit_stream_at(idx);

  // ── Seqlock writer: bump to ODD (release), publish ring + total
  //    (relaxed), bump to EVEN (release). Single declared writer per cell,
  //    so the odd/even toggle is uncontended on the write side. ──
  //
  // S2 (adversarial substrate review 2026-06-13): a producer that crashed
  // mid-append (between the odd-store and the even-store) leaves `seq` ODD.
  // env_open.cpp adopts such an orphaned segment without re-initializing its
  // cells (adopt() reuses the inode so live readers keep working), so a
  // newly-attached writer can inherit an odd `seq`. Blindly storing s+1/s+2
  // from an odd base would drive odd→even *during* the write and even→odd
  // when "stable" — permanently INVERTING the odd/even meaning for that cell
  // (every future reader would treat the stable state as "writing" and spin,
  // and treat the writing state as stable and tear). Normalize here: if the
  // inherited seq is odd, round it up to the next even value before the
  // protocol's odd-store. We do this in the writer (not the adopt path)
  // because append_bits is the single guaranteed entry point for *any* newly
  // attached writer — adopt() doesn't know which cells are bit-streams, and a
  // creator that crashed mid-init is handled by declare() zeroing seq. The
  // single-producer invariant means this normalization is uncontended.
  std::uint64_t s = bs->seq.load(std::memory_order_relaxed);
  if ((s & 1u) != 0u) {
    ++s;  // odd (orphaned mid-append) → next even; resume from a clean base
  }

  bs->seq.store(s + 1, std::memory_order_release);  // → odd: "writing"
  // N1 (review 2026-06-13): a standalone release fence here does NOT publish
  // the odd marker before the ring/total relaxed stores — a release fence
  // only orders stores that *precede* it against a subsequent release
  // operation, and there is no later release op tied to it that a reader
  // synchronizes-with at this point. Publication of every ring/total store
  // is carried solely by the even-store (seq.store(s+2, release)) below,
  // paired with the reader's acquire load + s2 recheck. The fence is dropped.

  std::uint64_t total = bs->total_bits.load(std::memory_order_relaxed);
  for (std::size_t k = 0; k < n; ++k) {
    ring_set_bit(bs->ring, total + k, bits[k] != false);
  }
  total += n;
  bs->total_bits.store(total, std::memory_order_relaxed);

  // write_gen mirror — saturating, same rationale as the scalar Cell. A
  // RELEASE store (not relaxed) so the type-agnostic write_gen() accessor's
  // bare acquire load (no seqlock) synchronizes-with it; the gen==0 →
  // gen>0 transition is what for_each_unwritten / the post-init watchdog
  // gate on, so it must be visible without holding the seqlock.
  std::uint32_t g = bs->write_gen.load(std::memory_order_relaxed);
  if (g != UINT32_MAX) bs->write_gen.store(g + 1, std::memory_order_release);

  bs->seq.store(s + 2, std::memory_order_release);  // → even: "stable"
  return true;
}

bool WireTable::read_bits_since(WireId id, std::uint64_t* cursor, bool* out,
                                std::size_t max, std::size_t* got,
                                bool* overran) const {
  if (got != nullptr) *got = 0;
  if (overran != nullptr) *overran = false;
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kBitStream, &idx)) return false;
  const BitStreamCell* bs = impl_->bit_stream_at(idx);

  // ── Seqlock reader: load seq (acquire); if odd, a writer is mid-append
  //    — retry. Read total + ring slice into a LOCAL staging buffer,
  //    fence(acquire), re-load seq; if it changed, a writer interleaved —
  //    retry. A snapshot of total_bits plus the bits we copy is consistent
  //    only when the two seq loads match and are even.
  //
  //    S3 (review 2026-06-13): decode into a stack staging buffer first and
  //    memcpy into the caller's out[] only AFTER the seqlock confirms a
  //    stable read. The prior code wrote speculatively into out[] on every
  //    retry (relying on the caller honoring `got`); honoring the documented
  //    contract keeps a torn snapshot from ever touching out[]. The staging
  //    buffer is bounded by the ring capacity (avail is clamped to both `max`
  //    and the ring window), so it's a fixed-size, allocation-free stack
  //    array — no VLA, no heap.
  //
  //    S1 (review 2026-06-13): the loop is BOUNDED. If a producer was killed
  //    mid-append, seq is stuck ODD and we'd otherwise spin forever, wedging
  //    the caller's tick loop. On exceeding kSeqlockMaxRetries we return
  //    "no new bits this tick" (got=0, ok=true, *cursor unchanged) — never
  //    block. ──
  bool staging[kBitStreamCapBits];  // ≤ 1024 bools; allocation-free

  const std::uint64_t start = *cursor;
  std::uint64_t commit_start = start;
  std::size_t n = 0;
  bool overrun = false;

  for (unsigned attempt = 0; attempt < kSeqlockMaxRetries; ++attempt) {
    const std::uint64_t s1 = bs->seq.load(std::memory_order_acquire);
    if ((s1 & 1u) != 0u) continue;  // writer in progress; spin (bounded)

    const std::uint64_t total = bs->total_bits.load(std::memory_order_relaxed);

    // Determine the window of recoverable absolute indices: [oldest, total).
    // If the reader's cursor predates the oldest bit still in the ring, the
    // gap was evicted — clamp up and flag the overrun.
    const std::uint64_t cap = kBitStreamCapBits;
    const std::uint64_t oldest = (total > cap) ? (total - cap) : 0;
    std::uint64_t from = start;
    bool this_overrun = false;
    if (from < oldest) {
      from = oldest;
      this_overrun = true;
    }
    // Bits available to replay, capped at max (and implicitly at cap, since
    // [from, total) spans at most the ring window) — so avail fits staging[].
    std::uint64_t avail = (total > from) ? (total - from) : 0;
    if (avail > max) avail = max;

    for (std::uint64_t k = 0; k < avail; ++k) {
      staging[k] = ring_get_bit(bs->ring, from + k);
    }

    std::atomic_thread_fence(std::memory_order_acquire);
    const std::uint64_t s2 = bs->seq.load(std::memory_order_acquire);
    if (s1 == s2) {
      // Consistent snapshot — only now copy the staged bits to the caller.
      const std::size_t avail_n = static_cast<std::size_t>(avail);
      for (std::size_t k = 0; k < avail_n; ++k) out[k] = staging[k];
      commit_start = from + avail;
      n = avail_n;
      overrun = this_overrun;
      if (got != nullptr) *got = n;
      if (overran != nullptr) *overran = overrun;
      *cursor = commit_start;
      return true;
    }
    // Writer interleaved (or seq odd) — discard staging and retry.
  }

  // S1: retry cap exhausted (a producer almost certainly died mid-append,
  // leaving seq odd). Treat as "no new bits this tick": do not advance the
  // cursor, do not block. got stays 0, overran stays false (set at entry).
  return true;
}

bool WireTable::bit_stream_total(WireId id, std::uint64_t* out_total) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kBitStream, &idx)) return false;
  const BitStreamCell* bs = impl_->bit_stream_at(idx);
  // S1 (review 2026-06-13): BOUNDED, same rationale as read_bits_since — a
  // producer killed mid-append leaves seq stuck ODD; an unbounded spin would
  // wedge the caller. On cap-exhaust, hand back the last-known (relaxed)
  // total rather than block. total_bits is monotonic, so a possibly-torn
  // value here is at worst slightly stale, never going backwards across the
  // 64-bit store (single producer; the field is naturally aligned).
  for (unsigned attempt = 0; attempt < kSeqlockMaxRetries; ++attempt) {
    const std::uint64_t s1 = bs->seq.load(std::memory_order_acquire);
    if ((s1 & 1u) != 0u) continue;
    const std::uint64_t total = bs->total_bits.load(std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_acquire);
    const std::uint64_t s2 = bs->seq.load(std::memory_order_acquire);
    if (s1 == s2) {
      *out_total = total;
      return true;
    }
  }
  *out_total = bs->total_bits.load(std::memory_order_relaxed);
  return true;
}

// ── TEST-ONLY fault-injection hooks (see wire_table.hpp) ──────────────
bool WireTable::debug_set_bit_stream_seq(WireId id, std::uint64_t raw_seq) {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kBitStream, &idx)) return false;
  BitStreamCell* bs = impl_->bit_stream_at(idx);
  bs->seq.store(raw_seq, std::memory_order_release);
  return true;
}

bool WireTable::debug_bit_stream_seq(WireId id, std::uint64_t* out_seq) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kBitStream, &idx)) return false;
  const BitStreamCell* bs = impl_->bit_stream_at(idx);
  *out_seq = bs->seq.load(std::memory_order_acquire);
  return true;
}

// ─── sample (value + generation) read accessors ───────────────────
//
// Same false-on-undeclared / type-mismatch contract as the value-only
// read_*(). On success, out->generation == 0 means "never written"
// (the substrate-side fact that dissolves Pattern A); out->generation > 0
// means "written at least that many times, and value is real." See
// read_raw_sample for the seqlock-style snapshot model.

bool WireTable::read_bit_sample(WireId id, Sample<bool>* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kBit, &idx)) return false;
  const auto s = read_raw_sample(impl_->cell_at(idx));
  out->value = (s.raw & 1u) != 0;
  out->generation = s.generation;
  return true;
}

bool WireTable::read_byte_sample(WireId id, Sample<std::uint8_t>* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kByte, &idx)) return false;
  const auto s = read_raw_sample(impl_->cell_at(idx));
  out->value = static_cast<std::uint8_t>(s.raw & 0xFFu);
  out->generation = s.generation;
  return true;
}

bool WireTable::read_uint16_sample(WireId id, Sample<std::uint16_t>* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kUint16, &idx)) return false;
  const auto s = read_raw_sample(impl_->cell_at(idx));
  out->value = static_cast<std::uint16_t>(s.raw & 0xFFFFu);
  out->generation = s.generation;
  return true;
}

bool WireTable::read_uint32_sample(WireId id, Sample<std::uint32_t>* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kUint32, &idx)) return false;
  const auto s = read_raw_sample(impl_->cell_at(idx));
  out->value = s.raw;
  out->generation = s.generation;
  return true;
}

bool WireTable::read_uint64_sample(WireId id, Sample<std::uint64_t>* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kUint64, &idx)) return false;
  const auto s = read_raw_sample64(impl_->cell_at(idx));
  out->value = s.raw;
  out->generation = s.generation;
  return true;
}

bool WireTable::read_float32_sample(WireId id, Sample<float>* out) const {
  std::uint32_t idx;
  if (!impl_->lookup(id, WireType::kFloat32, &idx)) return false;
  const auto s = read_raw_sample(impl_->cell_at(idx));
  std::memcpy(&out->value, &s.raw, sizeof(float));
  out->generation = s.generation;
  return true;
}

// ─── introspection ────────────────────────────────────────────────

std::uint32_t WireTable::topology_hash() const noexcept {
  return impl_->header ? impl_->header->topology_hash : 0u;
}

std::size_t WireTable::cell_count() const noexcept {
  return impl_->header
       ? impl_->header->cell_count.load(std::memory_order_acquire)
       : 0u;
}

bool WireTable::is_creator() const noexcept {
  return impl_->is_creator;
}

// ── Co-sim tick barrier (primitive-4 B) ─────────────────────────────────────

void WireTable::barrier_arm() {
  BarrierState* b = barrier_of(impl_->header);
  // publish_gen/ack_count are already zero from the creator's reserved[] memset;
  // arm last (release) so a consumer that observes enabled==1 also sees them.
  b->enabled.store(1u, std::memory_order_release);
}

void WireTable::barrier_publish_tick() {
  BarrierState* b = barrier_of(impl_->header);
  // Reset the ack counter for the new tick BEFORE advancing the generation, so
  // a consumer woken by the gen bump never observes a stale (previous-tick) ack
  // count. The barrier guarantees no consumer is still acking the prior tick
  // here: the leader only calls this after barrier_await_acks() returned for it.
  b->ack_count.store(0u, std::memory_order_release);
  b->publish_gen.fetch_add(1u, std::memory_order_acq_rel);
  barrier_futex_wake(&b->publish_gen);
}

bool WireTable::barrier_await_acks(std::uint32_t consumer_count, int timeout_ms) {
  BarrierState* b = barrier_of(impl_->header);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  for (;;) {
    const std::uint32_t acked = b->ack_count.load(std::memory_order_acquire);
    if (acked >= consumer_count) return true;
    if (std::chrono::steady_clock::now() >= deadline) {
      return b->ack_count.load(std::memory_order_acquire) >= consumer_count;
    }
    // Block on the ack word; consumers wake it after each fetch_add. The slice
    // bounds a missed wake; re-loop re-reads ack_count and the deadline.
    barrier_futex_wait_slice(&b->ack_count, acked);
  }
}

bool WireTable::barrier_active() const {
  return barrier_of(impl_->header)->enabled.load(std::memory_order_acquire) != 0u;
}

std::uint32_t WireTable::barrier_publish_gen() const {
  return barrier_of(impl_->header)->publish_gen.load(std::memory_order_acquire);
}

bool WireTable::barrier_await_tick(std::uint32_t* prev_gen, int timeout_ms) const {
  BarrierState* b = barrier_of(impl_->header);
  if (barrier_wait_word_changed(&b->publish_gen, *prev_gen, timeout_ms)) {
    *prev_gen = b->publish_gen.load(std::memory_order_acquire);
    return true;
  }
  return false;
}

void WireTable::barrier_ack() {
  BarrierState* b = barrier_of(impl_->header);
  b->ack_count.fetch_add(1u, std::memory_order_acq_rel);
  barrier_futex_wake(&b->ack_count);
}

bool WireTable::has_cell(WireId id) const {
  std::uint32_t idx = 0;
  return impl_->resolve_index(id, &idx);
}

bool WireTable::write_gen(WireId id, std::uint32_t* out_gen) const {
  std::uint32_t idx = 0;
  if (!impl_->resolve_index(id, &idx)) return false;
  // Acquire ordering pairs with the writer's release on write_gen
  // (see write_*()). The gen word lives at a DIFFERENT offset in a
  // kBitStream cell than in a scalar Cell (the scalar Cell's write_gen
  // is bytes 4..7, which overlap a BitStreamCell's seq high half), so
  // dispatch on the declared type. write_gen == 0 still means "never
  // written" for both shapes, which is what for_each_unwritten and the
  // post-init watchdog rely on.
  if (impl_->directory[idx].type ==
      static_cast<std::uint8_t>(WireType::kBitStream)) {
    *out_gen =
        impl_->bit_stream_at(idx)->write_gen.load(std::memory_order_acquire);
  } else {
    *out_gen = impl_->cell_at(idx)->write_gen.load(std::memory_order_acquire);
  }
  return true;
}

}  // namespace electricsim::io
