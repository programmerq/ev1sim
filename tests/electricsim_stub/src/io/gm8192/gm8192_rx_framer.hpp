/*
 * Gm8192RxFramer — stream-to-frame bridge for legacy-ring-free GM-8192 consumers.
 *
 * Wraps one electricsim::io::UartRx attached to a bit-stream wire cell with a
 * small bounded byte buffer + the generic gm8192_decode() decoder, so a
 * consumer migrating off the legacy `kSig*UartFrame` ring deltas can replace
 *
 *     auto polled = transport.poll_frame(...);
 *     if (polled.delta.signal_id == kSigBtcmUartFrame) {
 *       handle_btcm_frame(polled.delta.payload);
 *     }
 *
 * with
 *
 *     framer.step(now_ns);                            // ticks UartRx + drains
 *     while (auto frame = framer.step(now_ns)) {      // one frame per call
 *       handle_btcm_frame(frame->payload, frame->n);
 *     }
 *
 * The framer owns NO transport state of its own beyond a small process-local
 * byte buffer; the underlying UartRx keeps its own bit-cursor into the shared
 * bit-stream ring. Each step() call:
 *
 *   1. ticks the UartRx (drains new bits, runs the framing FSM, decoding any
 *      complete byte frames into the UartRx's internal byte queue),
 *   2. pops every newly available byte into this framer's bounded buffer,
 *   3. decodes the frame at the head of the current Idle-Line segment, and on
 *      success returns the frame view (its bytes — header + payload +
 *      sumcheck — are consumed at the start of the next step()).
 *
 * Idle-Line framing: the UartRx flags each byte that follows an Idle Line
 * (≥ 10 bit times of logic-1 — the GM-8192 frame boundary, docs/
 * gm8192_protocol.md §"Physical layer": "Frames between Idle Lines") or an
 * overrun hole. The framer records those byte offsets as segment boundaries
 * and parses each segment the way a GM-8192 receiver does: the first frame
 * starts at the segment's first byte, its Length says where it ends, and a
 * back-to-back next frame (gap under 10 bit times) starts right after its
 * SumCheck. A frame with a bad SumCheck is rejected by its own extent. When
 * the head of a segment is not a frame start at all (bad ID or Length — a
 * receiver that joined mid-frame, an overrun hole, noise), or a frame is
 * still incomplete when the Idle Line closes its segment (a corrupted
 * Length that over-claims), the framer drops bytes up to the next Idle Line
 * and resumes there. No real frame spans an Idle Line, so resyncing there
 * loses no real frame; and with no byte-by-byte scan, a window of garbage
 * that happens to pass ID + Length + SumCheck cannot surface as a frame
 * unless it sits where a frame must start.
 *
 * `Gm8192RxFramer` is safe to construct with a null WireTable* — every step()
 * call simply returns std::nullopt, mirroring the disabled-substrate seam the
 * follower and other migrated consumers already follow.
 *
 * The returned `gm8192_frame_t` view's `payload` pointer references the
 * framer's internal byte buffer; it stays valid until the NEXT step() call
 * (the frame's bytes are removed from the buffer at the start of that call,
 * not before step() returns, so a burst of back-to-back frames decoded in
 * one coarse tick each keep their own payload). Copy the payload out before
 * the next step() if it must outlive that boundary. Single-
 * threaded; one Gm8192RxFramer per producer cell per consumer thread.
 *
 * Capacity choice: a single GM-8192 frame is at most 173 bytes (ID + Length +
 * 170 payload + SumCheck — gm8192_frame.h §GM8192_MAX_FRAME_LEN, widened
 * 2026-09-08 from 68 when real hardware produced an N=68 $F1). The 512-byte
 * buffer is ~3× that ceiling, so a coarse host tick that decodes several
 * back-to-back frames in one step() still has headroom; the EV1 periodic set
 * tops out at N=11 (14 bytes), so in practice the margin is far larger than
 * 3×. Overruns drop the
 * OLDEST buffered bytes (so the framer always tries to recover on the
 * freshest data the wire has produced), bumping bytes_dropped() so a
 * diagnostic loop can spot persistent drops.
 *
 * @design 2026-06-14 claude — stream-to-frame bridge for legacy-ring-free
 *   GM-8192 consumers (Phase A step 0; notes/phase_a_gm8192_rx_scope.md).
 */

#ifndef ELECTRICSIM_SRC_IO_GM8192_GM8192_RX_FRAMER_HPP_
#define ELECTRICSIM_SRC_IO_GM8192_GM8192_RX_FRAMER_HPP_

#include "gm8192/gm8192_frame.h"
#include "uart/uart_rx.hpp"
#include "wire_table.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace electricsim::io {

class Gm8192RxFramer {
 public:
  // Buffer cap chosen as ~3× the GM8192_MAX_FRAME_LEN ceiling (173 bytes since
  // the 2026-09-08 widen; was ~7.5× the old 68). A coarse host tick that decodes
  // a small burst of back-to-back frames still fits comfortably — the EV1
  // periodic set's largest frame is 14 bytes — and overrun is treated as a
  // diagnostic event, not a routine condition. @design 2026-06-14 claude.
  static constexpr std::size_t kBufferCapBytes = 512;

  // Construct a framer that drains `tx_cell` (a kBitStream cell) in `table` at
  // `bit_period_ns`. A null `table` is permitted (every step() returns nullopt;
  // no allocation, no harm). The wire cell is the per-module TX cell — the
  // framer is reading what that module is transmitting onto the bus, exactly
  // like a passive eavesdropper would.
  Gm8192RxFramer(WireTable* table, WireId tx_cell, std::uint64_t bit_period_ns);

  // Disable copies (the underlying UartRx is non-copyable too) but allow moves.
  Gm8192RxFramer(const Gm8192RxFramer&) = delete;
  Gm8192RxFramer& operator=(const Gm8192RxFramer&) = delete;
  Gm8192RxFramer(Gm8192RxFramer&&) = default;
  Gm8192RxFramer& operator=(Gm8192RxFramer&&) = default;

  // Tick the UartRx, drain any newly decoded bytes into the buffer, then try
  // to extract ONE GM-8192 frame. Returns the frame on success; std::nullopt
  // if the buffer holds no complete frame yet (or the framer is disabled).
  // Callers loop until nullopt to drain every frame produced this pass.
  //
  // The returned `gm8192_frame_t.payload` pointer references this framer's
  // internal byte buffer and remains valid only until the NEXT step() call —
  // copy out before re-calling step() if the caller needs the bytes longer.
  std::optional<gm8192_frame_t> step(std::uint64_t now_ns);

  // Count of buffered bytes evicted due to buffer overflow (oldest-drops-first).
  // A persistently growing counter means upstream is producing faster than the
  // caller is consuming step()s, OR the bit stream is full of garbage that
  // never lines up to a valid frame. Either way: a diagnostic hint.
  std::uint64_t bytes_dropped() const noexcept { return bytes_dropped_; }

  // Count of buffered bytes discarded by the receive rule rather than
  // returned as frames: rejected frames, a partial frame from a mid-frame
  // join, bytes skipped while hunting for an Idle Line. A diagnostic, like
  // bytes_dropped().
  std::uint64_t bytes_discarded_at_idle() const noexcept {
    return bytes_discarded_at_idle_;
  }

  // Frames rejected by the receive rule (bad SumCheck, cut short by an Idle
  // Line, or an unparseable segment head that started a hunt). Diagnostic.
  std::uint64_t frames_rejected() const noexcept { return frames_rejected_; }

  // Idle Lines the underlying receiver has seen (diagnostic passthrough).
  std::uint32_t idle_lines() const noexcept { return rx_.idle_lines(); }

 private:
  // Remove the first `n` buffered bytes, shifting segment boundaries; when
  // the new head is a segment start, hunting_ takes that segment's state.
  void erase_front(std::size_t n);

  WireTable*               table_;        // borrowed; nullptr-safe
  UartRx                   rx_;
  std::vector<std::uint8_t> buffer_;       // bounded; oldest-drops-first
  // A byte offset into buffer_ at which a new segment starts: an Idle Line
  // preceded that byte (a frame starts there), or an overrun hole did
  // (`hunt`: the segment is mid-stream, skip it to the next Idle Line).
  struct Boundary {
    std::size_t offset;  // ascending across boundaries_; never 0
    bool        hunt;
  };
  std::vector<Boundary>    boundaries_;
  // Bytes of the frame returned by the last step(), erased at the start of
  // the next step() so the returned payload pointer stays valid until then.
  std::size_t              pending_consume_{0};
  std::uint64_t            bytes_dropped_{0};
  std::uint64_t            bytes_discarded_at_idle_{0};
  std::uint64_t            frames_rejected_{0};
  // True while the head of the buffer is known not to be a frame start (an
  // unparseable Length, an overrun hole, an evicted byte): drop bytes until
  // the next Idle Line boundary.
  bool                     hunting_{false};
};

}  // namespace electricsim::io

#endif  // ELECTRICSIM_SRC_IO_GM8192_GM8192_RX_FRAMER_HPP_
