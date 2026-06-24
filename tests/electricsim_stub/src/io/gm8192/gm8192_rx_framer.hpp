/*
 * Gm8192RxFramer — stream-to-frame bridge for legacy-ring-free GM-8192 consumers.
 *
 * Wraps one electricsim::io::UartRx attached to a bit-stream wire cell with a
 * small bounded byte buffer + the generic gm8192_decode_next() decoder, so a
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
 *   3. runs gm8192_decode_next() over the buffer, and on success consumes the
 *      framed bytes (header + payload + sumcheck) and returns the frame view.
 *
 * `Gm8192RxFramer` is safe to construct with a null WireTable* — every step()
 * call simply returns std::nullopt, mirroring the disabled-substrate seam the
 * follower and other migrated consumers already follow.
 *
 * The returned `gm8192_frame_t` view's `payload` pointer references the
 * framer's internal byte buffer; it stays valid until the NEXT step() call
 * that consumes from the buffer (so the caller should copy the payload bytes
 * before the next step() if they need to outlive that boundary). Single-
 * threaded; one Gm8192RxFramer per producer cell per consumer thread.
 *
 * Capacity choice: a single GM-8192 frame is at most 68 bytes (ID + Length +
 * 65 payload + SumCheck — gm8192_frame.h §GM8192_MAX_FRAME_LEN). The
 * 512-byte buffer is ~7.5× margin so a coarse host tick that decodes several
 * back-to-back frames in one step() still has headroom. Overruns drop the
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
  // Buffer cap chosen as ~7.5× the GM8192_MAX_FRAME_LEN ceiling (68 bytes).
  // A coarse host tick that decodes a small burst of back-to-back frames still
  // fits comfortably; overrun is treated as a diagnostic event, not a routine
  // condition. @design 2026-06-14 claude.
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

 private:
  WireTable*               table_;        // borrowed; nullptr-safe
  UartRx                   rx_;
  std::vector<std::uint8_t> buffer_;       // bounded; oldest-drops-first
  std::uint64_t            bytes_dropped_{0};
};

}  // namespace electricsim::io

#endif  // ELECTRICSIM_SRC_IO_GM8192_GM8192_RX_FRAMER_HPP_
