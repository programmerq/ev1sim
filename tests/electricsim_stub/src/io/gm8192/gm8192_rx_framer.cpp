/*
 * Gm8192RxFramer implementation — stream-to-frame bridge.
 *
 * step(now_ns):
 *   1. tick the underlying UartRx (drains every bit appended since this
 *      reader's cursor, runs the framing FSM, decodes complete bytes into
 *      the UartRx's internal queue);
 *   2. pop every newly available byte into the framer's bounded buffer
 *      (oldest-drops-first on overflow, bumping bytes_dropped_);
 *   3. run gm8192_decode_next() over the buffer; on GM8192_OK consume the
 *      framed bytes from the buffer head and return the decoded frame view
 *      (whose payload pointer references the buffer until the next step());
 *      on GM8192_ERR_TRUNCATED with leading-garbage advance, drop only the
 *      definitively-garbage bytes and return nullopt.
 *
 * Buffer policy: oldest-drops-first keeps the framer recovering on the
 * freshest wire bytes (a wedged consumer that fell wildly behind cannot
 * pin stale garbage at the buffer head). Each evicted byte bumps the
 * diagnostic counter so a stuck loop is visible.
 *
 * @design 2026-06-14 claude — stream-to-frame bridge for legacy-ring-free
 *   GM-8192 consumers (Phase A step 0; notes/phase_a_gm8192_rx_scope.md).
 */

#include "gm8192/gm8192_rx_framer.hpp"

#include "gm8192/gm8192_frame.h"

namespace electricsim::io {

Gm8192RxFramer::Gm8192RxFramer(WireTable* table, WireId tx_cell,
                               std::uint64_t bit_period_ns)
    : table_(table), rx_(table, tx_cell, bit_period_ns) {
  buffer_.reserve(kBufferCapBytes);
}

std::optional<gm8192_frame_t> Gm8192RxFramer::step(std::uint64_t now_ns) {
  if (table_ == nullptr) {
    return std::nullopt;  // disabled-substrate seam
  }

  // 1. Drain bits appended since this reader's cursor and decode bytes.
  rx_.tick(now_ns);

  // 2. Pull every freshly decoded byte into the bounded buffer. Oldest-
  //    drops-first on overflow so the framer always tries to recover on
  //    the freshest data the wire produced.
  std::uint8_t b = 0;
  while (rx_.pop_byte(&b)) {
    if (buffer_.size() >= kBufferCapBytes) {
      // Drop the oldest byte to make room. Persistent eviction here means
      // upstream is producing faster than the caller is consuming step()s,
      // or the bit stream is full of garbage that never aligns to a valid
      // frame — bytes_dropped() surfaces both cases.
      buffer_.erase(buffer_.begin());
      ++bytes_dropped_;
    }
    buffer_.push_back(b);
  }

  if (buffer_.empty()) {
    return std::nullopt;
  }

  // 3. Try to extract one frame. gm8192_decode_next() walks the buffer,
  //    discarding leading garbage and returning the next valid frame.
  gm8192_frame_t frame{};
  std::size_t consumed = 0;
  const gm8192_status_t status =
      gm8192_decode_next(buffer_.data(), buffer_.size(), &frame, &consumed);

  if (status == GM8192_OK) {
    // The frame's payload points into our buffer; the caller is documented
    // to use it only until the NEXT step(). erase() invalidates pointers,
    // but the API contract handles that: the next call repopulates.
    buffer_.erase(buffer_.begin(),
                  buffer_.begin() + static_cast<std::ptrdiff_t>(consumed));
    return frame;
  }

  // GM8192_ERR_TRUNCATED: `consumed` bytes are definitively-garbage bytes
  // before any plausible frame start; drop them. The remainder is held
  // until more bytes land. (Other error codes can't be returned by
  // gm8192_decode_next() — it only ever yields OK or TRUNCATED — but
  // guard against future expansion by treating non-OK as nullopt.)
  if (consumed > 0 && consumed <= buffer_.size()) {
    buffer_.erase(buffer_.begin(),
                  buffer_.begin() + static_cast<std::ptrdiff_t>(consumed));
  }
  return std::nullopt;
}

}  // namespace electricsim::io
