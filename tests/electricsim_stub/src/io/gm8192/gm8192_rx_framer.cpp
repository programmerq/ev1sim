/*
 * Gm8192RxFramer implementation — stream-to-frame bridge.
 *
 * step(now_ns):
 *   0. erase the frame returned by the previous step() (deferred so its
 *      payload pointer stayed valid until now);
 *   1. tick the underlying UartRx (drains every bit appended since this
 *      reader's cursor, runs the framing FSM, decodes complete bytes into
 *      the UartRx's internal queue);
 *   2. pop every newly available byte into the framer's bounded buffer
 *      (oldest-drops-first on overflow, bumping bytes_dropped_);
 *   3. gm8192_decode() the frame at the head of the first segment (the
 *      bytes before the first Idle-Line boundary, or the whole buffer when
 *      there is none):
 *        valid frame      -> return the frame view (its bytes are erased at
 *                            the start of the next step());
 *        bad SumCheck or  -> the Length gives the extent: drop exactly that
 *        reserved ID         frame and parse on right after it;
 *        incomplete       -> wait for more bytes; but if an Idle Line already
 *                            closed the segment, drop the segment (no frame
 *                            spans an Idle Line);
 *        Length < 0x55    -> the head is not a frame start: drop bytes until
 *                            the next Idle Line ("hunting").
 *      There is no byte-by-byte scan for a frame start: that is what let a
 *      garbage window pass for a frame, and it is not what a GM-8192
 *      receiver does.
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

void Gm8192RxFramer::erase_front(std::size_t n) {
  if (n == 0) return;
  if (n > buffer_.size()) n = buffer_.size();
  buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(n));
  // Shift boundaries. One that lands exactly on the new head makes the head
  // a segment start, carrying that segment's hunt state; one erased past no
  // longer separates anything.
  std::size_t w = 0;
  for (std::size_t r = 0; r < boundaries_.size(); ++r) {
    if (boundaries_[r].offset > n) {
      boundaries_[w++] = Boundary{boundaries_[r].offset - n, boundaries_[r].hunt};
    } else if (boundaries_[r].offset == n) {
      hunting_ = boundaries_[r].hunt;
    }
  }
  boundaries_.resize(w);
}

std::optional<gm8192_frame_t> Gm8192RxFramer::step(std::uint64_t now_ns) {
  if (table_ == nullptr) {
    return std::nullopt;  // disabled-substrate seam
  }

  // 0. The frame handed out by the previous step() is consumed now, not
  //    before that step() returned: its payload pointer referenced these
  //    bytes, and erasing them early would shift the NEXT frame's bytes
  //    underneath it when a coarse tick decoded several frames at once.
  erase_front(pending_consume_);
  pending_consume_ = 0;

  // 1. Drain bits appended since this reader's cursor and decode bytes.
  rx_.tick(now_ns);

  // 2. Pull every freshly decoded byte into the bounded buffer, recording an
  //    Idle-Line boundary in front of each byte the receiver flagged.
  //    Oldest-drops-first on overflow so the framer always tries to recover
  //    on the freshest data the wire produced.
  std::uint8_t b = 0;
  UartRx::Gap gap = UartRx::Gap::kNone;
  while (rx_.pop_byte(&b, &gap)) {
    if (buffer_.size() >= kBufferCapBytes) {
      // Drop the oldest byte to make room. Persistent eviction here means
      // upstream is producing faster than the caller is consuming step()s,
      // or the bit stream is full of garbage that never aligns to a valid
      // frame — bytes_dropped() surfaces both cases.
      // Evicting the head byte misaligns the head frame unless the next
      // byte starts a segment, so hunt for the next Idle Line.
      const bool next_is_segment_start =
          !boundaries_.empty() && boundaries_.front().offset == 1u;
      erase_front(1);
      ++bytes_dropped_;
      if (!next_is_segment_start) hunting_ = true;
    }
    if (gap != UartRx::Gap::kNone) {
      const bool hunt = (gap == UartRx::Gap::kLostBits);
      if (buffer_.empty()) {
        hunting_ = hunt;  // this byte is the head, and starts a segment
      } else {
        boundaries_.push_back(Boundary{buffer_.size(), hunt});
      }
    }
    buffer_.push_back(b);
  }

  // 3. Extract one frame. A frame must START at the head of its segment (the
  //    byte after an Idle Line, or right after the previous frame's
  //    SumCheck) and may not cross the segment's closing Idle Line — the
  //    receive rule a real GM-8192 receiver follows: reset at the Idle Line,
  //    take the Length at face value, and when the frame at hand cannot be
  //    parsed, ignore the wire until the next Idle Line.
  while (!buffer_.empty()) {
    const bool closed = !boundaries_.empty();
    const std::size_t window =
        closed ? boundaries_.front().offset : buffer_.size();

    if (hunting_) {
      // The head is not a frame start: drop what is buffered up to the next
      // boundary (everything, if none has arrived yet). Erasing up to the
      // boundary makes its segment the head and takes its hunt state.
      bytes_discarded_at_idle_ += window;
      erase_front(window);
      if (closed) continue;
      return std::nullopt;
    }

    // The envelope's extent comes from the Length byte alone, so the frame
    // at the head can be delimited before its ID or SumCheck is judged.
    std::size_t extent = 0;
    if (window >= 2u) {
      const std::uint8_t n = gm8192_n_from_length(buffer_[1]);
      if (n == GM8192_N_INVALID) {
        // Length below 0x55: the head is not a frame start (a receiver that
        // joined mid-frame, an overrun hole, noise). Where the next frame
        // starts is unknowable until the next Idle Line: hunt for it.
        ++frames_rejected_;
        hunting_ = true;
        continue;
      }
      extent = static_cast<std::size_t>(GM8192_HEADER_LEN) + n +
               static_cast<std::size_t>(GM8192_TRAILER_LEN);
    }

    if (extent == 0u || window < extent) {
      if (!closed) {
        return std::nullopt;  // wait for the rest of the frame
      }
      // The Idle Line closed this segment before the frame completed (a
      // frame cut short, or a corrupted Length claiming more bytes than
      // were sent). No frame spans an Idle Line: drop the segment.
      ++frames_rejected_;
      bytes_discarded_at_idle_ += window;
      erase_front(window);
      continue;
    }

    gm8192_frame_t frame{};
    std::size_t consumed = 0;
    if (gm8192_decode(buffer_.data(), extent, &frame, &consumed) ==
        GM8192_OK) {
      // The payload points into buffer_; the bytes stay put until the next
      // step() erases them (step 0 above).
      pending_consume_ = consumed;
      return frame;
    }

    // Bad SumCheck, or a reserved ID (0x00 / 0xFF) on an otherwise
    // well-formed envelope: the Length still delimits the frame, so reject
    // exactly that frame and parse on from the byte after it — a
    // back-to-back next frame starts there.
    ++frames_rejected_;
    bytes_discarded_at_idle_ += extent;
    erase_front(extent);
  }
  return std::nullopt;
}

}  // namespace electricsim::io
