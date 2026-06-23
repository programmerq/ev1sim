/*
 * UartTx implementation — GM-8192 host-side UART transmitter.
 *
 * Bit clock: `next_edge_ns_` is the monotonic time of the next bit boundary.
 * tick(now_ns) emits bits while that boundary is at or before now_ns. A frame
 * is 10 bits (Start, 8 data LSB-first, Stop); when one completes the next
 * queued byte starts on the very next boundary (back-to-back), otherwise the
 * machine goes idle.
 *
 * TRANSPORT (round-19, sub-PR 4-C½): the transmit cell is a bit-STREAM cell
 * (WireType::kBitStream), not a level-held single-bit register. Each emitted
 * frame bit is APPENDED to the cell's bounded bit ring rather than
 * overwriting a single level. This is what dissolves the bit-cadence problem
 * (notes/manual_supplements.yaml#2026-06-13-gm8192-bit-ring-cell): a coarse
 * host tick (5–50 ms) that crosses many 122 µs bit boundaries now appends
 * EVERY bit it produces — all preserved for the consumer — instead of letting
 * all but the last overwrite vanish. The `now_ns`/`bit_period_ns_` cadence
 * still governs HOW MANY bits a tick produces (the producer paces bit
 * production by the wall clock); they just land in the ring now.
 *
 * Idle Line: when the queue drains, the transmitter simply STOPS appending —
 * it does NOT flood the ring with idle 1-bits. The receiver's edge detector
 * keys off its own last-sampled level, so the next frame's Start bit presents
 * as a 1→0 transition from the previous Stop bit regardless of how long the
 * gap was. (A level-held cell needed a held logic-1; a stream does not, and
 * appending unbounded idle bits would overrun the ring during long quiets.)
 *
 * @design 2026-06-13 claude — docs/wire_truth_gm8192_step4.md §"Sub-PR 4-C½"
 *   (transport); §"Sub-PR 4-B" (state machine); docs/gm8192_protocol.md
 *   §"Physical layer".
 */

#include "uart/uart_tx.hpp"

namespace electricsim::io {

UartTx::UartTx(WireTable* table, WireId tx_cell, std::uint64_t bit_period_ns)
    : table_(table), tx_cell_(tx_cell), bit_period_ns_(bit_period_ns) {
  // The bit-stream cell starts empty (no bits appended). The Idle Line is
  // the absence of appended bits — a receiver that has sampled nothing yet
  // treats the line as resting logic-1 (@source:manual; @source:redux
  // bus/ev1_uart_bus.yaml). Nothing to write at construction.
}

void UartTx::enqueue(const std::uint8_t* data, std::size_t len) {
  for (std::size_t i = 0; i < len; ++i) {
    queue_.push_back(data[i]);
  }
}

void UartTx::enqueue(std::uint8_t byte) { queue_.push_back(byte); }

bool UartTx::idle() const noexcept {
  return state_ == State::kIdle && queue_.empty();
}

bool UartTx::frame_bit(std::uint8_t byte, int pos) {
  if (pos == 0) return false;                       // Start bit: logic-0.
  if (pos == 9) return true;                        // Stop bit: logic-1.
  return ((byte >> (pos - 1)) & 1u) != 0u;          // Data, LSB-first.
}

void UartTx::tick(std::uint64_t now_ns) {
  for (;;) {
    if (state_ == State::kIdle) {
      if (queue_.empty()) {
        return;  // Nothing to send; cell already holds the Idle Line (logic-1).
      }
      // Begin a new frame. Align the bit clock to now_ns on the first frame
      // after construction or after an idle gap, so the Start bit goes out on
      // this tick rather than at a stale deadline.
      if (!seeded_ || next_edge_ns_ < now_ns) {
        next_edge_ns_ = now_ns;
        seeded_ = true;
      }
      cur_byte_ = queue_.front();
      queue_.pop_front();
      bit_pos_ = 0;
      state_ = State::kInFrame;
    }

    // state_ == kInFrame
    if (now_ns < next_edge_ns_) {
      return;  // Next bit boundary is still in the future.
    }

    if (table_ != nullptr) {
      // APPEND the bit to the bit-stream ring (preserved for the consumer)
      // rather than overwriting a single level. @design 2026-06-13 claude —
      // sub-PR 4-C½.
      table_->append_bit(tx_cell_, frame_bit(cur_byte_, bit_pos_));
    }
    next_edge_ns_ += bit_period_ns_;
    ++bit_pos_;

    if (bit_pos_ == 10) {
      // Frame complete. Chain to the next queued byte back-to-back, or go idle
      // (leaving the cell at the Stop bit's logic-1 — the Idle Line).
      if (queue_.empty()) {
        state_ = State::kIdle;
        return;
      }
      cur_byte_ = queue_.front();
      queue_.pop_front();
      bit_pos_ = 0;
      // next_edge_ns_ already points one bit period past the Stop bit, which
      // is exactly the next Start bit boundary — no inter-byte gap.
    }
  }
}

}  // namespace electricsim::io
