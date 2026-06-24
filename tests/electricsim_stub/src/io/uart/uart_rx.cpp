/*
 * UartRx implementation — GM-8192 host-side UART receiver.
 *
 * Bit-stream transport (round-19, sub-PR 4-C½): tick() drains the bits this
 * reader has not yet seen (read_bits_since against a process-local cursor)
 * and feeds each through the per-bit framing FSM in consume_bit(). One
 * appended bit is exactly one GM-8192 bit period, so the receiver consumes
 * one bit per bit — no oversampling, no mid-bit re-timing. Bits are drained
 * in bounded batches so a long-idle producer that then bursts a frame is
 * fully replayed in a single tick.
 *
 * @design 2026-06-13 claude — docs/wire_truth_gm8192_step4.md §"Sub-PR 4-C½"
 *   (transport); §"Sub-PR 4-B" (framing FSM); docs/gm8192_protocol.md
 *   §"Physical layer".
 */

#include "uart/uart_rx.hpp"

namespace electricsim::io {

UartRx::UartRx(WireTable* table, WireId rx_cell, std::uint64_t bit_period_ns,
               int oversample_factor)
    : table_(table), rx_cell_(rx_cell), bit_period_ns_(bit_period_ns) {
  // oversample_factor is retained for API stability but unused under the
  // bit-stream transport (one appended bit == one bit period).
  (void)oversample_factor;
  // last_high_ starts true: the Idle Line rests at logic-1, so the first
  // Start bit presents as a 1→0 edge. @source:manual; docs/gm8192_protocol.md.
}

bool UartRx::pop_byte(std::uint8_t* out) {
  if (byte_q_.empty()) return false;
  *out = byte_q_.front();
  byte_q_.pop_front();
  return true;
}

void UartRx::consume_bit(bool bit) {
  switch (state_) {
    case State::kIdle:
      // Hunt for the Start-bit falling edge (1→0).
      if (last_high_ && !bit) {
        shift_reg_ = 0;
        bit_idx_ = 0;
        state_ = State::kData;
      }
      // Any other bit (idle high, or a stray low without a preceding high)
      // keeps the line idle.
      break;

    case State::kData:
      // Accumulate data bits LSB-first. @source:manual; docs/gm8192_protocol.md.
      if (bit) {
        shift_reg_ |= static_cast<std::uint8_t>(1u << bit_idx_);
      }
      ++bit_idx_;
      if (bit_idx_ == 8) {
        state_ = State::kStop;
      }
      break;

    case State::kStop:
      if (bit) {
        byte_q_.push_back(shift_reg_);  // Valid Stop bit (logic-1).
      } else {
        ++framing_errors_;              // Stop bit was logic-0.
      }
      // Return to idle. A back-to-back byte's Start bit (0) is detected on
      // the next bit because last_high_ is set from THIS Stop bit (1 on a
      // good frame). On a framing error (Stop = 0) the next bit cannot be a
      // valid Start edge anyway; the receiver re-syncs on the following 1→0.
      state_ = State::kIdle;
      break;
  }

  last_high_ = bit;
}

void UartRx::tick(std::uint64_t now_ns) {
  (void)now_ns;  // advisory under the bit-stream transport (bits self-pace)
  if (table_ == nullptr) return;

  // Drain the stream in bounded batches until caught up. A batch buffer of
  // one full frame's worth of bits keeps the stack footprint small while
  // still replaying an arbitrarily long backlog across multiple iterations.
  constexpr std::size_t kBatch = 64;
  bool buf[kBatch];

  for (;;) {
    std::size_t got = 0;
    bool overran = false;
    if (!table_->read_bits_since(rx_cell_, &cursor_, buf, kBatch, &got,
                                 &overran)) {
      return;  // undeclared / type mismatch — nothing to do
    }
    if (overran) {
      // The reader fell more than the ring capacity behind: bits were lost.
      // Reset framing to idle and treat the line as resting high so the next
      // genuine Start edge re-syncs cleanly, rather than decoding a byte that
      // straddles the gap.
      ++overruns_;
      state_ = State::kIdle;
      last_high_ = true;
    }
    for (std::size_t i = 0; i < got; ++i) {
      consume_bit(buf[i]);
    }
    if (got < kBatch) {
      return;  // caught up to the producer's latest bit
    }
  }
}

}  // namespace electricsim::io
