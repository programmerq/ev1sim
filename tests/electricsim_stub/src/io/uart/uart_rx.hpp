/*
 * UartRx — host-side UART receiver state machine for the GM-8192 bit-wire
 * substrate.
 *
 * TRANSPORT (round-19, sub-PR 4-C½): the receive cell is a bit-STREAM cell
 * (WireType::kBitStream). Each UartRx keeps its OWN cursor in process-local
 * memory (there is no per-reader state in the shared cell — many independent
 * eavesdroppers can read the same stream, each at its own pace). tick()
 * drains the bits appended since this reader's cursor and runs the framing
 * state machine on EXACTLY that bit sequence — one appended bit == one
 * GM-8192 bit period, so no oversampling or mid-bit re-timing is needed (the
 * bits are already discretised at bit-period granularity by the producer).
 * This dissolves the bit-cadence problem the level-held cell suffered (notes/
 * manual_supplements.yaml#2026-06-13-gm8192-bit-ring-cell): a coarse tick
 * replays every bit it missed instead of re-reading one stale level.
 *
 * Physical layer (docs/gm8192_protocol.md §"Physical layer"; @source:manual;
 * @source:redux bus/ev1_uart_bus.yaml):
 *
 *   Frame:    Start(0) + 8 data bits LSB-first + Stop(1).
 *   Baud:     8192 baud; bit period = 122.07 µs (122070 ns).
 *   Idle Line: continuous logic-1; a 1→0 transition marks a Start bit.
 *
 * Per-bit framing FSM (one transition per appended bit):
 *   - kIdle: a 1→0 transition (prev bit logic-1, this bit logic-0) is a Start
 *     bit → begin a frame. Any other bit keeps the line idle.
 *   - kData: accumulate 8 data bits LSB-first.
 *   - kStop: logic-1 → push the assembled byte; logic-0 → framing error
 *     (byte discarded, counter bumped). Either way the Stop bit becomes the
 *     new "previous" level, so a back-to-back next Start (0) is caught on the
 *     following bit.
 *
 * Overrun: if this reader falls more than the ring capacity behind the
 * producer (a tick so coarse / a producer so fast that the ring wrapped past
 * the cursor), read_bits_since reports it; UartRx counts the event, resets to
 * idle, and re-syncs framing on the next Start edge rather than decoding
 * corrupted bits. overruns() exposes the count.
 *
 * Driven by an external monotonic timestamp, like UartTx — no owned thread or
 * real-time clock. `now_ns` is advisory under the bit-stream transport (the
 * appended bits self-pace); it is retained for API stability.
 * @design 2026-06-13 claude — docs/wire_truth_gm8192_step4.md §"Sub-PR 4-C½"
 *   (transport); §"Sub-PR 4-B" (framing FSM).
 *
 * Host-side only. No simavr dependency, no module controller changes.
 */

#ifndef ELECTRICSIM_SRC_IO_UART_UART_RX_HPP_
#define ELECTRICSIM_SRC_IO_UART_UART_RX_HPP_

#include "wire_table.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>

namespace electricsim::io {

class UartRx {
 public:
  // Construct a receiver draining `rx_cell` (a kBitStream cell) in `table`.
  // `bit_period_ns` is the time per bit (122070 for 8192 baud). The
  // `oversample_factor` argument is retained for API stability (sub-PRs 4-B
  // / 4-C call it) but is no longer used: a bit-stream cell delivers discrete
  // per-bit-period samples, so the receiver consumes one appended bit per bit
  // and does not oversample. @design 2026-06-13 claude — sub-PR 4-C½.
  UartRx(WireTable* table, WireId rx_cell, std::uint64_t bit_period_ns,
         int oversample_factor = 4);

  // Advance the receiver: drain every bit appended to the stream since this
  // reader's cursor and run the framing FSM over them, decoding bytes into
  // the output queue. `now_ns` is advisory under the bit-stream transport.
  void tick(std::uint64_t now_ns);

  // Pop the next decoded byte. Returns true and writes *out on success; false
  // if the decode queue is empty.
  bool pop_byte(std::uint8_t* out);

  // Framing errors observed since construction (Stop bit sampled logic-0).
  std::uint32_t framing_errors() const noexcept { return framing_errors_; }

  // Overrun events observed since construction (the reader fell more than the
  // ring capacity behind the producer; bits were lost and framing re-synced).
  std::uint32_t overruns() const noexcept { return overruns_; }

 private:
  // Process one decoded bit through the framing FSM.
  void consume_bit(bool bit);

  WireTable*    table_;
  WireId        rx_cell_;
  std::uint64_t bit_period_ns_;

  // Process-local replay cursor into the shared bit-stream ring. Absolute
  // bit index of the next bit this reader will consume; NOT shared.
  std::uint64_t cursor_{0};

  enum class State { kIdle, kData, kStop };
  State         state_{State::kIdle};
  std::uint8_t  shift_reg_{0};       // data bits assembled LSB-first
  int           bit_idx_{0};         // next data bit index (0..7)
  bool          last_high_{true};    // previous bit (for 1→0 edge detection)
  std::uint32_t framing_errors_{0};
  std::uint32_t overruns_{0};

  std::deque<std::uint8_t> byte_q_;  // decoded bytes awaiting pop_byte()
};

}  // namespace electricsim::io

#endif  // ELECTRICSIM_SRC_IO_UART_UART_RX_HPP_
