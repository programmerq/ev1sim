/*
 * UartTx — host-side UART transmitter state machine for the GM-8192 bit-wire
 * substrate.
 *
 * Serialises a queue of bytes onto a `bit_stream`-typed WireTable cell
 * (WireType::kBitStream), one bit per GM-8192 bit period, following the EV1
 * UART physical layer. Each bit is APPENDED to the cell's bounded bit ring
 * (round-19, sub-PR 4-C½) rather than overwriting a single level, so a
 * coarse host tick that crosses many 122 µs bit boundaries preserves every
 * bit for the consumer instead of clobbering all but the last. The frame
 * grammar is unchanged:
 *
 *   Frame:    Start(0) + 8 data bits LSB-first + Stop(1)   — 10-bit word.
 *   Baud:     8192 baud; bit period = 122.07 µs (122070 ns).
 *   Idle Line: continuous logic-1 (≥ 10 bit times is the bus-quiet minimum).
 *   @source:manual; @source:redux bus/ev1_uart_bus.yaml
 *   docs/gm8192_protocol.md §"Physical layer".
 *
 * Idle Line is a *state-machine boundary, not a byte* (gm8192_protocol.md
 * §Notes 5): the transmitter does NOT inject a fixed inter-byte gap. While the
 * queue holds bytes it emits them back-to-back (each Stop bit is immediately
 * followed by the next Start bit); the line only floats high when the queue
 * drains. This is what makes both "≥10-bit idle between bursts" and
 * "back-to-back bytes with no inter-byte gap" fall out of the same logic.
 *
 * The state machine is driven by an external monotonic timestamp (`now_ns`)
 * supplied by the caller's tick loop — it owns no thread or real-time clock,
 * keeping it decoupled from scheduling policy (synthetic time in tests, the
 * fleet wall clock in production).
 * @design 2026-06-13 claude — docs/wire_truth_gm8192_step4.md §"Sub-PR 4-B".
 *
 * Host-side only. No simavr dependency, no module controller changes. Wire
 * coupling to specific GM8192_*_TX topology cells is sub-PRs 4-D through 4-L.
 */

#ifndef ELECTRICSIM_SRC_IO_UART_UART_TX_HPP_
#define ELECTRICSIM_SRC_IO_UART_UART_TX_HPP_

#include "wire_table.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>

namespace electricsim::io {

class UartTx {
 public:
  // Construct a transmitter driving `tx_cell` (a kBitStream cell) in
  // `table`. `bit_period_ns` is the time per bit (122070 for 8192 baud).
  // The stream starts empty; the Idle Line is the absence of appended bits
  // (a receiver treats an un-sampled line as resting logic-1).
  // @source:manual; docs/gm8192_protocol.md.
  UartTx(WireTable* table, WireId tx_cell, std::uint64_t bit_period_ns);

  // Enqueue bytes for transmission (shifted out LSB-first per byte). enqueue()
  // and tick() must not be called concurrently without external synchronisation.
  void enqueue(const std::uint8_t* data, std::size_t len);
  void enqueue(std::uint8_t byte);

  // Advance to `now_ns`, emitting every bit whose boundary falls at or before
  // it. One call may emit several bits if `now_ns` jumped multiple bit periods
  // since the last tick. Each emitted bit is written to the wire cell.
  void tick(std::uint64_t now_ns);

  // True when no frame is in flight and the queue is empty (cell held at the
  // last Stop bit's logic-1 = Idle Line).
  bool idle() const noexcept;

 private:
  // Frame bit value for position `pos` (0=Start, 1..8=data LSB-first, 9=Stop).
  // @source:manual; docs/gm8192_protocol.md §"Physical layer".
  static bool frame_bit(std::uint8_t byte, int pos);

  WireTable*    table_;
  WireId        tx_cell_;
  std::uint64_t bit_period_ns_;

  enum class State { kIdle, kInFrame };
  State         state_{State::kIdle};
  std::uint8_t  cur_byte_{0};       // byte currently being serialised
  int           bit_pos_{0};        // 0..9 within the current frame
  std::uint64_t next_edge_ns_{0};   // monotonic time of the next bit boundary
  bool          seeded_{false};     // has next_edge_ns_ been aligned to real time?

  std::deque<std::uint8_t> queue_;  // bytes awaiting transmission
};

}  // namespace electricsim::io

#endif  // ELECTRICSIM_SRC_IO_UART_UART_TX_HPP_
