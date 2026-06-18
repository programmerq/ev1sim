/*
 * Wire-truth default / init-policy helpers.
 *
 * Layered on top of WireTable::Sample / read_*_sample (round-4 step 4a)
 * and the `default` / `init_policy` per-net YAML schema (round-4 step
 * 4b). A consumer that chooses the `default` policy for a wire can use
 * the read_*_or_default() wrappers below; consumers that choose `hold`
 * or `fault` express that branch themselves (see
 * docs/wire_truth_written_ness.md §"Layer 2 — per-net defaults + init
 * policy" for the per-policy shape).
 *
 * The substrate-side fact these helpers consume is the gen==0 marker
 * (Cell.write_gen, round-4 step 4a). Without it the substrate could
 * not distinguish a never-written cell from a producer that wrote 0;
 * read_or_default could only return the live value or fall back on
 * undeclared, neither of which dissolves the "Pattern A" trap that
 * motivated this round.
 *
 * Per-net constants — kWire<NAME>_Default and kWire<NAME>_InitPolicy
 * — are emitted into topology_generated.h by
 * scripts/gen_topology_header.py and read in lockstep with the named
 * kWire<NAME> WireId. Their values are folded into kTopologyHash, so a
 * producer / consumer disagreement on a default or policy is caught
 * by the same attach-time check that catches a type or order drift.
 *
 * @design 2026-06-11 claude — round-4, step 4b.
 */

#ifndef ELECTRICSIM_SRC_IO_TOPOLOGY_WIRE_DEFAULTS_HPP_
#define ELECTRICSIM_SRC_IO_TOPOLOGY_WIRE_DEFAULTS_HPP_

#include "wire_table.hpp"

#include <cstdint>

namespace electricsim::topology {

// Consumer-side behaviour for a wire whose cell has never been written
// (Cell.write_gen == 0). Per-net via the YAML's `init_policy:` key;
// emitted as kWire<NAME>_InitPolicy in topology_generated.h.
//
// kHold (default) — keep whatever the consumer already had: its
//   initialised internal state, its dying ring-bus reading, etc. Chosen
//   for nets where "no producer yet" is benign and the consumer's
//   pre-substrate behaviour is the correct fallback. Most wires want
//   this; RUN1 in particular (BPM supervisor initialises run1_present
//   = true, so holding the init is what avoids latching DTC 281 under
//   an absent producer).
//
// kDefault — substitute the declared default value (kWire<NAME>_Default).
//   Chosen for nets where the consumer needs a concrete value to compute
//   with and the "no producer yet" baseline is a known good level (e.g.
//   a level that the real hardware sits at when no driver is asserting).
//
// kFault — treat unwritten-after-init as a fault condition. Chosen for
//   safety-critical interlocks where silence is itself an error (e.g.
//   HV interlock continuity).
enum class InitPolicy : std::uint8_t {
  kHold = 0,
  kDefault = 1,
  kFault = 2,
};

// read_*_or_default — convenience for the kDefault policy. Returns the
// live wire value when the cell has been written (gen > 0); returns the
// declared default when never written (gen == 0) OR on
// undeclared/type-mismatch. The undeclared branch returns the default
// because the call-site usage pattern fuses the WireId, the type, and
// the default value via compile-time constants
// (kWire<NAME>, kWire<NAME>_Default), so an undeclared id at runtime
// would indicate a topology mismatch — and in that case substituting
// the declared default is at least as safe as any other choice. (A
// consumer that wants a hard fault on type mismatch should use
// read_*_sample directly.)
//
// One per cell type; signatures mirror the WireType set. Each is a
// thin inline so the binary size cost is zero.

inline bool read_bit_or_default(const ::electricsim::io::WireTable& table,
                                ::electricsim::io::WireId id,
                                bool default_value) {
  ::electricsim::io::WireTable::Sample<bool> s;
  if (!table.read_bit_sample(id, &s)) return default_value;
  return s.written() ? s.value : default_value;
}

inline std::uint8_t read_byte_or_default(const ::electricsim::io::WireTable& table,
                                         ::electricsim::io::WireId id,
                                         std::uint8_t default_value) {
  ::electricsim::io::WireTable::Sample<std::uint8_t> s;
  if (!table.read_byte_sample(id, &s)) return default_value;
  return s.written() ? s.value : default_value;
}

inline std::uint16_t read_uint16_or_default(const ::electricsim::io::WireTable& table,
                                            ::electricsim::io::WireId id,
                                            std::uint16_t default_value) {
  ::electricsim::io::WireTable::Sample<std::uint16_t> s;
  if (!table.read_uint16_sample(id, &s)) return default_value;
  return s.written() ? s.value : default_value;
}

inline std::uint32_t read_uint32_or_default(const ::electricsim::io::WireTable& table,
                                            ::electricsim::io::WireId id,
                                            std::uint32_t default_value) {
  ::electricsim::io::WireTable::Sample<std::uint32_t> s;
  if (!table.read_uint32_sample(id, &s)) return default_value;
  return s.written() ? s.value : default_value;
}

inline float read_float32_or_default(const ::electricsim::io::WireTable& table,
                                     ::electricsim::io::WireId id,
                                     float default_value) {
  ::electricsim::io::WireTable::Sample<float> s;
  if (!table.read_float32_sample(id, &s)) return default_value;
  return s.written() ? s.value : default_value;
}

}  // namespace electricsim::topology

#endif  // ELECTRICSIM_SRC_IO_TOPOLOGY_WIRE_DEFAULTS_HPP_
