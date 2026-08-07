// src/net_host/conductor_publisher.hpp — THE publish edge.
//
// This is the one and only holder of io::SolverToken's private constructor. Nothing
// else in the tree may name SolverToken, and nothing else can therefore reach
// WireTable::publish_conductor().
//
// WHY IT IS HERE AND NOT IN src/net/solver/publish.cpp. src/net/ compiles
// STANDALONE against nothing but the C++ standard library — no embedded-hardware
// dependencies — and
// tests/circuit_truth/net_core_build.sh enforces it by GLOBBING every .cpp under
// src/net/ and compiling each with `-I src/net` and no other include path. A
// publish.cpp under src/net/solver/ would have to include src/io/wire_table.hpp, so
// it would break that build — the one suite that must stay green at every commit.
// The publish edge therefore lives one directory out, in src/net_host/, which hosts
// the solver's I/O-facing integration points.
//
// The rule that mattered is preserved exactly: ONE place constructs the token —
// "one friend"; if a caller needs two, the fix is to fix the split, not grow the
// friend list.

#pragma once

#include <cstdint>

#include "../io/wire_table.hpp"

namespace electricsim {
namespace net_host {

class ConductorPublisher {
 public:
  explicit ConductorPublisher(io::WireTable& table) : table_(&table) {}

  // Publish one conductor cell's derived energisation.
  //
  // `energised` must be the output of the solver's provenance computation for the
  // node this cell belongs to. There is no runtime
  // way to check that here, which is precisely why the capability is scoped to this
  // class instead of being a flag on a call: the check is "who is allowed to hold
  // the token", and the answer is one class in one file.
  bool publish(io::ConductorId id, bool energised) {
    const bool ok = table_->publish_conductor(id, energised, token_);
    if (ok) {
      ++published_;
      if (energised) ++energised_;
    } else {
      ++refused_;
    }
    return ok;
  }

  // Receipt counters. `refused` is not decoration: publish_conductor returns false on
  // an undeclared id or a type mismatch, and a host that published nothing all tick
  // because every id was wrong would otherwise look identical to a dark vehicle.
  std::uint64_t published() const noexcept { return published_; }
  std::uint64_t published_energised() const noexcept { return energised_; }
  std::uint64_t refused() const noexcept { return refused_; }
  void reset_counters() noexcept { published_ = energised_ = refused_ = 0; }

 private:
  io::WireTable* table_;
  // The token. Its constructor is private to io::SolverToken and this class is its
  // only friend, so this member declaration is the single line in the repository
  // that can bring one into existence.
  io::SolverToken token_{};
  std::uint64_t published_ = 0;
  std::uint64_t energised_ = 0;
  std::uint64_t refused_ = 0;
};

}  // namespace net_host
}  // namespace electricsim
