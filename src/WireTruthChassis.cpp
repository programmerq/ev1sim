// WireTruthChassis implementation. See WireTruthChassis.h for the contract.
//
// @design 2026-06-15 — wire-truth migration kickoff (proof-of-life batch).

#include "WireTruthChassis.h"

#if defined(EV1SIM_HAVE_WIRE_TRUTH)

// Real implementation: attach electricsim's shared WireTable. These headers
// come from the electricsim tree (compiled into the electricsim_wire_substrate
// static lib); the include root is ${ELECTRICSIM_DIR}/src/io.
#include "topology/env_open.hpp"          // electricsim::topology::try_open_from_env
#include "topology/topology_generated.h"  // kTopologyHash, kWireHORN_DRIVE_LINE_*
#include "wire_table.hpp"                 // electricsim::io::WireTable

#include <utility>

namespace ev1sim {

struct WireTruthChassis::Impl {
    std::unique_ptr<electricsim::io::WireTable> table;
};

WireTruthChassis::WireTruthChassis() : impl_(std::make_unique<Impl>()) {}
WireTruthChassis::~WireTruthChassis() = default;

std::unique_ptr<WireTruthChassis> WireTruthChassis::OpenFromEnv(
    const char* default_role) {
    auto table = electricsim::topology::try_open_from_env(default_role);
    if (!table) return nullptr;  // disabled / segment down / hash mismatch
    std::unique_ptr<WireTruthChassis> self(new WireTruthChassis());
    self->impl_->table = std::move(table);
    return self;
}

std::unique_ptr<WireTruthChassis> WireTruthChassis::Attach(
    const std::string& segment_name) {
    electricsim::io::WireTableOptions opts;
    opts.name = segment_name;
    opts.topology_hash = electricsim::topology::kTopologyHash;
    auto table = electricsim::io::WireTable::attach(opts);
    if (!table) return nullptr;
    std::unique_ptr<WireTruthChassis> self(new WireTruthChassis());
    self->impl_->table = std::move(table);
    return self;
}

bool WireTruthChassis::attached() const {
    return impl_ && impl_->table != nullptr;
}

std::optional<bool> WireTruthChassis::read_bit(std::uint32_t wire_id) const {
    if (!attached()) return std::nullopt;
    electricsim::io::WireTable::Sample<bool> sample;
    if (!impl_->table->read_bit_sample(wire_id, &sample)) return std::nullopt;
    if (!sample.written()) return std::nullopt;  // producer hasn't moved yet
    return sample.value;
}

bool WireTruthChassis::write_bit(std::uint32_t wire_id, bool value) {
    if (!attached()) return false;
    return impl_->table->write_bit(wire_id, value);
}

std::optional<bool> WireTruthChassis::horn_low_drive() const {
    return read_bit(electricsim::topology::kWireHORN_DRIVE_LINE_LOW);
}

std::optional<bool> WireTruthChassis::horn_high_drive() const {
    return read_bit(electricsim::topology::kWireHORN_DRIVE_LINE_HIGH);
}

}  // namespace ev1sim

#else  // !EV1SIM_HAVE_WIRE_TRUTH

// Disabled stub: the electricsim wire-truth substrate is not available at build
// time (e.g. the CI integrated-build against tests/electricsim_stub, which has
// no wire_table.cpp / topology_generated.h). Factories return nullptr so every
// caller's `if (wire)` guard takes the legacy-ring path; the accessors exist
// only to satisfy the linker and never run.

namespace ev1sim {

struct WireTruthChassis::Impl {};

WireTruthChassis::WireTruthChassis() = default;
WireTruthChassis::~WireTruthChassis() = default;

std::unique_ptr<WireTruthChassis> WireTruthChassis::OpenFromEnv(const char*) {
    return nullptr;
}
std::unique_ptr<WireTruthChassis> WireTruthChassis::Attach(const std::string&) {
    return nullptr;
}
bool WireTruthChassis::attached() const { return false; }
std::optional<bool> WireTruthChassis::read_bit(std::uint32_t) const {
    return std::nullopt;
}
bool WireTruthChassis::write_bit(std::uint32_t, bool) { return false; }
std::optional<bool> WireTruthChassis::horn_low_drive() const {
    return std::nullopt;
}
std::optional<bool> WireTruthChassis::horn_high_drive() const {
    return std::nullopt;
}

}  // namespace ev1sim

#endif  // EV1SIM_HAVE_WIRE_TRUTH
