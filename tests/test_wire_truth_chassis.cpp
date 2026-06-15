// Proof-of-life for the wire-truth migration: ev1sim attaches electricsim's
// shared WireTable and round-trips the horn drive-line cells (chassis
// 4020/4021 -> HORN_DRIVE_LINE_LOW/HIGH) through WireTruthChassis.
//
// The test plays BOTH cross-repo roles in-process: it CREATES the shared table
// the way an electricsim driver does (env_open's creator path: declare_all +
// the canonical topology hash), then ATTACHES through ev1sim's WireTruthChassis
// and asserts the values match. This proves, against the REAL generated
// topology (not a stand-in): the topology-hash attach gate, the WireId
// resolution (kWireHORN_DRIVE_LINE_*), the bit type, the written()/freshness
// fallback contract, and both read (consumer) and write (producer) directions.
//
// Only compiled when EV1SIM_HAVE_WIRE_TRUTH (the electricsim substrate is
// checked out); see CMakeLists.txt.
//
// @design 2026-06-15 — wire-truth migration kickoff.

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>

#include <unistd.h>  // getpid

#include "WireTruthChassis.h"

// electricsim substrate (creator side) — same headers env_open.cpp uses.
#include "topology/topology_generated.h"
#include "wire_table.hpp"

namespace {

using electricsim::io::WireTable;
using electricsim::io::WireTableOptions;

// A unique-per-run segment name so a killed test never collides with a leaked
// segment (mirrors electricsim's $ELECTRICSIM_BUS_NAME convention). The creator
// WireTable unlinks the segment on destruction.
std::string unique_segment(const char* tag) {
    return std::string("ev1sim_wt_") + tag + "_" +
           std::to_string(static_cast<long>(::getpid()));
}

// Create the shared table as an electricsim driver would: declare the full
// topology under the canonical hash before publishing init_complete.
std::unique_ptr<WireTable> create_fleet_table(const std::string& name,
                                              std::uint32_t hash) {
    WireTableOptions opts;
    opts.name = name;
    opts.topology_hash = hash;
    return WireTable::create(opts, [](WireTable& t) {
        return electricsim::topology::declare_all(t);
    });
}

}  // namespace

TEST_CASE("WireTruthChassis: horn drive lines round-trip electricsim -> ev1sim",
          "[wire_truth]") {
    const std::string seg = unique_segment("horn");
    auto producer = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(producer != nullptr);  // creator path must succeed

    // electricsim (LHJB) writes the low-tone horn drive line on the wire.
    REQUIRE(producer->write_bit(
        electricsim::topology::kWireHORN_DRIVE_LINE_LOW, true));

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);
    REQUIRE(wire->attached());

    // Consumer side: low was written -> live value; high never written ->
    // nullopt, so the connector keeps its legacy-ring fallback (kHold-safe).
    REQUIRE(wire->horn_low_drive() == std::optional<bool>(true));
    REQUIRE(wire->horn_high_drive() == std::nullopt);

    // Producer flips low off and writes high on; the consumer tracks it.
    REQUIRE(producer->write_bit(
        electricsim::topology::kWireHORN_DRIVE_LINE_LOW, false));
    REQUIRE(producer->write_bit(
        electricsim::topology::kWireHORN_DRIVE_LINE_HIGH, true));
    REQUIRE(wire->horn_low_drive() == std::optional<bool>(false));
    REQUIRE(wire->horn_high_drive() == std::optional<bool>(true));
}

TEST_CASE("WireTruthChassis: never-written cell reads as nullopt (fallback)",
          "[wire_truth]") {
    const std::string seg = unique_segment("fresh");
    auto producer = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(producer != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);
    REQUIRE(wire->attached());

    // Nothing written yet: every read is nullopt so the consumer stays on its
    // legacy value. This is the seam that lets ev1sim migrate a consumer cell
    // BEFORE electricsim's producer has moved onto the wire.
    REQUIRE(wire->horn_low_drive() == std::nullopt);
    REQUIRE(wire->horn_high_drive() == std::nullopt);
    REQUIRE(wire->read_bit(electricsim::topology::kWireHORN_DRIVE_LINE_LOW) ==
            std::nullopt);
}

TEST_CASE("WireTruthChassis: producer write is visible to a peer reader",
          "[wire_truth]") {
    const std::string seg = unique_segment("prod");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // ev1sim writes a bit (the producer direction for ev1sim-sourced cells);
    // the creator peer observes it through the shared table.
    REQUIRE(wire->write_bit(electricsim::topology::kWireHORN_DRIVE_LINE_HIGH,
                            true));
    bool seen = false;
    REQUIRE(creator->read_bit(
        electricsim::topology::kWireHORN_DRIVE_LINE_HIGH, &seen));
    REQUIRE(seen == true);
}

TEST_CASE("WireTruthChassis: topology-hash mismatch refuses to attach",
          "[wire_truth]") {
    const std::string seg = unique_segment("hash");
    // A segment created under a DIFFERENT topology hash: WireTruthChassis
    // attaches with the canonical kTopologyHash, so the gate must reject it.
    auto stale = create_fleet_table(seg, electricsim::topology::kTopologyHash ^ 0x1U);
    REQUIRE(stale != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire == nullptr);  // hash gate fired
}

TEST_CASE("WireTruthChassis: attach to a missing segment returns nullptr",
          "[wire_truth]") {
    auto wire = ev1sim::WireTruthChassis::Attach(unique_segment("absent"));
    REQUIRE(wire == nullptr);
}
