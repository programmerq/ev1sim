// The silent-detach guard: a failed attach must be distinguishable from a
// deliberate standalone run.
//
// WHAT WENT WRONG WITHOUT THIS
//
// WireTruthChassis::OpenFromEnv() returns nullptr in two opposite situations:
// nobody asked for the wire-truth substrate, and somebody asked and it did not come
// up. The connector's `if (wire)` guard is false either way, so ev1sim would run a
// complete session with every wire dark — lamps, telltales and horn all sitting on
// their "producer hasn't written this yet" defaults, at full frame rate, with no
// diagnostic anywhere. The recorded stats from such a run are indistinguishable
// from a real one.
//
// The likeliest trigger is also the quietest: the external sim's topology hash
// moves and this binary is not rebuilt, so the substrate's attach gate refuses. In
// the external sim's own tree that refusal is loud. Here it was not.
//
// These tests pin the DISTINCTION, which is the part that was missing. They assert
// on AttachOutcome rather than on the fatal exit, because the fatal path lives in
// the connector and calls std::exit — see the note on the third case.
//
// @design 2026-08-07 — circuit-truth downstream upgrade.

#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <string>

#include <unistd.h>  // getpid

#include "WireTruthChassis.h"

#include "topology/topology_generated.h"
#include "wire_table.hpp"

namespace {

using electricsim::io::WireTable;
using electricsim::io::WireTableOptions;
using Outcome = ev1sim::WireTruthChassis::AttachOutcome;

std::string unique_segment(const char* tag) {
    return std::string("ev1sim_ao_") + tag + "_" +
           std::to_string(static_cast<long>(::getpid()));
}

std::unique_ptr<WireTable> create_fleet_table(const std::string& name,
                                              std::uint32_t hash) {
    WireTableOptions opts;
    opts.name = name;
    opts.topology_hash = hash;
    return WireTable::create(opts, [](WireTable& t) {
        return electricsim::topology::declare_all(t);
    });
}

// RAII for ELECTRICSIM_WIRES_NAME so one case cannot leak its environment into the
// next — these tests are precisely about what that variable being set/unset means.
class ScopedWiresName {
public:
    explicit ScopedWiresName(const char* value) {
        const char* prev = std::getenv("ELECTRICSIM_WIRES_NAME");
        had_prev_ = prev != nullptr;
        if (had_prev_) prev_ = prev;
        if (value != nullptr) {
            ::setenv("ELECTRICSIM_WIRES_NAME", value, 1);
        } else {
            ::unsetenv("ELECTRICSIM_WIRES_NAME");
        }
    }
    ~ScopedWiresName() {
        if (had_prev_) {
            ::setenv("ELECTRICSIM_WIRES_NAME", prev_.c_str(), 1);
        } else {
            ::unsetenv("ELECTRICSIM_WIRES_NAME");
        }
    }
private:
    bool had_prev_ = false;
    std::string prev_;
};

}  // namespace

TEST_CASE("AttachOutcome: env unset reports DISABLED, not a failure",
          "[wire_truth][attach_outcome]") {
    ScopedWiresName guard(nullptr);  // wire truth deliberately not requested

    auto wire = ev1sim::WireTruthChassis::OpenFromEnv("attacher");
    REQUIRE(wire == nullptr);
    // The supported way to run ev1sim standalone. It must NOT be an error, or the
    // fatal path in the connector would fire on every plain run.
    REQUIRE(ev1sim::WireTruthChassis::LastAttachOutcome() == Outcome::kDisabled);
}

TEST_CASE("AttachOutcome: a live segment reports ATTACHED",
          "[wire_truth][attach_outcome]") {
    const std::string seg = unique_segment("live");
    auto producer = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(producer != nullptr);

    ScopedWiresName guard(seg.c_str());

    auto wire = ev1sim::WireTruthChassis::OpenFromEnv("attacher");
    REQUIRE(wire != nullptr);
    REQUIRE(wire->attached());
    REQUIRE(ev1sim::WireTruthChassis::LastAttachOutcome() == Outcome::kAttached);
}

TEST_CASE("AttachOutcome: a topology-hash mismatch is REQUESTED_BUT_UNAVAILABLE",
          "[wire_truth][attach_outcome]") {
    // THE CASE THIS FILE EXISTS FOR. A segment created under a different topology
    // hash is exactly what a producer-side topology change looks like to a binary
    // that was not rebuilt. Before the outcome enum, this was indistinguishable
    // from the DISABLED case above — same nullptr, same silence, and ev1sim went on
    // to render a vehicle with no electrical fleet behind it.
    const std::string seg = unique_segment("stalehash");
    auto stale = create_fleet_table(seg, electricsim::topology::kTopologyHash ^ 0x1U);
    REQUIRE(stale != nullptr);

    ScopedWiresName guard(seg.c_str());

    auto wire = ev1sim::WireTruthChassis::OpenFromEnv("attacher");
    REQUIRE(wire == nullptr);
    REQUIRE(ev1sim::WireTruthChassis::LastAttachOutcome() ==
            Outcome::kRequestedButUnavailable);

    // The connector turns this outcome into a loud diagnostic and a refusal to run
    // (exit 70). That is not asserted here because it calls std::exit and would
    // take the test binary with it; what is asserted is the fact the connector
    // branches on, which is the part that did not exist before.
}

TEST_CASE("AttachOutcome: an absent segment is REQUESTED_BUT_UNAVAILABLE",
          "[wire_truth][attach_outcome]") {
    // The other way a requested substrate fails to appear: nobody created it. Same
    // verdict as the hash mismatch, deliberately — from ev1sim's side both mean
    // "you asked for the fleet and it is not there", and both must stop the run.
    ScopedWiresName guard(unique_segment("never_created").c_str());

    auto wire = ev1sim::WireTruthChassis::OpenFromEnv("attacher");
    REQUIRE(wire == nullptr);
    REQUIRE(ev1sim::WireTruthChassis::LastAttachOutcome() ==
            Outcome::kRequestedButUnavailable);
}

TEST_CASE("AttachOutcome: DISABLED and REQUESTED_BUT_UNAVAILABLE are distinct",
          "[wire_truth][attach_outcome]") {
    // The whole point, stated as one assertion: the two nullptr cases must not
    // collapse. If this ever passes by both sides being equal, the silent-detach
    // defect is back regardless of what the other cases say.
    Outcome disabled_outcome;
    {
        ScopedWiresName guard(nullptr);
        (void)ev1sim::WireTruthChassis::OpenFromEnv("attacher");
        disabled_outcome = ev1sim::WireTruthChassis::LastAttachOutcome();
    }

    Outcome unavailable_outcome;
    {
        ScopedWiresName guard(unique_segment("distinct").c_str());
        (void)ev1sim::WireTruthChassis::OpenFromEnv("attacher");
        unavailable_outcome = ev1sim::WireTruthChassis::LastAttachOutcome();
    }

    REQUIRE(disabled_outcome != unavailable_outcome);
    REQUIRE(std::string(ev1sim::WireTruthChassis::AttachOutcomeName(disabled_outcome)) ==
            "DISABLED");
    REQUIRE(std::string(
                ev1sim::WireTruthChassis::AttachOutcomeName(unavailable_outcome)) ==
            "REQUESTED_BUT_UNAVAILABLE");
}

TEST_CASE("CompiledTopologyHash reports this binary's hash",
          "[wire_truth][attach_outcome]") {
    // Half of what the fatal diagnostic prints. A reader comparing this against the
    // segment's hash is how a stale-binary mismatch gets diagnosed in one step
    // instead of by rebuilding and hoping.
    REQUIRE(ev1sim::WireTruthChassis::CompiledTopologyHash() ==
            electricsim::topology::kTopologyHash);
    REQUIRE(ev1sim::WireTruthChassis::CompiledTopologyHash() != 0U);
}
