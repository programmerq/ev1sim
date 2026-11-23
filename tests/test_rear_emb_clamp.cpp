// Tests for RearEmbClamp: the clamp force one EMB rear brake holds under the
// BTCM's signed apply motor command (+1 apply, -1 release, 0 undriven) and its
// park latch motor (+1 latch, -1 unlatch, 0 undriven).
//
// The mechanism under test (RearEmbClamp.h): a driven apply motor holds the
// clamp its current supports; the shoe return spring backdrives an undriven
// motor until the shoes leave the drum; only the park latch holds a clamp with
// no current, and it holds through a dead BTCM.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "BrakeDrum.h"
#include "RearEmbClamp.h"

using Catch::Matchers::WithinAbs;
using ev1sim::BrakeDrum;
using ev1sim::RearEmbClamp;

namespace {

constexpr double kStep = 0.002;  // the sim's default 2 ms physics step

// Drive `cmd` and `latch` for `seconds` at kStep; return the force after the
// last step.
double drive(RearEmbClamp& c, float cmd, double seconds,
             const BrakeDrum::Params& p, bool fresh = true, int latch = 0) {
    const int n = static_cast<int>(seconds / kStep + 0.5);
    double f = c.ForceN();
    for (int i = 0; i < n; ++i) f = c.Update(cmd, latch, fresh, p, kStep);
    return f;
}

}  // namespace

// ---------------------------------------------------------------------------
// Settled map (no step size): every move completes within the call.
// ---------------------------------------------------------------------------

TEST_CASE("RearEmbClamp: an undriven apply motor lets the clamp go",
          "[RearEmbClamp]") {
    // US5366281 rehome Mode 1: the return spring backdrives the motor.
    BrakeDrum::Params p;
    RearEmbClamp c;
    CHECK_THAT(c.Update(+1.0f, 0, true, p), WithinAbs(p.max_shoe_force_n, 1e-9));
    CHECK(c.Update(0.0f, 0, true, p) == 0.0);
    // ...to the running clearance, not to home: inside it nothing pushes.
    CHECK_FALSE(c.InContact());
    CHECK_FALSE(c.AtHome());
    CHECK_THAT(c.TravelS(), WithinAbs(c.TravelParams().clearance_s, 1e-12));
}

TEST_CASE("RearEmbClamp: a part command holds a part clamp", "[RearEmbClamp]") {
    // Current-controlled apply (US5366281 Fig 10): the clamp follows the
    // effort, so a hold needs current and a lower current holds less.
    BrakeDrum::Params p;
    RearEmbClamp c;
    CHECK_THAT(c.Update(0.6f, 0, true, p), WithinAbs(0.6 * p.max_shoe_force_n, 1e-3));
    for (int i = 0; i < 10; ++i) {
        CHECK_THAT(c.Update(0.6f, 0, true, p), WithinAbs(0.6 * p.max_shoe_force_n, 1e-3));
    }
    CHECK_THAT(c.Update(0.25f, 0, true, p), WithinAbs(0.25 * p.max_shoe_force_n, 1e-3));
}

TEST_CASE("RearEmbClamp: RELEASE drives the actuator home", "[RearEmbClamp]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    c.Update(+1.0f, 0, true, p);
    CHECK(c.Update(-1.0f, 0, true, p) == 0.0);
    CHECK(c.AtHome());
    CHECK(c.Update(0.0f, 0, true, p) == 0.0);
    CHECK(c.AtHome());  // the spring does not push a home actuator anywhere
}

TEST_CASE("RearEmbClamp: a stale BTCM lets an unlatched clamp go",
          "[RearEmbClamp]") {
    // No hydraulic backup on the EV1 rear, and a dead BTCM drives nothing.
    BrakeDrum::Params p;
    RearEmbClamp c;
    c.Update(+1.0f, 0, true, p);
    CHECK(c.Update(+1.0f, 0, false, p) == 0.0);
}

TEST_CASE("RearEmbClamp: a set park latch holds the clamp with no power",
          "[RearEmbClamp][Latch]") {
    // US5366281 Fig 18a/18b: the latch is its own motor, and a set latch
    // stays set undriven, so the park brake holds through total power loss.
    BrakeDrum::Params p;
    RearEmbClamp c;
    c.Update(+1.0f, 0, true, p);
    c.Update(+1.0f, +1, true, p);
    CHECK(c.Latched());
    CHECK_THAT(c.Update(0.0f, 0, true, p), WithinAbs(p.max_shoe_force_n, 1e-9));
    CHECK_THAT(c.Update(0.0f, 0, false, p), WithinAbs(p.max_shoe_force_n, 1e-9));
    // A release drive against the latch moves nothing.
    CHECK_THAT(c.Update(-1.0f, 0, true, p), WithinAbs(p.max_shoe_force_n, 1e-9));
    CHECK(c.Latched());
    // Clearing the latch lets the spring take the clamp off.
    c.Update(0.0f, -1, true, p);
    CHECK_FALSE(c.Latched());
    CHECK(c.Update(0.0f, 0, true, p) == 0.0);
}

// ---------------------------------------------------------------------------
// With a step size.
// ---------------------------------------------------------------------------

TEST_CASE("RearEmbClamp: APPLY takes up the clearance before any force",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    CHECK(drive(c, +1.0f, t.clearance_s - 2 * kStep, p) == 0.0);
    CHECK(c.TravelS() > 0.0);
    CHECK_FALSE(c.InContact());
    CHECK(drive(c, +1.0f, 4 * kStep, p) > 0.0);
    CHECK(c.InContact());
}

TEST_CASE("RearEmbClamp: full apply from home takes clearance plus rise",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    const double total = t.clearance_s + t.rise_s;
    CHECK(drive(c, +1.0f, total - 4 * kStep, p) < p.max_shoe_force_n);
    CHECK_THAT(drive(c, +1.0f, 4 * kStep, p), WithinAbs(p.max_shoe_force_n, 1e-6));
    // A stalled motor goes no further however long it is driven.
    const double travel_at_stall = c.TravelS();
    drive(c, +1.0f, 1.0, p);
    CHECK(c.TravelS() == travel_at_stall);
}

TEST_CASE("RearEmbClamp: a part command rises at its speed to its clamp",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    // Half speed for twice the clearance time: only just at contact.
    CHECK_THAT(drive(c, 0.5f, 2.0 * t.clearance_s, p), WithinAbs(0.0, 1e-6));
    // Then on to half the maximum, and no further.
    drive(c, 0.5f, 2.0, p);
    CHECK_THAT(c.ForceN(), WithinAbs(0.5 * p.max_shoe_force_n, 1e-6));
}

TEST_CASE("RearEmbClamp: the spring takes the clamp off over its travel",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    drive(c, +1.0f, 1.0, p);
    // Undriven for 40 % of the spring's time: 40 % of the force gone.
    const double part = drive(c, 0.0f, 0.4 * t.rise_s / t.spring_speed, p);
    CHECK_THAT(part, WithinAbs(0.6 * p.max_shoe_force_n, 1e-6 * p.max_shoe_force_n));
    // Then all of it, stopping at the clearance.
    CHECK(drive(c, 0.0f, 1.0, p) == 0.0);
    CHECK_THAT(c.TravelS(), WithinAbs(t.clearance_s, 1e-9));
}

TEST_CASE("RearEmbClamp: an anti-lock DUMP then HOLD keeps the dumped clamp",
          "[RearEmbClamp][Travel]") {
    // The BTCM's anti-lock HOLD keeps the apply lead driven at its walked-down
    // level, so the clamp it holds is the one the dump left.
    BrakeDrum::Params p;
    RearEmbClamp c;
    drive(c, 0.8f, 1.0, p);
    const double dumped = drive(c, -1.0f, 0.020, p);
    CHECK(dumped < 0.8 * p.max_shoe_force_n);
    const float hold = static_cast<float>(dumped / p.max_shoe_force_n);
    CHECK_THAT(drive(c, hold, 0.100, p), WithinAbs(dumped, 1e-6 * p.max_shoe_force_n));
}

TEST_CASE("RearEmbClamp: a latch drive shorter than its stroke changes nothing",
          "[RearEmbClamp][Latch]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    drive(c, +1.0f, 1.0, p);
    drive(c, +1.0f, t.latch_stroke_s / 2.0, p, true, +1);
    CHECK_FALSE(c.Latched());
    drive(c, +1.0f, t.latch_stroke_s, p, true, +1);
    CHECK(c.Latched());
    drive(c, 0.0f, t.latch_stroke_s / 2.0, p, true, -1);
    CHECK(c.Latched());
    CHECK_THAT(drive(c, 0.0f, 1.0, p), WithinAbs(p.max_shoe_force_n, 1e-9));
}

TEST_CASE("RearEmbClamp: travel times are configurable", "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp::Travel tr;
    tr.clearance_s = 0.0;
    tr.rise_s = 0.100;
    RearEmbClamp c(tr);
    CHECK(c.Update(+1.0f, 0, true, p, kStep) > 0.0);  // no clearance: force at once
    CHECK_THAT(drive(c, +1.0f, 0.100, p), WithinAbs(p.max_shoe_force_n, 1e-9));
}
