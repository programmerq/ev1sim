// Tests for RearEmbClamp: the clamp force one EMB rear brake holds under the
// BTCM's signed motor command (+1 apply, -1 release, 0 hold).
//
// Two things matter. HOLD: an anti-lock HOLD on the rear axle is the BTCM
// latching the actuator where it is, so a 0 must keep the force, not drop it.
// And travel: the command drives a motor, so with a step size the force
// builds through clearance take-up and a compliant rise, and falls back the
// same way, instead of jumping to the commanded level.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "BrakeDrum.h"
#include "RearEmbClamp.h"

using Catch::Matchers::WithinAbs;
using ev1sim::BrakeDrum;
using ev1sim::RearEmbClamp;

namespace {

constexpr double kStep = 0.002;  // the sim's default 2 ms physics step

// Drive `cmd` for `seconds` at kStep; return the force after the last step.
double drive(RearEmbClamp& c, float cmd, double seconds,
             const BrakeDrum::Params& p, bool fresh = true) {
    const int n = static_cast<int>(seconds / kStep + 0.5);
    double f = c.ForceN();
    for (int i = 0; i < n; ++i) f = c.Update(cmd, fresh, p, kStep);
    return f;
}

}  // namespace

// ---------------------------------------------------------------------------
// Settled map (no step size): what SimApp::ApplyRearEmbBrake gets today.
// ---------------------------------------------------------------------------

TEST_CASE("RearEmbClamp: HOLD keeps the clamp force an APPLY built",
          "[RearEmbClamp]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    CHECK_THAT(c.Update(+1.0f, true, p), WithinAbs(p.max_shoe_force_n, 1e-9));
    for (int i = 0; i < 100; ++i) {
        CHECK_THAT(c.Update(0.0f, true, p), WithinAbs(p.max_shoe_force_n, 1e-9));
    }
}

TEST_CASE("RearEmbClamp: RELEASE drops the force and HOLD then keeps it off",
          "[RearEmbClamp]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    c.Update(+1.0f, true, p);
    CHECK(c.Update(-1.0f, true, p) == 0.0);
    CHECK(c.AtHome());
    CHECK(c.Update(0.0f, true, p) == 0.0);
}

TEST_CASE("RearEmbClamp: an anti-lock APPLY/HOLD/DUMP cycle", "[RearEmbClamp]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const float cmds[]    = {+1.0f, 0.0f, 0.0f, -1.0f, 0.0f, +1.0f, 0.0f};
    const double expect[] = {1.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0};
    for (int i = 0; i < 7; ++i) {
        CHECK_THAT(c.Update(cmds[i], true, p),
                   WithinAbs(expect[i] * p.max_shoe_force_n, 1e-9));
    }
}

TEST_CASE("RearEmbClamp: a stale BTCM holds nothing", "[RearEmbClamp]") {
    // No hydraulic backup on the EV1 rear: a module that is not commanding the
    // motors is not holding a latch either.
    BrakeDrum::Params p;
    RearEmbClamp c;
    c.Update(+1.0f, true, p);
    CHECK(c.Update(0.0f, false, p) == 0.0);
    CHECK(c.AtHome());
    CHECK(c.Update(0.0f, true, p) == 0.0);  // and a HOLD after it holds zero
}

// ---------------------------------------------------------------------------
// Shoe-position integration (with a step size).
// ---------------------------------------------------------------------------

TEST_CASE("RearEmbClamp: APPLY takes up the clearance before any force",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();

    // Just short of contact: the motor has been running, the shoes have moved,
    // and there is still no force on the drum.
    CHECK(drive(c, +1.0f, t.clearance_s - 2 * kStep, p) == 0.0);
    CHECK(c.TravelS() > 0.0);
    CHECK_FALSE(c.InContact());

    // A few steps further the shoes are on the drum and force has begun.
    CHECK(drive(c, +1.0f, 4 * kStep, p) > 0.0);
    CHECK(c.InContact());
}

TEST_CASE("RearEmbClamp: force rises with travel past contact to the stall",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();

    drive(c, +1.0f, t.clearance_s, p);
    double prev = c.ForceN();
    // 40 % of the way up the rise: 40 % of the maximum force (linear
    // compliance).
    const double partial = drive(c, +1.0f, 0.4 * t.rise_s, p);
    CHECK_THAT(partial, WithinAbs(0.4 * p.max_shoe_force_n, 1e-6 * p.max_shoe_force_n));
    CHECK(partial > prev);
    prev = partial;

    // Every further APPLY step adds force until the motor stalls at the max.
    for (int i = 0; i < 200; ++i) {
        const double f = c.Update(+1.0f, true, p, kStep);
        CHECK(f >= prev);
        prev = f;
    }
    CHECK_THAT(prev, WithinAbs(p.max_shoe_force_n, 1e-9));
    // A stalled motor goes no further however long it is driven.
    const double travel_at_stall = c.TravelS();
    drive(c, +1.0f, 1.0, p);
    CHECK(c.TravelS() == travel_at_stall);
    CHECK_THAT(c.ForceN(), WithinAbs(p.max_shoe_force_n, 1e-9));
}

TEST_CASE("RearEmbClamp: full apply from home takes clearance plus rise",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    const double total = t.clearance_s + t.rise_s;

    CHECK(drive(c, +1.0f, total - 4 * kStep, p) < p.max_shoe_force_n);
    CHECK_THAT(drive(c, +1.0f, 4 * kStep, p), WithinAbs(p.max_shoe_force_n, 1e-6));
}

TEST_CASE("RearEmbClamp: RELEASE backs the force off and stops at home",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    drive(c, +1.0f, 1.0, p);  // stalled at max

    // Reversing for 40 % of the rise takes 40 % of the force off.
    CHECK_THAT(drive(c, -1.0f, 0.4 * t.rise_s, p),
               WithinAbs(0.6 * p.max_shoe_force_n, 1e-6 * p.max_shoe_force_n));
    // The rest of the rise brings it to zero; the shoes are off the drum
    // before the actuator is home.
    CHECK(drive(c, -1.0f, 0.6 * t.rise_s + 2 * kStep, p) == 0.0);
    CHECK_FALSE(c.InContact());
    CHECK_FALSE(c.AtHome());
    // Driving on in RELEASE reaches the home stop and stays there.
    drive(c, -1.0f, 1.0, p);
    CHECK(c.AtHome());
    CHECK(c.TravelS() == 0.0);
}

TEST_CASE("RearEmbClamp: HOLD mid-rise keeps a partial force",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    drive(c, +1.0f, t.clearance_s + t.rise_s / 4.0, p);
    const double held = c.ForceN();
    CHECK(held > 0.0);
    CHECK(held < p.max_shoe_force_n);
    CHECK(drive(c, 0.0f, 0.5, p) == held);
}

TEST_CASE("RearEmbClamp: an anti-lock cycle modulates without stepping",
          "[RearEmbClamp][Travel]") {
    // A short DUMP pulse takes some force off, not all of it: the release
    // rate is the motor's, so the force moves by the travel the pulse buys.
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    drive(c, +1.0f, 1.0, p);                              // APPLY to stall
    const double dumped = drive(c, -1.0f, 0.020, p);      // 20 ms DUMP
    const double per_s = p.max_shoe_force_n / t.rise_s;
    CHECK_THAT(dumped, WithinAbs(p.max_shoe_force_n - 0.020 * per_s, 1e-6));
    CHECK(drive(c, 0.0f, 0.100, p) == dumped);            // HOLD
    const double reapplied = drive(c, +1.0f, 0.010, p);   // 10 ms APPLY
    CHECK_THAT(reapplied, WithinAbs(dumped + 0.010 * per_s, 1e-6));
}

TEST_CASE("RearEmbClamp: a fractional command scales the motor speed",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    drive(c, 0.5f, 2.0 * t.clearance_s, p);
    // Half speed for twice the clearance time: only just at contact.
    CHECK_THAT(c.ForceN(), WithinAbs(0.0, 1e-6 * p.max_shoe_force_n));
    CHECK_THAT(c.TravelS(), WithinAbs(t.clearance_s, 1e-9));
    // An out-of-range command saturates at full speed.
    RearEmbClamp d;
    drive(d, 3.0f, t.clearance_s, p);
    CHECK_THAT(d.TravelS(), WithinAbs(t.clearance_s, 1e-9));
}

TEST_CASE("RearEmbClamp: a stale BTCM mid-apply sends the actuator home",
          "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    const auto& t = c.TravelParams();
    drive(c, +1.0f, 1.0, p);
    CHECK(c.Update(+1.0f, false, p, kStep) == 0.0);
    CHECK(c.AtHome());
    // When the BTCM comes back the clearance has to be taken up again.
    CHECK(drive(c, +1.0f, t.clearance_s - 2 * kStep, p) == 0.0);
}

TEST_CASE("RearEmbClamp: travel times are configurable", "[RearEmbClamp][Travel]") {
    BrakeDrum::Params p;
    RearEmbClamp::Travel tr;
    tr.clearance_s = 0.0;
    tr.rise_s = 0.100;
    RearEmbClamp c(tr);
    CHECK(c.Update(+1.0f, true, p, kStep) > 0.0);  // no clearance: force at once
    CHECK_THAT(drive(c, +1.0f, 0.100, p), WithinAbs(p.max_shoe_force_n, 1e-9));
}
