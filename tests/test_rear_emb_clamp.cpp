// Tests for RearEmbClamp: the clamp force one EMB rear brake holds under the
// BTCM's signed motor command (+1 apply, -1 release, 0 hold).
//
// The case that matters is HOLD. An anti-lock HOLD on the rear axle is the
// BTCM latching the actuator where it is, and SimApp::ApplyRearEmbBrake used
// to read the 0 it publishes for that as "no force", which released the rear
// axle every time the anti-lock kernel held it.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "BrakeDrum.h"
#include "RearEmbClamp.h"

using Catch::Matchers::WithinAbs;
using ev1sim::BrakeDrum;
using ev1sim::RearEmbClamp;

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
    CHECK(c.Update(0.0f, true, p) == 0.0);  // and a HOLD after it holds zero
}

TEST_CASE("RearEmbClamp: a positive command scales and saturates",
          "[RearEmbClamp]") {
    BrakeDrum::Params p;
    RearEmbClamp c;
    CHECK_THAT(c.Update(0.25f, true, p), WithinAbs(0.25 * p.max_shoe_force_n, 1e-6));
    CHECK_THAT(c.Update(3.0f, true, p), WithinAbs(p.max_shoe_force_n, 1e-9));
    CHECK(c.ForceN() == p.max_shoe_force_n);
}
