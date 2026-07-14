// Unit tests for the Chrono-free longitudinal slip-ratio helper
// (src/SlipRatio.h).
//
// VehicleWorld computes the published per-wheel slip_ratio through this same
// helper, so pinning the formula, the clamp, and — above all — the standstill
// speed floor here makes the behaviour verifiable without booting Chrono.
//
// The floor exists because recorded braking-to-stop runs show the vehicle
// (TMeasy tire, no true stiction) settling into a slow residual creep of
// ~0.10-0.12 m/s for several seconds after a stop.  The old 0.1 m/s guard sat
// exactly on that band, so the near-zero/near-zero quotient railed to a
// clamped +/-1 in bursts on a stationary car.  These tests pin that the whole
// creep band now reads slip == 0, and that real slip is still computed just
// above the floor.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "SlipRatio.h"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using ev1sim::SlipRatio;

// EV1 TMeasy unloaded radius (m) — same value VehicleWorld passes in.
static constexpr double kTireRadius = 0.2915;

// Wheel angular speed (rad/s) that free-rolls at the given vehicle speed.
static double free_rolling_omega(double v_mps) { return v_mps / kTireRadius; }

TEST_CASE("SlipRatio: zero at exact standstill", "[SlipRatio]") {
    CHECK(SlipRatio::longitudinal(0.0, 0.0, kTireRadius) == 0.0);
    // Spinning wheels on a stationary car still report 0 (below the floor).
    CHECK(SlipRatio::longitudinal(0.0, 50.0, kTireRadius) == 0.0);
}

TEST_CASE("SlipRatio: zero across the post-stop creep band (0.05-0.49 m/s)",
          "[SlipRatio]") {
    // The no-stiction standstill creep measures ~0.10-0.12 m/s; sample the
    // whole band up to just under the floor, wheels held locked (brakes on),
    // which is exactly the case that used to rail to +1.
    for (double v : {0.05, 0.08, 0.10, 0.105, 0.12, 0.15, 0.3, 0.49}) {
        INFO("v = " << v << " m/s, wheels locked");
        CHECK(SlipRatio::longitudinal(v, 0.0, kTireRadius) == 0.0);
        CHECK(SlipRatio::longitudinal(-v, 0.0, kTireRadius) == 0.0);
    }
}

TEST_CASE("SlipRatio: the floor itself is inclusive (zero AT 0.5 m/s)",
          "[SlipRatio]") {
    CHECK(SlipRatio::kSpeedFloorMps == 0.5);
    CHECK(SlipRatio::longitudinal(SlipRatio::kSpeedFloorMps, 0.0, kTireRadius)
          == 0.0);
}

TEST_CASE("SlipRatio: computed and clamped just above the floor", "[SlipRatio]") {
    // v = 0.6 m/s, wheel locked -> full braking slip, clamped at +1.
    CHECK_THAT(SlipRatio::longitudinal(0.6, 0.0, kTireRadius),
               WithinAbs(1.0, 1e-12));
    // Free rolling -> 0.
    CHECK_THAT(SlipRatio::longitudinal(0.6, free_rolling_omega(0.6), kTireRadius),
               WithinAbs(0.0, 1e-12));
    // Wheel spin (wheel twice as fast as the car) -> -1 after the clamp.
    CHECK_THAT(SlipRatio::longitudinal(0.6, 2.0 * free_rolling_omega(0.6),
                                       kTireRadius),
               WithinAbs(-1.0, 1e-12));
}

TEST_CASE("SlipRatio: braking-convention value at normal speed", "[SlipRatio]") {
    // 20 m/s, wheels turning at 80% of free rolling -> slip = +0.2.
    CHECK_THAT(SlipRatio::longitudinal(20.0, 0.8 * free_rolling_omega(20.0),
                                       kTireRadius),
               WithinRel(0.2, 1e-9));
    // Reverse travel uses magnitudes: same result.
    CHECK_THAT(SlipRatio::longitudinal(-20.0, -0.8 * free_rolling_omega(20.0),
                                       kTireRadius),
               WithinRel(0.2, 1e-9));
}

TEST_CASE("SlipRatio: clamp bounds hold under extreme wheel spin", "[SlipRatio]") {
    CHECK(SlipRatio::longitudinal(1.0, 1000.0, kTireRadius) == -1.0);
}
