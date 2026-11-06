// The tire's grip-peak lookup (src/TireGripPeak.h), on a table built here.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "TireGripPeak.h"

#include <stdexcept>

using Catch::Approx;

namespace {
ev1sim::TireGripPeak Table() {
    ev1sim::TireGripPeak g;
    g.friction = {0.2, 0.8, 1.2};
    g.peak_slip = {0.06, 0.12, 0.18};
    return g;
}
}  // namespace

TEST_CASE("TireGripPeak: linear between points, held past the ends", "[tire]") {
    const auto g = Table();
    CHECK(g.slip_at(0.2) == Approx(0.06));
    CHECK(g.slip_at(0.5) == Approx(0.09));
    CHECK(g.slip_at(1.0) == Approx(0.15));
    CHECK(g.slip_at(0.05) == Approx(0.06));
    CHECK(g.slip_at(2.0) == Approx(0.18));
}

TEST_CASE("TireGripPeak: the scale is 1 at the reference and falls slower than friction",
          "[tire]") {
    const auto g = Table();
    CHECK(g.slip_scale(0.8, 0.8) == Approx(1.0));
    // A road at a quarter of the reference friction keeps half the peak slip,
    // where TMeasy's own scaling would keep a quarter.
    CHECK(g.slip_scale(0.2, 0.8) == Approx(0.5));
    CHECK(g.slip_scale(0.2, 0.8) > 0.2 / 0.8);
}

TEST_CASE("TireGripPeak: validate refuses an unusable table", "[tire]") {
    auto g = Table();
    REQUIRE_NOTHROW(g.validate());
    g.peak_slip.pop_back();
    CHECK_THROWS_AS(g.validate(), std::invalid_argument);
    g = Table();
    g.friction[1] = 0.1;
    CHECK_THROWS_AS(g.validate(), std::invalid_argument);
    g = Table();
    g.peak_slip[0] = 0.0;
    CHECK_THROWS_AS(g.validate(), std::invalid_argument);
}
