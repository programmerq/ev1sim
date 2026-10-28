// AbsRumble: when the force-feedback wheel rumbles for ABS.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ForceFeedback.h"

using ev1sim::AbsRumble;
using ev1sim::FfbConfig;
using Catch::Matchers::WithinAbs;

TEST_CASE("AbsRumble: quiet until the ABS modulates, then at the configured amplitude", "[ffb][abs_rumble]") {
    AbsRumble r;
    CHECK_FALSE(r.Active());
    CHECK(r.Magnitude() == 0.0);
    r.Update(0.01, false);
    CHECK_FALSE(r.Active());
    r.Update(0.01, true);
    CHECK(r.Active());
    CHECK_THAT(r.Magnitude(), WithinAbs(FfbConfig{}.abs_rumble_magnitude, 1e-12));
}

TEST_CASE("AbsRumble: holds across re-apply legs, then stops after the hold time", "[ffb][abs_rumble]") {
    FfbConfig cfg;
    cfg.abs_rumble_hold_ms = 100;
    AbsRumble r(cfg);
    r.Update(0.01, true);
    for (int i = 0; i < 9; ++i) r.Update(0.01, false);   // 90 ms of APPLY
    CHECK(r.Active());
    r.Update(0.01, true);                                 // next HOLD restarts the hold
    for (int i = 0; i < 9; ++i) r.Update(0.01, false);
    CHECK(r.Active());
    for (int i = 0; i < 3; ++i) r.Update(0.01, false);   // past 100 ms
    CHECK_FALSE(r.Active());
    CHECK(r.Magnitude() == 0.0);
}

TEST_CASE("AbsRumble: disabled by config, and amplitude clamped to [0,1]", "[ffb][abs_rumble]") {
    FfbConfig off;
    off.abs_rumble = false;
    AbsRumble quiet(off);
    quiet.Update(0.01, true);
    CHECK_FALSE(quiet.Active());
    CHECK(quiet.Magnitude() == 0.0);

    FfbConfig loud;
    loud.abs_rumble_magnitude = 3.0;
    AbsRumble r(loud);
    r.Update(0.01, true);
    CHECK(r.Magnitude() == 1.0);
}
