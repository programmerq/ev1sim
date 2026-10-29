// AbsFeedback and ControllerRumbleLevels: when the ABS feedback channel is live,
// its pulse, and the motor levels a rumble controller plays.

#include <catch2/catch_test_macros.hpp>

#include "FeedbackChannel.h"

using ev1sim::AbsFeedback;
using ev1sim::AbsFeedbackConfig;
using ev1sim::ControllerRumbleConfig;
using ev1sim::ControllerRumbleLevels;

TEST_CASE("AbsFeedback: quiet until the ABS modulates", "[feedback][abs]") {
    AbsFeedback f;
    CHECK_FALSE(f.Active());
    CHECK_FALSE(f.PulseOn());
    f.Update(0.01, false);
    CHECK_FALSE(f.Active());
    f.Update(0.01, true);
    CHECK(f.Active());
}

TEST_CASE("AbsFeedback: holds across re-apply legs, then stops after the hold time", "[feedback][abs]") {
    AbsFeedbackConfig cfg;
    cfg.hold_ms = 100;
    AbsFeedback f(cfg);
    f.Update(0.01, true);
    for (int i = 0; i < 9; ++i) f.Update(0.01, false);   // 90 ms of APPLY
    CHECK(f.Active());
    f.Update(0.01, true);                                 // next HOLD restarts the hold
    for (int i = 0; i < 9; ++i) f.Update(0.01, false);
    CHECK(f.Active());
    for (int i = 0; i < 3; ++i) f.Update(0.01, false);   // past 100 ms
    CHECK_FALSE(f.Active());
    CHECK_FALSE(f.PulseOn());
}

TEST_CASE("AbsFeedback: pulse is on for the first half of each period, from the event start", "[feedback][abs]") {
    AbsFeedbackConfig cfg;
    cfg.pulse_period_ms = 40;
    cfg.hold_ms         = 1000;
    AbsFeedback f(cfg);
    f.Update(0.005, true);    // 5 ms in
    CHECK(f.PulseOn());
    f.Update(0.020, true);    // 25 ms: second half
    CHECK_FALSE(f.PulseOn());
    f.Update(0.020, false);   // 45 ms: next period, first half
    CHECK(f.PulseOn());
}

TEST_CASE("AbsFeedback: disabled by config", "[feedback][abs]") {
    AbsFeedbackConfig off;
    off.enabled = false;
    AbsFeedback f(off);
    f.Update(0.01, true);
    CHECK_FALSE(f.Active());
    CHECK_FALSE(f.PulseOn());
}

TEST_CASE("ControllerRumbleLevels: drives the chosen motor at the configured strength", "[feedback][rumble]") {
    ControllerRumbleConfig cfg;
    cfg.abs_magnitude = 1.0;
    using M = ControllerRumbleConfig::Motor;

    cfg.abs_motor = M::Low;
    auto lv = ControllerRumbleLevels(cfg, true);
    CHECK(lv.low == 65535);
    CHECK(lv.high == 0);

    cfg.abs_motor = M::High;
    lv = ControllerRumbleLevels(cfg, true);
    CHECK(lv.low == 0);
    CHECK(lv.high == 65535);

    cfg.abs_motor = M::Both;
    lv = ControllerRumbleLevels(cfg, true);
    CHECK(lv.low == 65535);
    CHECK(lv.high == 65535);

    lv = ControllerRumbleLevels(cfg, false);
    CHECK(lv.low == 0);
    CHECK(lv.high == 0);
}

TEST_CASE("ControllerRumbleLevels: strength clamped to [0,1]", "[feedback][rumble]") {
    ControllerRumbleConfig cfg;
    cfg.abs_magnitude = 3.0;
    CHECK(ControllerRumbleLevels(cfg, true).low == 65535);
    cfg.abs_magnitude = -1.0;
    CHECK(ControllerRumbleLevels(cfg, true).low == 0);
    cfg.abs_magnitude = 0.5;
    CHECK(ControllerRumbleLevels(cfg, true).low == 32768);
}
