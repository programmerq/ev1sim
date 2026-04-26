#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "BrakeActuator.h"
#include "DriverCommand.h"

using Catch::Matchers::WithinAbs;

TEST_CASE("DriverCommand default-initialises to safe values", "[DriverCommand]") {
    DriverCommand cmd;

    CHECK(cmd.throttle    == 0.0);
    CHECK(cmd.front_brake == 0.0);
    CHECK(cmd.rear_brake  == 0.0);
    CHECK(cmd.steering    == 0.0);
    CHECK(cmd.parking_brake  == false);
    CHECK(cmd.reset_vehicle  == false);
    CHECK(cmd.horn_low   == false);
    CHECK(cmd.horn_high  == false);
}

TEST_CASE("DriverCommand fields are independently assignable", "[DriverCommand]") {
    DriverCommand cmd;
    cmd.throttle    = 0.75;
    cmd.front_brake = 0.5;
    cmd.rear_brake  = 0.3;
    cmd.steering    = -0.4;
    cmd.parking_brake = true;

    CHECK(cmd.throttle    == 0.75);
    CHECK(cmd.front_brake == 0.5);
    CHECK(cmd.rear_brake  == 0.3);
    CHECK(cmd.steering    == -0.4);
    CHECK(cmd.parking_brake == true);
    CHECK(cmd.reset_vehicle == false);  // untouched
}

// ---------------------------------------------------------------------------
// BrakeActuator dynamics
// ---------------------------------------------------------------------------

TEST_CASE("BrakeActuator starts at zero", "[BrakeActuator]") {
    BrakeActuator a;
    CHECK(a.front_pressure == 0.0);
    CHECK(a.rear_position  == 0.0);
}

TEST_CASE("Front brake rises faster than it falls", "[BrakeActuator]") {
    // Apply full front brake for 150 ms and measure pressure; then release
    // for the same duration and compare.
    constexpr double step = 0.002;   // 2 ms physics step
    constexpr int    N    = 75;      // 150 ms worth of steps

    BrakeActuator apply;
    for (int i = 0; i < N; ++i)
        apply.Advance(step, 1.0, 0.0);
    const double p_apply = apply.front_pressure;

    BrakeActuator release;
    release.front_pressure = 1.0;
    for (int i = 0; i < N; ++i)
        release.Advance(step, 0.0, 0.0);
    const double p_release = release.front_pressure;

    // After 150 ms from zero with τ=50ms: p ≈ 1 − e^(−3) ≈ 0.95
    // After 150 ms from one  with τ=80ms: p ≈ e^(−150/80) ≈ 0.15
    CHECK(p_apply   > 0.90);   // well above 90 % — rises quickly
    CHECK(p_release < 0.20);   // well below 20 % — falls more slowly
    CHECK(p_apply   > p_release);
}

TEST_CASE("Front brake pressure is clamped to [0,1]", "[BrakeActuator]") {
    BrakeActuator a;
    // Advance many steps with full command — must not exceed 1.
    for (int i = 0; i < 1000; ++i)
        a.Advance(0.002, 1.0, 0.0);
    CHECK_THAT(a.front_pressure, WithinAbs(1.0, 1e-6));
}

TEST_CASE("Rear drum actuator is rate-limited", "[BrakeActuator]") {
    // With max rate ≈ 3.333/s and step = 2ms the actuator takes ~300ms
    // to reach full engagement.  After only 100 ms it should be ~1/3 applied.
    constexpr double step = 0.002;
    constexpr int    N50  = 25;    //  50 ms
    constexpr int    N150 = 75;    // 150 ms
    constexpr int    N350 = 175;   // 350 ms — enough to fully engage

    BrakeActuator a;
    for (int i = 0; i < N50;  ++i)  a.Advance(step, 0.0, 1.0);
    const double p50 = a.rear_position;

    for (int i = 0; i < N150 - N50; ++i) a.Advance(step, 0.0, 1.0);
    const double p150 = a.rear_position;

    for (int i = 0; i < N350 - N150; ++i) a.Advance(step, 0.0, 1.0);
    const double p350 = a.rear_position;

    CHECK(p50  < 0.25);            // still engaging at 50 ms
    CHECK(p150 > 0.40);            // meaningfully engaged at 150 ms
    CHECK_THAT(p350, WithinAbs(1.0, 0.01));  // fully engaged by 350 ms
}

TEST_CASE("Rear drum actuator retracts symmetrically", "[BrakeActuator]") {
    constexpr double step = 0.002;

    BrakeActuator a;
    // Fully engage.
    for (int i = 0; i < 200; ++i) a.Advance(step, 0.0, 1.0);
    CHECK_THAT(a.rear_position, WithinAbs(1.0, 0.01));

    // Retract — should take the same time (rate-limited, symmetric).
    for (int i = 0; i < 200; ++i) a.Advance(step, 0.0, 0.0);
    CHECK_THAT(a.rear_position, WithinAbs(0.0, 0.01));
}

TEST_CASE("Front and rear brake channels are independent", "[BrakeActuator]") {
    constexpr double step = 0.002;
    BrakeActuator a;

    // Drive only the front — rear must stay zero.
    for (int i = 0; i < 100; ++i) a.Advance(step, 1.0, 0.0);
    CHECK(a.front_pressure > 0.9);
    CHECK(a.rear_position  == 0.0);

    // Drive only the rear — front must fall back.
    BrakeActuator b;
    b.front_pressure = 1.0;
    for (int i = 0; i < 100; ++i) b.Advance(step, 0.0, 1.0);
    CHECK(b.front_pressure < 0.1);   // released
    CHECK(b.rear_position  > 0.5);   // engaging
}
