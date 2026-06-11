// Unit tests for the Chrono-free DC traction-pack current model
// (src/MotorCurrent.h).  SimApp feeds it the engine's shaft torque + speed and
// publishes the result on chassis bus 4072, so pinning the model here makes the
// published current verifiable without booting Chrono.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "MotorCurrent.h"

using Catch::Matchers::WithinAbs;
using ev1sim::MotorCurrent;

TEST_CASE("MotorCurrent: zero shaft power draws only the accessory load", "[MotorCurrent]") {
    // P_mech = 0 → I = accessory_load / pack_voltage = 1200 / 312 ≈ 3.85 A.
    const double idle = MotorCurrent::kAccessoryLoadW / MotorCurrent::kPackVoltageV;
    CHECK_THAT(MotorCurrent::dc_bus_current_a(0.0, 0.0),   WithinAbs(idle, 1e-6));
    CHECK_THAT(MotorCurrent::dc_bus_current_a(0.0, 250.0), WithinAbs(idle, 1e-6));  // coasting
    CHECK_THAT(idle, WithinAbs(3.8462, 1e-3));
}

TEST_CASE("MotorCurrent: motoring discharges (positive), regen charges (negative)",
          "[MotorCurrent]") {
    // Motoring: T=100 N·m, ω=200 rad/s → (20 kW / 0.9 + 1200 W) / 312 V ≈ 75.07 A.
    CHECK_THAT(MotorCurrent::dc_bus_current_a(100.0, 200.0), WithinAbs(75.07, 0.05));
    // Regen: T=−100, ω=200 → (−20 kW · 0.9 + 1200 W) / 312 V ≈ −53.85 A (charging).
    const double regen = MotorCurrent::dc_bus_current_a(-100.0, 200.0);
    CHECK(regen < 0.0);
    CHECK_THAT(regen, WithinAbs(-53.85, 0.05));
}

TEST_CASE("MotorCurrent: current rises monotonically with torque at fixed speed",
          "[MotorCurrent]") {
    const double lo = MotorCurrent::dc_bus_current_a(50.0,  200.0);
    const double hi = MotorCurrent::dc_bus_current_a(150.0, 200.0);
    CHECK(hi > lo);
}

TEST_CASE("MotorCurrent: depends only on mechanical power, not its direction",
          "[MotorCurrent]") {
    // T·ω is the same for (100, 200) and (−100, −200) — both are motoring.
    CHECK_THAT(MotorCurrent::dc_bus_current_a(-100.0, -200.0),
               WithinAbs(MotorCurrent::dc_bus_current_a(100.0, 200.0), 1e-9));
}

TEST_CASE("MotorCurrent: drivetrain losses make motoring draw exceed regen return",
          "[MotorCurrent]") {
    // Isolate the traction term (no accessory load): for equal |P_mech|, the
    // motoring magnitude (÷η) must exceed the regen magnitude (×η).
    MotorCurrent::Params p;
    p.accessory_load_w = 0.0;
    const double drive = MotorCurrent::dc_bus_current_a( 100.0, 200.0, p);
    const double regen = MotorCurrent::dc_bus_current_a(-100.0, 200.0, p);
    CHECK(drive > 0.0);
    CHECK(regen < 0.0);
    CHECK(drive > -regen);   // |discharge| > |charge| for the same shaft power
}

TEST_CASE("MotorCurrent: EV1 Gen 2 pack constants are wired", "[MotorCurrent]") {
    CHECK_THAT(MotorCurrent::kPackVoltageV,   WithinAbs(312.0, 1e-9));
    CHECK_THAT(MotorCurrent::kDriveEfficiency, WithinAbs(0.90, 1e-9));
    CHECK_THAT(MotorCurrent::kRegenEfficiency, WithinAbs(0.90, 1e-9));
    CHECK_THAT(MotorCurrent::kAccessoryLoadW, WithinAbs(1200.0, 1e-9));
}

TEST_CASE("MotorCurrent: custom params override the EV1 defaults", "[MotorCurrent]") {
    MotorCurrent::Params p;
    p.pack_voltage_v = 300.0; p.drive_efficiency = 1.0; p.accessory_load_w = 0.0;
    // 100 N·m × 300 rad/s = 30 kW, ÷1.0, ÷300 V = 100 A.
    CHECK_THAT(MotorCurrent::dc_bus_current_a(100.0, 300.0, p), WithinAbs(100.0, 1e-6));
}

TEST_CASE("MotorCurrent: a non-positive pack voltage is handled defensively",
          "[MotorCurrent]") {
    MotorCurrent::Params p;
    p.pack_voltage_v = 0.0;
    CHECK(MotorCurrent::dc_bus_current_a(100.0, 200.0, p) == 0.0);
}

TEST_CASE("MotorCurrent: a non-positive drive efficiency can't leak inf/NaN",
          "[MotorCurrent]") {
    MotorCurrent::Params p;
    p.drive_efficiency = 0.0;   // misconfigured — would divide by zero unguarded
    const double i = MotorCurrent::dc_bus_current_a(100.0, 200.0, p);
    CHECK(std::isfinite(i));
    // Falls back to a lossless pass-through: (20 kW + 1200 W) / 312 V.
    CHECK_THAT(i, WithinAbs((20000.0 + 1200.0) / 312.0, 0.05));
}
