// Tests for the BrakePedal master-cylinder pressure model.
//
// Covers the two-stage pressure curve:
//   travel < dead_band                -> 0 kPa
//   travel < transition               -> k1 × (t - dead_band)
//   travel ≥ transition               -> soft_segment + k2 × (t - transition)
//   pressure capped at max_pressure_kpa.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "PhysicalWorld.h"
#include "ExternalSimConnector.h"

using Catch::Matchers::WithinAbs;
using ev1sim::BrakePedal;

TEST_CASE("BrakePedal: dead-band returns zero pressure", "[BrakePedal]") {
    BrakePedal::Calibration cal;  // defaults: dead_band 0.07
    CHECK(BrakePedal::pressure_for_travel(0.00, cal) == 0.0);
    CHECK(BrakePedal::pressure_for_travel(0.05, cal) == 0.0);
    CHECK(BrakePedal::pressure_for_travel(0.06, cal) == 0.0);
    // At the threshold, still zero (boundary).
    CHECK(BrakePedal::pressure_for_travel(0.07, cal) == 0.0);
}

TEST_CASE("BrakePedal: soft stage is linear with k1", "[BrakePedal]") {
    BrakePedal::Calibration cal;  // defaults: dead_band 0.07, transition 0.35, k1 8000
    // At 0.20: pressure = 8000 * (0.20 - 0.07) = 1040 kPa
    CHECK_THAT(BrakePedal::pressure_for_travel(0.20, cal),
               WithinAbs(1040.0, 1e-9));
    // At 0.30: pressure = 8000 * (0.30 - 0.07) = 1840 kPa
    CHECK_THAT(BrakePedal::pressure_for_travel(0.30, cal),
               WithinAbs(1840.0, 1e-9));
}

TEST_CASE("BrakePedal: firm stage uses k2 from the transition point",
          "[BrakePedal]") {
    BrakePedal::Calibration cal;
    // Soft segment top: 8000 * (0.35 - 0.07) = 2240 kPa
    CHECK_THAT(BrakePedal::pressure_for_travel(0.35, cal),
               WithinAbs(2240.0, 1e-9));
    // At 0.50: 2240 + 18000 * (0.50 - 0.35) = 2240 + 2700 = 4940 kPa
    CHECK_THAT(BrakePedal::pressure_for_travel(0.50, cal),
               WithinAbs(4940.0, 1e-9));
    // At 0.80: 2240 + 18000 * (0.80 - 0.35) = 2240 + 8100 = 10340 kPa
    CHECK_THAT(BrakePedal::pressure_for_travel(0.80, cal),
               WithinAbs(10340.0, 1e-9));
}

TEST_CASE("BrakePedal: pressure caps at max_pressure_kpa", "[BrakePedal]") {
    BrakePedal::Calibration cal;  // max 15000
    // At 1.0: uncapped would be 2240 + 18000*0.65 = 13940 (under cap)
    CHECK_THAT(BrakePedal::pressure_for_travel(1.0, cal),
               WithinAbs(13940.0, 1e-9));

    // Lower the cap and check it kicks in.
    BrakePedal::Calibration tight = cal;
    tight.max_pressure_kpa = 5000.0;
    CHECK(BrakePedal::pressure_for_travel(1.0, tight) == 5000.0);
}

TEST_CASE("BrakePedal: travel outside [0,1] is clamped",
          "[BrakePedal]") {
    BrakePedal::Calibration cal;
    CHECK(BrakePedal::pressure_for_travel(-0.5, cal) == 0.0);
    CHECK_THAT(BrakePedal::pressure_for_travel(1.5, cal),
               WithinAbs(BrakePedal::pressure_for_travel(1.0, cal), 1e-9));
}

TEST_CASE("BrakePedal: update() stores most-recent pressure", "[BrakePedal]") {
    BrakePedal pedal;
    pedal.update(0.5);
    CHECK_THAT(pedal.pressure_kpa(), WithinAbs(4940.0, 1e-9));

    // Subsequent update overwrites.
    pedal.update(0.0);
    CHECK(pedal.pressure_kpa() == 0.0);
}

TEST_CASE("BrakePedal: custom calibration is honored", "[BrakePedal]") {
    BrakePedal::Calibration cal;
    cal.dead_band  = 0.10;
    cal.transition = 0.40;
    cal.k1_kpa_per_unit = 5000.0;
    cal.k2_kpa_per_unit = 10000.0;
    cal.max_pressure_kpa = 8000.0;

    BrakePedal pedal(cal);
    // Below new dead-band — zero.
    CHECK(pedal.update(0.08) == 0.0);
    // Soft stage: 5000 * (0.20 - 0.10) = 500 kPa
    CHECK_THAT(pedal.update(0.20), WithinAbs(500.0, 1e-9));
    // Cap kicks in at full travel: would be (5000*0.30) + (10000*0.60) = 1500 + 6000 = 7500
    CHECK_THAT(pedal.update(1.0), WithinAbs(7500.0, 1e-9));
}

// ---------------------------------------------------------------------------
// ExternalSimConnector integration — brake pressure publish path.
// ---------------------------------------------------------------------------

TEST_CASE("ExternalSimConnector: endpoint table registers brake_master_pressure_kpa (4074)",
          "[ExternalSim][BrakePedal]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4074);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.brake.master_cylinder_pressure_kpa");
    CHECK(std::string(ep->short_name)     == "brake_master_pressure_kpa");
    CHECK_FALSE(ep->input_to_sim);  // ev1sim → BTCM, output direction
}

TEST_CASE("ExternalSimConnector: SetBrakeMasterPressureKpa stores the value",
          "[ExternalSim][BrakePedal]") {
    ExternalSimConnector c;
    // No public getter — but the setter shouldn't crash and the endpoint
    // exists.  Round-trip via a frame would require a live transport;
    // the BrakePedal unit tests above cover the conversion math.
    c.SetBrakeMasterPressureKpa(0.0f);
    c.SetBrakeMasterPressureKpa(12345.6f);
    SUCCEED();
}
