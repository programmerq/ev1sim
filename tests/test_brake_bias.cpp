// Verifies the EV1 brake-bias contract from the shipped JSON (Round 4).
//
// These tests don't boot Chrono — they read the same brake/vehicle JSON that
// Chrono::Vehicle loads, so the front/rear split, the preserved total, and the
// axle wiring are guarded against regressions.  The rear value is checked
// against the code constant SimApp::kRearBrakeMaxTorqueNm (defined as
// ev1sim::kRearBrakeMaxTorqueNm in RearEmbClamp.h) used by the rear-EMB
// torque→ratio math — never against a re-typed literal.  The torque values
// themselves are not pinned (owner ruling 2026-10-08, ev1-canon:ci-checks):
// they are engineering choices a better source may replace.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fstream>
#include <string>
#include <nlohmann/json.hpp>

#include "BrakeDrum.h"
#include "RearEmbClamp.h"

using json = nlohmann::json;
using Catch::Matchers::WithinAbs;

#ifndef EV1SIM_SOURCE_DIR
#define EV1SIM_SOURCE_DIR "."
#endif

static json ReadJson(const std::string& relative_path) {
    std::string path = std::string(EV1SIM_SOURCE_DIR) + "/" + relative_path;
    std::ifstream f(path);
    REQUIRE(f.is_open());
    return json::parse(f);
}

static constexpr const char* kFront = "data/vehicle/ev1/brake/EV1_BrakeSimple_Front.json";
static constexpr const char* kRear  = "data/vehicle/ev1/brake/EV1_BrakeSimple_Rear.json";
static constexpr const char* kVeh   = "data/vehicle/ev1/vehicle/EV1_Vehicle.json";

TEST_CASE("Brake bias: front/rear JSONs use the BrakeSimple brake template", "[Brake][Bias]") {
    auto front = ReadJson(kFront);
    auto rear  = ReadJson(kRear);
    CHECK(front.at("Type")     == "Brake");
    CHECK(rear.at("Type")      == "Brake");
    CHECK(front.at("Template") == "BrakeSimple");
    CHECK(rear.at("Template")  == "BrakeSimple");
}

TEST_CASE("Brake bias: both axles brake and the split is front-biased", "[Brake][Bias]") {
    const double front = ReadJson(kFront).at("Maximum Torque").get<double>();
    const double rear  = ReadJson(kRear).at("Maximum Torque").get<double>();

    CHECK(rear > 0.0);
    CHECK(front > rear);                              // front-biased
}

TEST_CASE("Brake bias: rear budget covers the EMB drum peak and matches the SimApp constant",
          "[Brake][Bias]") {
    // The rear allocation must clear the BrakeDrum model's own peak torque at
    // speed (full shoe force, well above the smoothing threshold) so the EMB
    // never clips, and it must equal the code constant SimApp uses for the
    // physical-torque→ratio convert.
    const double rear = ReadJson(kRear).at("Maximum Torque").get<double>();
    const ev1sim::BrakeDrum::Params drum;
    const double drum_peak_nm = ev1sim::BrakeDrum::torque_magnitude_nm(
        drum.max_shoe_force_n, 100.0 * drum.omega_threshold_rad_s, drum);
    CHECK(rear >= drum_peak_nm);
    CHECK_THAT(rear, WithinAbs(ev1sim::kRearBrakeMaxTorqueNm, 1e-9));
}

TEST_CASE("Brake bias: EV1_Vehicle.json wires front→Front and rear→Rear brakes", "[Brake][Bias]") {
    auto veh = ReadJson(kVeh);
    const auto& axles = veh.at("Axles");
    REQUIRE(axles.size() >= 2);

    for (const char* side : {"Left Brake Input File", "Right Brake Input File"}) {
        CHECK(axles[0].at(side).get<std::string>().find("Front") != std::string::npos);
        CHECK(axles[1].at(side).get<std::string>().find("Rear")  != std::string::npos);
    }

    // The superseded shared brake file must no longer be referenced by the
    // vehicle definition.  (This guards EV1_Vehicle.json specifically — a stray
    // mention elsewhere, e.g. a historical note in docs, isn't caught here.)
    const std::string dump = veh.dump();
    CHECK(dump.find("EV1_BrakeSimple.json") == std::string::npos);
}
