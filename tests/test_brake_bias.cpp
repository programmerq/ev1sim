// Verifies the EV1 brake-bias contract from the shipped JSON (Round 4).
//
// These tests don't boot Chrono — they read the same brake/vehicle JSON that
// Chrono::Vehicle loads, so the front/rear split, the preserved total, and the
// axle wiring are guarded against regressions.  The rear value also pins the
// SimApp::kRearBrakeMaxTorqueNm coupling used by the rear-EMB torque→ratio math.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <fstream>
#include <string>
#include <nlohmann/json.hpp>

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

TEST_CASE("Brake bias: front carries ~70% and the 4-wheel total is preserved", "[Brake][Bias]") {
    const double front = ReadJson(kFront).at("Maximum Torque").get<double>();
    const double rear  = ReadJson(kRear).at("Maximum Torque").get<double>();

    CHECK(front > rear);                              // front-biased

    const double total = 2.0 * front + 2.0 * rear;   // 4 wheels
    CHECK_THAT(total, WithinAbs(3200.0, 1.0));        // preserve ~0.88g peak decel

    const double front_frac = (2.0 * front) / total;
    CHECK_THAT(front_frac, WithinAbs(0.70, 0.02));    // ~70/30 split
}

TEST_CASE("Brake bias: rear budget covers the EMB drum peak and pins the SimApp constant",
          "[Brake][Bias]") {
    // BrakeDrum at speed: μ0.38 · 4000 N · 0.10 m · (1 + α2.0) = 456 N·m.
    // The rear allocation must clear that so the EMB never clips, and it must
    // equal SimApp::kRearBrakeMaxTorqueNm for the physical-torque→ratio convert.
    const double rear = ReadJson(kRear).at("Maximum Torque").get<double>();
    CHECK(rear >= 456.0);
    CHECK(rear == 480.0);   // keep in sync with SimApp::kRearBrakeMaxTorqueNm
}

TEST_CASE("Brake bias: EV1_Vehicle.json wires front→Front and rear→Rear brakes", "[Brake][Bias]") {
    auto veh = ReadJson(kVeh);
    const auto& axles = veh.at("Axles");
    REQUIRE(axles.size() >= 2);

    for (const char* side : {"Left Brake Input File", "Right Brake Input File"}) {
        CHECK(axles[0].at(side).get<std::string>().find("Front") != std::string::npos);
        CHECK(axles[1].at(side).get<std::string>().find("Rear")  != std::string::npos);
    }

    // The superseded shared brake file must no longer be referenced anywhere.
    const std::string dump = veh.dump();
    CHECK(dump.find("EV1_BrakeSimple.json") == std::string::npos);
}
