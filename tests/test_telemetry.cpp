// Telemetry — the CSV log's rate gating, header and row, and the HUD's text.

#include <catch2/catch_test_macros.hpp>

#include "Telemetry.h"

#include <irrlicht.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<std::string> ReadLines(const std::filesystem::path& p) {
    std::ifstream f(p);
    std::vector<std::string> lines;
    for (std::string l; std::getline(f, l);) lines.push_back(l);
    return lines;
}

std::vector<std::string> Split(const std::string& row) {
    std::vector<std::string> out;
    std::stringstream ss(row);
    for (std::string c; std::getline(ss, c, ',');) out.push_back(c);
    return out;
}

}  // namespace

TEST_CASE("Telemetry logs one CSV row per interval, header once", "[telemetry]") {
    const auto path = std::filesystem::temp_directory_path() / "ev1sim_telemetry_test.csv";
    {
        Telemetry t(10.0, true, path.string(), false);   // one row per 0.1 s
        VehicleState s;
        s.sim_time = 1.5;
        s.speed_mps = 12.5;
        s.brake_cmd = {0.25, 0.25, 0.5, 0.5};
        s.wheel_omega = {40, 41, 42, 43};
        s.slip_ratio = {0, 0, 0, 1};
        t.Record(s, 0.05);   // half an interval: nothing yet
        t.Record(s, 0.05);   // interval reached: header + row
        t.Record(s, 0.02);
        t.Record(s, 0.09);   // second row
    }
    const auto lines = ReadLines(path);
    std::filesystem::remove(path);
    REQUIRE(lines.size() == 3);
    const auto header = Split(lines[0]);
    const auto row = Split(lines[1]);
    REQUIRE(header.size() == 30);
    REQUIRE(row.size() == header.size());
    auto col = [&](const std::string& name) {
        for (std::size_t i = 0; i < header.size(); ++i)
            if (header[i] == name) return row[i];
        FAIL("no column " << name);
        return std::string();
    };
    CHECK(col("sim_time") == "1.5");
    CHECK(col("speed_mps") == "12.5");
    CHECK(col("brake_rl") == "0.5");
    CHECK(col("wheel_rr_omega") == "43");
    CHECK(col("wheel_mu_fl") == "-1");   // VehicleState's "unavailable"
    CHECK(col("slip_rr") == "1");
}

TEST_CASE("Telemetry writes no file when file logging is off", "[telemetry]") {
    const auto path = std::filesystem::temp_directory_path() / "ev1sim_telemetry_off_test.csv";
    std::filesystem::remove(path);
    Telemetry t(10.0, false, path.string(), false);
    t.Record(VehicleState{}, 1.0);
    CHECK_FALSE(std::filesystem::exists(path));
}

TEST_CASE("Telemetry HUD text: speed, pedals, camera, surface, time", "[telemetry]") {
    VehicleState s;
    s.speed_mps = 10.0;
    s.applied_throttle = 0.5;
    s.applied_front_brake = 0.25;
    s.applied_steering = -0.1;
    s.sim_time = 3.04;
    const auto lines = TelemetryHudLines(s, "Chase", "asphalt", nullptr, nullptr);
    REQUIRE(lines.size() == 5);
    CHECK(lines[0] == "Speed: 36.0 km/h  (10.0 m/s)");
    CHECK(lines[1] == "Throttle: 50%   Brake: 25%   Steer: -10%");
    CHECK(lines[2] == "Camera: Chase");
    CHECK(lines[3] == "Surface: asphalt");
    CHECK(lines[4] == "Sim time: 3.0 s");
}

TEST_CASE("Telemetry HUD shows ABS phase and staleness, and per-wheel friction",
          "[telemetry]") {
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;
    ExternalSimConnector::AbsPhaseFront abs;
    abs.fl = Phase::DUMP;
    abs.fl_fresh = true;
    abs.fr = Phase::APPLY;
    abs.fr_fresh = false;
    const double mu[4] = {0.9, 0.08, -1.0, 0.9};
    const auto lines = TelemetryHudLines(VehicleState{}, "Hood", "split", mu, &abs);
    REQUIRE(lines.size() == 7);
    CHECK(lines[2] == "ABS:  FL=DUMP  FR=APPLY(stale)");
    CHECK(lines[5] == "mu:  FL=0.90  FR=0.08  RL=  -    RR=0.90");

    // Neither wheel fresh: the car is on its own brake logic.
    ExternalSimConnector::AbsPhaseFront none;
    CHECK(TelemetryHudLines(VehicleState{}, "Hood", "split", nullptr, &none)[2] ==
          "ABS:  FL=local  FR=local");
}

TEST_CASE("Telemetry HUD draws on a null driver and skips when hidden", "[telemetry]") {
    Telemetry hidden(10.0, false, "", false);
    hidden.DrawHUD(nullptr, VehicleState{}, "Chase", "asphalt", nullptr);   // no-op
    irr::IrrlichtDevice* dev = irr::createDevice(irr::video::EDT_NULL,
                                                 irr::core::dimension2d<irr::u32>(640, 480));
    REQUIRE(dev != nullptr);
    Telemetry shown(10.0, false, "", true);
    dev->getVideoDriver()->beginScene();
    shown.DrawHUD(dev, VehicleState{}, "Chase", "asphalt", nullptr);
    hidden.DrawHUD(dev, VehicleState{}, "Chase", "asphalt", nullptr);
    dev->getVideoDriver()->endScene();
    dev->drop();
}
