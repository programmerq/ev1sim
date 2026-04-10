#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "Config.h"

#include <fstream>
#include <nlohmann/json.hpp>

using Catch::Matchers::WithinAbs;

// Write a temporary JSON file and return its path.
static std::string WriteTempJson(const std::string& content) {
    static int counter = 0;
    std::string path = "/tmp/ev1sim_test_config_" + std::to_string(counter++) + ".json";
    std::ofstream f(path);
    f << content;
    return path;
}

// -----------------------------------------------------------------------
TEST_CASE("Config built-in defaults are sane", "[Config]") {
    Config cfg;
    CHECK(cfg.vehicle_model == "sedan");
    CHECK(cfg.terrain.friction == 0.9);
    CHECK(cfg.simulation.step_size_s == 0.002);
    CHECK(cfg.simulation.realtime == true);
    CHECK(cfg.camera.default_mode == "chase");
    CHECK(cfg.input.steer_rate == 1.8);
    CHECK(cfg.telemetry.show_hud == true);
}

// -----------------------------------------------------------------------
TEST_CASE("Config loads values from JSON file", "[Config]") {
    auto path = WriteTempJson(R"({
        "vehicle_model": "hmmwv",
        "terrain": { "friction": 0.3, "surface": "low_mu" },
        "simulation": { "step_size_s": 0.001, "render_fps": 30 },
        "camera": { "chase_distance": 8.0 }
    })");

    Config cfg = Config::LoadFromFile(path);

    CHECK(cfg.vehicle_model == "hmmwv");
    CHECK_THAT(cfg.terrain.friction, WithinAbs(0.3, 1e-9));
    CHECK(cfg.terrain.surface == "low_mu");
    CHECK_THAT(cfg.simulation.step_size_s, WithinAbs(0.001, 1e-9));
    CHECK(cfg.simulation.render_fps == 30);
    CHECK_THAT(cfg.camera.chase_distance, WithinAbs(8.0, 1e-9));

    // Keys not in JSON should keep defaults.
    CHECK(cfg.simulation.realtime == true);
    CHECK(cfg.input.steer_rate == 1.8);
}

// -----------------------------------------------------------------------
TEST_CASE("Config missing file falls back to defaults", "[Config]") {
    Config cfg = Config::LoadFromFile("/tmp/ev1sim_nonexistent_file.json");
    CHECK(cfg.vehicle_model == "sedan");
    CHECK(cfg.terrain.friction == 0.9);
}

// -----------------------------------------------------------------------
TEST_CASE("Config CLI overrides take precedence", "[Config]") {
    Config cfg;  // starts with defaults

    const char* args[] = {
        "ev1sim",
        "--vehicle", "hmmwv",
        "--surface", "low_mu",
        "--step-size", "0.005",
        "--realtime", "false",
    };
    int argc = sizeof(args) / sizeof(args[0]);
    cfg.ApplyCliOverrides(argc, const_cast<char**>(args));

    CHECK(cfg.vehicle_model == "hmmwv");
    CHECK(cfg.terrain.surface == "low_mu");
    CHECK_THAT(cfg.terrain.friction, WithinAbs(0.3, 1e-9));  // preset applied
    CHECK_THAT(cfg.simulation.step_size_s, WithinAbs(0.005, 1e-9));
    CHECK(cfg.simulation.realtime == false);
}

// -----------------------------------------------------------------------
TEST_CASE("Config CLI --config flag is skipped by ApplyCliOverrides", "[Config]") {
    Config cfg;
    const char* args[] = {"ev1sim", "--config", "some/path.json", "--vehicle", "hmmwv"};
    int argc = sizeof(args) / sizeof(args[0]);
    cfg.ApplyCliOverrides(argc, const_cast<char**>(args));

    // --config should be silently consumed, vehicle override still applies.
    CHECK(cfg.vehicle_model == "hmmwv");
}
