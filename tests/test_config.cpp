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
    CHECK(cfg.vehicle_model == "ev1");
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
    CHECK(cfg.vehicle_model == "ev1");
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
TEST_CASE("Config loads external_sim and lights sections", "[Config]") {
    auto path = WriteTempJson(R"({
        "lights":       { "demo_mode": "chase" },
        "external_sim": { "enabled": true, "bus_name": "custom_bus",
                          "reconnect_period_s": 2.5 }
    })");

    Config cfg = Config::LoadFromFile(path);
    CHECK(cfg.lights.demo_mode == "chase");
    CHECK(cfg.external_sim.enabled == true);
    CHECK(cfg.external_sim.bus_name == "custom_bus");
    CHECK_THAT(cfg.external_sim.reconnect_period_s, WithinAbs(2.5, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Config accepts legacy boolean demo_mode", "[Config]") {
    // Earlier versions stored demo_mode as a bool; preserve compatibility.
    auto path_true  = WriteTempJson(R"({ "lights": { "demo_mode": true  } })");
    auto path_false = WriteTempJson(R"({ "lights": { "demo_mode": false } })");

    CHECK(Config::LoadFromFile(path_true ).lights.demo_mode == "blink");
    CHECK(Config::LoadFromFile(path_false).lights.demo_mode == "off");
}

// -----------------------------------------------------------------------
TEST_CASE("Config external_sim defaults off with stock bus name", "[Config]") {
    Config cfg;
    CHECK(cfg.external_sim.enabled == false);
    CHECK(cfg.external_sim.bus_name == "electricsim_harness_bus");
    CHECK(cfg.lights.demo_mode == "off");
}

// -----------------------------------------------------------------------
TEST_CASE("Config --external-sim and --lights-demo CLI flags", "[Config]") {
    Config cfg;
    const char* args[] = {
        "ev1sim",
        "--external-sim", "true",
        "--external-sim-bus", "alt_bus",
        "--lights-demo", "chase",
    };
    int argc = sizeof(args) / sizeof(args[0]);
    cfg.ApplyCliOverrides(argc, const_cast<char**>(args));

    CHECK(cfg.external_sim.enabled == true);
    CHECK(cfg.external_sim.bus_name == "alt_bus");
    CHECK(cfg.lights.demo_mode == "chase");

    // "false"/"0" disables external sim; bool-style synonyms map to "blink"/"off".
    Config cfg2;
    cfg2.external_sim.enabled = true;
    const char* off_args[] = {"ev1sim", "--external-sim", "false",
                              "--lights-demo", "on"};
    cfg2.ApplyCliOverrides(5, const_cast<char**>(off_args));
    CHECK(cfg2.external_sim.enabled == false);
    CHECK(cfg2.lights.demo_mode == "blink");
}

// -----------------------------------------------------------------------
TEST_CASE("Config headless + max-time defaults and JSON", "[Config][headless]") {
    Config cfg;
    CHECK(cfg.simulation.headless == false);
    CHECK(cfg.simulation.max_time_s == 0.0);

    auto path = WriteTempJson(R"({
        "simulation": { "headless": true, "max_time_s": 12.5 }
    })");
    Config loaded = Config::LoadFromFile(path);
    CHECK(loaded.simulation.headless == true);
    CHECK_THAT(loaded.simulation.max_time_s, WithinAbs(12.5, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Config --headless and --max-time CLI flags", "[Config][headless]") {
    Config cfg;
    const char* args[] = {"ev1sim", "--headless", "--max-time", "7.5"};
    cfg.ApplyCliOverrides(4, const_cast<char**>(args));

    CHECK(cfg.simulation.headless == true);
    CHECK_THAT(cfg.simulation.max_time_s, WithinAbs(7.5, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Config scripted block defaults and JSON", "[Config][scripted]") {
    Config cfg;
    CHECK(cfg.scripted.enabled == false);
    CHECK_THAT(cfg.scripted.target_speed_kph, WithinAbs(40.0, 1e-9));
    CHECK_THAT(cfg.scripted.hold_time_s,      WithinAbs(1.0,  1e-9));
    CHECK_THAT(cfg.scripted.stop_threshold_mps, WithinAbs(0.1, 1e-9));

    auto path = WriteTempJson(R"({
        "scripted": {
            "enabled": true,
            "target_speed_kph": 55.0,
            "hold_time_s": 2.5,
            "stop_threshold_mps": 0.25
        }
    })");
    Config loaded = Config::LoadFromFile(path);
    CHECK(loaded.scripted.enabled == true);
    CHECK_THAT(loaded.scripted.target_speed_kph,   WithinAbs(55.0,  1e-9));
    CHECK_THAT(loaded.scripted.hold_time_s,        WithinAbs(2.5,   1e-9));
    CHECK_THAT(loaded.scripted.stop_threshold_mps, WithinAbs(0.25,  1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Config scripted CLI flags (--target-kph implies enabled)",
          "[Config][scripted]") {
    // --scripted-accel-brake alone enables with defaults.
    {
        Config cfg;
        const char* args[] = {"ev1sim", "--scripted-accel-brake"};
        cfg.ApplyCliOverrides(2, const_cast<char**>(args));
        CHECK(cfg.scripted.enabled == true);
        CHECK_THAT(cfg.scripted.target_speed_kph, WithinAbs(40.0, 1e-9));
    }

    // --target-kph implies enabled and sets target.
    {
        Config cfg;
        const char* args[] = {"ev1sim", "--target-kph", "25", "--hold-time", "0.5"};
        cfg.ApplyCliOverrides(5, const_cast<char**>(args));
        CHECK(cfg.scripted.enabled == true);
        CHECK_THAT(cfg.scripted.target_speed_kph, WithinAbs(25.0, 1e-9));
        CHECK_THAT(cfg.scripted.hold_time_s,      WithinAbs(0.5,  1e-9));
    }
}

// -----------------------------------------------------------------------
TEST_CASE("Config environment defaults to day preset values", "[Config][Env]") {
    Config cfg;
    CHECK(cfg.environment.time_of_day == "day");
    CHECK_THAT(cfg.environment.ambient_r, WithinAbs(0.8, 1e-9));
    CHECK_THAT(cfg.environment.ambient_g, WithinAbs(0.8, 1e-9));
    CHECK_THAT(cfg.environment.ambient_b, WithinAbs(0.8, 1e-9));
    CHECK_THAT(cfg.environment.sun_elevation_deg, WithinAbs(60.0, 1e-9));
    CHECK_THAT(cfg.environment.ambient_temp_c,    WithinAbs(20.0, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Config environment preset resolves ambient + sun", "[Config][Env]") {
    auto path = WriteTempJson(R"({
        "environment": { "time_of_day": "night" }
    })");
    Config cfg = Config::LoadFromFile(path);
    CHECK(cfg.environment.time_of_day == "night");
    CHECK(cfg.environment.ambient_r < 0.3);
    CHECK(cfg.environment.sun_elevation_deg < 0.0);
}

// -----------------------------------------------------------------------
TEST_CASE("Config environment explicit overrides beat the preset", "[Config][Env]") {
    auto path = WriteTempJson(R"({
        "environment": {
            "time_of_day": "day",
            "ambient": [0.1, 0.2, 0.3],
            "sun_elevation_deg": 12.5,
            "ambient_temp_c": -5.0
        }
    })");
    Config cfg = Config::LoadFromFile(path);
    CHECK_THAT(cfg.environment.ambient_r, WithinAbs(0.1, 1e-9));
    CHECK_THAT(cfg.environment.ambient_g, WithinAbs(0.2, 1e-9));
    CHECK_THAT(cfg.environment.ambient_b, WithinAbs(0.3, 1e-9));
    CHECK_THAT(cfg.environment.sun_elevation_deg, WithinAbs(12.5, 1e-9));
    CHECK_THAT(cfg.environment.ambient_temp_c,    WithinAbs(-5.0, 1e-9));
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
