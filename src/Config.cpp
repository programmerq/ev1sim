#include "Config.h"

#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ---------------------------------------------------------------------------
// JSON helpers — pull value only if the key exists
// ---------------------------------------------------------------------------
namespace {

template <typename T>
void read_if(const json& j, const char* key, T& out) {
    if (j.contains(key)) j.at(key).get_to(out);
}

}  // namespace

// ---------------------------------------------------------------------------
Config Config::LoadFromFile(const std::string& path) {
    Config cfg;

    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[Config] Cannot open " << path
                  << " — using built-in defaults.\n";
        return cfg;
    }

    json j;
    try {
        j = json::parse(f);
    } catch (const json::parse_error& e) {
        std::cerr << "[Config] JSON parse error in " << path << ": "
                  << e.what() << "\n";
        return cfg;
    }

    read_if(j, "vehicle_model", cfg.vehicle_model);

    if (j.contains("terrain")) {
        auto& t = j["terrain"];
        read_if(t, "type",     cfg.terrain.type);
        read_if(t, "surface",  cfg.terrain.surface);
        read_if(t, "length_m", cfg.terrain.length_m);
        read_if(t, "width_m",  cfg.terrain.width_m);
        read_if(t, "friction", cfg.terrain.friction);
    }

    if (j.contains("simulation")) {
        auto& s = j["simulation"];
        read_if(s, "step_size_s", cfg.simulation.step_size_s);
        read_if(s, "render_fps",  cfg.simulation.render_fps);
        read_if(s, "realtime",    cfg.simulation.realtime);
    }

    if (j.contains("spawn")) {
        auto& sp = j["spawn"];
        read_if(sp, "x",       cfg.spawn.x);
        read_if(sp, "y",       cfg.spawn.y);
        read_if(sp, "z",       cfg.spawn.z);
        read_if(sp, "yaw_deg", cfg.spawn.yaw_deg);
    }

    if (j.contains("camera")) {
        auto& c = j["camera"];
        read_if(c, "default_mode",   cfg.camera.default_mode);
        read_if(c, "chase_distance", cfg.camera.chase_distance);
        read_if(c, "chase_height",   cfg.camera.chase_height);
    }

    if (j.contains("input")) {
        auto& i = j["input"];
        read_if(i, "steer_rate",         cfg.input.steer_rate);
        read_if(i, "steer_return_rate",  cfg.input.steer_return_rate);
        read_if(i, "throttle_rise_rate", cfg.input.throttle_rise_rate);
        read_if(i, "brake_rise_rate",    cfg.input.brake_rise_rate);
    }

    if (j.contains("telemetry")) {
        auto& tl = j["telemetry"];
        read_if(tl, "log_rate_hz",  cfg.telemetry.log_rate_hz);
        read_if(tl, "log_to_file",  cfg.telemetry.log_to_file);
        read_if(tl, "log_file",     cfg.telemetry.log_file);
        read_if(tl, "show_hud",     cfg.telemetry.show_hud);
    }

    return cfg;
}

// ---------------------------------------------------------------------------
void Config::ApplyCliOverrides(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        auto next = [&]() -> std::string {
            if (i + 1 < argc) return argv[++i];
            std::cerr << "[Config] Missing value after " << arg << "\n";
            return {};
        };

        if (arg == "--config") {
            // Already handled by caller before this method runs.
            ++i;  // skip value
        } else if (arg == "--vehicle") {
            vehicle_model = next();
        } else if (arg == "--surface") {
            terrain.surface = next();
            // Apply well-known friction presets
            if (terrain.surface == "asphalt")       terrain.friction = 0.9;
            else if (terrain.surface == "concrete")  terrain.friction = 0.8;
            else if (terrain.surface == "low_mu")    terrain.friction = 0.3;
        } else if (arg == "--step-size") {
            auto v = next();
            if (!v.empty()) simulation.step_size_s = std::stod(v);
        } else if (arg == "--realtime") {
            auto v = next();
            simulation.realtime = (v == "true" || v == "1");
        } else if (arg == "--friction") {
            auto v = next();
            if (!v.empty()) terrain.friction = std::stod(v);
        } else if (arg == "--render-fps") {
            auto v = next();
            if (!v.empty()) simulation.render_fps = std::stoi(v);
        } else if (arg == "--hud") {
            auto v = next();
            telemetry.show_hud = (v == "true" || v == "1");
        } else {
            std::cerr << "[Config] Unknown flag: " << arg << "\n";
        }
    }
}
