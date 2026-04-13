#pragma once

#include <string>

struct Config {
    std::string vehicle_model = "ev1";  // "ev1", "sedan", or "hmmwv"

    struct Terrain {
        std::string type    = "rigid_plane";  // "rigid_plane" or "level"
        std::string surface = "asphalt";
        double length_m  = 400.0;
        double width_m   = 400.0;
        double friction  = 0.9;

        // When type == "level", path to a JSON level file that defines
        // mesh patches, surface types, friction, and spawn point.
        std::string level_file;
    } terrain;

    struct Simulation {
        double step_size_s = 0.002;
        int    render_fps  = 60;
        bool   realtime    = true;
    } simulation;

    struct Spawn {
        double x       = 0.0;
        double y       = 0.0;
        double z       = 0.5;
        double yaw_deg = 0.0;
    } spawn;

    struct Camera {
        std::string default_mode = "chase";
        double chase_distance = 6.0;
        double chase_height   = 2.0;
    } camera;

    struct Input {
        double steer_rate        = 1.8;
        double steer_return_rate = 2.5;
        double throttle_rise_rate = 1.5;
        double brake_rise_rate   = 2.0;
    } input;

    struct Telemetry {
        double      log_rate_hz = 10.0;
        bool        log_to_file = false;
        std::string log_file    = "telemetry.csv";
        bool        show_hud    = true;
    } telemetry;

    bool start_paused = false;

    // Load from JSON file.  Missing keys keep their defaults.
    static Config LoadFromFile(const std::string& path);

    // Merge CLI arguments on top of current values.
    // Recognised flags: --config, --vehicle, --surface, --step-size, --realtime
    void ApplyCliOverrides(int argc, char* argv[]);
};
