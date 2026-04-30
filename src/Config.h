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

        // Headless run: skip Irrlicht window and rendering.  Physics, telemetry,
        // and the external-sim connector still run normally.  Intended for
        // CI-style scenario runs — see max_time_s for automatic termination.
        bool   headless    = false;

        // Exit after this many seconds of simulated time (0 = no limit).
        // Applies in both headless and interactive modes; in interactive mode
        // closing the window or pressing Esc still works.
        double max_time_s  = 0.0;
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

    // Ambient lighting preset for the Irrlicht scene.  Headless runs
    // ignore this block.  Presets set ambient RGB and sun elevation; the
    // explicit override fields (when not negative / empty) win over the
    // preset.  ambient_temp_c is a stub for future brake/tire thermal
    // hooks — no consumer reads it yet.
    struct Environment {
        std::string time_of_day      = "day";   // "day" | "dusk" | "night"
        double      ambient_r        = 0.8;
        double      ambient_g        = 0.8;
        double      ambient_b        = 0.8;
        double      sun_elevation_deg = 60.0;
        double      ambient_temp_c   = 20.0;    // stub — not yet consumed
    } environment;

    struct Lights {
        // Built-in demo pattern, for diagnostics independent of the
        // electrical sim:
        //   "off"   — all bulbs off (default; electric sim drives them)
        //   "blink" — every bulb blinks at a unique frequency
        //   "chase" — one bulb at a time, walking around the vehicle
        std::string demo_mode = "off";
    } lights;

    struct ExternalSim {
        bool        enabled            = false;
        std::string bus_name           = "electricsim_chassis_bus";
        double      reconnect_period_s = 1.0;
    } external_sim;

    // Vehicle dynamics authority — selects whether Chrono's powertrain is
    // driven from the local pedal (default; legacy behavior) or from the
    // electronics-side commanded throttle on the chassis bus.
    //
    //   "local"        — direct keyboard / scripted-driver pedal input.
    //   "electronics"  — subscribe to kSigChassisThrottleCmdQ8 (4073) from PIM
    //                    and override the local pedal when the bus value is
    //                    fresh (within throttle_freshness_window_ms).
    //
    // The freshness fallback is critical: when ev1sim runs in "electronics"
    // mode but PIM is not running (e.g. no controller spawned, or it crashes
    // mid-run), the local pedal still drives the car so the user can recover.
    struct VehicleDynamics {
        std::string driver = "local";
        double      throttle_freshness_window_ms = 200.0;
    } vehicle_dynamics;

    // Built-in accel-hold-brake scripted scenario.  Only active when
    // enabled == true (usually paired with --headless for CI use).
    struct Scripted {
        bool   enabled            = false;
        double target_speed_kph   = 40.0;
        double hold_time_s        = 1.0;
        double stop_threshold_mps = 0.1;
    } scripted;

    // Data-driven scenario harness — load timed events + stats capture
    // from a JSON file.  When `path` is non-empty SimApp loads the file at
    // construction, applies driver_mode + max_time_s overrides, and runs
    // the scenario in place of (or alongside) the built-in scripted driver.
    // Mutually exclusive with `scripted.enabled` — if both are set the
    // scenario file wins and a warning is logged.
    struct Scenario {
        std::string path;
    } scenario;

    bool start_paused = false;

    // Load from JSON file.  Missing keys keep their defaults.
    static Config LoadFromFile(const std::string& path);

    // Merge CLI arguments on top of current values.
    // Recognised flags: --config, --vehicle, --surface, --step-size, --realtime,
    // --headless, --max-time, ...
    void ApplyCliOverrides(int argc, char* argv[]);
};
