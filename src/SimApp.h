#pragma once

#include "CameraManager.h"
#include "Config.h"
#include "ExternalSimConnector.h"
#include "HornAudio.h"
#include "KeyboardInputController.h"
#include "PhysicalWorld.h"
#include "ScriptedDriver.h"
#include "Telemetry.h"
#include "VehicleLights.h"
#include "VehiclePanels.h"
#include "VehicleWorld.h"
#include "WiperRenderer.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include <memory>

class SimApp {
public:
    explicit SimApp(const Config& config);
    ~SimApp();

    // Exit codes returned from Run().  Chosen to be CI-friendly:
    //   0   — successful completion (scripted Done, user-closed window, or
    //          max_time with no scripted scenario active)
    //   2   — max_time_s expired while a scripted scenario was still running
    //   130 — SIGINT (conventional 128+SIGINT_number)
    //   64  — usage/config error (EX_USAGE)
    static constexpr int kExitSuccess     = 0;
    static constexpr int kExitTimeout     = 2;
    static constexpr int kExitInterrupted = 130;
    static constexpr int kExitUsage       = 64;

    // Blocking.  In interactive mode, runs until the user closes the window,
    // presses Esc, or simulation.max_time_s elapses.  In headless mode, runs
    // until max_time_s elapses, the scripted scenario finishes, or SIGINT is
    // received.  Returns one of the kExit* codes.
    int Run();

private:
    void SetupVisualization();
    int  RunWithVisualization();
    int  RunHeadless();

    /// Apply BTCM ABS per-wheel front brake modulation for the current tick.
    /// Reads the current AbsPhaseFront from m_external_sim, computes modulated
    /// per-wheel values, and (if fresh) calls ApplyFrontBrakePerWheel().
    /// Also emits one INFO line per wheel on freshness transitions.
    /// @param time       current sim time (seconds), forwarded to Chrono brake API
    /// @param local_front_brake  the local (driver-commanded) front brake ratio [0..1]
    void ApplyAbsFrontBrake(double time, double local_front_brake);

    Config m_config;

    std::unique_ptr<VehicleWorld>           m_world;
    std::unique_ptr<KeyboardInputController> m_keyboard;
    std::unique_ptr<CameraManager>          m_camera;
    std::unique_ptr<Telemetry>              m_telemetry;
    std::unique_ptr<HornAudio>              m_horn;
    std::unique_ptr<VehicleLights>          m_lights;
    std::unique_ptr<VehiclePanels>          m_panels;
    std::unique_ptr<ev1sim::PhysicalWorld>  m_physical;
    std::unique_ptr<ExternalSimConnector>   m_external_sim;
    std::unique_ptr<ScriptedDriver>         m_scripted;
    std::unique_ptr<WiperRenderer>          m_wiper;

    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> m_vis;
    chrono::ChRealtimeStepTimer m_realtime_timer;
    bool m_paused = false;

    // Propulsion enable gate — false at startup so Chrono doesn't behave like
    // the vehicle is already running before the ECU pipeline is ready.
    // Brakes are clamped to 1.0 and throttle is forced to 0 while false.
    // Driven by RSA run-mode broadcast (kSigRunModeBroadcast, ID 5711).
    bool m_propulsion_enabled = false;

    // Per-wheel front brake modulation state (BTCM ABS integration).
    // Holds the previous modulated brake torque ratio for each front wheel;
    // used to implement HOLD phase (freeze-previous) semantics.
    double m_abs_fl_prev = 0.0;  ///< previous FL modulated brake (0..1)
    double m_abs_fr_prev = 0.0;  ///< previous FR modulated brake (0..1)

    // Freshness-fallback logging state — emit one INFO line on transition.
    bool m_abs_fl_was_fresh = false;  ///< was FL fresh on the previous tick?
    bool m_abs_fr_was_fresh = false;  ///< was FR fresh on the previous tick?

    // Freshness window for BTCM solenoid signals.
    static constexpr std::chrono::milliseconds kAbsFreshnessWindow{200};

    // Help overlay — show keyboard controls in a translucent box.
    // Auto-show for the first 5 seconds, then toggle with '?'.
    bool   m_show_help    = true;   // shown at startup
    double m_help_hide_time = 5.0; // sim_time_s to auto-hide

    // Demo pattern for the bulbs — "off", "blink", or "chase".  Kept in the
    // codebase for diagnostics; defaults off now that the electrical sim
    // drives the lamps.
    std::string m_lights_demo = "off";

    // Physical-world snapshot overlay — toggled by Z key.
    bool m_show_snapshot = false;

    // Running count of IPC trip-reset button presses (HUD display only; not on bus).
    int m_trip_reset_count = 0;
};
