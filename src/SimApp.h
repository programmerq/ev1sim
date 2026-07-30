#pragma once

#include "CameraManager.h"
#include "Config.h"
#include "DriverCommand.h"
#include "ExternalSimConnector.h"
#include "FloatingUiPanel.h"
#include "HornAudio.h"
#include "SounderAudio.h"
#include "KeyboardInputController.h"
#include "PhysicalWorld.h"
#include "SdlContext.h"
#include "Scenario.h"
#include "ScriptedDriver.h"
#include "Telemetry.h"
#include "VehicleLights.h"
#include "VehiclePanels.h"
#include "VehicleWorld.h"
#include "WiperRenderer.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include <memory>

class SimApp : public ev1sim::ScenarioHooks {
public:
    explicit SimApp(const Config& config);
    ~SimApp();

    // ScenarioHooks (public so the Scenario can invoke them; SimApp owns
    // both objects so the lifetime is fine).
    void KeyOnCycle()        override;
    void HeadlightCycle()    override;
    void PrndUp()            override;
    void PrndDown()          override;
    void TurnSignalLeft()    override;
    void TurnSignalRight()   override;
    void HazardToggle()      override;
    void IpcTripResetPress() override;
    void CruiseSet()         override;
    void CruiseResume()      override;
    void CruiseCancel()      override;
    void CruiseSpeedUp()     override;
    void CruiseSpeedDown()   override;
    void DoorLockAll()       override;
    void DoorUnlockAll()     override;
    void ExteriorKeypadCode() override;
    void DoorHandleDriver()  override;
    void FlashToPass(bool held) override;
    void FailThrottleInput(bool fail) override;

    // Dispatch a named input action (from a wheel button / Arduino contact) by
    // reusing the same handlers the keyboard drives.  Horn is a held input
    // folded into DriverCommand, so it is not routed here.
    void DispatchAction(ev1sim::InputAction action);

    // Exit codes returned from Run().  Chosen to be CI-friendly:
    //   0   — successful completion (scripted Done, user-closed window, or
    //          max_time with no scripted scenario active)
    //   2   — max_time_s expired while a scripted scenario was still running
    //   130 — SIGINT (conventional 128+SIGINT_number)
    //   64  — usage/config error (EX_USAGE)
    static constexpr int kExitSuccess           = 0;
    static constexpr int kExitTimeout           = 2;
    static constexpr int kExitScenarioAssertion = 3;  ///< scenario assert_* failed
    static constexpr int kExitInterrupted       = 130;
    static constexpr int kExitUsage             = 64;

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
    /// @param dt_s       physics step size in seconds — used by the
    ///                   finite-rate hydraulic model to integrate
    ///                   APPLY/HOLD/DUMP pressure changes.
    /// @param local_front_brake  the local (driver-commanded) front brake ratio [0..1]
    void ApplyAbsFrontBrake(double time, double dt_s, double local_front_brake);

    // Consume BTCM rear-motor commands (kSigRearMotorLR/RR), compute
    // self-energizing drum torque per wheel using the current wheel omega,
    // and apply the resulting brake ratio to Chrono's rear axle.  When
    // BTCM data is stale or missing, falls back to the local rear-brake
    // pedal value (passed in the same way ApplyAbsFrontBrake takes it).
    // Logs freshness transitions on tick boundaries.
    void ApplyRearEmbBrake(double time, double local_rear_brake);

    // Override cmd.throttle in place with the bus throttle command from
    // PIM when m_driver_mode == "electronics" and the bus value is fresh.
    // No-op in "local" mode or when the bus is stale / never received.
    // Logs freshness transitions on tick boundaries.
    void ApplyElectronicsThrottle(DriverCommand& cmd);

    // Symmetric to ApplyElectronicsThrottle: override cmd.steering from the
    // chassis steering command (4076) when m_driver_mode == "electronics" and
    // the bus value is fresh; fall back to the local input otherwise.  No
    // external sim producer today — for closed-loop / physical-rig steering.
    void ApplyElectronicsSteering(DriverCommand& cmd);

    // Consume the externally-driven body actuator peripherals each tick:
    //   - power-steering pump motor (chassis 4097 in / 4098 out)
    //   - sounder / piezo "click"     (chassis 4096 in)
    //   - door-lock motors LH/RH      (chassis 4092-4095 in)
    // Advances the PhysicalWorld plant models from the connector's latched
    // inputs and mirrors door-lock end-of-travel into DoorLocks.  Safe to call
    // when m_external_sim is null (no-op).  dt is the tick duration in seconds.
    void ConsumeBodyActuatorPeripherals(double dt);

    Config m_config;

    std::unique_ptr<VehicleWorld>           m_world;
    std::unique_ptr<KeyboardInputController> m_keyboard;
    std::unique_ptr<CameraManager>          m_camera;
    std::unique_ptr<Telemetry>              m_telemetry;
    std::unique_ptr<HornAudio>              m_horn;
    std::unique_ptr<SounderAudio>           m_sounder_audio;
    std::unique_ptr<VehicleLights>          m_lights;
    std::unique_ptr<VehiclePanels>          m_panels;
    std::unique_ptr<ev1sim::PhysicalWorld>  m_physical;
    std::unique_ptr<ExternalSimConnector>   m_external_sim;
    std::unique_ptr<ScriptedDriver>         m_scripted;
    std::unique_ptr<ev1sim::Scenario>       m_scenario;
    /// Scenario clock origin in sim time (s); < 0 = not armed yet.  A
    /// `requires_external_sim` scenario arms only once both bus transports
    /// are up (ExternalSimConnector::BusesUp) — events that fired earlier
    /// would publish one-shot driver inputs onto a bus no peer has joined,
    /// and they are not retransmitted.  Armed immediately when the scenario
    /// does not need the bus (or it can never come up), and unconditionally
    /// at kScenarioArmFallbackS so a half-configured run still terminates.
    double m_scenario_t0 = -1.0;
    /// True when m_config.simulation.max_time_s was COPIED from the scenario's
    /// own max_time_s (vs. set explicitly in the config JSON).  The shared
    /// max-time exit is then compared against scenario-relative time, so a
    /// late clock arm (co-sim bus wait) can't terminate the run before the
    /// scenario's intended duration elapses — matching the scenario's own
    /// IsDone() check.  An explicit config cap stays a raw wall on sim time.
    bool m_max_time_from_scenario = false;
    /// Returns scenario-relative time, or a negative value while unarmed.
    double ScenarioTime(double sim_t);
    std::unique_ptr<WiperRenderer>          m_wiper;
    std::unique_ptr<FloatingUiPanel>        m_floating_ui;
#ifdef EV1SIM_HAVE_WHEEL_IO
    std::unique_ptr<ev1sim::SdlContext>           m_sdl;
    std::unique_ptr<ev1sim::WheelInputController> m_wheel;
    std::unique_ptr<ev1sim::ForceFeedback>        m_ffb;
#endif

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

    /// Park-brake edge detector — drives the 3D-sim contract set/release
    /// momentary signals (6946/6947).  Tracks the previous tick's level
    /// state from DriverCommand.parking_brake; a false→true transition
    /// emits a one-tick set request, true→false emits a one-tick release.
    bool m_park_brake_prev = false;

    /// Push 3D-sim integration contract driver-side inputs to the
    /// external sim connector (6940-6947, 6966).  Pulls headlight switch
    /// + dim from CombinationSwitch; horn and park-brake state from the
    /// passed DriverCommand; telltale test + key position are stubbed
    /// (no ev1sim UI source).  Park-brake set/release are edge-detected
    /// using m_park_brake_prev; this method updates m_park_brake_prev
    /// as a side effect.  Safe to call when m_external_sim is null.
    void PushExtContractDriverInputs(const struct DriverCommand& cmd);

    // Freshness window for BTCM liveness signal (kSigBtcmUartFrame
    // heartbeat).  See ExternalSimConnector::GetAbsPhaseFront for the
    // architecture: liveness comes from the BTCM's 5 Hz
    // canonical-frame heartbeat (200 ms cadence per the
    // BTCM_UART_FRAME_BROADCAST_PERIOD_MS contract), and iso/dump
    // pin states are persistent commanded state — they don't have
    // their own freshness check.  The 3-second window matches the
    // tightest peer-side BTCM-loss tolerance documented in the EV1
    // manual corpus (IPC DTC 015 "Loss of BTCM Eavesdrop", page 27
    // of the EV1 electrical service manual): a shorter window would
    // trip abs-phase-stale before that peer tolerance elapses, a
    // longer one would keep reporting a lost BTCM as live.
    static constexpr std::chrono::milliseconds kAbsFreshnessWindow{3000};

    // Rear EMB drum brake state (BTCM rear-motor integration).
    // The ApplyRearEmbBrake helper consumes kSigRearMotorLR/RR (5014/5015),
    // converts the float command to a shoe force, computes torque via the
    // BrakeDrum self-energizing model, and applies per-wheel rear brake
    // ratio against the BrakeSimple max torque.
    bool m_rear_lr_was_fresh = false;
    bool m_rear_rr_was_fresh = false;
    /// Rear brake "Maximum Torque" (N·m) — converts the physical drum torque
    /// from the BrakeDrum model into the [0,1] ratio Chrono's brake takes.
    /// MUST equal data/vehicle/ev1/brake/EV1_BrakeSimple_Rear.json's
    /// "Maximum Torque" (Round 4 brake bias: front 1120 / rear 480, 70/30).
    static constexpr double kRearBrakeMaxTorqueNm = 480.0;

    // Throttle authority — when m_driver_mode == "electronics", subscribe
    // to PIM's commanded throttle (kSigChassisThrottleCmdQ8 = 4073) and
    // override cmd.throttle when the bus value is fresh.  Falls back to
    // the local pedal when stale or never received.  Logs freshness
    // transitions on tick boundaries.
    std::string m_driver_mode = "local";          ///< "local" | "electronics"
    bool        m_throttle_bus_was_fresh = false; ///< was bus fresh last tick?
    std::chrono::milliseconds m_throttle_freshness_window{200};
    bool        m_steering_bus_was_fresh = false; ///< steering bus fresh last tick?
    std::chrono::milliseconds m_steering_freshness_window{200};

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

    // UI mode — when true the cursor is shown and mouse clicks go to the
    // floating panel instead of the camera.  Toggled by TAB.
    bool m_ui_mode = false;

    // Running count of IPC trip-reset button presses (HUD display only; not on bus).
    int m_trip_reset_count = 0;

    // Last-applied driver command — updated each tick after all overrides
    // (propulsion gate, electronics throttle, scenario) are applied.
    // Read by the floating-UI throttle/brake display rows each frame.
    DriverCommand m_last_cmd;

    // Last-seen door lock cmd from RSA — used to suppress repeated log lines.
    // 0xFF = never received.
    std::uint8_t m_last_door_lock_cmd[2] = {0xFFu, 0xFFu};  // [0]=driver,[1]=passenger

    // Last-seen power window motor cmd — used to suppress repeated log lines.
    // 0xFF = never received.
    std::uint8_t m_last_pw_motor_cmd[2]  = {0xFFu, 0xFFu};  // [0]=driver,[1]=passenger

    // Last-applied door-lock motor stroke per door (cast of DoorLockMotor::Stroke;
    // 0 == UNLOCKED, the motors' initial stroke).  Suppresses repeated DoorLocks
    // writes / log lines, including a spurious "reached UNLOCKED" on the first
    // tick after the motor-leg drives first appear.
    int m_last_dlm_stroke[2] = {0, 0};                 // [0]=LH/driver, [1]=RH/passenger
};
