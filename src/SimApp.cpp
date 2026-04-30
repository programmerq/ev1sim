#include "SimApp.h"
#include "MacOSPlatform.h"

#include "chrono_vehicle/ChEngine.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

using namespace chrono;
using namespace chrono::vehicle;

namespace {
// Headless-only: flipped by SIGINT so the run loop can exit cleanly.
// Static lifetime is fine — only SimApp::RunHeadless installs/reads it.
std::atomic<bool> g_stop_requested{false};

extern "C" void HeadlessSigintHandler(int) {
    g_stop_requested.store(true, std::memory_order_relaxed);
}
}  // namespace

// ---------------------------------------------------------------------------
SimApp::SimApp(const Config& config) : m_config(config) {
    const bool headless = m_config.simulation.headless;

    // 1. Physics world.
    m_world = std::make_unique<VehicleWorld>(m_config);

    // 2. Keyboard input — only when we have an Irrlicht window to attach to.
    if (!headless) {
        KeyboardInputController::Rates rates;
        rates.steer_rate        = m_config.input.steer_rate;
        rates.steer_return_rate = m_config.input.steer_return_rate;
        rates.throttle_rise     = m_config.input.throttle_rise_rate;
        rates.brake_rise        = m_config.input.brake_rise_rate;
        m_keyboard = std::make_unique<KeyboardInputController>(rates);
    }

    // 3. Telemetry.
    m_telemetry = std::make_unique<Telemetry>(
        m_config.telemetry.log_rate_hz,
        m_config.telemetry.log_to_file,
        m_config.telemetry.log_file,
        m_config.telemetry.show_hud);

    // 4. Horn audio — CoreAudio on macOS, no-op elsewhere.  Safe headless.
    m_horn = std::make_unique<HornAudio>();

    // 5. Visualization (creates window).  Skipped entirely in headless mode;
    //    no Irrlicht device, no window, no OpenGL context.
    if (!headless) {
        SetupVisualization();

        // 6. Camera manager — needs the Irrlicht camera node from vis.
        auto* cam_node = m_vis->GetActiveCamera();
        m_camera = std::make_unique<CameraManager>(
            cam_node, m_config.camera.chase_distance, m_config.camera.chase_height);
        m_camera->SetModeFromString(m_config.camera.default_mode);

        // Register camera manager's mouse handler with Irrlicht.
        m_vis->AddUserEventReceiver(m_camera.get());

        // 7. Vehicle lights — needs Irrlicht scene graph to be populated.
        m_lights = std::make_unique<VehicleLights>();
    }

    // 8. Vehicle panels (hood, trunk, doors) — state-only until panel OBJs exist.
    m_panels = std::make_unique<VehiclePanels>();

    // 8b. Physical-world inputs (combination switch first; more components
    //     to follow per docs/TODO.md).  These are driver-actuated switches
    //     whose state is published wire-level on the chassis bus.
    m_physical = std::make_unique<ev1sim::PhysicalWorld>();

    // 8c. Wiper renderer — phase-based sweep animation driven by RHJB motor command.
    m_wiper = std::make_unique<WiperRenderer>();

    // 9. External electrical-simulator connector.  Non-blocking — if the
    //    electric sim isn't running yet, the connector retries each Tick().
    ExternalSimConnector::Options ext_opts;
    ext_opts.enabled            = m_config.external_sim.enabled;
    ext_opts.bus_name           = m_config.external_sim.bus_name;
    ext_opts.reconnect_period_s = m_config.external_sim.reconnect_period_s;
    m_external_sim = std::make_unique<ExternalSimConnector>(ext_opts);
    m_external_sim->Start();
    if (ext_opts.enabled) {
        std::cout << "[SimApp] External sim: enabled, bus='"
                  << ext_opts.bus_name << "' ("
                  << m_external_sim->StatusString() << ")\n";
    }

    // Scripted driver (optional).  Currently one built-in scenario:
    // accel → hold → brake → done.
    if (m_config.scripted.enabled) {
        ScriptedDriver::Params p;
        p.target_speed_mps   = m_config.scripted.target_speed_kph / 3.6;
        p.hold_time_s        = m_config.scripted.hold_time_s;
        p.stop_threshold_mps = m_config.scripted.stop_threshold_mps;
        m_scripted = std::make_unique<ScriptedDriver>(p);
    }

    m_lights_demo = m_config.lights.demo_mode;

    m_paused = m_config.start_paused;

    // Startup banner — always printed regardless of headless/interactive mode.
    std::cout << "[SimApp] Vehicle: KEY OFF — press K to cycle RSA state (OFF→RUN→ACC→OFF)\n";

    if (headless) {
        std::cout << "[SimApp] Headless mode — no window.  Exits ";
        if (m_scripted)
            std::cout << "when the scripted scenario completes";
        if (m_config.simulation.max_time_s > 0.0) {
            if (m_scripted) std::cout << ", ";
            std::cout << "at sim_time " << m_config.simulation.max_time_s << "s";
        }
        std::cout << ", or on SIGINT.\n";
    } else {
        std::cout << "[SimApp] Ready.  Controls: WASD=drive  Space=park brake  "
                     "P=pause  R=respawn  C=camera  Scroll=zoom  B=horn  O=hi  L=lo  "
                     "H=headlights  K=key-on  .=PRND-up  ,=PRND-down  "
                     "Q=turn-L  E=turn-R  X=hazard  "
                     "F=hood  T=trunk  [=doorL  ]=doorR  Z=snapshot  Esc=quit\n";
        if (m_paused)
            std::cout << "[SimApp] Started PAUSED — press P to begin simulation\n";
    }
}

SimApp::~SimApp() = default;

// ---------------------------------------------------------------------------
void SimApp::SetupVisualization() {
    m_vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    m_vis->SetWindowTitle("EV1 Simulator");
    m_vis->SetWindowSize(1280, 720);
    m_vis->SetDriverType(irr::video::EDT_OPENGL);

    // Chase camera defaults (we override positioning via CameraManager,
    // but Chrono needs a chase camera configured to initialise properly).
    m_vis->SetChaseCamera(
        ChVector3d(0, 0, 1.5),  // track point on vehicle
        m_config.camera.chase_distance,
        0.5);

    m_vis->Initialize();
    m_vis->AddSkyBox();

    // Directional sun light with ambient from the environment preset.
    // AddLightDirectional(elevation_deg, azimuth_deg, ambient, specular, diffuse).
    const auto& env = m_config.environment;
    m_vis->AddLightDirectional(
        static_cast<float>(env.sun_elevation_deg), 60.0f,
        chrono::ChColor(static_cast<float>(env.ambient_r),
                        static_cast<float>(env.ambient_g),
                        static_cast<float>(env.ambient_b)),
        chrono::ChColor(0.3f, 0.3f, 0.3f),   // specular
        chrono::ChColor(1.0f, 1.0f, 0.9f));  // diffuse (warm sun)
    m_vis->AttachVehicle(&m_world->GetVehicle());

    // macOS platform fixes (no-ops on other platforms).
    macos_activate_app();
    macos_setup_menu_bar();
    macos_fix_retina_viewport();
    macos_enable_fullscreen();

    // Register keyboard handler.
    m_vis->AddUserEventReceiver(m_keyboard.get());
}

// ---------------------------------------------------------------------------
void SimApp::ApplyAbsFrontBrake(double time, double local_front_brake) {
    using Phase = ExternalSimConnector::AbsPhaseFront::Phase;

    const auto abs_phase = m_external_sim->GetAbsPhaseFront(kAbsFreshnessWindow);

    // Log freshness transitions (one line per wheel).
    if (abs_phase.fl_fresh != m_abs_fl_was_fresh) {
        if (abs_phase.fl_fresh)
            std::cout << "[SimApp] front-brake from BTCM (live) wheel=FL\n";
        else
            std::cout << "[SimApp] front-brake fallback (BTCM stale) wheel=FL\n";
        m_abs_fl_was_fresh = abs_phase.fl_fresh;
    }
    if (abs_phase.fr_fresh != m_abs_fr_was_fresh) {
        if (abs_phase.fr_fresh)
            std::cout << "[SimApp] front-brake from BTCM (live) wheel=FR\n";
        else
            std::cout << "[SimApp] front-brake fallback (BTCM stale) wheel=FR\n";
        m_abs_fr_was_fresh = abs_phase.fr_fresh;
    }

    // If neither wheel has fresh BTCM data, the symmetric front_pressure from
    // CommandDriver::ApplyBrakes() (called inside VehicleWorld::Synchronize) is
    // already in effect — nothing to do.
    if (!abs_phase.fl_fresh && !abs_phase.fr_fresh) {
        // Keep prev values in sync with local so HOLD doesn't freeze at stale values.
        m_abs_fl_prev = local_front_brake;
        m_abs_fr_prev = local_front_brake;
        return;
    }

    // Compute per-wheel modulated brake ratio.
    auto modulate = [&](Phase phase, double prev, double local) -> double {
        switch (phase) {
            case Phase::APPLY: return local;
            case Phase::HOLD:  return prev;
            case Phase::DUMP:  return 0.2 * local;
        }
        return local;  // unreachable
    };

    const double fl = modulate(abs_phase.fl, m_abs_fl_prev, local_front_brake);
    const double fr = modulate(abs_phase.fr, m_abs_fr_prev, local_front_brake);

    m_abs_fl_prev = fl;
    m_abs_fr_prev = fr;

    // Apply per-wheel front brakes via Chrono API, overriding the symmetric
    // front_pressure CommandDriver::ApplyBrakes() already set in Synchronize().
    // TODO: rear EMB clamp-position model needed for full rear-wheel modulation.
    m_world->ApplyFrontBrakePerWheel(time, fl, fr);
}

// ---------------------------------------------------------------------------
int SimApp::Run() {
    if (m_config.simulation.headless) {
        // Guard against the hang-forever foot-gun: headless with no terminator
        // and no scripted scenario would loop until SIGINT, which is a bad
        // default for CI.  Require at least one way to exit automatically.
        const bool has_max_time = m_config.simulation.max_time_s > 0.0;
        const bool has_scripted = m_config.scripted.enabled;
        if (!has_max_time && !has_scripted) {
            std::cerr << "[SimApp] --headless requires at least one of "
                         "--max-time <s> or a scripted scenario "
                         "(e.g. --scripted-accel-brake).  Otherwise the "
                         "run can only be ended by SIGINT.\n";
            return kExitUsage;
        }
        return RunHeadless();
    }
    return RunWithVisualization();
}

// ---------------------------------------------------------------------------
int SimApp::RunWithVisualization() {
    double step     = m_config.simulation.step_size_s;
    double render_dt = 1.0 / std::max(1, m_config.simulation.render_fps);
    int    steps_per_frame = std::max(1, static_cast<int>(std::round(render_dt / step)));
    double max_time = m_config.simulation.max_time_s;

    while (m_vis->Run()) {
        // --- Input (once per render frame) ---
        // Keyboard always runs to provide one-shot actions (pause, camera,
        // quit, etc.), but the drive command comes from the scripted driver
        // when configured — useful for visually debugging a scenario.
        DriverCommand cmd = m_keyboard->Update(render_dt);
        if (m_scripted && !m_paused)
            cmd = m_scripted->Update(m_world->GetState());

        // --- Propulsion enable gate (KEY OFF override) ---
        // While m_propulsion_enabled is false, clamp brakes at full and zero
        // throttle so Chrono reflects the vehicle's actual "key off" state.
        if (!m_propulsion_enabled) {
            cmd.throttle    = 0.0;
            cmd.front_brake = 1.0;
            cmd.rear_brake  = 1.0;
        }

        m_world->GetDriver().SetCommand(cmd);

        // Handle one-shot actions.
        if (cmd.reset_vehicle)
            m_world->ResetVehicle();
        if (m_keyboard->ConsumeCameraCycle())
            m_camera->CycleMode();
        if (m_keyboard->ConsumePauseToggle()) {
            m_paused = !m_paused;
            std::cout << (m_paused ? "[SimApp] PAUSED" : "[SimApp] RESUMED") << std::endl;
        }
        if (m_keyboard->QuitRequested())
            break;
        // K = cycle RSA key state: OFF → RUN → ACC → OFF → …
        // Publishes momentary RSA keypad code_ok and mode_button signals.
        if (m_keyboard->ConsumeKeyOnToggle()) {
            m_physical->rsa_keypad().cycle_k();
            std::cout << "[SimApp] Key request: "
                      << m_physical->rsa_keypad().expected_state_name() << "\n";
        }
        if (m_keyboard->ConsumeHeadlightToggle()) {
            // H cycles the combination switch through OFF → PARK → ON → HI.
            // The wire-level pin states are published on the chassis bus and
            // LHJB drives the actual bulb feed lines back from there.
            if (m_lights_demo != "off") {
                std::cout << "[SimApp] Lights demo OFF (was " << m_lights_demo << ")\n";
                m_lights_demo = "off";
            }
            m_physical->combination_switch().cycle_h();
            const char* names[] = {"OFF", "PARK", "ON", "HI"};
            int idx = static_cast<int>(m_physical->combination_switch().position());
            std::cout << "[SimApp] CombSw: " << names[idx] << "\n";
        }
        // U is momentary flash-to-pass (pulled-back lever).  Update each frame.
        m_physical->combination_switch().set_flash_to_pass(
            m_keyboard->IsFlashToPassHeld());

        // '.' / ',' cycle PRND up/down (clamped at ends; no wraparound).
        if (m_keyboard->ConsumePrndUp()) {
            m_physical->prnd_selector().cycle_up();
            const char* prnd_names[] = {"P", "R", "N", "D"};
            int idx = static_cast<int>(m_physical->prnd_selector().position());
            std::cout << "[SimApp] PRND -> " << prnd_names[idx] << "\n";
        }
        if (m_keyboard->ConsumePrndDown()) {
            m_physical->prnd_selector().cycle_down();
            const char* prnd_names[] = {"P", "R", "N", "D"};
            int idx = static_cast<int>(m_physical->prnd_selector().position());
            std::cout << "[SimApp] PRND -> " << prnd_names[idx] << "\n";
        }
        // Q = turn signal LEFT; E = turn signal RIGHT; X = hazard toggle.
        if (m_keyboard->ConsumeTurnSignalLeft()) {
            m_physical->turn_signal_stalk().toggle_left();
            const auto pos = m_physical->turn_signal_stalk().position();
            using P = ev1sim::TurnSignalStalk::Position;
            const char* ts_name = (pos == P::LEFT) ? "LEFT" : (pos == P::RIGHT) ? "RIGHT" : "OFF";
            std::cout << "[SimApp] Turn signal: " << ts_name << "\n";
        }
        if (m_keyboard->ConsumeTurnSignalRight()) {
            m_physical->turn_signal_stalk().toggle_right();
            const auto pos = m_physical->turn_signal_stalk().position();
            using P = ev1sim::TurnSignalStalk::Position;
            const char* ts_name = (pos == P::LEFT) ? "LEFT" : (pos == P::RIGHT) ? "RIGHT" : "OFF";
            std::cout << "[SimApp] Turn signal: " << ts_name << "\n";
        }
        if (m_keyboard->ConsumeHazardToggle()) {
            m_physical->hazard_switch().toggle();
            std::cout << "[SimApp] Hazard: "
                      << (m_physical->hazard_switch().on() ? "ON" : "OFF") << "\n";
        }

        // I = IPC trip-reset (momentary).
        if (m_keyboard->ConsumeIpcTripReset()) {
            m_physical->ipc_trip_reset().press();
            ++m_trip_reset_count;
            std::cout << "[SimApp] IPC trip-reset pressed\n";
        }

        // Z = toggle physical-world snapshot overlay.
        if (m_keyboard->ConsumeSnapshotToggle())
            m_show_snapshot = !m_show_snapshot;

        // Cruise stalk: G=SET, Y=RESUME, N=CANCEL, +(=)=SPEED+, -=SPEED-.
        if (m_keyboard->ConsumeCruiseSet()) {
            m_physical->cruise_stalk().press_set();
            std::cout << "[SimApp] Cruise: SET\n";
        }
        if (m_keyboard->ConsumeCruiseResume()) {
            m_physical->cruise_stalk().press_resume();
            std::cout << "[SimApp] Cruise: RESUME\n";
        }
        if (m_keyboard->ConsumeCruiseCancel()) {
            m_physical->cruise_stalk().press_cancel();
            std::cout << "[SimApp] Cruise: CANCEL\n";
        }
        if (m_keyboard->ConsumeCruiseSpeedUp()) {
            m_physical->cruise_stalk().press_speed_up();
            std::cout << "[SimApp] Cruise: SPEED+\n";
        }
        if (m_keyboard->ConsumeCruiseSpeedDown()) {
            m_physical->cruise_stalk().press_speed_down();
            std::cout << "[SimApp] Cruise: SPEED-\n";
        }

        // V = wiper cycle (OFF → INT → LOW → HIGH → OFF).
        if (m_keyboard->ConsumeWiperCycle()) {
            m_physical->wiper_stalk().cycle_position();
            const char* wiper_names[] = {"OFF", "INT", "LOW", "HIGH"};
            int widx = static_cast<int>(m_physical->wiper_stalk().position());
            std::cout << "[SimApp] Wiper: " << wiper_names[widx] << "\n";
        }

        // M = wiper wash (momentary).
        if (m_keyboard->ConsumeWiperWash()) {
            m_physical->wiper_stalk().press_wash();
            std::cout << "[SimApp] Wiper wash\n";
        }

        // Auto-cancel turn signal from steering travel.
        // run every frame regardless of pause so the state machine stays current.
        m_physical->turn_signal_stalk().update_for_steering(cmd.steering, render_dt);
        if (m_physical->turn_signal_stalk().consume_auto_cancel_event())
            std::cout << "[SimApp] Turn signal: AUTO-CANCEL\n";

        // --- Physics sub-stepping (skipped when paused) ---
        if (!m_paused) {
            for (int i = 0; i < steps_per_frame; ++i) {
                double t = m_world->GetSimTime();
                m_world->Synchronize(t);
                // Per-wheel BTCM ABS modulation (front axle).
                // Must follow Synchronize() (which calls ApplyBrakes internally)
                // so we can override the symmetric front pressure when BTCM is live.
                ApplyAbsFrontBrake(t, cmd.front_brake);
                m_world->Advance(step);
            }
        }

        // --- Visualisation sync/advance ---
        double t = m_world->GetSimTime();
        m_vis->Synchronize(t, m_world->GetDriver().GetInputs());
        m_vis->Advance(render_dt);

        // --- Camera override ---
        m_camera->Update(m_world->GetPose());

        // --- Panel toggles (1-4) ---
        for (int i = 0; i < VehiclePanels::NUM_PANELS; ++i) {
            if (m_keyboard->ConsumePanelToggle(i))
                m_panels->Toggle(static_cast<PanelID>(i));
        }

        // --- Vehicle lights ---
        // Lazy-init: the Irrlicht scene nodes are created during the
        // first Synchronize/Advance cycle, so we initialise on first use.
        if (!m_lights->IsInitialized()) {
            auto* smgr = m_vis->GetDevice()->getSceneManager();
            m_lights->Initialize(smgr);
        }

        // --- External sim sync (publish panel sensors + dynamics + combo
        //     switch + driver inputs; drain bulb/horn cmds) ---
        for (int i = 0; i < VehiclePanels::NUM_PANELS; ++i) {
            m_external_sim->SetPanelSensor(static_cast<PanelID>(i),
                                           m_panels->IsOpen(static_cast<PanelID>(i)));
        }
        m_external_sim->SetVehicleState(m_world->GetState());
        // Combination switch wire-level outputs (chassis bus, IDs 4040-4042).
        {
            const auto& cs = m_physical->combination_switch();
            m_external_sim->SetCombSwOutputs(cs.pin_low_beam_out(),
                                             cs.pin_flash_to_pass_out(),
                                             cs.pin_park_headlamp_out());
        }
        // Charge coupler presence (chassis bus, ID 4060).  Stubbed false until
        // a floating-UI panel or charge-door animation sets it.
        m_external_sim->SetChargeCouplerPresent(
            m_physical->charge_coupler().present());
        // PRND selector wire-level pins (chassis bus, IDs 4050-4053).
        {
            const auto& prnd = m_physical->prnd_selector();
            m_external_sim->SetPrndSelector(prnd.pin_a(), prnd.pin_b(),
                                            prnd.pin_c(), prnd.pin_d());
        }
        // Driver inputs (main harness segment, IDs 6900-6903).
        {
            auto clamp01_q8 = [](double v) -> std::uint8_t {
                if (v < 0.0) v = 0.0;
                if (v > 1.0) v = 1.0;
                return static_cast<std::uint8_t>(v * 255.0 + 0.5);
            };
            // Steering Q8 in degrees, signed.  cmd.steering is normalized
            // -1..+1 (Chrono convention); map to ±90 deg as a placeholder.
            auto steer_deg_q8 = [](double v) -> std::int16_t {
                if (v < -1.0) v = -1.0;
                if (v >  1.0) v =  1.0;
                double deg_q8 = v * 90.0 * 256.0;
                if (deg_q8 >  32767.0) deg_q8 =  32767.0;
                if (deg_q8 < -32768.0) deg_q8 = -32768.0;
                return static_cast<std::int16_t>(deg_q8);
            };
            m_external_sim->SetDriverBrakePedalQ8(clamp01_q8(cmd.front_brake));
            m_external_sim->SetDriverThrottleQ8(clamp01_q8(cmd.throttle));
            m_external_sim->SetDriverSteeringDegQ8(steer_deg_q8(cmd.steering));
            // Gear selector: map PrndSelector::Position to enum (P=0, R=1, N=2, D=3).
            m_external_sim->SetDriverGearSelector(
                static_cast<std::uint8_t>(m_physical->prnd_selector().position()));
            // Brake switch (6904): derive from brake travel with hysteresis.
            bool brake_sw = m_physical->brake_switch().update(cmd.front_brake);
            m_external_sim->SetDriverBrakeSwitch(brake_sw);
            // Seatbelt (6964): default true — driver always buckled.
            // TODO: add a floating-UI toggle in docs/TODO.md panel item so
            // the user can unbuckle during development testing.
            m_external_sim->SetDriverSeatbeltBuckled(true);
            // Turn signal stalk (6948, 6949) and hazard switch (6944).
            m_external_sim->SetDriverTurnSignalLeft(
                m_physical->turn_signal_stalk().active_left());
            m_external_sim->SetDriverTurnSignalRight(
                m_physical->turn_signal_stalk().active_right());
            m_external_sim->SetDriverHazardRequest(
                m_physical->hazard_switch().on());
            // RSA keypad buttons (6975-6979) and mode button (6971) — tick the
            // scheduler and consume whatever it has ready for this frame.
            // button_value encoding: 0=idle, 1=tap, 2=long-press.
            m_physical->rsa_keypad().update(render_dt);
            {
                auto fires = m_physical->rsa_keypad().consume_fires_now();
                m_external_sim->SetDriverRsaKeypadButton1(fires.button_value[0]);
                m_external_sim->SetDriverRsaKeypadButton2(fires.button_value[1]);
                m_external_sim->SetDriverRsaKeypadButton3(fires.button_value[2]);
                m_external_sim->SetDriverRsaKeypadButton4(fires.button_value[3]);
                m_external_sim->SetDriverRsaKeypadButton5(fires.button_value[4]);
                m_external_sim->SetDriverRsaModeButton(fires.mode_button);
            }
            // IPC trip-reset (6952), cruise stalk (6953-6957), wiper (6958, 6959).
            // Consume one-shot events and publish to the main harness segment.
            m_external_sim->SetDriverIpcTripReset(
                m_physical->ipc_trip_reset().consume_press_event());
            m_external_sim->SetDriverCruiseSet(
                m_physical->cruise_stalk().consume_set());
            m_external_sim->SetDriverCruiseResume(
                m_physical->cruise_stalk().consume_resume());
            m_external_sim->SetDriverCruiseCancel(
                m_physical->cruise_stalk().consume_cancel());
            m_external_sim->SetDriverCruiseSpeedUp(
                m_physical->cruise_stalk().consume_speed_up());
            m_external_sim->SetDriverCruiseSpeedDown(
                m_physical->cruise_stalk().consume_speed_down());
            m_external_sim->SetDriverWiperSwitch(
                static_cast<std::uint8_t>(m_physical->wiper_stalk().position()));
            m_external_sim->SetDriverWiperWashRequest(
                m_physical->wiper_stalk().consume_wash());
        }
        // Motor RPM and torque (chassis bus 4070-4071).
        {
            auto* engine = m_world->GetVehicle().GetEngine().get();
            float motor_rpm = 0.0f;
            float motor_torque = 0.0f;
            if (engine) {
                // GetMotorSpeed() returns rad/s; convert to RPM.
                motor_rpm    = static_cast<float>(engine->GetMotorSpeed() * 60.0 / (2.0 * 3.14159265358979323846));
                motor_torque = static_cast<float>(engine->GetOutputMotorshaftTorque());
            }
            m_external_sim->SetMotorRpm(motor_rpm);
            m_external_sim->SetMotorTorqueNm(motor_torque);
        }
        m_external_sim->Tick(t);

        // Subscribe to RSA run-mode broadcast and update propulsion gate.
        if (m_external_sim->HasReceivedRunMode()) {
            const std::uint8_t run_mode = m_external_sim->GetRsaRunMode();
            // RSA run modes: 0=OFF, 1=ACC, 2=RUN (per rsa_scan.h).
            const bool new_prop = (run_mode == 2 /*RUN*/);
            if (new_prop != m_propulsion_enabled) {
                m_propulsion_enabled = new_prop;
                const char* mode_names[] = {"OFF", "ACC", "RUN"};
                const char* mode_str = (run_mode < 3) ? mode_names[run_mode] : "UNKNOWN";
                std::cout << "[SimApp] Run mode (from RSA): " << mode_str << "\n";
            }
        }

        const bool ext_driving_bulbs =
            m_external_sim->IsConnected() && m_external_sim->HasReceivedBulbData();

        if (m_lights_demo == "blink") {
            // All bulbs blink at unique frequencies for identification.
            m_lights->UpdateDemoMode(t);
        } else if (m_lights_demo == "chase") {
            // One bulb at a time, ~0.2s each, walking around the vehicle.
            m_lights->UpdateChaseDemo(t);
        } else if (ext_driving_bulbs) {
            // Bulb state is fully authored by the external electrical sim.
            for (int i = 0; i < NUM_LIGHTS; ++i) {
                m_lights->SetState(static_cast<LightID>(i),
                                   m_external_sim->GetBulbCmd(static_cast<LightID>(i)));
            }
        } else {
            // No demo, no external sim — all bulbs stay off.  We deliberately
            // do NOT have a local-fallback headlamp path: the user wants the
            // ECU pipeline to be the only source of bulb state, so missing
            // ECU output makes the missing wire visible rather than masking it.
            for (int i = 0; i < NUM_LIGHTS; ++i)
                m_lights->SetState(static_cast<LightID>(i), false);
        }

        m_lights->ApplyToScene();

        // --- Wiper renderer tick ---
        // Pull the latest motor command from RHJB (0xFF if never received →
        // Tick() treats it as OFF since no valid enum matches).
        {
            const std::uint8_t wiper_cmd =
                m_external_sim->HasReceivedWiperMotorCommand()
                    ? m_external_sim->GetWiperMotorCommand()
                    : 0u;  // default OFF when no RHJB command yet
            m_wiper->Tick(render_dt, wiper_cmd);
        }
        m_wiper->ApplyToScene();

        // --- Render ---
        m_vis->BeginScene();
        macos_apply_viewport();   // override glViewport for Retina / resize
        m_vis->Render();
        // Ask the world for the truthful terrain label — it reflects the
        // actual loaded terrain, including the rigid-plane fallback when
        // a requested level file was missing or invalid.
        auto mu = m_world->GetWheelFrictions();
        const auto abs_phase = m_external_sim->GetAbsPhaseFront(kAbsFreshnessWindow);
        m_telemetry->DrawHUD(m_vis->GetDevice(),
                             m_world->GetState(),
                             m_camera->GetModeName(),
                             m_world->GetTerrainLabel(),
                             mu.mu,
                             &abs_phase);
        m_lights->DrawHUD(m_vis->GetDevice());
        m_panels->DrawHUD(m_vis->GetDevice());
        m_wiper->DrawHUD(m_vis->GetDevice());
        m_physical->DrawHUD(m_vis->GetDevice(),
                            m_external_sim->GetRsaRunMode(),
                            m_external_sim->HasReceivedRunMode());
        m_physical->DrawSnapshotOverlay(m_vis->GetDevice(),
                                        m_show_snapshot,
                                        m_external_sim->GetRsaRunMode(),
                                        m_external_sim->HasReceivedRunMode(),
                                        m_trip_reset_count);

        // Draw PAUSED overlay.
        if (m_paused) {
            auto* gui  = m_vis->GetDevice()->getGUIEnvironment();
            auto* font = gui->getBuiltInFont();
            if (font) {
                auto dim = m_vis->GetDevice()->getVideoDriver()->getScreenSize();
                irr::core::recti rect(dim.Width / 2 - 40, 30, dim.Width / 2 + 40, 50);
                font->draw(L"[ PAUSED ]", rect,
                           irr::video::SColor(255, 255, 200, 0), true, true);
            }
        }

        // Help overlay toggle (? key).
        if (m_keyboard->ConsumeHelpToggle())
            m_show_help = !m_show_help;
        // Auto-hide after first 5 sim seconds.
        const double sim_now = m_world->GetSimTime();
        if (m_show_help && sim_now > m_help_hide_time) {
            // Auto-hide only once (disable the timer by setting to a huge value).
            m_show_help = false;
            m_help_hide_time = 1e18;
        }

        // Draw keyboard help overlay.
        if (m_show_help) {
            auto* drv  = m_vis->GetDevice()->getVideoDriver();
            auto* gui  = m_vis->GetDevice()->getGUIEnvironment();
            auto* font = gui->getBuiltInFont();
            if (drv && font) {
                auto dim = drv->getScreenSize();
                // Translucent dark background box.
                const int bx = dim.Width / 2 - 220;
                const int by = 60;
                const int bw = 440;
                const int bh = 296;
                drv->draw2DRectangle(
                    irr::video::SColor(180, 20, 20, 30),
                    irr::core::recti(bx, by, bx + bw, by + bh));

                const int h = 16;
                int ry = by + 6;
                irr::video::SColor hdr(255, 255, 220,  80);
                irr::video::SColor dim_col(255, 160, 160, 160);
                irr::video::SColor norm(255, 220, 220, 220);

                auto drawL = [&](const wchar_t* text, irr::video::SColor col) {
                    font->draw(text,
                        irr::core::rect<irr::s32>(bx + 8, ry, bx + bw - 8, ry + h),
                        col);
                    ry += h;
                };

                drawL(L"KEYBOARD CONTROLS              [? to toggle]", hdr);
                drawL(L"-----------------------------------------------", dim_col);
                drawL(L"WASD/Arrows  drive (throttle, brake, steering)", norm);
                drawL(L"Space        parking brake",                       norm);
                drawL(L"K            key cycle: OFF -> RUN -> ACC -> OFF", norm);
                drawL(L"H            headlamp: OFF -> PARK -> ON -> HI",  norm);
                drawL(L"U            flash-to-pass (held)",                norm);
                drawL(L"Q / E        turn signal left / right (toggle)",  norm);
                drawL(L"X            hazard toggle",                       norm);
                drawL(L", / .        PRND down / up",                      norm);
                drawL(L"B / O / L    horn (both / high / low)",           norm);
                drawL(L"F            hood toggle",                         norm);
                drawL(L"T            trunk toggle",                        norm);
                drawL(L"[ / ]        door L / R toggle",                  norm);
                drawL(L"Z            physical-world snapshot overlay",     norm);
                drawL(L"C            camera mode cycle",                   norm);
                drawL(L"P            pause",                               norm);
                drawL(L"R            respawn",                             norm);
                drawL(L"Esc          quit",                                norm);
            }
        }

        m_vis->EndScene();

        // --- Horn audio (external sim commands OR'd with keyboard input) ---
        bool horn_low  = cmd.horn_low;
        bool horn_high = cmd.horn_high;
        if (m_external_sim->IsConnected()) {
            horn_low  = horn_low  || m_external_sim->GetHornLowCmd();
            horn_high = horn_high || m_external_sim->GetHornHighCmd();
        }
        m_horn->SetTones(horn_low, horn_high);

        // --- Telemetry logging ---
        m_telemetry->Record(m_world->GetState(), render_dt);

        // --- Realtime pacing ---
        if (m_config.simulation.realtime)
            m_realtime_timer.Spin(step * steps_per_frame);

        // --- Scripted-scenario complete ---
        if (m_scripted && m_scripted->IsDone()) {
            std::cout << "[SimApp] Scripted scenario complete at t="
                      << m_world->GetSimTime() << "s — exiting.\n";
            return kExitSuccess;
        }

        // --- Max-time exit (shared with headless) ---
        if (max_time > 0.0 && m_world->GetSimTime() >= max_time) {
            const bool scripted_unfinished = m_scripted && !m_scripted->IsDone();
            if (scripted_unfinished) {
                std::cerr << "[SimApp] max_time_s reached with scripted "
                             "scenario still in phase '"
                          << m_scripted->PhaseName() << "' — timeout.\n";
                return kExitTimeout;
            }
            std::cout << "[SimApp] max_time_s reached — exiting.\n";
            return kExitSuccess;
        }
    }
    // Window closed / Esc pressed — normal exit.
    return kExitSuccess;
}

// ---------------------------------------------------------------------------
int SimApp::RunHeadless() {
    // Install SIGINT handler so Ctrl-C breaks out of the loop cleanly.
    g_stop_requested.store(false, std::memory_order_relaxed);
    struct sigaction new_sa{}, old_sa{};
    new_sa.sa_handler = &HeadlessSigintHandler;
    sigemptyset(&new_sa.sa_mask);
    sigaction(SIGINT, &new_sa, &old_sa);

    const double step     = m_config.simulation.step_size_s;
    const double tick_dt  = 1.0 / std::max(1, m_config.simulation.render_fps);
    const int    steps_per_tick =
        std::max(1, static_cast<int>(std::round(tick_dt / step)));
    const double max_time = m_config.simulation.max_time_s;

    // Default driver command — zero throttle/brake/steering when no scripted
    // driver is configured.  Apply startup override (brakes clamped, throttle
    // zeroed) so the initial SetCommand also reflects "key off".
    {
        DriverCommand init_cmd{};
        if (!m_propulsion_enabled) {
            init_cmd.front_brake = 1.0;
            init_cmd.rear_brake  = 1.0;
        }
        m_world->GetDriver().SetCommand(init_cmd);
    }

    const auto wall_start = std::chrono::steady_clock::now();

    while (!g_stop_requested.load(std::memory_order_relaxed)) {
        // --- Scripted driver (if any) — reads previous-tick state and
        //     emits a new command before we step physics.
        DriverCommand cmd{};
        if (m_scripted) {
            cmd = m_scripted->Update(m_world->GetState());
        }

        // --- Propulsion enable gate (KEY OFF override) ---
        // In headless mode no keyboard presses cycle the RSA state.
        // Propulsion is enabled only if RSA broadcasts RUN on the bus.
        if (!m_propulsion_enabled) {
            cmd.throttle    = 0.0;
            cmd.front_brake = 1.0;
            cmd.rear_brake  = 1.0;
        }

        m_world->GetDriver().SetCommand(cmd);

        // Auto-cancel turn signal from steering travel (headless: scripted driver
        // may supply non-zero steering; keyboard path is absent).
        m_physical->turn_signal_stalk().update_for_steering(cmd.steering, tick_dt);
        if (m_physical->turn_signal_stalk().consume_auto_cancel_event())
            std::cout << "[SimApp] Turn signal: AUTO-CANCEL\n";

        // --- Physics sub-stepping (no pause control in headless) ---
        for (int i = 0; i < steps_per_tick; ++i) {
            const double t = m_world->GetSimTime();
            m_world->Synchronize(t);
            // Per-wheel BTCM ABS modulation (front axle).
            // Must follow Synchronize() so we override the symmetric front
            // pressure when BTCM is live.
            ApplyAbsFrontBrake(t, cmd.front_brake);
            m_world->Advance(step);
        }

        const double t = m_world->GetSimTime();

        // --- External sim sync (panel sensors + dynamics + combo switch
        //     + driver inputs; bulb/horn cmds drained inside Tick()) ---
        for (int i = 0; i < VehiclePanels::NUM_PANELS; ++i) {
            m_external_sim->SetPanelSensor(
                static_cast<PanelID>(i),
                m_panels->IsOpen(static_cast<PanelID>(i)));
        }
        m_external_sim->SetVehicleState(m_world->GetState());
        // Combination switch: in headless mode no keyboard cycles it, so the
        // pin states stay at their default (OFF position → all pins low).
        // Still publish so downstream consumers see a defined initial state.
        {
            const auto& cs = m_physical->combination_switch();
            m_external_sim->SetCombSwOutputs(cs.pin_low_beam_out(),
                                             cs.pin_flash_to_pass_out(),
                                             cs.pin_park_headlamp_out());
        }
        // Charge coupler presence (chassis bus, ID 4060).  Stubbed false until
        // a floating-UI panel or charge-door animation sets it.
        m_external_sim->SetChargeCouplerPresent(
            m_physical->charge_coupler().present());
        // PRND selector wire-level pins (chassis bus, IDs 4050-4053).
        // In headless mode stays at default Park; no keyboard cycling.
        {
            const auto& prnd = m_physical->prnd_selector();
            m_external_sim->SetPrndSelector(prnd.pin_a(), prnd.pin_b(),
                                            prnd.pin_c(), prnd.pin_d());
        }
        // Driver inputs — publish the override-adjusted values so the bus
        // reflects what the vehicle is actually doing (IDs 6900-6903).
        {
            auto clamp01_q8 = [](double v) -> std::uint8_t {
                if (v < 0.0) v = 0.0;
                if (v > 1.0) v = 1.0;
                return static_cast<std::uint8_t>(v * 255.0 + 0.5);
            };
            m_external_sim->SetDriverBrakePedalQ8(clamp01_q8(cmd.front_brake));
            m_external_sim->SetDriverThrottleQ8(clamp01_q8(cmd.throttle));
            m_external_sim->SetDriverSteeringDegQ8(0);  // no keyboard in headless
            // Gear selector: map PrndSelector::Position to enum (P=0, R=1, N=2, D=3).
            m_external_sim->SetDriverGearSelector(
                static_cast<std::uint8_t>(m_physical->prnd_selector().position()));
            // Brake switch (6904): derive from brake travel with hysteresis.
            bool brake_sw = m_physical->brake_switch().update(cmd.front_brake);
            m_external_sim->SetDriverBrakeSwitch(brake_sw);
            // Seatbelt (6964): default true — driver always buckled.
            // TODO: add a floating-UI toggle in docs/TODO.md panel item so
            // the user can unbuckle during development testing.
            m_external_sim->SetDriverSeatbeltBuckled(true);
            // Turn signal stalk (6948, 6949) and hazard switch (6944).
            // In headless mode no keyboard cycles them; publish stable defaults
            // (both false) so LHJB sees a defined initial state.
            m_external_sim->SetDriverTurnSignalLeft(
                m_physical->turn_signal_stalk().active_left());
            m_external_sim->SetDriverTurnSignalRight(
                m_physical->turn_signal_stalk().active_right());
            m_external_sim->SetDriverHazardRequest(
                m_physical->hazard_switch().on());
            // RSA keypad signals — headless: no key presses; tick scheduler
            // and publish whatever it has (likely idle zeros).
            // button_value encoding: 0=idle, 1=tap, 2=long-press.
            m_physical->rsa_keypad().update(tick_dt);
            {
                auto fires = m_physical->rsa_keypad().consume_fires_now();
                m_external_sim->SetDriverRsaKeypadButton1(fires.button_value[0]);
                m_external_sim->SetDriverRsaKeypadButton2(fires.button_value[1]);
                m_external_sim->SetDriverRsaKeypadButton3(fires.button_value[2]);
                m_external_sim->SetDriverRsaKeypadButton4(fires.button_value[3]);
                m_external_sim->SetDriverRsaKeypadButton5(fires.button_value[4]);
                m_external_sim->SetDriverRsaModeButton(fires.mode_button);
            }
            // IPC trip-reset (6952), cruise stalk (6953-6957), wiper (6958, 6959).
            // Headless: no keyboard input; consume pending events (all idle in
            // headless) and publish stable defaults so the bus sees defined state.
            m_external_sim->SetDriverIpcTripReset(
                m_physical->ipc_trip_reset().consume_press_event());
            m_external_sim->SetDriverCruiseSet(
                m_physical->cruise_stalk().consume_set());
            m_external_sim->SetDriverCruiseResume(
                m_physical->cruise_stalk().consume_resume());
            m_external_sim->SetDriverCruiseCancel(
                m_physical->cruise_stalk().consume_cancel());
            m_external_sim->SetDriverCruiseSpeedUp(
                m_physical->cruise_stalk().consume_speed_up());
            m_external_sim->SetDriverCruiseSpeedDown(
                m_physical->cruise_stalk().consume_speed_down());
            m_external_sim->SetDriverWiperSwitch(
                static_cast<std::uint8_t>(m_physical->wiper_stalk().position()));
            m_external_sim->SetDriverWiperWashRequest(
                m_physical->wiper_stalk().consume_wash());
        }
        // Motor RPM and torque (chassis bus 4070-4071).
        {
            auto* engine = m_world->GetVehicle().GetEngine().get();
            float motor_rpm = 0.0f;
            float motor_torque = 0.0f;
            if (engine) {
                motor_rpm    = static_cast<float>(engine->GetMotorSpeed() * 60.0 / (2.0 * 3.14159265358979323846));
                motor_torque = static_cast<float>(engine->GetOutputMotorshaftTorque());
            }
            m_external_sim->SetMotorRpm(motor_rpm);
            m_external_sim->SetMotorTorqueNm(motor_torque);
        }
        m_external_sim->Tick(t);

        // Subscribe to RSA run-mode broadcast and update propulsion gate.
        if (m_external_sim->HasReceivedRunMode()) {
            const std::uint8_t run_mode = m_external_sim->GetRsaRunMode();
            const bool new_prop = (run_mode == 2 /*RUN*/);
            if (new_prop != m_propulsion_enabled) {
                m_propulsion_enabled = new_prop;
                const char* mode_names[] = {"OFF", "ACC", "RUN"};
                const char* mode_str = (run_mode < 3) ? mode_names[run_mode] : "UNKNOWN";
                std::cout << "[SimApp] Run mode (from RSA): " << mode_str << "\n";
            }
        }

        // --- Wiper renderer tick (headless — no DrawHUD, just phase tracking) ---
        {
            const std::uint8_t wiper_cmd =
                m_external_sim->HasReceivedWiperMotorCommand()
                    ? m_external_sim->GetWiperMotorCommand()
                    : 0u;
            m_wiper->Tick(tick_dt, wiper_cmd);
        }

        // --- Horn audio (external-sim-driven only in headless) ---
        bool horn_low = false, horn_high = false;
        if (m_external_sim->IsConnected()) {
            horn_low  = m_external_sim->GetHornLowCmd();
            horn_high = m_external_sim->GetHornHighCmd();
        }
        m_horn->SetTones(horn_low, horn_high);

        // --- Telemetry logging ---
        m_telemetry->Record(m_world->GetState(), tick_dt);

        // --- Realtime pacing (sim-time vs wall-time) ---
        if (m_config.simulation.realtime) {
            const auto target = wall_start +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(t));
            std::this_thread::sleep_until(target);
        }

        // --- Scripted-scenario complete ---
        if (m_scripted && m_scripted->IsDone()) {
            std::cout << "[SimApp] Scripted scenario complete at t="
                      << t << "s — exiting.\n";
            sigaction(SIGINT, &old_sa, nullptr);
            return kExitSuccess;
        }

        // --- Max-time exit ---
        if (max_time > 0.0 && t >= max_time) {
            const bool scripted_unfinished = m_scripted && !m_scripted->IsDone();
            if (scripted_unfinished) {
                std::cerr << "[SimApp] max_time_s reached with scripted "
                             "scenario still in phase '"
                          << m_scripted->PhaseName() << "' — timeout.\n";
                sigaction(SIGINT, &old_sa, nullptr);
                return kExitTimeout;
            }
            std::cout << "[SimApp] max_time_s reached — exiting.\n";
            sigaction(SIGINT, &old_sa, nullptr);
            return kExitSuccess;
        }
    }

    // Fell out of the loop -> SIGINT was the only possible cause.
    std::cout << "[SimApp] SIGINT — exiting.\n";
    sigaction(SIGINT, &old_sa, nullptr);
    return kExitInterrupted;
}
