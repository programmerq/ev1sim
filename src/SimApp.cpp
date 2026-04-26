#include "SimApp.h"
#include "MacOSPlatform.h"

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
                     "H=headlights  F=hood  T=trunk  [=doorL  ]=doorR  Esc=quit\n";
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
        if (m_keyboard->ConsumeHeadlightToggle()) {
            // H disables any running demo and cycles the headlamps.
            if (m_lights_demo != "off") {
                std::cout << "[SimApp] Lights demo OFF (was " << m_lights_demo << ")\n";
                m_lights_demo = "off";
            }
            m_headlight_mode = (m_headlight_mode + 1) % 3;
            const char* names[] = {"OFF", "LOW BEAM", "HIGH BEAM"};
            std::cout << "[SimApp] Headlamps: " << names[m_headlight_mode] << "\n";
        }

        // --- Physics sub-stepping (skipped when paused) ---
        if (!m_paused) {
            for (int i = 0; i < steps_per_frame; ++i) {
                double t = m_world->GetSimTime();
                m_world->Synchronize(t);
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

        // --- External sim sync (publish panel sensors + dynamics, drain bulb/horn cmds) ---
        for (int i = 0; i < VehiclePanels::NUM_PANELS; ++i) {
            m_external_sim->SetPanelSensor(static_cast<PanelID>(i),
                                           m_panels->IsOpen(static_cast<PanelID>(i)));
        }
        m_external_sim->SetVehicleState(m_world->GetState());
        m_external_sim->Tick(t);

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
            // Demo off, no external sim — all bulbs default off, H drives headlamps.
            for (int i = 0; i < NUM_LIGHTS; ++i)
                m_lights->SetState(static_cast<LightID>(i), false);
            bool low  = (m_headlight_mode >= 1);
            bool high = (m_headlight_mode >= 2);
            m_lights->SetState(LightID::LLBH, low);
            m_lights->SetState(LightID::RLBH, low);
            m_lights->SetState(LightID::LHBH, high);
            m_lights->SetState(LightID::RHBH, high);
        }

        m_lights->ApplyToScene();

        // --- Render ---
        m_vis->BeginScene();
        macos_apply_viewport();   // override glViewport for Retina / resize
        m_vis->Render();
        // Ask the world for the truthful terrain label — it reflects the
        // actual loaded terrain, including the rigid-plane fallback when
        // a requested level file was missing or invalid.
        auto mu = m_world->GetWheelFrictions();
        m_telemetry->DrawHUD(m_vis->GetDevice(),
                             m_world->GetState(),
                             m_camera->GetModeName(),
                             m_world->GetTerrainLabel(),
                             mu.mu);
        m_lights->DrawHUD(m_vis->GetDevice());
        m_panels->DrawHUD(m_vis->GetDevice());

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
    // driver is configured.
    if (!m_scripted)
        m_world->GetDriver().SetCommand(DriverCommand{});

    const auto wall_start = std::chrono::steady_clock::now();

    while (!g_stop_requested.load(std::memory_order_relaxed)) {
        // --- Scripted driver (if any) — reads previous-tick state and
        //     emits a new command before we step physics.
        if (m_scripted) {
            DriverCommand cmd = m_scripted->Update(m_world->GetState());
            m_world->GetDriver().SetCommand(cmd);
        }

        // --- Physics sub-stepping (no pause control in headless) ---
        for (int i = 0; i < steps_per_tick; ++i) {
            const double t = m_world->GetSimTime();
            m_world->Synchronize(t);
            m_world->Advance(step);
        }

        const double t = m_world->GetSimTime();

        // --- External sim sync (panel sensors + dynamics, bulb/horn cmds) ---
        for (int i = 0; i < VehiclePanels::NUM_PANELS; ++i) {
            m_external_sim->SetPanelSensor(
                static_cast<PanelID>(i),
                m_panels->IsOpen(static_cast<PanelID>(i)));
        }
        m_external_sim->SetVehicleState(m_world->GetState());
        m_external_sim->Tick(t);

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
