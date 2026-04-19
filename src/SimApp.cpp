#include "SimApp.h"
#include "MacOSPlatform.h"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace chrono;
using namespace chrono::vehicle;

// ---------------------------------------------------------------------------
SimApp::SimApp(const Config& config) : m_config(config) {
    // 1. Physics world.
    m_world = std::make_unique<VehicleWorld>(m_config);

    // 2. Keyboard input.
    KeyboardInputController::Rates rates;
    rates.steer_rate        = m_config.input.steer_rate;
    rates.steer_return_rate = m_config.input.steer_return_rate;
    rates.throttle_rise     = m_config.input.throttle_rise_rate;
    rates.brake_rise        = m_config.input.brake_rise_rate;
    m_keyboard = std::make_unique<KeyboardInputController>(rates);

    // 3. Telemetry.
    m_telemetry = std::make_unique<Telemetry>(
        m_config.telemetry.log_rate_hz,
        m_config.telemetry.log_to_file,
        m_config.telemetry.log_file,
        m_config.telemetry.show_hud);

    // 4. Horn audio.
    m_horn = std::make_unique<HornAudio>();

    // 5. Visualization (creates window).
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

    m_lights_demo = m_config.lights.demo_mode;

    m_paused = m_config.start_paused;

    std::cout << "[SimApp] Ready.  Controls: WASD=drive  Space=park brake  "
                 "P=pause  R=respawn  C=camera  Scroll=zoom  B=horn  O=hi  L=lo  "
                 "H=headlights  F=hood  T=trunk  [=doorL  ]=doorR  Esc=quit\n";
    if (m_paused)
        std::cout << "[SimApp] Started PAUSED — press P to begin simulation\n";
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

    // Directional sun light with strong ambient so the whole map is lit
    // (AddTypicalLights only adds two point lights near the origin).
    m_vis->AddLightDirectional(60, 60,
        chrono::ChColor(0.8f, 0.8f, 0.8f),   // ambient
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
void SimApp::Run() {
    double step     = m_config.simulation.step_size_s;
    double render_dt = 1.0 / std::max(1, m_config.simulation.render_fps);
    int    steps_per_frame = std::max(1, static_cast<int>(std::round(render_dt / step)));

    while (m_vis->Run()) {
        // --- Input (once per render frame) ---
        DriverCommand cmd = m_keyboard->Update(render_dt);
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

        // --- External sim sync (publish panel sensors, drain bulb/horn cmds) ---
        for (int i = 0; i < VehiclePanels::NUM_PANELS; ++i) {
            m_external_sim->SetPanelSensor(static_cast<PanelID>(i),
                                           m_panels->IsOpen(static_cast<PanelID>(i)));
        }
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
        m_telemetry->DrawHUD(m_vis->GetDevice(),
                             m_world->GetState(),
                             m_camera->GetModeName(),
                             m_config.terrain.surface);
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
    }
}
