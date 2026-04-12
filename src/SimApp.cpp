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

    std::cout << "[SimApp] Ready.  Controls: WASD=drive  Space=park brake  "
                 "Q/R=respawn  C=camera  +/-=zoom  B=horn  O=hi  L=lo  Esc=quit\n";
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
    m_vis->AddTypicalLights();
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
        double zoom = m_keyboard->ConsumeZoomDelta();
        if (zoom != 0.0)
            m_camera->Zoom(zoom);
        if (m_keyboard->QuitRequested())
            break;

        // --- Physics sub-stepping ---
        for (int i = 0; i < steps_per_frame; ++i) {
            double t = m_world->GetSimTime();
            m_world->Synchronize(t);
            m_world->Advance(step);
        }

        // --- Visualisation sync/advance ---
        double t = m_world->GetSimTime();
        m_vis->Synchronize(t, m_world->GetDriver().GetInputs());
        m_vis->Advance(render_dt);

        // --- Camera override ---
        m_camera->Update(m_world->GetPose());

        // --- Render ---
        m_vis->BeginScene();
        macos_apply_viewport();   // override glViewport for Retina / resize
        m_vis->Render();
        m_telemetry->DrawHUD(m_vis->GetDevice(),
                             m_world->GetState(),
                             m_camera->GetModeName(),
                             m_config.terrain.surface);
        m_vis->EndScene();

        // --- Horn audio ---
        m_horn->SetTones(cmd.horn_low, cmd.horn_high);

        // --- Telemetry logging ---
        m_telemetry->Record(m_world->GetState(), render_dt);

        // --- Realtime pacing ---
        if (m_config.simulation.realtime)
            m_realtime_timer.Spin(step * steps_per_frame);
    }
}
