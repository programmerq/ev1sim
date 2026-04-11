#pragma once

#include "CameraManager.h"
#include "Config.h"
#include "HornAudio.h"
#include "KeyboardInputController.h"
#include "Telemetry.h"
#include "VehicleWorld.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include <memory>

class SimApp {
public:
    explicit SimApp(const Config& config);
    ~SimApp();

    // Blocking — runs until the user closes the window or presses Esc.
    void Run();

private:
    void SetupVisualization();

    Config m_config;

    std::unique_ptr<VehicleWorld>           m_world;
    std::unique_ptr<KeyboardInputController> m_keyboard;
    std::unique_ptr<CameraManager>          m_camera;
    std::unique_ptr<Telemetry>              m_telemetry;
    std::unique_ptr<HornAudio>              m_horn;

    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> m_vis;
    chrono::ChRealtimeStepTimer m_realtime_timer;
};
