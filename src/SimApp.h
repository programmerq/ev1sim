#pragma once

#include "CameraManager.h"
#include "Config.h"
#include "ExternalSimConnector.h"
#include "HornAudio.h"
#include "KeyboardInputController.h"
#include "Telemetry.h"
#include "VehicleLights.h"
#include "VehiclePanels.h"
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
    std::unique_ptr<VehicleLights>          m_lights;
    std::unique_ptr<VehiclePanels>          m_panels;
    std::unique_ptr<ExternalSimConnector>   m_external_sim;

    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht> m_vis;
    chrono::ChRealtimeStepTimer m_realtime_timer;
    bool m_paused = false;

    // Headlight mode: 0=off, 1=low beam, 2=high beam
    int  m_headlight_mode = 0;
    // Demo pattern for the bulbs — "off", "blink", or "chase".  Kept in the
    // codebase for diagnostics; defaults off now that the electrical sim
    // drives the lamps.
    std::string m_lights_demo = "off";
};
