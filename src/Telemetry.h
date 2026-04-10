#pragma once

#include "VehicleState.h"

#include <irrlicht.h>

#include <fstream>
#include <string>

class Telemetry {
public:
    Telemetry(double log_rate_hz, bool log_to_file,
              const std::string& log_file, bool show_hud);
    ~Telemetry();

    // Call every render frame.
    void Record(const VehicleState& state, double dt);

    // Call between BeginScene/EndScene.
    void DrawHUD(irr::IrrlichtDevice* device,
                 const VehicleState& state,
                 const std::string& camera_mode,
                 const std::string& surface);

private:
    double m_log_interval;
    double m_accum = 0.0;
    bool   m_show_hud;

    std::ofstream m_file;
    bool          m_file_header_written = false;
};
