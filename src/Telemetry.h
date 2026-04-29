#pragma once

#include "ExternalSimConnector.h"
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

    // Call between BeginScene/EndScene.  `wheel_mu` holds the ground
    // friction coefficient under each wheel (FL, FR, RL, RR); values
    // <0 are skipped in the display.  Pass nullptr to omit the line.
    // `abs_phase` is optional — pass nullptr to omit the ABS line.
    void DrawHUD(irr::IrrlichtDevice* device,
                 const VehicleState& state,
                 const std::string& camera_mode,
                 const std::string& surface,
                 const double* wheel_mu,
                 const ExternalSimConnector::AbsPhaseFront* abs_phase = nullptr);

private:
    double m_log_interval;
    double m_accum = 0.0;
    bool   m_show_hud;

    std::ofstream m_file;
    bool          m_file_header_written = false;
};
