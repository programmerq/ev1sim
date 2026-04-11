#pragma once

#include "DriverCommand.h"

#include "chrono_vehicle/ChDriver.h"

// Bridge between our framework-agnostic DriverCommand and Chrono's
// ChDriver interface.  VehicleWorld passes this to Chrono's
// Synchronize/Advance calls so the physics engine sees a proper driver.
class CommandDriver : public chrono::vehicle::ChDriver {
public:
    explicit CommandDriver(chrono::vehicle::ChVehicle& vehicle);

    void SetCommand(const DriverCommand& cmd);
    const DriverCommand& GetCommand() const { return m_cmd; }

    // ChDriver overrides
    void Synchronize(double time) override;
    void Advance(double step) override {}

private:
    DriverCommand m_cmd;
};
