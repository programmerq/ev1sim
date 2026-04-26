#include "CommandDriver.h"

#include <algorithm>

using chrono::vehicle::LEFT;
using chrono::vehicle::RIGHT;

CommandDriver::CommandDriver(chrono::vehicle::ChWheeledVehicle& vehicle)
    : ChDriver(vehicle), m_wheeled(vehicle) {}

void CommandDriver::SetCommand(const DriverCommand& cmd) {
    m_cmd = cmd;
}

void CommandDriver::Synchronize(double time) {
    m_throttle = m_cmd.throttle;
    m_steering = m_cmd.steering;
    // Zero the unified braking channel.  Per-axle actuation is applied in
    // ApplyBrakes() after the vehicle's own Synchronize() runs, so zeroing
    // here prevents Chrono from doing a redundant unified-braking pass.
    m_braking = 0.0;
}

void CommandDriver::Advance(double step) {
    m_actuator.Advance(step, m_cmd.front_brake, m_cmd.rear_brake);
}

void CommandDriver::ApplyBrakes(double time) {
    const int num_axles = static_cast<int>(m_wheeled.GetNumberAxles());

    // Axle 0 = front — hydraulic disc, driven by actual pressure.
    if (num_axles > 0) {
        m_wheeled.GetBrake(0, LEFT )->Synchronize(time, m_actuator.front_pressure);
        m_wheeled.GetBrake(0, RIGHT)->Synchronize(time, m_actuator.front_pressure);
    }

    // Axle 1 = rear — electric drum, driven by actuator position.
    if (num_axles > 1) {
        m_wheeled.GetBrake(1, LEFT )->Synchronize(time, m_actuator.rear_position);
        m_wheeled.GetBrake(1, RIGHT)->Synchronize(time, m_actuator.rear_position);
    }
}
