#include "CommandDriver.h"

#include <algorithm>

CommandDriver::CommandDriver(chrono::vehicle::ChVehicle& vehicle)
    : ChDriver(vehicle) {}

void CommandDriver::SetCommand(const DriverCommand& cmd) {
    m_cmd = cmd;
}

void CommandDriver::Synchronize(double time) {
    m_throttle = m_cmd.throttle;
    m_steering = m_cmd.steering;

    // First pass: single brake channel fed by max of front/rear.
    // Future: apply per-axle torques directly through the brake subsystem.
    m_braking = std::max(m_cmd.front_brake, m_cmd.rear_brake);
}
