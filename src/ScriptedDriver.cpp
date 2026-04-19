#include "ScriptedDriver.h"

#include <iostream>

ScriptedDriver::ScriptedDriver(const Params& params) : m_params(params) {
    std::cout << "[ScriptedDriver] target="
              << (m_params.target_speed_mps * 3.6) << " km/h, hold="
              << m_params.hold_time_s << "s, stop<"
              << m_params.stop_threshold_mps << " m/s\n";
    std::cout << "[ScriptedDriver] Phase: Accel\n";
}

const char* ScriptedDriver::PhaseName() const {
    switch (m_phase) {
        case Phase::Accel: return "Accel";
        case Phase::Hold:  return "Hold";
        case Phase::Brake: return "Brake";
        case Phase::Done:  return "Done";
    }
    return "?";
}

void ScriptedDriver::Transition(Phase next, double sim_time) {
    m_phase = next;
    std::cout << "[ScriptedDriver] t=" << sim_time
              << "s  Phase -> " << PhaseName() << "\n";
}

DriverCommand ScriptedDriver::Update(const VehicleState& s) {
    DriverCommand cmd{};

    switch (m_phase) {
        case Phase::Accel:
            cmd.throttle = 1.0;
            if (s.speed_mps >= m_params.target_speed_mps) {
                m_hold_start_time = s.sim_time;
                Transition(Phase::Hold, s.sim_time);
            }
            break;

        case Phase::Hold: {
            // Bang-bang throttle to hold near target.  Good enough for CI —
            // we don't need a smooth cruise controller here.
            cmd.throttle = (s.speed_mps < m_params.target_speed_mps) ? 0.5 : 0.0;
            if (s.sim_time - m_hold_start_time >= m_params.hold_time_s)
                Transition(Phase::Brake, s.sim_time);
            break;
        }

        case Phase::Brake:
            cmd.front_brake = 1.0;
            cmd.rear_brake  = 1.0;
            if (s.speed_mps <= m_params.stop_threshold_mps)
                Transition(Phase::Done, s.sim_time);
            break;

        case Phase::Done:
            // All zeros.
            break;
    }

    return cmd;
}
