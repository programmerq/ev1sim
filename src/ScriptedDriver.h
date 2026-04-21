#pragma once

#include "DriverCommand.h"
#include "VehicleState.h"

// Minimal state-machine driver for CI scenarios.
//
// Phases (run in order, one-way transitions):
//   Accel  — full throttle until speed reaches target_speed_mps
//   Hold   — keep speed near target for hold_time_s
//   Brake  — full brake until speed falls below stop_threshold_mps
//   Done   — zero command; SimApp::RunHeadless will exit
//
// The driver is stateless w.r.t. the physics engine — it only reads
// VehicleState and produces DriverCommand.
class ScriptedDriver {
public:
    struct Params {
        double target_speed_mps   = 10.0;
        double hold_time_s        = 1.0;
        double stop_threshold_mps = 0.1;
    };

    enum class Phase { Accel, Hold, Brake, Done };

    explicit ScriptedDriver(const Params& params);

    // Produce the next driver command given the latest VehicleState.
    DriverCommand Update(const VehicleState& state);

    bool   IsDone()     const { return m_phase == Phase::Done; }
    Phase  CurrentPhase() const { return m_phase; }
    const char* PhaseName() const;

private:
    void Transition(Phase next, double sim_time);

    Params m_params;
    Phase  m_phase           = Phase::Accel;
    double m_hold_start_time = -1.0;
};
