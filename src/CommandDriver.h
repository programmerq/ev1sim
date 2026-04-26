#pragma once

#include "BrakeActuator.h"
#include "DriverCommand.h"

#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

/// Bridges our framework-agnostic DriverCommand to Chrono's ChDriver
/// interface, and models the independent brake actuator dynamics for the
/// two axles:
///
///   Front (axle 0) — Hydraulic disc caliper
///     Brake-fluid pressure follows a first-order lag:
///       τ_apply  ≈  50 ms  (master cylinder compresses fluid)
///       τ_release ≈  80 ms (return spring bleeds pressure)
///
///   Rear  (axle 1) — Electrically actuated drum
///     A threaded-rod leadscrew driven by a small DC motor limits how fast
///     the shoes can engage or retract.
///       max rate ≈ 3.33 /s  (full 0→1 stroke in ~300 ms)
///
/// ApplyBrakes() must be called after ChWheeledVehicle::Synchronize() each
/// step so the per-axle Chrono brake subsystems are driven by the actuated
/// pressures rather than the unified m_braking value Chrono would apply.
class CommandDriver : public chrono::vehicle::ChDriver {
public:
    explicit CommandDriver(chrono::vehicle::ChWheeledVehicle& vehicle);

    void SetCommand(const DriverCommand& cmd);
    const DriverCommand& GetCommand() const { return m_cmd; }

    /// Actual brake actuator state after dynamics are applied.
    /// Read by VehicleWorld::GetState() for telemetry and bus publishing.
    const BrakeActuator& GetActuatorState() const { return m_actuator; }

    /// Apply per-axle brake Synchronize() calls with the actuated pressure
    /// and position values.  Call AFTER ChWheeledVehicle::Synchronize() so
    /// these override the unified m_braking the vehicle would have applied.
    void ApplyBrakes(double time);

    // ChDriver overrides
    void Synchronize(double time) override;
    void Advance(double step) override;

private:
    chrono::vehicle::ChWheeledVehicle& m_wheeled;
    DriverCommand  m_cmd;
    BrakeActuator  m_actuator;
};
