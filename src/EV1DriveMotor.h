#pragma once

#include "chrono_vehicle/powertrain/EngineSimpleMap.h"

#include <optional>
#include <string>

namespace ev1sim {

/// The EV1 drive motor: Chrono's simple-map engine, read from the same JSON,
/// whose zero-pedal torque the propulsion controller can own.
///
/// @source:manual propulsion p57, COAST DOWN FUNCTION: with the pedal released
/// "the EV1 will coast freely like an automatic transmission in neutral", and
/// with coast-down selected "The PCM controls coast down by providing a
/// negative torque current".  So the zero-pedal torque is the PIM's coast-down
/// drag, which it publishes on CHASSIS_PIM_COAST_DRAG_NM.  While
/// SetCoastTorque() holds a value it replaces the map's "Map Zero Throttle";
/// otherwise the map stands in for the PIM (a run with no electronics, or
/// before the PIM publishes):
///
///     torque = Z * (1 - throttle) + F(speed) * throttle
///     Z      = the coast torque while the PIM owns it, else the coast map
///
/// The engine sees only the shaft speed's magnitude, so the caller signs the
/// torque against the shaft's rotation (ev1sim::SetCoastDrag in EV1Vehicle.h).
/// Speed clamping and the full-throttle map are Chrono's, unchanged.
class EV1DriveMotor : public chrono::vehicle::EngineSimpleMap {
  public:
    explicit EV1DriveMotor(const std::string& filename);

    /// Hand the zero-pedal torque to the PIM: `torque_nm` in the motor's own
    /// frame (negative brakes a shaft turning the way positive torque drives
    /// it) until ClearCoastTorque().
    void SetCoastTorque(double torque_nm) { m_coast_torque_nm = torque_nm; }
    void ClearCoastTorque() { m_coast_torque_nm.reset(); }
    std::optional<double> GetCoastTorque() const { return m_coast_torque_nm; }

    double GetMotorSpeed() const override { return m_speed; }
    double GetOutputMotorshaftTorque() const override { return m_torque; }

  private:
    void SetEngineTorqueMaps(chrono::ChFunctionInterp& map0,
                             chrono::ChFunctionInterp& mapF) override;
    void Synchronize(double time,
                     const chrono::vehicle::DriverInputs& driver_inputs,
                     double motorshaft_speed) override;

    chrono::ChFunctionInterp m_zero_map;
    chrono::ChFunctionInterp m_full_map;
    std::optional<double>    m_coast_torque_nm;
    double                   m_speed  = 0.0;
    double                   m_torque = 0.0;
};

}  // namespace ev1sim
