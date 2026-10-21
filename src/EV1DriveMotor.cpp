#include "EV1DriveMotor.h"

#include "chrono/utils/ChUtils.h"

namespace ev1sim {

using namespace chrono;
using namespace chrono::vehicle;

EV1DriveMotor::EV1DriveMotor(const std::string& filename)
    : EngineSimpleMap(filename) {}

void EV1DriveMotor::SetEngineTorqueMaps(ChFunctionInterp& map0,
                                        ChFunctionInterp& mapF) {
    EngineSimpleMap::SetEngineTorqueMaps(map0, mapF);
    m_zero_map = map0;
    m_full_map = mapF;
}

void EV1DriveMotor::Synchronize(double /*time*/,
                                const DriverInputs& driver_inputs,
                                double motorshaft_speed) {
    // Same clamp and blend as ChEngineSimpleMap::Synchronize.
    m_speed = ChClamp(motorshaft_speed, 0.0, GetMaxEngineSpeed());
    const double throttle = driver_inputs.m_throttle;
    const double zero = m_coast_torque_nm ? *m_coast_torque_nm
                                          : m_zero_map.GetVal(m_speed);
    m_torque = zero * (1.0 - throttle) + m_full_map.GetVal(m_speed) * throttle;
}

}  // namespace ev1sim
