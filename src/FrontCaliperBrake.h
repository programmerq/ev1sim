#pragma once

// The front calipers' brake command while the BTCM is alive: each corner's
// share of the front brake's Maximum Torque, taken from the caliper pressure
// the BTCM publishes for that corner (chassis 4153/4154,
// ExternalSimConnector::GetFrontWheelCylinderPressuresKpa).
//
// The BTCM's wheel-hydraulics model is the one front pressure model: its
// firmware reads the same numbers through the LF/RF downstream sensors, so
// the car brakes on exactly the pressure the controller senses.  The model
// lives in electricsim ev1/btcm/wheel_hydraulics_model.hpp.
//
// Torque is proportional to caliper pressure, with one gain shared with the
// BTCM's front regen blend (electricsim btcm_front_regen_blend.h
// BTCM_FRONT_AXLE_NM_PER_KPA = 2 x 1120 / 3448): the brake's Maximum Torque
// (data/vehicle/ev1/brake/EV1_BrakeSimple_Front.json) is reached at the
// upstream sensors' full scale, which is also the ceiling of the BTCM's
// caliper model.
// @inferred: full pedal reaches the upstream sensors' 0-3448 kPa full scale
//   (brakes-310); no page prints the front caliper's torque per kPa.

#include <algorithm>

#include "PhysicalWorld.h"

namespace ev1sim {

/// The caliper pressure at which a front brake gives its Maximum Torque.
inline constexpr double kFrontCaliperFullTorqueKpa = BrakePedal::kSensorFullScaleKpa;

/// A corner's brake command (0..1 of Maximum Torque) for a caliper pressure.
inline double front_brake_command_for_kpa(double caliper_kpa) {
    return std::clamp(caliper_kpa / kFrontCaliperFullTorqueKpa, 0.0, 1.0);
}

}  // namespace ev1sim
