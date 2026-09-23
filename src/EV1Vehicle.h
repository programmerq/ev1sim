#pragma once

#include "chrono/core/ChCoordsys.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include <memory>

namespace ev1sim {

/// Assemble the EV1 plant from its JSON tree: chassis/suspension/driveline,
/// body aerodynamics, the motor + single-speed powertrain, and the EV1 tire on
/// every wheel (tire step = step_size_s).  The one place the plant is put
/// together, so the sim app (VehicleWorld) and the plant tests run the same
/// car.  vehicle::SetDataPath() must already point at the EV1 data root.
std::unique_ptr<chrono::vehicle::WheeledVehicle> BuildEV1Vehicle(
    const chrono::ChCoordsys<>& pose, double step_size_s);

}  // namespace ev1sim
