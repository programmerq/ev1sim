#pragma once

#include "chrono/core/ChCoordsys.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

#include <memory>
#include <optional>

namespace ev1sim {

/// Assemble the EV1 plant from its JSON tree: chassis/suspension/driveline,
/// body aerodynamics, the motor + single-speed powertrain, and the EV1 tire on
/// every wheel (tire step = step_size_s).  The one place the plant is put
/// together, so the sim app (VehicleWorld) and the plant tests run the same
/// car.  vehicle::SetDataPath() must already point at the EV1 data root.
/// fwd_speed_mps starts the car rolling along its heading (0 = at rest).
std::unique_ptr<chrono::vehicle::WheeledVehicle> BuildEV1Vehicle(
    const chrono::ChCoordsys<>& pose, double step_size_s,
    double fwd_speed_mps = 0.0);

/// Which way the drive motor turns its torque.  @source:manual propulsion
/// p250: "When shifted to reverse the motor torque direction is, counter
/// clockwise, causing the drive shafts to rotate in the reverse direction."
enum class MotorDirection { kForward, kReverse };

/// Point the motor's torque the way `dir` says.  The EV1 has no gearbox, so
/// this is the plant's only reversing element: Chrono's single-speed
/// transmission in its reverse gear (ratio -1).  No-op on a vehicle whose
/// transmission is not automatic.
void SetMotorDirection(chrono::vehicle::ChWheeledVehicle& vehicle, MotorDirection dir);

class EV1DriveMotor;

/// The vehicle's drive motor (EV1DriveMotor.h), or nullptr on a vehicle built
/// some other way.
EV1DriveMotor* DriveMotor(chrono::vehicle::ChWheeledVehicle& vehicle);

/// Below this shaft speed the coast drag is withheld: a regen drag opposes
/// rotation and cannot start a shaft turning.  The PIM's own drag is zero
/// below the p.209 regen cutoff (~996 rpm), far above this.
inline constexpr double kCoastDragDeadbandRadS = 1.0;

/// Give the PIM's coast-down drag (`drag_nm`, Nm at the shaft, >= 0) to the
/// drive motor as torque opposing the shaft's current rotation, or hand the
/// zero-pedal torque back to the motor map (nullopt).  Call once per step
/// before Synchronize.  No-op on a vehicle without an EV1DriveMotor.
void SetCoastDrag(chrono::vehicle::ChWheeledVehicle& vehicle,
                  std::optional<double> drag_nm);

/// Motor shaft speed in rad/s, signed: positive turning the car forward,
/// negative turning it backward, whichever way the torque points.  The
/// magnitude is the transmission's output-motorshaft speed (unclamped, unlike
/// the engine's); the sign is the driveshaft's direction of rotation, which
/// Chrono's simple-map transmission discards (std::abs).
double SignedMotorShaftSpeed(const chrono::vehicle::ChWheeledVehicle& vehicle);

}  // namespace ev1sim
