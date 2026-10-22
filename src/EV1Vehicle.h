#pragma once

#include "chrono/core/ChCoordsys.h"
#include "chrono_vehicle/ChTerrain.h"
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

/// What SettleAtRest calls "at rest": chassis speed, vertical speed and pitch
/// rate all under these, held for kSettleHoldS.
inline constexpr double kSettleSpeedMps   = 0.002;
inline constexpr double kSettlePitchRadS  = 0.002;
inline constexpr double kSettleHoldS      = 0.2;
inline constexpr double kSettleMaxS       = 10.0;

/// Let a car created at rest drop onto its tyres and stop moving before
/// anything watches it: step it, unbraked, until it is at rest (above), then
/// stop every part and put the system clock back to 0.  A car spawned above its ride
/// height otherwise lands and pitches fore-aft for ~2 s, turning the motor
/// shaft back and forth while the electronics are already reading it.
/// Returns the sim time the settle took, or a negative value when the car was
/// still moving at kSettleMaxS (the clock is reset either way).
double SettleAtRest(chrono::vehicle::ChWheeledVehicle& vehicle,
                    chrono::vehicle::ChTerrain& terrain, double step_size_s);

}  // namespace ev1sim
