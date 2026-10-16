// Reverse: with the motor's torque pointed backward the car drives backward,
// and the signed motor shaft speed (what 4070 publishes) reads negative.  The
// sign follows the direction the shaft turns, not the direction the torque
// points: forward drive reads positive, and so does a car still rolling
// forward when R is selected.

#include <catch2/catch_test_macros.hpp>

#include "EV1Vehicle.h"

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <cmath>

using namespace chrono;
using namespace chrono::vehicle;

namespace {

constexpr double kStep = 1e-3;      // the app's step
constexpr double kSettle = 0.5;     // [s] past the landing from the spawn height
constexpr double kEnd = 3.0;        // [s]
constexpr double kThrottle = 0.3;

struct Run {
    double speed;           // chassis forward speed at kEnd [m/s]
    double x;               // chassis x at kEnd [m] (spawned at 0, heading +x)
    double motor_speed;     // SignedMotorShaftSpeed at kEnd [rad/s]
    double driveshaft;      // driveline output driveshaft speed at kEnd [rad/s]
};

Run Drive(ev1sim::MotorDirection dir, double spawn_speed = 0.0, double throttle = kThrottle,
          double end = kEnd) {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);

    auto ev1 = ev1sim::BuildEV1Vehicle(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT), kStep,
                                       spawn_speed);
    ev1sim::SetMotorDirection(*ev1, dir);

    RigidTerrain terrain(ev1->GetSystem());
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.01f);
    terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 200.0, 100.0);
    terrain.Initialize();

    while (ev1->GetSystem()->GetChTime() < end) {
        const double t = ev1->GetSystem()->GetChTime();
        DriverInputs in{};
        in.m_throttle = t >= kSettle ? throttle : 0.0;
        terrain.Synchronize(t);
        ev1->Synchronize(t, in, terrain);
        terrain.Advance(kStep);
        ev1->Advance(kStep);
    }
    return {ev1->GetSpeed(), ev1->GetChassis()->GetPos().x(),
            ev1sim::SignedMotorShaftSpeed(*ev1),
            ev1->GetDriveline()->GetOutputDriveshaftSpeed()};
}

}  // namespace

TEST_CASE("Reverse: the car drives backward and the motor speed reads negative",
          "[plant][reverse]") {
    const Run rev = Drive(ev1sim::MotorDirection::kReverse);
    INFO("reverse: speed " << rev.speed << " m/s, x " << rev.x << " m, motor "
         << rev.motor_speed << " rad/s, driveshaft " << rev.driveshaft << " rad/s");
    CHECK(rev.speed < -1.0);
    CHECK(rev.x < -1.0);
    CHECK(rev.motor_speed < 0.0);
    // No gearbox: the motor shaft turns with the driveshaft, one to one.
    CHECK(std::abs(rev.motor_speed - rev.driveshaft) < 1e-3 * std::abs(rev.driveshaft) + 1e-6);
}

TEST_CASE("Reverse: forward drive keeps a positive motor speed",
          "[plant][reverse]") {
    const Run fwd = Drive(ev1sim::MotorDirection::kForward);
    INFO("forward: speed " << fwd.speed << " m/s, x " << fwd.x << " m, motor "
         << fwd.motor_speed << " rad/s, driveshaft " << fwd.driveshaft << " rad/s");
    CHECK(fwd.speed > 1.0);
    CHECK(fwd.x > 1.0);
    CHECK(fwd.motor_speed > 0.0);
    CHECK(std::abs(fwd.motor_speed - fwd.driveshaft) < 1e-3 * std::abs(fwd.driveshaft) + 1e-6);
}

TEST_CASE("Reverse: a car rolling forward in R still reads a positive motor speed",
          "[plant][reverse]") {
    const Run roll = Drive(ev1sim::MotorDirection::kReverse, /*spawn_speed=*/5.0,
                           /*throttle=*/0.0, /*end=*/1.0);
    INFO("rolling in R: speed " << roll.speed << " m/s, motor " << roll.motor_speed
         << " rad/s, driveshaft " << roll.driveshaft << " rad/s");
    REQUIRE(roll.speed > 1.0);
    CHECK(roll.motor_speed > 0.0);
    CHECK(std::abs(roll.motor_speed - roll.driveshaft) < 1e-3 * std::abs(roll.driveshaft) + 1e-6);
}
