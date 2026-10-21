// The drive motor's zero-pedal torque is the PIM's coast-down drag while the
// PIM owns it (EV1DriveMotor.h, ev1sim::SetCoastDrag).  Steps the real EV1
// build at speed with the pedal released three ways (coast map, PIM drag 0,
// PIM drag 12 Nm) and checks the shaft torque each delivers and the order the
// car slows in, then holds a drag on a car at rest and checks it does not roll
// back.  Runs with the plant suite, ~11 s of simulated time in all.

#include <catch2/catch_test_macros.hpp>

#include "EV1DriveMotor.h"
#include "EV1Vehicle.h"

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <cmath>
#include <optional>

using namespace chrono;
using namespace chrono::vehicle;

namespace {

constexpr double kStep = 1e-3;
constexpr double kSpawnSpeed = 25.0;  // [m/s], well above the p.209 regen cutoff
constexpr double kEnd = 3.0;          // [s]

struct CoastRun {
    double speed_loss_mps = 0.0;  // speed at 1 s minus speed at kEnd
    double motor_torque_nm = 0.0;  // shaft torque on the last step
};

CoastRun Coast(std::optional<double> pim_drag_nm) {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);
    auto ev1 = ev1sim::BuildEV1Vehicle(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT),
                                       kStep, kSpawnSpeed);
    auto* motor = ev1sim::DriveMotor(*ev1);
    REQUIRE(motor != nullptr);

    RigidTerrain terrain(ev1->GetSystem());
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.01f);
    terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(200, 0, 0), QUNIT), 600.0, 100.0);
    terrain.Initialize();

    DriverInputs in{};  // pedal released, no brake
    CoastRun r;
    double speed_at_1s = 0.0;
    while (ev1->GetSystem()->GetChTime() < kEnd) {
        const double t = ev1->GetSystem()->GetChTime();
        ev1sim::SetCoastDrag(*ev1, pim_drag_nm);
        terrain.Synchronize(t);
        ev1->Synchronize(t, in, terrain);
        terrain.Advance(kStep);
        ev1->Advance(kStep);
        if (speed_at_1s == 0.0 && t >= 1.0) speed_at_1s = ev1->GetSpeed();
    }
    r.speed_loss_mps = speed_at_1s - ev1->GetSpeed();
    r.motor_torque_nm = motor->GetOutputMotorshaftTorque();
    return r;
}

}  // namespace

TEST_CASE("Coast drag: the PIM's drag replaces the coast map at zero pedal",
          "[plant][coast]") {
    const CoastRun map = Coast(std::nullopt);
    const CoastRun free_coast = Coast(0.0);
    const CoastRun drag = Coast(12.0);

    INFO("map: loss " << map.speed_loss_mps << " m/s, torque " << map.motor_torque_nm);
    INFO("PIM 0 Nm: loss " << free_coast.speed_loss_mps << " m/s, torque "
         << free_coast.motor_torque_nm);
    INFO("PIM 12 Nm: loss " << drag.speed_loss_mps << " m/s, torque "
         << drag.motor_torque_nm);

    CHECK(map.motor_torque_nm < 0.0);           // the map's own coast regen
    CHECK(free_coast.motor_torque_nm == 0.0);   // button off: coasts freely
    CHECK(drag.motor_torque_nm == -12.0);       // exactly the PIM's drag
    CHECK(drag.speed_loss_mps > free_coast.speed_loss_mps);
    CHECK(map.speed_loss_mps > free_coast.speed_loss_mps);
}

TEST_CASE("Coast drag: a car at rest is not driven backward", "[plant][coast]") {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);
    auto ev1 = ev1sim::BuildEV1Vehicle(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT), kStep);
    auto* motor = ev1sim::DriveMotor(*ev1);
    REQUIRE(motor != nullptr);
    RigidTerrain terrain(ev1->GetSystem());
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.01f);
    terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 100.0, 100.0);
    terrain.Initialize();
    const double x0 = ev1->GetPos().x();
    DriverInputs in{};  // no pedal, no brake
    while (ev1->GetSystem()->GetChTime() < 2.0) {
        const double t = ev1->GetSystem()->GetChTime();
        ev1sim::SetCoastDrag(*ev1, 12.0);  // a stale drag at rest
        terrain.Synchronize(t);
        ev1->Synchronize(t, in, terrain);
        terrain.Advance(kStep);
        ev1->Advance(kStep);
    }
    INFO("x moved " << ev1->GetPos().x() - x0 << " m");
    // 12 Nm through the 10.95:1 reduction would roll the car back metres.
    CHECK(ev1->GetPos().x() - x0 > -0.05);
}
