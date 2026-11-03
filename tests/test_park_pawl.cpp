// The park pawl: engaged, it holds a car rolling at walking pace still; released,
// the same car keeps rolling.  Drives the real EV1 plant (BuildEV1Vehicle).

#include <catch2/catch_test_macros.hpp>

#include "EV1Vehicle.h"

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <cmath>

using namespace chrono;
using namespace chrono::vehicle;

namespace {

constexpr double kStep = 1e-3;   // the app's step
constexpr double kEnd = 2.0;     // [s]

// Spawn rolling at 1 m/s, the pawl engaged or not from the start; return the
// chassis speed and the motor shaft speed at kEnd.
std::pair<double, double> Roll(bool pawl_engaged) {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);

    auto ev1 = ev1sim::BuildEV1Vehicle(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT), kStep, 1.0);
    ev1sim::SetParkPawlEngaged(*ev1sim::ParkPawlCarrier(*ev1), pawl_engaged);

    RigidTerrain terrain(ev1->GetSystem());
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.01f);
    terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 200.0, 100.0);
    terrain.Initialize();

    while (ev1->GetSystem()->GetChTime() < kEnd) {
        const double t = ev1->GetSystem()->GetChTime();
        DriverInputs in{};
        terrain.Synchronize(t);
        ev1->Synchronize(t, in, terrain);
        terrain.Advance(kStep);
        ev1->Advance(kStep);
    }
    return {ev1->GetSpeed(), ev1sim::SignedMotorShaftSpeed(*ev1)};
}

}  // namespace

TEST_CASE("Park pawl: engaged it holds the car and the motor shaft still", "[plant][park]") {
    const auto [speed, shaft] = Roll(true);
    INFO("speed " << speed << " m/s, motor shaft " << shaft << " rad/s");
    CHECK(std::abs(speed) < 0.02);
    CHECK(std::abs(shaft) < 0.5);
}

TEST_CASE("Park pawl: released, the car keeps rolling", "[plant][park]") {
    const auto [speed, shaft] = Roll(false);
    INFO("speed " << speed << " m/s, motor shaft " << shaft << " rad/s");
    CHECK(speed > 0.3);
}
