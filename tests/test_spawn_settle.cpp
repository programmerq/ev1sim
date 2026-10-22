// A car spawned at rest is at rest when the clock starts.
//
// Checks ev1sim::SettleAtRest, which VehicleWorld runs on every at-rest spawn:
// the car built at the app's 0.5 m spawn height comes back settled with the
// clock at 0, and over the next 3 s with no driver input the chassis stays
// still and the motor shaft never turns more than a fraction of a sensor pulse
// either way.  The control case runs the same car without the settle and
// checks it does rock, so the test can tell the two apart.  Runs with the
// plant tests on every build.

#include <catch2/catch_test_macros.hpp>

#include "EV1Vehicle.h"

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <algorithm>
#include <cmath>
#include <memory>

using namespace chrono;
using namespace chrono::vehicle;

namespace {

constexpr double kStep = 1e-3;   // the app's step
constexpr double kWatch = 3.0;   // [s] watched after the clock starts

// A turn of the motor shaft counts as a reversal once it has gone this far
// back from its last turning point: well under one speed-sensor pulse on any
// plausible tooth count, so a reversal the controller could see is counted.
constexpr double kReversalRad = 0.05;

struct Watch {
    double max_speed = 0.0;      // [m/s] |chassis speed|
    int reversals = 0;           // motor shaft direction changes
};

std::unique_ptr<RigidTerrain> FlatGround(ChSystem* sys) {
    auto terrain = std::make_unique<RigidTerrain>(sys);
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.01f);
    terrain->AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 200.0, 100.0);
    terrain->Initialize();
    return terrain;
}

Watch Coast(ChWheeledVehicle& ev1, RigidTerrain& terrain) {
    Watch w;
    DriverInputs in{};  // no throttle, no brake
    double angle = 0.0, turn = 0.0;
    int dir = 0;
    const double end = ev1.GetSystem()->GetChTime() + kWatch;
    while (ev1.GetSystem()->GetChTime() < end) {
        const double t = ev1.GetSystem()->GetChTime();
        terrain.Synchronize(t);
        ev1.Synchronize(t, in, terrain);
        terrain.Advance(kStep);
        ev1.Advance(kStep);
        w.max_speed = std::max(w.max_speed, std::abs(ev1.GetSpeed()));
        angle += ev1sim::SignedMotorShaftSpeed(ev1) * kStep;
        if (dir == 0) {
            if (std::abs(angle - turn) > kReversalRad) dir = angle > turn ? 1 : -1, turn = angle;
        } else if ((angle - turn) * dir > 0) {
            turn = angle;                       // still going the same way
        } else if (std::abs(angle - turn) > kReversalRad) {
            ++w.reversals;                      // came back far enough to count
            dir = -dir, turn = angle;
        }
    }
    return w;
}

std::unique_ptr<WheeledVehicle> SpawnAtRest() {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);
    return ev1sim::BuildEV1Vehicle(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT), kStep);
}

}  // namespace

TEST_CASE("Spawn settle: a car created at rest is still when the clock starts",
          "[plant][spawn]") {
    auto ev1 = SpawnAtRest();
    auto terrain = FlatGround(ev1->GetSystem());

    const double took = ev1sim::SettleAtRest(*ev1, *terrain, kStep);
    INFO("settle took " << took << " s");
    REQUIRE(took > 0.0);
    CHECK(ev1->GetSystem()->GetChTime() == 0.0);
    CHECK(std::abs(ev1->GetSpeed()) < ev1sim::kSettleSpeedMps);

    const Watch w = Coast(*ev1, *terrain);
    INFO("max |speed| " << w.max_speed << " m/s, shaft reversals " << w.reversals);
    CHECK(w.max_speed < 0.01);
    CHECK(w.reversals == 0);
}

TEST_CASE("Spawn settle: without it the dropped car rocks", "[plant][spawn]") {
    auto ev1 = SpawnAtRest();
    auto terrain = FlatGround(ev1->GetSystem());

    const Watch w = Coast(*ev1, *terrain);
    INFO("max |speed| " << w.max_speed << " m/s, shaft reversals " << w.reversals);
    CHECK(w.max_speed > 0.1);
    CHECK(w.reversals >= 2);
}
