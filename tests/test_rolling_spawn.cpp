// A car spawned rolling is a car already at speed, not one being launched.
//
// WHY THIS TEST EXISTS
// --------------------
// spawn.speed_mps lets a VAT case that is about braking or cruising start at
// its test speed instead of spending ~15 s of sim time accelerating there.
// That is only honest if the plant really starts at speed: chassis moving AND
// every wheel already spinning to match, so the first second is a coast, not a
// skid (wheels at rest under a moving car) or a launch (car at rest).  Chrono's
// WheeledVehicle::Initialize(pose, fwd_vel) is meant to set both; this pins
// that it does for the EV1 build, through the same builder the app uses.
//
// The car is dropped from the usual 0.5 m spawn height, so the first ~0.5 s
// includes the landing; the checks are taken after it settles.

#include <catch2/catch_test_macros.hpp>

#include "EV1Vehicle.h"
#include "SlipRatio.h"

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <algorithm>
#include <cmath>

using namespace chrono;
using namespace chrono::vehicle;

namespace {

constexpr double kStep = 1e-3;          // the app's step
constexpr double kTireRadius = 0.2915;  // EV1_TMeasyTire.json "Unloaded Radius [m]"
constexpr double kSpawnSpeed = 30.0;    // [m/s] the ABS high-mu entry speed
constexpr double kSettle = 1.0;         // [s] past the landing
constexpr double kEnd = 2.0;            // [s]

}  // namespace

TEST_CASE("Rolling spawn: the car starts at speed with its wheels already turning",
          "[plant][spawn]") {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);

    auto ev1 = ev1sim::BuildEV1Vehicle(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT),
                                       kStep, kSpawnSpeed);

    // Every wheel is spun up at creation, before the first step.
    for (int a = 0; a < 2; ++a)
        for (auto side : {LEFT, RIGHT}) {
            const double omega = ev1->GetSpindleOmega(a, side);
            INFO("axle " << a << " side " << side << " omega at t=0 = " << omega);
            CHECK(std::abs(std::abs(omega) * kTireRadius - kSpawnSpeed) < 0.05 * kSpawnSpeed);
        }

    RigidTerrain terrain(ev1->GetSystem());
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.01f);
    terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(200, 0, 0), QUNIT), 600.0, 100.0);
    terrain.Initialize();

    DriverInputs in{};  // coasting: no throttle, no brake
    double max_slip = 0.0;
    double speed_at_settle = 0.0;
    while (ev1->GetSystem()->GetChTime() < kEnd) {
        const double t = ev1->GetSystem()->GetChTime();
        terrain.Synchronize(t);
        ev1->Synchronize(t, in, terrain);
        terrain.Advance(kStep);
        ev1->Advance(kStep);
        if (t >= kSettle) {
            if (speed_at_settle == 0.0)
                speed_at_settle = ev1->GetSpeed();
            for (int a = 0; a < 2; ++a)
                for (auto side : {LEFT, RIGHT})
                    max_slip = std::max(max_slip, std::abs(ev1sim::SlipRatio::longitudinal(
                        ev1->GetSpeed(), ev1->GetSpindleOmega(a, side), kTireRadius)));
        }
    }

    INFO("speed at " << kSettle << " s = " << speed_at_settle
         << ", at " << kEnd << " s = " << ev1->GetSpeed());
    INFO("max |slip| after settle = " << max_slip);
    // Still at (nearly) the spawn speed: a coast loses well under 1 m/s here.
    CHECK(speed_at_settle > kSpawnSpeed - 1.0);
    CHECK(speed_at_settle <= kSpawnSpeed + 0.1);
    // And moving forward along +x, not skidding: free-rolling wheels.
    CHECK(ev1->GetChassis()->GetPos().x() > 0.9 * kSpawnSpeed * kEnd);
    CHECK(max_slip < 0.02);
}
