// Front ride rate and ride frequency, measured on the plant.
//
// The car settles at rest on a flat plane.  A vertical load is then held on
// the chassis directly over the front axle, so it all goes into that axle,
// and released.  Checked against what the front suspension file gives:
//   * the wheel rate is within 5 % of the coil-over's rate through its
//     motion ratio, both measured in the run, so no other part of the
//     linkage adds or takes rate;
//   * after release the front rings within 10 % of the ride frequency that
//     wheel rate and the corner's sprung mass give;
//   * and the ring decays at a damping ratio between 0.6 and 1.1 times the
//     one the damper rate gives through the same motion ratio.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "EV1Vehicle.h"

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

using namespace chrono;
using namespace chrono::vehicle;
using Catch::Matchers::WithinRel;

namespace {

constexpr double kStep = 1e-3;     // the app's step
constexpr double kLoadN = 2000.0;  // held over the front axle, 1000 N per side
constexpr double kLoadOn = 3.0;    // [s] settled at rest by then
constexpr double kRelease = 6.0;   // [s] settled under the load by then
constexpr double kEnd = 9.0;       // [s]
constexpr double kG = 9.81;
constexpr double kPi = 3.14159265358979323846;

nlohmann::json ReadVehicleJson(const std::string& rel) {
    std::ifstream f(vehicle::GetDataFile(rel));
    return nlohmann::json::parse(f, nullptr, true, true);
}

}  // namespace

TEST_CASE("Ride: the front wheel rate, ride frequency and damping are the front file's",
          "[plant][ride]") {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);

    const auto front = ReadVehicleJson("ev1/suspension/EV1_FrontDoubleWishbone.json");
    const double k_spring = front.at("Spring").at("Spring Coefficient").get<double>();
    const double c_damper = front.at("Shock").at("Damping Coefficient").get<double>();
    const double x_front = ReadVehicleJson("ev1/vehicle/EV1_Vehicle.json")
                               .at("Axles")[0].at("Suspension Location")[0].get<double>();

    auto ev1 = ev1sim::BuildEV1Vehicle(ChCoordsys<>(ChVector3d(0, 0, 0.40), QUNIT), kStep, 0.0);
    RigidTerrain terrain(ev1->GetSystem());
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 100.0, 100.0);
    terrain.Initialize();

    auto chassis = ev1->GetChassisBody();
    const std::string susp = ev1->GetAxle(0)->m_suspension->GetName();
    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    for (auto& l : ev1->GetSystem()->GetLinks())
        if (auto t = std::dynamic_pointer_cast<ChLinkTSDA>(l))
            if (t->GetName() == susp + "_spring_L" || t->GetName() == susp + "_spring_R")
                springs.push_back(t);
    REQUIRE(springs.size() == 2);

    // Spindle height in the chassis frame, mean of the two sides: the
    // suspension's own travel, whatever the body does.
    auto travel = [&] {
        double s = 0.0;
        for (auto side : {LEFT, RIGHT})
            s += chassis->TransformPointParentToLocal(ev1->GetSpindlePos(0, side)).z();
        return s / 2.0;
    };
    auto spring_len = [&] { return (springs[0]->GetLength() + springs[1]->GetLength()) / 2.0; };
    auto spring_force = [&] { return (springs[0]->GetForce() + springs[1]->GetForce()) / 2.0; };

    double z_rest = 0, len_rest = 0, force_rest = 0, z_loaded = 0, len_loaded = 0;
    std::vector<std::pair<double, double>> ring;  // (t, travel) after release
    DriverInputs in{};
    while (ev1->GetSystem()->GetChTime() < kEnd) {
        const double t = ev1->GetSystem()->GetChTime();
        if (t < kLoadOn) {
            z_rest = travel(); len_rest = spring_len(); force_rest = spring_force();
        } else if (t < kRelease) {
            z_loaded = travel(); len_loaded = spring_len();
        } else {
            ring.emplace_back(t, travel());
        }
        terrain.Synchronize(t);
        ev1->Synchronize(t, in, terrain);
        terrain.Advance(kStep);
        chassis->EmptyAccumulators();
        if (t >= kLoadOn && t < kRelease)
            chassis->AccumulateForce(ChVector3d(0, 0, -kLoadN),
                                     ev1->GetChassis()->GetPointLocation(ChVector3d(x_front, 0, 0)),
                                     false);
        ev1->Advance(kStep);
    }

    // Static: rate and motion ratio, both measured.
    const double travel_m = z_loaded - z_rest;  // the spindle rises toward the body
    const double motion_ratio = (len_rest - len_loaded) / travel_m;
    const double wheel_rate = kLoadN / 2.0 / travel_m;
    const double wheel_rate_from_spring = k_spring * motion_ratio * motion_ratio;
    // The corner's sprung mass is what the spring holds up at rest, at the wheel.
    const double corner_mass = force_rest * motion_ratio / kG;
    const double f_pred = std::sqrt(wheel_rate_from_spring / corner_mass) / (2.0 * kPi);
    const double zeta_pred = c_damper * motion_ratio * motion_ratio /
                             (2.0 * std::sqrt(wheel_rate_from_spring * corner_mass));

    // Free ring: period from successive upward crossings of the height it
    // settles back to (the last half second); damping from the log decrement
    // over one full period.
    double z_end = 0.0;
    int n_end = 0;
    for (const auto& [t, z] : ring)
        if (t > kEnd - 0.5) { z_end += z; ++n_end; }
    z_end /= n_end;
    std::vector<double> ups;
    std::vector<double> peaks;  // |excursion| at each local extremum
    for (size_t i = 1; i + 1 < ring.size(); ++i) {
        const double a = ring[i - 1].second - z_end, b = ring[i].second - z_end,
                     c = ring[i + 1].second - z_end;
        if (a < 0.0 && b >= 0.0) ups.push_back(ring[i].first);
        if (std::abs(b) >= std::abs(a) && std::abs(b) > std::abs(c) && std::abs(b) > 2e-4)
            peaks.push_back(std::abs(b));
    }
    REQUIRE(ups.size() >= 3);
    REQUIRE(peaks.size() >= 3);
    const double f_meas = (ups.size() - 1) / (ups.back() - ups.front());
    const double delta = std::log(peaks[0] / peaks[2]);
    const double zeta_meas = delta / std::sqrt(4.0 * kPi * kPi + delta * delta);

    INFO("travel " << travel_m * 1000 << " mm, motion ratio " << motion_ratio
         << ", wheel rate " << wheel_rate << " N/m vs spring through it " << wheel_rate_from_spring
         << "; corner mass " << corner_mass << " kg; ride " << f_meas << " Hz vs " << f_pred
         << "; damping ratio " << zeta_meas << " vs " << zeta_pred);
    CHECK_THAT(wheel_rate, WithinRel(wheel_rate_from_spring, 0.05));
    CHECK_THAT(f_meas, WithinRel(f_pred, 0.10));
    // The ring is the car's coupled heave-and-pitch mode, with the tyre in
    // series, so it decays a little slower than the corner alone would.
    CHECK(zeta_meas > 0.6 * zeta_pred);
    CHECK(zeta_meas < 1.1 * zeta_pred);
}
