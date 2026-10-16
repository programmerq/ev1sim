// Ride rates, ride frequency and damping, measured on the plant.
//
// The car settles at rest on a flat plane.  A vertical load is held on the
// chassis directly over one axle, so it all goes into that axle, then
// released; the front first, then the rear.  For each axle:
//   * the wheel rate is the spring's rate through its motion ratio, both
//     measured in the run: within 5 % on the front, within 12 % on the rear,
//     where the upper link carries a share of the load as the axle winds up;
//   * after release it rings within 10 % of the body's heave-mode frequency,
//     from the two axles' wheel rates and the chassis file's mass, COM and
//     pitch inertia;
//   * and the ring decays at between 0.6 and 1.5 times that mode's damping
//     ratio, from the two dampers' rates through their motion ratios (the
//     ring carries some of the more damped pitch mode too, and the tyres sit
//     in series).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "EV1Vehicle.h"

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <algorithm>
#include <array>
#include <cctype>
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
constexpr double kLoadN = 2000.0;  // held over one axle, 1000 N per side
constexpr double kPhase = 3.0;     // [s] settle, load front, ring, load rear, ring
constexpr double kG = 9.81;
constexpr double kPi = 3.14159265358979323846;

nlohmann::json ReadVehicleJson(const std::string& rel) {
    std::ifstream f(vehicle::GetDataFile(rel));
    return nlohmann::json::parse(f, nullptr, true, true);
}

double NamedValue(const nlohmann::json& list, const std::string& name, const char* key) {
    for (const auto& t : list)
        if (t.at("Name").get<std::string>() == name) return t.at(key).get<double>();
    FAIL("no entry named " << name);
    return 0.0;
}

std::string Lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

// What one axle's static load and free ring measure.
struct AxleRide {
    double z_rest = 0, spring_rest = 0, damper_rest = 0, force_rest = 0;
    double z_loaded = 0, spring_loaded = 0, damper_loaded = 0;
    std::vector<std::pair<double, double>> ring;  // (t, travel) after release

    double travel() const { return z_loaded - z_rest; }  // the spindle rises toward the body
    double spring_mr() const { return (spring_rest - spring_loaded) / travel(); }
    double damper_mr() const { return (damper_rest - damper_loaded) / travel(); }
    double wheel_rate() const { return kLoadN / 2.0 / travel(); }
    // The corner's sprung mass is what the spring holds up at rest, at the wheel.
    double corner_mass() const { return force_rest * spring_mr() / kG; }

    // Frequency from upward crossings of the height the ring settles back to
    // (its last half second); damping from the log decrement over one period.
    void FreeRing(double& f, double& zeta) const {
        double z_end = 0.0;
        int n = 0;
        for (const auto& [t, z] : ring)
            if (t > ring.back().first - 0.5) { z_end += z; ++n; }
        z_end /= n;
        std::vector<double> ups, peaks;
        for (size_t i = 1; i + 1 < ring.size(); ++i) {
            const double a = ring[i - 1].second - z_end, b = ring[i].second - z_end,
                         c = ring[i + 1].second - z_end;
            if (a < 0.0 && b >= 0.0) ups.push_back(ring[i].first);
            if (std::abs(b) >= std::abs(a) && std::abs(b) > std::abs(c) && std::abs(b) > 2e-4)
                peaks.push_back(std::abs(b));
        }
        REQUIRE(ups.size() >= 3);
        REQUIRE(peaks.size() >= 3);
        f = (ups.size() - 1) / (ups.back() - ups.front());
        const double delta = std::log(peaks[0] / peaks[2]);
        zeta = delta / std::sqrt(4.0 * kPi * kPi + delta * delta);
    }
};

}  // namespace

TEST_CASE("Ride: each axle's wheel rate, ride frequency and damping are its file's",
          "[plant][ride]") {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);

    const auto front = ReadVehicleJson("ev1/suspension/EV1_FrontDoubleWishbone.json");
    const auto rear = ReadVehicleJson("ev1/suspension/EV1_RearFiveLink.json");
    const std::array<double, 2> k_spring = {
        front.at("Spring").at("Spring Coefficient").get<double>(),
        NamedValue(rear.at("TSDAs"), "Spring", "Spring Coefficient")};
    const std::array<double, 2> c_damper = {
        front.at("Shock").at("Damping Coefficient").get<double>(),
        NamedValue(rear.at("TSDAs"), "Shock", "Damping Coefficient")};
    const auto axles = ReadVehicleJson("ev1/vehicle/EV1_Vehicle.json").at("Axles");
    const std::array<double, 2> x_axle = {axles[0].at("Suspension Location")[0].get<double>(),
                                          axles[1].at("Suspension Location")[0].get<double>()};
    const auto body = ReadVehicleJson("ev1/chassis/EV1_Chassis.json").at("Components")[0];
    const double m_body = body.at("Mass").get<double>();
    const double i_pitch = body.at("Moments of Inertia")[1].get<double>();
    const double x_com = body.at("Centroidal Frame").at("Location")[0].get<double>();

    auto ev1 = ev1sim::BuildEV1Vehicle(ChCoordsys<>(ChVector3d(0, 0, 0.40), QUNIT), kStep, 0.0);
    RigidTerrain terrain(ev1->GetSystem());
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 100.0, 100.0);
    terrain.Initialize();
    auto chassis = ev1->GetChassisBody();

    // Each axle's spring and damper pair, by the suspension's own names.
    std::array<std::vector<std::shared_ptr<ChLinkTSDA>>, 2> springs, dampers;
    for (int a = 0; a < 2; ++a) {
        const std::string susp = Lower(ev1->GetAxle(a)->m_suspension->GetName());
        for (auto& l : ev1->GetSystem()->GetLinks())
            if (auto t = std::dynamic_pointer_cast<ChLinkTSDA>(l)) {
                const std::string n = Lower(t->GetName());
                if (n == susp + "_spring_l" || n == susp + "_spring_r") springs[a].push_back(t);
                if (n == susp + "_shock_l" || n == susp + "_shock_r") dampers[a].push_back(t);
            }
        REQUIRE(springs[a].size() == 2);
        REQUIRE(dampers[a].size() == 2);
    }
    // Spindle height in the chassis frame, mean of the two sides: the
    // suspension's own travel, whatever the body does.
    auto travel = [&](int a) {
        double s = 0.0;
        for (auto side : {LEFT, RIGHT})
            s += chassis->TransformPointParentToLocal(ev1->GetSpindlePos(a, side)).z();
        return s / 2.0;
    };
    auto mean_len = [](const auto& pair) { return (pair[0]->GetLength() + pair[1]->GetLength()) / 2.0; };

    std::array<AxleRide, 2> ride;
    DriverInputs in{};
    while (ev1->GetSystem()->GetChTime() < 5 * kPhase) {
        const double t = ev1->GetSystem()->GetChTime();
        const int phase = std::min(4, static_cast<int>(t / kPhase));
        const int a = phase < 2 ? 0 : (phase == 2 ? -1 : 1);
        if (phase == 0 || phase == 2) {  // at rest before each axle's load
            const int next = phase == 0 ? 0 : 1;
            auto& r = ride[next];
            r.z_rest = travel(next);
            r.spring_rest = mean_len(springs[next]);
            r.damper_rest = mean_len(dampers[next]);
            r.force_rest = (springs[next][0]->GetForce() + springs[next][1]->GetForce()) / 2.0;
        }
        if (phase == 1 || phase == 3) {
            auto& r = ride[a];
            r.z_loaded = travel(a);
            r.spring_loaded = mean_len(springs[a]);
            r.damper_loaded = mean_len(dampers[a]);
        }
        if (phase == 2) ride[0].ring.emplace_back(t, travel(0));
        if (phase == 4) ride[1].ring.emplace_back(t, travel(1));
        terrain.Synchronize(t);
        ev1->Synchronize(t, in, terrain);
        terrain.Advance(kStep);
        chassis->EmptyAccumulators();
        if (phase == 1 || phase == 3)
            chassis->AccumulateForce(ChVector3d(0, 0, -kLoadN),
                                     ev1->GetChassis()->GetPointLocation(ChVector3d(x_axle[a], 0, 0)),
                                     false);
        ev1->Advance(kStep);
    }

    // The body's heave mode, bouncing and pitching about its COM on the two
    // axles' wheel rates as the springs give them; its damping ratio by modal
    // projection of the dampers' rates.
    std::array<double, 2> k_axle{}, c_axle{}, arm{};
    for (int a = 0; a < 2; ++a) {
        k_axle[a] = 2.0 * k_spring[a] * ride[a].spring_mr() * ride[a].spring_mr();
        c_axle[a] = 2.0 * c_damper[a] * ride[a].damper_mr() * ride[a].damper_mr();
        arm[a] = x_axle[a] - x_com;
    }
    auto matrix = [&](const std::array<double, 2>& v) {  // [zz, zp, pp]
        return std::array<double, 3>{v[0] + v[1], -(v[0] * arm[0] + v[1] * arm[1]),
                                     v[0] * arm[0] * arm[0] + v[1] * arm[1] * arm[1]};
    };
    const auto K = matrix(k_axle), C = matrix(c_axle);
    const double qa = m_body * i_pitch, qb = -(K[0] * i_pitch + K[2] * m_body),
                 qc = K[0] * K[2] - K[1] * K[1];
    const double lambda = (-qb - std::sqrt(qb * qb - 4 * qa * qc)) / (2 * qa);
    const double phi_z = K[1], phi_p = lambda * m_body - K[0];
    const double modal_m = m_body * phi_z * phi_z + i_pitch * phi_p * phi_p;
    const double modal_c = C[0] * phi_z * phi_z + 2 * C[1] * phi_z * phi_p + C[2] * phi_p * phi_p;
    const double f_heave = std::sqrt(lambda) / (2.0 * kPi);
    const double zeta_heave = modal_c / (2.0 * std::sqrt(lambda) * modal_m);

    const std::array<double, 2> rate_tol = {0.05, 0.12};
    for (int a = 0; a < 2; ++a) {
        const auto& r = ride[a];
        double f = 0, zeta = 0;
        r.FreeRing(f, zeta);
        const double from_spring = k_spring[a] * r.spring_mr() * r.spring_mr();
        INFO((a == 0 ? "front" : "rear") << ": travel " << r.travel() * 1000 << " mm, motion ratio "
             << r.spring_mr() << ", wheel rate " << r.wheel_rate() << " N/m vs spring through it "
             << from_spring << "; corner mass " << r.corner_mass() << " kg; ring " << f
             << " Hz vs heave " << f_heave << "; damping ratio " << zeta << " vs heave "
             << zeta_heave << "; damper motion ratio " << r.damper_mr());
        CHECK_THAT(r.wheel_rate(), WithinRel(from_spring, rate_tol[a]));
        CHECK_THAT(f, WithinRel(f_heave, 0.10));
        CHECK(zeta > 0.6 * zeta_heave);
        CHECK(zeta < 1.5 * zeta_heave);
    }
}
