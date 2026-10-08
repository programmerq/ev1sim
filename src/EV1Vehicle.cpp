#include "EV1Vehicle.h"

#include "Aerodynamics.h"
#include "EV1TMeasyTire.h"

#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace chrono;
using namespace chrono::vehicle;

namespace ev1sim {

namespace {

// Spin every rotating part up to match a car rolling at fwd_speed_mps.
//
// Chrono 9's WheeledVehicle::Initialize(pose, fwd_vel) moves the chassis
// body only.  Every other body (suspension links, uprights, spindles,
// steering) starts at rest, so the first constraint solve shares the
// chassis's momentum out and the car drops from 30 to ~27.8 m/s in one step;
// and every wheel and shaft starts unspun, so the first contact is a
// four-wheel skid that sheds a further ~2.5 m/s (both measured with
// tests/test_rolling_spawn.cpp before this existed).  So every moving body
// gets the chassis's forward velocity, and the spin is set with the sign
// conventions measured off a normal forward launch of this same build:
//   spindle body:    +w about its local y (the axle)
//   axle shafts:     -w (each suspension's ChShaft, both axles)
//   differential box:-w (open differential, both driven axles at -w)
//   driveshaft:      +w / conical ratio (motor side of the 10.946:1 reduction)
// The two driveline shafts are private to ChShaftsDriveline2WD, so they are
// found among the system's shafts by the inertias the driveline JSON gives
// them; a JSON that made them indistinguishable fails loudly.
void SpinUpRotatingParts(WheeledVehicle& ev1, double fwd_speed_mps,
                         double wheel_radius_m, const std::string& driveline_json) {
    const double w = fwd_speed_mps / wheel_radius_m;

    const ChVector3d fwd = ev1.GetChassisBody()->GetRot().GetAxisX() * fwd_speed_mps;
    for (auto& body : ev1.GetSystem()->GetBodies())
        if (!body->IsFixed())
            body->SetPosDt(fwd);

    std::vector<ChShaft*> axle_shafts;
    for (auto& axle : ev1.GetAxles())
        for (auto side : {LEFT, RIGHT}) {
            axle->m_suspension->GetSpindle(side)->SetAngVelLocal(ChVector3d(0, w, 0));
            auto shaft = axle->m_suspension->GetAxle(side);
            shaft->SetPosDt(-w);
            axle_shafts.push_back(shaft.get());
        }

    std::ifstream f(driveline_json);
    const auto j = nlohmann::json::parse(f, nullptr, true, true);
    const double j_drive = j.at("Shaft Inertia").at("Driveshaft").get<double>();
    const double j_diff  = j.at("Shaft Inertia").at("Differential Box").get<double>();
    const double conical = j.at("Gear Ratio").at("Conical Gear").get<double>();
    if (j_drive == j_diff)
        throw std::runtime_error(
            "rolling spawn: driveshaft and differential box share an inertia in " +
            driveline_json + "; cannot tell them apart to spin them up");

    int found = 0;
    for (auto& shaft : ev1.GetSystem()->GetShafts()) {
        if (std::find(axle_shafts.begin(), axle_shafts.end(), shaft.get()) != axle_shafts.end())
            continue;
        if (shaft->GetInertia() == j_drive) {
            shaft->SetPosDt(w / conical);
            ++found;
        } else if (shaft->GetInertia() == j_diff) {
            shaft->SetPosDt(-w);
            ++found;
        }
    }
    if (found != 2)
        throw std::runtime_error("rolling spawn: expected the driveshaft and differential "
                                 "box among the system's shafts, matched " +
                                 std::to_string(found));
}

}  // namespace

std::unique_ptr<WheeledVehicle> BuildEV1Vehicle(const ChCoordsys<>& pose,
                                                double step_size_s,
                                                double fwd_speed_mps) {
    const std::string vehicle_json = GetDataFile("ev1/vehicle/EV1_Vehicle.json");
    const std::string engine_json  = GetDataFile("ev1/powertrain/EV1_EngineSimpleMap.json");
    const std::string trans_json   = GetDataFile("ev1/powertrain/EV1_AutomaticTransmissionSimpleMap.json");
    const std::string tire_json    = GetDataFile("ev1/tire/EV1_TMeasyTire.json");
    const std::string driveline_json = GetDataFile("ev1/driveline/EV1_Driveline2WD.json");

    // Create the vehicle from JSON (without powertrain/tires — attached below).
    auto ev1 = std::make_unique<WheeledVehicle>(vehicle_json, ChContactMethod::SMC, false, false);
    ev1->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ev1->Initialize(pose, fwd_speed_mps);

    // --- Body aerodynamics (Round 4) -------------------------------------
    // The EV1's signature 0.19 Cd was previously lumped into tire dissipation;
    // apply it explicitly to the chassis instead.  ChChassis::SetAerodynamicDrag
    // applies F = 0.5·rho·Cd·A·v² at the COM opposing chassis velocity each
    // Synchronize.  The constants (and the matching formula the unit tests pin)
    // live in Aerodynamics.h.  NOTE: now that drag is no longer baked into the
    // tire model, tire rolling/slip dissipation may want a small downward
    // recalibration to keep top speed / coastdown honest.
    ev1->GetChassis()->SetAerodynamicDrag(Aerodynamics::kEV1DragCoefficient,
                                          Aerodynamics::kEV1FrontalAreaM2,
                                          Aerodynamics::kAirDensityIsaSeaLevel);

    // Powertrain: engine + single-speed transmission.
    auto engine       = ReadEngineJSON(engine_json);
    auto transmission = ReadTransmissionJSON(trans_json);
    ev1->InitializePowertrain(
        chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission));

    // Tires on all wheels: TMeasy plus first-order longitudinal tire dynamics
    // (EV1TMeasyTire.h says why the plain Chrono 9 TMeasy rings at 1 ms).
    for (auto& axle : ev1->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadEV1TireJSON(tire_json);
            ev1->InitializeTire(tire, wheel, VisualizationType::MESH);
            tire->SetStepsize(step_size_s);
        }
    }

    if (fwd_speed_mps != 0.0) {
        const double radius = ev1->GetAxle(0)->GetWheel(LEFT)->GetTire()->GetRadius();
        SpinUpRotatingParts(*ev1, fwd_speed_mps, radius, driveline_json);
    }
    return ev1;
}

}  // namespace ev1sim
