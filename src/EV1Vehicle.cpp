#include "EV1Vehicle.h"

#include "Aerodynamics.h"
#include "EV1TMeasyTire.h"

#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace ev1sim {

std::unique_ptr<WheeledVehicle> BuildEV1Vehicle(const ChCoordsys<>& pose,
                                                double step_size_s) {
    const std::string vehicle_json = GetDataFile("ev1/vehicle/EV1_Vehicle.json");
    const std::string engine_json  = GetDataFile("ev1/powertrain/EV1_EngineSimpleMap.json");
    const std::string trans_json   = GetDataFile("ev1/powertrain/EV1_AutomaticTransmissionSimpleMap.json");
    const std::string tire_json    = GetDataFile("ev1/tire/EV1_TMeasyTire.json");

    // Create the vehicle from JSON (without powertrain/tires — attached below).
    auto ev1 = std::make_unique<WheeledVehicle>(vehicle_json, ChContactMethod::SMC, false, false);
    ev1->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    ev1->Initialize(pose);

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
    return ev1;
}

}  // namespace ev1sim
