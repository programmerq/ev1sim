#include "VehicleWorld.h"

#include "chrono/core/ChTypes.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/sedan/Sedan.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace chrono;
using namespace chrono::vehicle;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

VehicleWorld::VehicleWorld(const Config& config) {
    // Compute spawn pose.
    double yaw_rad = config.spawn.yaw_deg * M_PI / 180.0;
    m_spawn_pos = ChVector3d(config.spawn.x, config.spawn.y, config.spawn.z);
    m_spawn_rot = QuatFromAngleZ(yaw_rad);

    // Set Chrono data path so vehicle JSON definitions are found.
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Build vehicle.
    if (config.vehicle_model == "hmmwv")
        CreateHMMWV(config);
    else
        CreateSedan(config);

    // Driver bridge.
    m_driver = std::make_unique<CommandDriver>(*m_vehicle);

    // Terrain.
    CreateTerrain(config);

    std::cout << "[VehicleWorld] Initialised: model=" << config.vehicle_model
              << "  surface=" << config.terrain.surface
              << "  friction=" << config.terrain.friction << "\n";
}

VehicleWorld::~VehicleWorld() = default;

// ---------------------------------------------------------------------------
// Vehicle creation helpers
// ---------------------------------------------------------------------------

void VehicleWorld::CreateSedan(const Config& cfg) {
    using namespace chrono::vehicle::sedan;

    m_sedan = std::make_unique<Sedan>();
    m_sedan->SetContactMethod(ChContactMethod::SMC);
    m_sedan->SetChassisFixed(false);
    m_sedan->SetInitPosition(ChCoordsys<>(m_spawn_pos, m_spawn_rot));
    m_sedan->SetTireType(TireModelType::TMEASY);
    m_sedan->SetTireStepSize(cfg.simulation.step_size_s);
    m_sedan->Initialize();

    m_vehicle = &m_sedan->GetVehicle();
    m_system  = m_sedan->GetSystem();

    m_vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    m_vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetWheelVisualizationType(VisualizationType::MESH);
    m_vehicle->SetTireVisualizationType(VisualizationType::MESH);
}

void VehicleWorld::CreateHMMWV(const Config& cfg) {
    using namespace chrono::vehicle::hmmwv;

    m_hmmwv = std::make_unique<HMMWV_Full>();
    m_hmmwv->SetContactMethod(ChContactMethod::SMC);
    m_hmmwv->SetChassisFixed(false);
    m_hmmwv->SetInitPosition(ChCoordsys<>(m_spawn_pos, m_spawn_rot));
    m_hmmwv->SetEngineType(EngineModelType::SHAFTS);
    m_hmmwv->SetTransmissionType(TransmissionModelType::AUTOMATIC_SHAFTS);
    m_hmmwv->SetDriveType(DrivelineTypeWV::AWD);
    m_hmmwv->SetTireType(TireModelType::TMEASY);
    m_hmmwv->SetTireStepSize(cfg.simulation.step_size_s);
    m_hmmwv->Initialize();

    m_vehicle = &m_hmmwv->GetVehicle();
    m_system  = m_hmmwv->GetSystem();

    m_vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    m_vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    m_vehicle->SetWheelVisualizationType(VisualizationType::MESH);
    m_vehicle->SetTireVisualizationType(VisualizationType::MESH);
}

void VehicleWorld::CreateTerrain(const Config& cfg) {
    m_terrain = std::make_unique<RigidTerrain>(m_system);

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(static_cast<float>(cfg.terrain.friction));
    mat->SetRestitution(0.01f);

    auto patch = m_terrain->AddPatch(
        mat,
        ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT),
        cfg.terrain.length_m,
        cfg.terrain.width_m);

    // Apply a simple texture if available (non-fatal if missing).
    patch->SetTexture(GetDataFile("terrain/textures/tile4.jpg"), 200, 200);

    m_terrain->Initialize();
}

// ---------------------------------------------------------------------------
// Simulation stepping
// ---------------------------------------------------------------------------

void VehicleWorld::Synchronize(double time) {
    auto inputs = m_driver->GetInputs();
    m_driver->Synchronize(time);
    m_terrain->Synchronize(time);

    if (m_sedan)
        m_sedan->Synchronize(time, inputs, *m_terrain);
    else
        m_hmmwv->Synchronize(time, inputs, *m_terrain);
}

void VehicleWorld::Advance(double step) {
    m_driver->Advance(step);
    m_terrain->Advance(step);

    if (m_sedan)
        m_sedan->Advance(step);
    else
        m_hmmwv->Advance(step);
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void VehicleWorld::ResetVehicle() {
    auto body = m_vehicle->GetChassisBody();
    body->SetPos(m_spawn_pos);
    body->SetRot(m_spawn_rot);
    body->SetPosDt(ChVector3d(0, 0, 0));
    body->SetRotDt(ChQuaterniond(1, 0, 0, 0));
    body->SetPosDt2(ChVector3d(0, 0, 0));
    body->SetRotDt2(ChQuaterniond(1, 0, 0, 0));
    // Note: this resets the chassis but not individual wheel/suspension states.
    // Good enough for a quick respawn; a full reset would recreate the vehicle.
}

// ---------------------------------------------------------------------------
// State queries
// ---------------------------------------------------------------------------

double VehicleWorld::GetSimTime() const {
    return m_system->GetChTime();
}

VehiclePose VehicleWorld::GetPose() const {
    auto pos = m_vehicle->GetPos();
    auto rot = m_vehicle->GetRot();

    // Extract forward (X) and up (Z) directions from quaternion.
    ChVector3d fwd = rot.GetAxisX();
    ChVector3d up  = rot.GetAxisZ();

    return VehiclePose{
        pos.x(), pos.y(), pos.z(),
        fwd.x(), fwd.y(), fwd.z(),
        up.x(),  up.y(),  up.z(),
    };
}

VehicleState VehicleWorld::GetState() const {
    VehicleState s;
    s.sim_time = m_system->GetChTime();

    // Chassis motion
    s.speed_mps = m_vehicle->GetSpeed();
    s.yaw_rate  = m_vehicle->GetChassisBody()->GetAngVelLocal().z();
    s.roll_rate = m_vehicle->GetChassisBody()->GetAngVelLocal().x();

    // Pose
    auto pos = m_vehicle->GetPos();
    auto rot = m_vehicle->GetRot();
    s.pos_x   = pos.x();
    s.pos_y   = pos.y();
    s.pos_z   = pos.z();

    // Yaw from quaternion (rotation about Z).
    ChVector3d fwd = rot.GetAxisX();
    s.yaw_deg = std::atan2(fwd.y(), fwd.x()) * 180.0 / M_PI;

    // Chassis-frame acceleration at the reference point.
    ChVector3d acc_global = m_vehicle->GetChassisBody()->GetPosDt2();
    ChMatrix33<> A(rot);
    ChVector3d acc_local = A.transpose() * acc_global;
    s.accel_long = acc_local.x();
    s.accel_lat  = acc_local.y();
    s.accel_vert = acc_local.z();

    // Wheel angular speeds (assumes 2 axles, 2 wheels each).
    int n_axles = std::min(static_cast<int>(m_vehicle->GetNumberAxles()), 2);
    int idx = 0;
    for (int a = 0; a < n_axles && idx < 4; ++a) {
        auto axle = m_vehicle->GetAxle(a);
        for (int side = 0; side < 2 && idx < 4; ++side) {
            s.wheel_omega[idx++] = axle->m_wheels[side]->GetPos_dt();
        }
    }

    // Steering angle (first steering subsystem).
    if (m_vehicle->GetNumberSteeringMechanisms() > 0)
        s.steering_angle = m_vehicle->GetSteering(0);

    // Echo applied commands.
    auto& cmd = m_driver->GetCommand();
    s.applied_throttle    = cmd.throttle;
    s.applied_front_brake = cmd.front_brake;
    s.applied_rear_brake  = cmd.rear_brake;
    s.applied_steering    = cmd.steering;

    return s;
}
