#include "VehicleWorld.h"

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChRotation.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_models/vehicle/sedan/Sedan.h"
#include "chrono_models/vehicle/hmmwv/HMMWV.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
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

VehicleWorld::VehicleWorld(const Config& config_in) {
    // Mutable copy — the level file may override spawn position.
    Config config = config_in;

    // If a level file is specified, load it first (overrides spawn + terrain).
    if (config.terrain.type == "level" && !config.terrain.level_file.empty())
        LoadLevelFile(config.terrain.level_file, config);

    // Compute spawn pose.
    double yaw_rad = config.spawn.yaw_deg * M_PI / 180.0;
    m_spawn_pos = ChVector3d(config.spawn.x, config.spawn.y, config.spawn.z);
    m_spawn_rot = QuatFromAngleZ(yaw_rad);

    // Set Chrono data paths so vehicle JSON definitions and meshes are found.
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(std::string(CHRONO_DATA_DIR) + "vehicle/");

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
              << "  terrain=" << config.terrain.type;
    if (config.terrain.type == "level")
        std::cout << "  level=" << config.terrain.level_file;
    else
        std::cout << "  surface=" << config.terrain.surface
                  << "  friction=" << config.terrain.friction;
    std::cout << "\n";
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

    if (cfg.terrain.type == "level" && !m_level_patches.empty()) {
        // ── Mesh-based level terrain ──
        // Resolve mesh paths relative to the level file's directory.
        std::string level_dir;
        auto slash = cfg.terrain.level_file.find_last_of("/\\");
        if (slash != std::string::npos)
            level_dir = cfg.terrain.level_file.substr(0, slash + 1);

        for (auto& lp : m_level_patches) {
            auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
            mat->SetFriction(static_cast<float>(lp.friction));
            mat->SetRestitution(0.01f);

            std::string mesh_path = level_dir + lp.mesh_file;
            std::cout << "[VehicleWorld] Loading patch: " << lp.surface
                      << "  mesh=" << mesh_path
                      << "  friction=" << lp.friction << "\n";

            auto patch = m_terrain->AddPatch(
                mat,
                ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT),
                mesh_path,
                true,   // connected mesh (better collision)
                0.0,    // sweep sphere radius
                true);  // visualization

            // Apply texture if specified.
            if (!lp.texture.empty()) {
                std::string tex_path = level_dir + lp.texture;
                float scale = static_cast<float>(lp.texture_scale);
                patch->SetTexture(tex_path, scale, scale);
            }
        }
    } else {
        // ── Flat box terrain (original default) ──
        auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
        mat->SetFriction(static_cast<float>(cfg.terrain.friction));
        mat->SetRestitution(0.01f);

        auto patch = m_terrain->AddPatch(
            mat,
            ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT),
            cfg.terrain.length_m,
            cfg.terrain.width_m);

        patch->SetTexture(GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    }

    m_terrain->Initialize();
}

// ---------------------------------------------------------------------------
// Level file loading
// ---------------------------------------------------------------------------

void VehicleWorld::LoadLevelFile(const std::string& level_file, Config& cfg) {
    std::ifstream f(level_file);
    if (!f.is_open()) {
        std::cerr << "[VehicleWorld] Cannot open level file: " << level_file
                  << " — falling back to flat terrain.\n";
        cfg.terrain.type = "rigid_plane";
        return;
    }

    nlohmann::json j;
    try {
        j = nlohmann::json::parse(f);
    } catch (const nlohmann::json::parse_error& e) {
        std::cerr << "[VehicleWorld] Level JSON error: " << e.what() << "\n";
        cfg.terrain.type = "rigid_plane";
        return;
    }

    // ── Spawn point (overrides config.spawn) ──
    if (j.contains("spawn")) {
        auto& sp = j["spawn"];
        if (sp.contains("x"))       cfg.spawn.x       = sp["x"].get<double>();
        if (sp.contains("y"))       cfg.spawn.y       = sp["y"].get<double>();
        if (sp.contains("z"))       cfg.spawn.z       = sp["z"].get<double>();
        if (sp.contains("yaw_deg")) cfg.spawn.yaw_deg = sp["yaw_deg"].get<double>();
        std::cout << "[VehicleWorld] Level spawn: ("
                  << cfg.spawn.x << ", " << cfg.spawn.y << ", " << cfg.spawn.z
                  << ")  yaw=" << cfg.spawn.yaw_deg << " deg\n";
    }

    // ── Terrain patches ──
    if (j.contains("patches")) {
        for (auto& p : j["patches"]) {
            LevelPatch lp;
            lp.mesh_file      = p.value("mesh", "");
            lp.surface        = p.value("surface", "unknown");
            lp.friction       = p.value("friction", 0.9);
            lp.texture        = p.value("texture", "");
            lp.texture_scale  = p.value("texture_scale", 10.0);

            if (lp.mesh_file.empty()) {
                std::cerr << "[VehicleWorld] Patch missing 'mesh' field — skipped.\n";
                continue;
            }
            m_level_patches.push_back(std::move(lp));
        }
    }

    std::cout << "[VehicleWorld] Level loaded: " << m_level_patches.size()
              << " patches from " << level_file << "\n";
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
    // Teleport the entire vehicle (chassis + wheels + suspension + steering)
    // by computing the rigid transform from current pose to spawn pose and
    // applying it to every non-fixed body in the system.
    auto chassis = m_vehicle->GetChassisBody();
    ChVector3d    old_pos = chassis->GetPos();
    ChQuaterniond old_rot = chassis->GetRot();

    // Rotation from old orientation to spawn orientation.
    ChQuaterniond rot_delta = m_spawn_rot * old_rot.GetInverse();

    for (auto& body : m_system->GetBodies()) {
        if (body->IsFixed())
            continue;

        // Rotate each body's offset from the old chassis position,
        // then translate to spawn.
        ChVector3d rel = body->GetPos() - old_pos;
        body->SetPos(m_spawn_pos + rot_delta.Rotate(rel));
        body->SetRot(rot_delta * body->GetRot());

        // Zero all velocities so the car is at rest.
        body->SetPosDt(ChVector3d(0, 0, 0));
        body->SetAngVelLocal(ChVector3d(0, 0, 0));
        body->SetPosDt2(ChVector3d(0, 0, 0));
    }
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
        s.wheel_omega[idx++] = m_vehicle->GetSpindleOmega(a, vehicle::LEFT);
        s.wheel_omega[idx++] = m_vehicle->GetSpindleOmega(a, vehicle::RIGHT);
    }

    // Steering angle (first axle, left side).
    if (n_axles > 0)
        s.steering_angle = m_vehicle->GetSteeringAngle(0, vehicle::LEFT);

    // Echo applied commands.
    auto& cmd = m_driver->GetCommand();
    s.applied_throttle    = cmd.throttle;
    s.applied_front_brake = cmd.front_brake;
    s.applied_rear_brake  = cmd.rear_brake;
    s.applied_steering    = cmd.steering;

    return s;
}
