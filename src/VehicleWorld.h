#pragma once

#include "CommandDriver.h"
#include "Config.h"
#include "VehiclePose.h"
#include "VehicleState.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

// Forward declarations for vehicle model wrappers
namespace chrono::vehicle::sedan { class Sedan; }
namespace chrono::vehicle::hmmwv { class HMMWV_Full; }

#include <memory>
#include <string>
#include <vector>

// Owns the Chrono physics system, vehicle model, terrain, and driver bridge.
// Provides framework-agnostic state queries for telemetry and camera.
class VehicleWorld {
public:
    explicit VehicleWorld(const Config& config);
    ~VehicleWorld();

    // Chrono Synchronize-Advance protocol (call in this order each step).
    void Synchronize(double time);
    void Advance(double step);

    // Teleport vehicle back to spawn.  Resets velocities.
    void ResetVehicle();

    // Accessors
    double       GetSimTime() const;
    VehicleState GetState() const;
    VehiclePose  GetPose() const;

    CommandDriver&                       GetDriver()  { return *m_driver; }
    chrono::vehicle::ChWheeledVehicle&   GetVehicle() { return *m_vehicle; }
    chrono::vehicle::RigidTerrain&       GetTerrain() { return *m_terrain; }
    chrono::ChSystem&                    GetSystem()  { return *m_system; }

private:
    void CreateSedan(const Config& cfg);
    void CreateHMMWV(const Config& cfg);
    void CreateTerrain(const Config& cfg);
    void LoadLevelFile(const std::string& level_file, Config& cfg);

    // One of these is populated depending on vehicle_model.
    std::unique_ptr<chrono::vehicle::sedan::Sedan>      m_sedan;
    std::unique_ptr<chrono::vehicle::hmmwv::HMMWV_Full> m_hmmwv;

    // Non-owning convenience pointers into the active model.
    chrono::vehicle::ChWheeledVehicle* m_vehicle = nullptr;
    chrono::ChSystem*                  m_system  = nullptr;

    std::unique_ptr<CommandDriver>                m_driver;
    std::unique_ptr<chrono::vehicle::RigidTerrain> m_terrain;

    // Spawn pose for reset.
    chrono::ChVector3d     m_spawn_pos;
    chrono::ChQuaterniond  m_spawn_rot;

    // Level file patch definitions (populated by LoadLevelFile).
    struct LevelPatch {
        std::string mesh_file;
        std::string surface;
        double      friction = 0.9;
    };
    std::vector<LevelPatch> m_level_patches;
};
