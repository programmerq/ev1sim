#pragma once

#include "CommandDriver.h"
#include "Config.h"
#include "VehiclePose.h"
#include "VehicleState.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"

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

    // HUD-facing label reflecting the actual terrain in use.  For a
    // successfully loaded level this is "level: <stem>"; if the level
    // file was missing/invalid and we fell back to a rigid plane, this
    // is "rigid_plane (fallback)" so the HUD doesn't lie.
    const std::string& GetTerrainLabel() const { return m_terrain_label; }

private:
    void CreateEV1(const Config& cfg);
    void CreateSedan(const Config& cfg);
    void CreateHMMWV(const Config& cfg);
    void CreateTerrain(const Config& cfg);
    void LoadLevelFile(const std::string& level_file, Config& cfg);

    // One of these is populated depending on vehicle_model.
    std::unique_ptr<chrono::vehicle::WheeledVehicle>    m_ev1;
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
        enum class Kind { Mesh, Plane };
        Kind        kind           = Kind::Mesh;
        std::string mesh_file;                   // Kind::Mesh
        double      center_x       = 0.0;        // Kind::Plane
        double      center_y       = 0.0;
        double      center_z       = 0.0;
        double      size_l         = 0.0;        // length along X (m)
        double      size_w         = 0.0;        // width  along Y (m)
        std::string surface;
        double      friction       = 0.9;
        std::string texture;                     // path relative to level JSON
        double      texture_scale  = 10.0;
    };
    std::vector<LevelPatch> m_level_patches;

    // Truthful HUD label computed after LoadLevelFile / CreateTerrain.
    std::string m_terrain_label;
};
