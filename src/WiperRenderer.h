#pragma once

#include <cstdint>
#include <optional>

// Forward declarations to avoid pulling in Irrlicht headers.
namespace irr {
    class IrrlichtDevice;
    namespace scene { class ISceneNode; }
}

/// Animates the windshield wiper sweep based on the motor-command signal
/// received from RHJB (ID 4080, kSigChassisWiperMotorCommand).
///
/// Motor command enum:
///   0 = OFF   — no animation
///   1 = INT   — ~10 wipes/min (one sweep every ~6 s)
///   2 = LOW   — ~45 wipes/min
///   3 = HIGH  — ~85 wipes/min
///
/// A "wipe" is one full out-and-back arc modelled as one revolution (2π) of
/// the internal phase accumulator.  The wiper angle exposed to the scene graph
/// (or HUD) is `sin(phase) × max_angle_deg`, which naturally produces the
/// symmetric left↔right sweep without any extra bookkeeping.
///
/// Visuals: a small "WIPER" HUD block above the lights snapshot panel, and
/// the wipers on the windshield, which WiperBlades3D draws from
/// GetSweepFraction() (the chassis mesh has no wiper parts, so FindWiperNode()
/// finds none).
class WiperRenderer {
public:
    /// Wiper motor command enum — mirrors kSigChassisWiperMotorCommand values.
    enum class Mode : std::uint8_t {
        OFF  = 0,
        INT  = 1,
        LOW  = 2,
        HIGH = 3,
    };

    WiperRenderer();

    // -----------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------
    /// Maximum sweep angle in degrees (one side from centre).
    /// Default: 60°, i.e. the blade sweeps ±60° from its park position.
    static constexpr float kDefaultMaxAngleDeg = 60.0f;

    /// Cadences (wipes per minute) per mode.
    static constexpr double kCadenceInt  = 10.0;   // ~6 s per wipe
    static constexpr double kCadenceLow  = 45.0;   // ~1.33 s per wipe
    static constexpr double kCadenceHigh = 85.0;   // ~0.71 s per wipe

    // -----------------------------------------------------------------
    // Runtime
    // -----------------------------------------------------------------
    /// Update the phase accumulator by dt seconds using the current motor
    /// command.  Call once per simulation tick before DrawHUD() / ApplyToScene().
    void Tick(double dt_s, std::uint8_t motor_cmd_raw);

    /// Convenience overload that takes the Mode enum directly.
    void Tick(double dt_s, Mode mode);

    /// The blade position the RHJB's wiper-motor plant publishes
    /// (CHASSIS_WIPER_SWEEP_POSITION, percent of one cycle, 0 = park, 50 = end
    /// of sweep), or nullopt before it has. When set, GetSweepFraction() and
    /// the HUD follow it instead of this class's own phase, so the park return
    /// after OFF shows as the plant runs it.
    void SetPlantSweepPosition(std::optional<std::uint8_t> position_pct) {
        m_plant_position_pct = position_pct;
    }

    /// Sweep fraction (0 = park, 1 = far end) from a plant position: percent
    /// of one full cycle, 0 = park, 50 = end of sweep, back to park at 100.
    static double SweepFromPlantPosition(std::uint8_t position_pct);

    /// Where the blades are: 0 = park, 1 = far end of the sweep. The plant's
    /// position when published, else (1 - cos(phase)) / 2.
    double GetSweepFraction() const;

    /// Attempt to find a wiper scene-graph node.  Returns true if found.
    /// When a node is found, ApplyToScene() rotates it instead of drawing HUD.
    bool FindWiperNode(irr::scene::ISceneNode* root);

    /// Apply the current wiper angle to the scene graph (if a mesh node was
    /// found) or do nothing (HUD path handles the visual).
    void ApplyToScene();

    /// Draw the WIPER HUD overlay block.  Should be called from the same render
    /// context as VehicleLights::DrawHUD().
    void DrawHUD(irr::IrrlichtDevice* device) const;

    // -----------------------------------------------------------------
    // Accessors (primarily for tests)
    // -----------------------------------------------------------------
    Mode   GetMode()     const { return m_mode; }
    double GetPhaseRad() const { return m_phase_rad; }

    /// Current wiper angle in degrees (sin(phase) × max_angle).
    float  GetAngleDeg() const;

    /// True if a wiper mesh node was found in the scene graph.
    bool   HasMeshNode() const { return m_wiper_node != nullptr; }

private:
    /// Returns cadence in wipes/minute for the given mode.
    static double CadenceForMode(Mode mode);

    Mode   m_mode      = Mode::OFF;
    double m_phase_rad = 0.0;
    float  m_max_angle_deg = kDefaultMaxAngleDeg;
    std::optional<std::uint8_t> m_plant_position_pct;

    /// Non-owning pointer to the wiper scene-graph node, or null if absent.
    irr::scene::ISceneNode* m_wiper_node = nullptr;
};
