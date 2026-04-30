#pragma once

#include <cstdint>

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
/// Mesh path: no wiper mesh exists in the EV1 chassis model
/// (data/vehicle/ev1/ev1_chassis_vis.obj contains no "wiper" or "windshield"
/// object groups).  The implementation therefore uses a HUD-only fallback:
/// a small "WIPER" overlay block rendered below the lights snapshot panel.
///
/// TODO: When wiper meshes are added to the chassis model, call
///       WiperRenderer::FindWiperNode(smgr->getRootSceneNode()) and the class
///       will automatically rotate them instead of drawing the HUD bar.
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

    /// Non-owning pointer to the wiper scene-graph node, or null if absent.
    irr::scene::ISceneNode* m_wiper_node = nullptr;
};
