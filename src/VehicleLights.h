#pragma once

#include <irrlicht.h>

#include <array>

/// Individual bulb identifiers from the EV1 electrical system manual.
/// Each ID represents a single physical bulb that the electronic
/// simulator can control independently.
enum class LightID : int {
    LHBH = 0,  // Left High Beam Headlamp
    LLBH,       // Left Low Beam Headlamp
    RHBH,       // Right High Beam Headlamp
    RLBH,       // Right Low Beam Headlamp
    LFTS,       // Left Front Turn Signal
    RFTS,       // Right Front Turn Signal
    LFML,       // Left Front Marker Lamp
    RFML,       // Right Front Marker Lamp
    LRSL,       // Left Rear Stop Lamp
    RRSL,       // Right Rear Stop Lamp
    LRTL,       // Left Rear Tail Lamp  (low-intensity running light)
    RRTL,       // Right Rear Tail Lamp (low-intensity running light)
    LRTS,       // Left Rear Turn Signal
    RRTS,       // Right Rear Turn Signal
    LRSM,       // Left Rear Side Marker
    RRSM,       // Right Rear Side Marker
    CHMSL,      // Center High Mount Stop Lamp
    LBL,        // Left Backup Lamp
    RBL,        // Right Backup Lamp
    COUNT
};

inline constexpr int NUM_LIGHTS = static_cast<int>(LightID::COUNT);

/// Returns a short human-readable name for a light (e.g. "LHBH").
const char* LightIDName(LightID id);

// ---------------------------------------------------------------------------
/// Manages the visual representation of vehicle lights via Irrlicht materials.
///
/// The chassis mesh has per-light material groups (headlamp_left, stoplamp_right,
/// etc.).  Multiple LightIDs can map to the same material group (e.g. LHBH and
/// LLBH both affect the headlamp_left lens).  The group's emissive colour is
/// computed from the combined state of its constituent bulbs.
///
/// Call Initialize() once after the Irrlicht scene is built, then
/// ApplyToScene() every frame before rendering.
class VehicleLights {
public:
    VehicleLights();

    /// Walk the Irrlicht scene graph to find the chassis mesh node and
    /// build a mapping from lamp groups to mesh buffer indices.
    /// Returns true if the chassis node was found and mapped.
    bool Initialize(irr::scene::ISceneManager* smgr);

    /// Set/get the on/off state of an individual bulb.
    void SetState(LightID id, bool on);
    bool GetState(LightID id) const;

    /// Demo mode — blink every light at a slightly different frequency
    /// so you can visually verify each lamp group works independently.
    void UpdateDemoMode(double sim_time);

    /// Chase demo — illuminate one bulb at a time, stepping clockwise
    /// around the vehicle starting at the CHMSL.  Useful for verifying
    /// that each LightID maps to the expected physical lamp.  Each bulb
    /// stays on for ~0.2 s, then advances.
    void UpdateChaseDemo(double sim_time);

    /// Push current light states to the Irrlicht mesh materials.
    /// Call once per frame, before BeginScene.
    void ApplyToScene();

    /// Draw a top-down vehicle silhouette with bulb status indicators.
    /// Call between BeginScene/EndScene.
    void DrawHUD(irr::IrrlichtDevice* device) const;

    /// The chassis scene node (may be null before initialization).
    irr::scene::ISceneNode* GetChassisNode() const { return m_chassis_node; }

    bool IsInitialized() const { return m_chassis_node != nullptr; }

private:
    // A lamp group is a set of mesh faces sharing one Irrlicht material
    // buffer.  Multiple LightIDs map to the same group when their bulbs
    // are behind a single shared lens (e.g. high + low beam headlamp).
    enum class LampGroup : int {
        HEADLAMP_HI_LEFT = 0,
        HEADLAMP_HI_RIGHT,
        HEADLAMP_LO_LEFT,
        HEADLAMP_LO_RIGHT,
        SIGNAL_FRONT_TURN_LEFT,
        SIGNAL_FRONT_TURN_RIGHT,
        SIGNAL_FRONT_MARKER_LEFT,
        SIGNAL_FRONT_MARKER_RIGHT,
        STOPLAMP_LEFT,
        STOPLAMP_RIGHT,
        SIGNAL_REAR_LEFT,
        SIGNAL_REAR_RIGHT,
        SIDE_MARKER_REAR_LEFT,
        SIDE_MARKER_REAR_RIGHT,
        BACKUP_LEFT,
        BACKUP_RIGHT,
        CHMSL,
        COUNT
    };
    static constexpr int NUM_GROUPS = static_cast<int>(LampGroup::COUNT);

    static const char* GroupName(LampGroup g);

    // Which lamp group does a given LightID control?
    static LampGroup GroupFor(LightID id);

    struct GroupInfo {
        int buffer_index = -1;  // Index into chassis IMesh buffers (-1 = not mapped)
    };

    // Per-light state
    bool m_state[NUM_LIGHTS] = {};

    // Per-group mesh buffer mapping
    GroupInfo m_groups[NUM_GROUPS];

    // Cached Irrlicht node
    irr::scene::ISceneNode* m_chassis_node = nullptr;

    // Demo blink frequencies (Hz) per light — each slightly different
    // so every lamp group blinks at its own cadence.
    static constexpr double BLINK_FREQ[NUM_LIGHTS] = {
        0.50, 0.55,   // LHBH, LLBH  (headlamps L high/low)
        0.62, 0.67,   // RHBH, RLBH  (headlamps R high/low)
        0.74, 0.80,   // LFTS, RFTS  (front turn signals)
        0.87, 0.93,   // LFML, RFML  (front marker lamps)
        1.00, 1.07,   // LRSL, RRSL  (rear stop lamps)
        1.14, 1.20,   // LRTL, RRTL  (rear tail lamps)
        1.27, 1.35,   // LRTS, RRTS  (rear turn signals)
        1.42, 1.50,   // LRSM, RRSM  (rear side markers)
        1.57,          // CHMSL
        1.65, 1.73     // LBL, RBL   (backup lamps)
    };

    // Compute the emissive colour for a lamp group given current states.
    irr::video::SColor ComputeGroupEmissive(LampGroup group) const;

    // Scene graph helpers
    void FindChassisMeshNode(irr::scene::ISceneNode* node);
    void BuildBufferMapping();

    // Material classification for buffer mapping
    enum class GlassType { NONE, CLEAR, ORANGE, RED };
    static GlassType ClassifyGlass(const irr::video::SMaterial& mat);
};
