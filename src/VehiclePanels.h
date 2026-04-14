#pragma once

#include <irrlicht.h>

/// Identifiers for each movable body panel.
enum class PanelID : int {
    HOOD = 0,
    TRUNK,
    DOOR_LEFT,
    DOOR_RIGHT,
    COUNT  // = 4
};

/// Tracks open/closed state of body panels (hood, trunk, doors).
/// Each panel maps to a physical sensor in the electronic harness.
/// No geometry or animation — just state and HUD indicators.
class VehiclePanels {
public:
    VehiclePanels();

    void Toggle(PanelID panel);
    bool IsOpen(PanelID panel) const;

    /// Draw panel status indicators on the top-down vehicle HUD.
    /// Call between BeginScene/EndScene.
    void DrawHUD(irr::IrrlichtDevice* device) const;

    static constexpr int NUM_PANELS = static_cast<int>(PanelID::COUNT);

private:
    struct Panel {
        const char* name;
        bool open = false;
    };

    Panel m_panels[NUM_PANELS];
};
