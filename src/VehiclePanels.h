#pragma once

#include "Hood.h"

#include <irrlicht.h>

/// Identifiers for each movable body panel.
enum class PanelID : int {
    HOOD = 0,
    TRUNK,
    DOOR_LEFT,
    DOOR_RIGHT,
    COUNT  // = 4
};

/// One lock as the body diagram shows it: the latch state, and where the lock
/// actuator's stroke is (0 = unlocked end, 1 = locked end). While a door-lock
/// motor is mid-stroke the knob follows the motor; otherwise it sits at the
/// end the latch says.
struct LockIndicator {
    bool   locked   = false;
    bool   moving   = false;
    double stroke01 = 0.0;
};

/// Pure: the indicator for a latch state plus, for a motorised door lock, its
/// motor's stroke (DoorLockMotor::position()/moving()). The trunk has no
/// motor model, so it passes motor_moving = false.
inline LockIndicator MakeLockIndicator(bool latch_locked, double motor_stroke01,
                                       bool motor_moving) {
    LockIndicator li;
    li.locked = latch_locked;
    li.moving = motor_moving;
    li.stroke01 = motor_moving ? motor_stroke01 : (latch_locked ? 1.0 : 0.0);
    return li;
}

/// The three locks on the diagram: driver door, passenger door, trunk.
struct BodyLockIndicators {
    LockIndicator driver, passenger, trunk;
};

/// One window as the body diagram shows it: how far the glass is down
/// (PowerWindowMotor::open_fraction(), 0 = closed) and whether it is moving.
struct WindowIndicator {
    double open01 = 0.0;
    bool   moving = false;
};

/// The two windows on the diagram: driver (LH) and passenger (RH).
struct BodyWindowIndicators {
    WindowIndicator driver, passenger;
};

/// Tracks open/closed state of body panels (hood, trunk, doors).
/// Each panel maps to a physical sensor in the electronic harness.
/// No geometry or animation — just state and HUD indicators.
class VehiclePanels {
public:
    VehiclePanels();

    void Toggle(PanelID panel);
    bool IsOpen(PanelID panel) const;

    /// The HOOD panel is richer than a simple open/closed bool: it has a
    /// two-stage latch (see Hood).  IsOpen(HOOD) returns the derived ajar
    /// sensor; this accessor exposes the full mechanical state for the staged
    /// release controls and the HUD.
    Hood&       hood()       { return m_hood; }
    const Hood& hood() const { return m_hood; }

    /// Draw panel status indicators on the top-down vehicle HUD, when `locks`
    /// is given a lock knob inside each door and the trunk, and when `windows`
    /// is given each door's window glass.  Call between BeginScene/EndScene.
    void DrawHUD(irr::IrrlichtDevice* device, const BodyLockIndicators* locks = nullptr,
                 const BodyWindowIndicators* windows = nullptr) const;

    /// Pixel offset of a lock knob along its travel (0 at the unlocked end,
    /// kLockKnobTravelPx at the locked end).
    static int LockKnobOffsetPx(const LockIndicator& li);
    static constexpr int kLockKnobTravelPx = 8;

    /// Pixel length of a window's glass bar: the full kWindowGlassPx when
    /// closed, shrinking to 0 as the glass goes down.
    static int WindowGlassPx(const WindowIndicator& wi);
    static constexpr int kWindowGlassPx = 22;

    static constexpr int NUM_PANELS = static_cast<int>(PanelID::COUNT);

private:
    struct Panel {
        const char* name;
        bool open = false;
    };

    Panel m_panels[NUM_PANELS];
    Hood  m_hood;   // HOOD panel's two-stage latch state machine
};
