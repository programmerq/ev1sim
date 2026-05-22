#pragma once

#include "InputActions.h"
#include "InputController.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ev1sim {

class SdlContext;

// One steering/pedal axis mapping: which SDL joystick axis drives which
// DriverCommand field, with calibration.  Raw SDL axis values are [-32768,
// 32767]; in_min/in_max say which raw endpoints map to the output range, so a
// pedal that rests at +32767 and floors at -32768 is handled by swapping them.
struct WheelAxisBinding {
    enum class Target { None, Steering, Throttle, FrontBrake, RearBrake };

    int    sdl_axis = -1;
    Target target   = Target::None;
    double in_min   = -32768.0;   // raw value mapped to the low end of the output
    double in_max   =  32767.0;   // raw value mapped to the high end of the output
    bool   invert   = false;      // flip after normalization
    double deadzone = 0.0;        // 0..1 band around center suppressed (steering)
};

// One button → named action mapping.
struct WheelButtonBinding {
    int         sdl_button = -1;
    InputAction action     = InputAction::None;
};

// Parsed wheel configuration (from config/input_bindings.json, see Config).
struct WheelConfig {
    bool        enabled = false;
    std::string device_name_match;          // substring; empty = first device
    std::vector<WheelAxisBinding>   axes;
    std::vector<WheelButtonBinding> buttons;
};

// Reads a Logitech-style wheel as a raw SDL joystick and produces a
// DriverCommand (steering / throttle / brake from axes; horn from a held
// "horn"-bound button).  Momentary button actions are edge-detected and queued
// for SimApp to drain into DispatchAction.  Hot-plug tolerant: starts with no
// device and opens one when it appears (and re-neutralizes when it's removed).
//
// Coexists with the keyboard — the keyboard stays the primary event receiver
// and the fallback driver; the wheel only overlays its axes + mapped buttons
// when a device is present.
class WheelInputController : public InputController {
public:
    WheelInputController(SdlContext* sdl, WheelConfig cfg);
    ~WheelInputController() override;

    WheelInputController(const WheelInputController&)            = delete;
    WheelInputController& operator=(const WheelInputController&) = delete;

    // Poll the wheel: returns axes + held horn in a DriverCommand and refreshes
    // the pending momentary-action queue.  Returns a neutral command when no
    // device is present.
    DriverCommand Update(double dt) override;

    bool HasDevice() const { return m_joystick != nullptr; }

    // Momentary actions (button rising edges) since the last drain.  SimApp
    // calls this after Update() and dispatches each, then clears.
    const std::vector<InputAction>& PendingActions() const { return m_pending; }
    void ClearPendingActions() { m_pending.clear(); }

    // Opaque SDL_Joystick* of the open device (for ForceFeedback to attach to),
    // or nullptr when no device is open.
    void* JoystickHandle() const { return m_joystick; }

private:
    void OpenMatchingDevice();   // scan + open the first matching wheel
    void CloseDevice();

    SdlContext*   m_sdl = nullptr;
    WheelConfig   m_cfg;

    void*         m_joystick    = nullptr;   // SDL_Joystick*
    std::uint32_t m_instance_id = 0;         // SDL_JoystickID of the open device

    std::vector<bool> m_button_prev;          // per-binding previous pressed state
    bool              m_fresh_open = false;   // first poll after (re)open: seed only, no edges
    std::vector<InputAction> m_pending;       // momentary actions this frame
};

}  // namespace ev1sim
