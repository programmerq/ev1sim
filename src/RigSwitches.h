#pragma once

#include "InputActions.h"

#include <cstddef>
#include <functional>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace ev1sim {

class PhysicalWorld;

// Real switches on a driving rig (a USB button box, an encoder board wired to
// cabin switches) that SDL sees as joystick buttons.  A button binds either to
// a momentary InputAction, fired on its press like a wheel button, or to a
// switch: a contact whose open/closed level sets a PhysicalWorld input directly,
// the way a seat-belt buckle or an A/C toggle holds its state.
//
// A switch is written only when its level changes (and once when its device
// appears), so the keyboard and the floating UI can still change the same input
// in between; the next real flip of the switch wins again.

enum class RigSwitchTarget {
    None,
    SeatbeltDriver,       // closed = buckled
    SeatbeltPassenger,
    ChargeCoupler,        // closed = paddle in the inlet
    AirConditioning,      // closed = A/C on
    Defrost,              // closed = rear defrost on
    WindowDriverUp,       // closed = held
    WindowDriverDown,
    WindowPassengerUp,
    WindowPassengerDown,
    // One contact per rotary position: closing it selects that position;
    // opening it changes nothing, since the next position's contact closes.
    HvacFanOff,
    HvacFanLow,
    HvacFanMed,
    HvacFanHigh,
    HvacModeFace,
    HvacModeBilevel,
    HvacModeFeet,
    HvacModeDefrost,

    Count,  // sentinel, keep last: ties kRigSwitchNames to the enum
};

namespace detail {
struct RigSwitchName {
    RigSwitchTarget  target;
    std::string_view name;
};
inline constexpr RigSwitchName kRigSwitchNames[] = {
    {RigSwitchTarget::SeatbeltDriver,      "seatbelt_driver"},
    {RigSwitchTarget::SeatbeltPassenger,   "seatbelt_passenger"},
    {RigSwitchTarget::ChargeCoupler,       "charge_coupler"},
    {RigSwitchTarget::AirConditioning,     "hvac_ac"},
    {RigSwitchTarget::Defrost,             "hvac_defrost"},
    {RigSwitchTarget::WindowDriverUp,      "window_driver_up"},
    {RigSwitchTarget::WindowDriverDown,    "window_driver_down"},
    {RigSwitchTarget::WindowPassengerUp,   "window_passenger_up"},
    {RigSwitchTarget::WindowPassengerDown, "window_passenger_down"},
    {RigSwitchTarget::HvacFanOff,          "hvac_fan_off"},
    {RigSwitchTarget::HvacFanLow,          "hvac_fan_low"},
    {RigSwitchTarget::HvacFanMed,          "hvac_fan_med"},
    {RigSwitchTarget::HvacFanHigh,         "hvac_fan_high"},
    {RigSwitchTarget::HvacModeFace,        "hvac_mode_face"},
    {RigSwitchTarget::HvacModeBilevel,     "hvac_mode_bilevel"},
    {RigSwitchTarget::HvacModeFeet,        "hvac_mode_feet"},
    {RigSwitchTarget::HvacModeDefrost,     "hvac_mode_defrost"},
};
}  // namespace detail

// Every RigSwitchTarget except None has exactly one name.
static_assert(std::size(detail::kRigSwitchNames) ==
                  static_cast<std::size_t>(RigSwitchTarget::Count) - 1,
              "add the new RigSwitchTarget to detail::kRigSwitchNames");

inline std::optional<RigSwitchTarget> RigSwitchTargetFromString(std::string_view name) {
    for (const auto& e : detail::kRigSwitchNames) {
        if (e.name == name) return e.target;
    }
    return std::nullopt;
}

struct RigButtonBinding {
    int         sdl_button = -1;
    InputAction action     = InputAction::None;
};

struct RigSwitchBinding {
    int             sdl_button = -1;
    RigSwitchTarget target     = RigSwitchTarget::None;
    bool            invert     = false;   // contact reads pressed when open (normally-closed)
};

// One rig device, matched by a substring of its SDL name (required: a rig never
// grabs "the first device", which would be the wheel).
struct RigDeviceConfig {
    std::string                   device_name_match;
    std::vector<RigButtonBinding> buttons;
    std::vector<RigSwitchBinding> switches;
};

struct RigConfig {
    std::vector<RigDeviceConfig> devices;
};

// Sets the PhysicalWorld input a switch drives to the switch's level.
void ApplyRigSwitch(PhysicalWorld& world, RigSwitchTarget target, bool closed);

// What one poll of a rig device asks of the sim.
struct RigSample {
    std::vector<InputAction>                       actions;        // momentary presses
    bool                                           horn = false;   // a horn-bound button is held
    std::vector<std::pair<RigSwitchTarget, bool>>  switch_changes; // (target, closed)
};

// Turns one device's raw button levels into a RigSample: rising edges for
// momentary buttons, level changes for switches.  SDL-free, so the decision
// logic is unit-tested without a device.
class RigSwitchTracker {
public:
    explicit RigSwitchTracker(RigDeviceConfig cfg = {}) : m_cfg(std::move(cfg)) { Reset(); }

    const RigDeviceConfig& config() const { return m_cfg; }

    // The device (re)appeared: the next Sample fires no momentary edges for
    // buttons already held, and reports every switch's level.
    void Reset() {
        m_button_prev.assign(m_cfg.buttons.size(), false);
        m_switch_prev.assign(m_cfg.switches.size(), std::nullopt);
        m_fresh = true;
    }

    // The device went away: momentary contacts it held (window switches) are
    // released so a motor does not run on; latched inputs keep their state.
    RigSample Disconnected() {
        RigSample s;
        for (std::size_t i = 0; i < m_cfg.switches.size(); ++i) {
            if (m_switch_prev[i].value_or(false) && IsMomentary(m_cfg.switches[i].target)) {
                s.switch_changes.emplace_back(m_cfg.switches[i].target, false);
            }
        }
        Reset();
        return s;
    }

    RigSample Sample(const std::function<bool(int sdl_button)>& down) {
        RigSample s;
        for (std::size_t i = 0; i < m_cfg.buttons.size(); ++i) {
            const auto& b      = m_cfg.buttons[i];
            const bool pressed = b.sdl_button >= 0 && down(b.sdl_button);
            if (b.action == InputAction::Horn) {
                s.horn = s.horn || pressed;
            } else if (b.action != InputAction::None && !InputActionIsHeld(b.action) &&
                       !m_fresh && pressed && !m_button_prev[i]) {
                s.actions.push_back(b.action);
            }
            m_button_prev[i] = pressed;
        }
        for (std::size_t i = 0; i < m_cfg.switches.size(); ++i) {
            const auto& sw = m_cfg.switches[i];
            if (sw.sdl_button < 0 || sw.target == RigSwitchTarget::None) continue;
            const bool closed = down(sw.sdl_button) != sw.invert;
            if (m_switch_prev[i] != closed) {
                s.switch_changes.emplace_back(sw.target, closed);
                m_switch_prev[i] = closed;
            }
        }
        m_fresh = false;
        return s;
    }

private:
    static bool IsMomentary(RigSwitchTarget t) {
        return t == RigSwitchTarget::WindowDriverUp || t == RigSwitchTarget::WindowDriverDown ||
               t == RigSwitchTarget::WindowPassengerUp || t == RigSwitchTarget::WindowPassengerDown;
    }

    RigDeviceConfig                  m_cfg;
    std::vector<bool>                m_button_prev;
    std::vector<std::optional<bool>> m_switch_prev;   // nullopt = not yet reported
    bool                             m_fresh = true;
};

}  // namespace ev1sim
