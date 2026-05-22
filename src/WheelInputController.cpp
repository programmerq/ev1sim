#include "WheelInputController.h"

#include "SdlContext.h"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

namespace ev1sim {

namespace {
// Map a raw SDL axis value [-32768, 32767] through a binding's calibration.
// in_min maps to the low end, in_max to the high end (swap them to invert a
// pedal that rests high).  bipolar=true yields [-1, 1] (steering, with an
// optional center deadzone); otherwise [0, 1] (throttle / brake pedals).
double normalize_axis(const WheelAxisBinding& b, int raw, bool bipolar) {
    double t = (b.in_max != b.in_min)
                   ? (static_cast<double>(raw) - b.in_min) / (b.in_max - b.in_min)
                   : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    if (b.invert) {
        t = 1.0 - t;
    }
    if (bipolar) {
        double v = t * 2.0 - 1.0;
        if (b.deadzone > 0.0 && std::abs(v) < b.deadzone) {
            v = 0.0;
        }
        return v;
    }
    return t;
}
}  // namespace

WheelInputController::WheelInputController(SdlContext* sdl, WheelConfig cfg)
    : m_sdl(sdl), m_cfg(std::move(cfg)) {
    m_button_prev.assign(m_cfg.buttons.size(), false);
    if (m_sdl && m_sdl->ok()) {
        OpenMatchingDevice();
    }
}

WheelInputController::~WheelInputController() { CloseDevice(); }

void WheelInputController::OpenMatchingDevice() {
    int count = 0;
    SDL_JoystickID* ids = SDL_GetJoysticks(&count);
    if (!ids) {
        return;
    }
    for (int i = 0; i < count; ++i) {
        const char* cname = SDL_GetJoystickNameForID(ids[i]);
        std::string name = cname ? cname : "";
        if (m_cfg.device_name_match.empty() ||
            name.find(m_cfg.device_name_match) != std::string::npos) {
            SDL_Joystick* joy = SDL_OpenJoystick(ids[i]);
            if (joy) {
                m_joystick    = joy;
                m_instance_id = ids[i];
                m_fresh_open  = true;  // first poll seeds m_button_prev, fires no edges
                std::cout << "[Wheel] opened '" << name << "' ("
                          << SDL_GetNumJoystickAxes(joy) << " axes, "
                          << SDL_GetNumJoystickButtons(joy) << " buttons)\n";
                break;
            }
        }
    }
    SDL_free(ids);
}

void WheelInputController::CloseDevice() {
    if (m_joystick) {
        SDL_CloseJoystick(static_cast<SDL_Joystick*>(m_joystick));
        m_joystick    = nullptr;
        m_instance_id = 0;
    }
}

DriverCommand WheelInputController::Update(double /*dt*/) {
    DriverCommand cmd;  // neutral defaults when no device is present
    m_pending.clear();
    if (!m_sdl || !m_sdl->ok()) {
        return cmd;
    }

    // Hot-plug: drop a device that went away, (re)open one if we have none.
    if (m_joystick &&
        !SDL_JoystickConnected(static_cast<SDL_Joystick*>(m_joystick))) {
        std::cout << "[Wheel] device disconnected\n";
        CloseDevice();
    }
    if (!m_joystick) {
        OpenMatchingDevice();
        if (!m_joystick) {
            return cmd;  // still nothing — stay neutral, keyboard drives
        }
    }

    SDL_Joystick* joy   = static_cast<SDL_Joystick*>(m_joystick);
    const int     n_axes = SDL_GetNumJoystickAxes(joy);
    const int     n_btn  = SDL_GetNumJoystickButtons(joy);

    // Axes -> DriverCommand fields.
    for (const auto& a : m_cfg.axes) {
        if (a.sdl_axis < 0 || a.sdl_axis >= n_axes ||
            a.target == WheelAxisBinding::Target::None) {
            continue;
        }
        const int raw = SDL_GetJoystickAxis(joy, a.sdl_axis);
        switch (a.target) {
            case WheelAxisBinding::Target::Steering:
                cmd.steering = normalize_axis(a, raw, /*bipolar=*/true);
                break;
            case WheelAxisBinding::Target::Throttle:
                cmd.throttle = normalize_axis(a, raw, /*bipolar=*/false);
                break;
            case WheelAxisBinding::Target::FrontBrake:
                cmd.front_brake = normalize_axis(a, raw, /*bipolar=*/false);
                break;
            case WheelAxisBinding::Target::RearBrake:
                cmd.rear_brake = normalize_axis(a, raw, /*bipolar=*/false);
                break;
            case WheelAxisBinding::Target::None:
                break;
        }
    }

    // Buttons -> held horn (level) + momentary action rising edges.
    for (std::size_t i = 0; i < m_cfg.buttons.size(); ++i) {
        const auto& b   = m_cfg.buttons[i];
        const bool down = (b.sdl_button >= 0 && b.sdl_button < n_btn)
                              ? SDL_GetJoystickButton(joy, b.sdl_button)
                              : false;
        if (b.action == InputAction::Horn) {
            if (down) {
                cmd.horn_low  = true;
                cmd.horn_high = true;
            }
        } else if (!m_fresh_open && !InputActionIsHeld(b.action) &&
                   b.action != InputAction::None) {
            if (down && !m_button_prev[i]) {
                m_pending.push_back(b.action);  // rising edge
            }
        }
        m_button_prev[i] = down;
    }
    // The first poll after a device (re)opens only seeds m_button_prev — a
    // button already held when the wheel appeared must not fire a phantom edge.
    m_fresh_open = false;
    return cmd;
}

}  // namespace ev1sim
