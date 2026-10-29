#include "ControllerRumble.h"

#include "SdlContext.h"

#include <SDL3/SDL.h>

#include <iostream>
#include <string>
#include <utility>

namespace ev1sim {

namespace {
// Each rumble request lasts this long, renewed every frame while it holds, so
// a stalled frame loop cannot leave a motor running.
constexpr Uint32 kRumbleRequestMs = 200;
}  // namespace

ControllerRumble::ControllerRumble(SdlContext* sdl, ControllerRumbleConfig cfg)
    : m_sdl(sdl), m_cfg(std::move(cfg)) {
    if (m_sdl && m_sdl->ok() && !m_cfg.device_name_match.empty()) {
        OpenMatchingDevice();
    }
}

ControllerRumble::~ControllerRumble() {
    if (m_joystick) {
        SDL_RumbleJoystick(static_cast<SDL_Joystick*>(m_joystick), 0, 0, 0);
    }
    CloseDevice();
}

void ControllerRumble::OpenMatchingDevice() {
    int count = 0;
    SDL_JoystickID* ids = SDL_GetJoysticks(&count);
    if (!ids) {
        return;
    }
    for (int i = 0; i < count; ++i) {
        const char* cname = SDL_GetJoystickNameForID(ids[i]);
        const std::string name = cname ? cname : "";
        if (name.find(m_cfg.device_name_match) == std::string::npos) {
            continue;
        }
        SDL_Joystick* joy = SDL_OpenJoystick(ids[i]);
        if (!joy) {
            continue;
        }
        m_joystick = joy;
        const bool can_rumble = SDL_GetBooleanProperty(
            SDL_GetJoystickProperties(joy), SDL_PROP_JOYSTICK_CAP_RUMBLE_BOOLEAN, false);
        std::cout << "[Rumble] opened '" << name << "'"
                  << (can_rumble ? "" : " (reports no rumble motors)") << "\n";
        break;
    }
    SDL_free(ids);
}

void ControllerRumble::CloseDevice() {
    if (m_joystick) {
        SDL_CloseJoystick(static_cast<SDL_Joystick*>(m_joystick));
        m_joystick = nullptr;
    }
    m_last = {};
}

void ControllerRumble::Update(bool abs_pulse_on) {
    if (!m_sdl || !m_sdl->ok() || m_cfg.device_name_match.empty()) {
        return;
    }
    if (m_joystick && !SDL_JoystickConnected(static_cast<SDL_Joystick*>(m_joystick))) {
        std::cout << "[Rumble] device disconnected\n";
        CloseDevice();
    }
    if (!m_joystick) {
        OpenMatchingDevice();
        if (!m_joystick) {
            return;
        }
    }
    const RumbleLevels lv = ControllerRumbleLevels(m_cfg, abs_pulse_on);
    const bool quiet = lv.low == 0 && lv.high == 0;
    if (quiet && m_last.low == 0 && m_last.high == 0) {
        return;   // nothing playing, nothing to stop
    }
    SDL_RumbleJoystick(static_cast<SDL_Joystick*>(m_joystick), lv.low, lv.high,
                       quiet ? 0 : kRumbleRequestMs);
    m_last = lv;
}

}  // namespace ev1sim
