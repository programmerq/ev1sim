#include "RigInputDevices.h"

#include "SdlContext.h"

#include <SDL3/SDL.h>

#include <iostream>
#include <string>

namespace ev1sim {

RigInputDevices::RigInputDevices(SdlContext* sdl, const RigConfig& cfg) : m_sdl(sdl) {
    for (const auto& dc : cfg.devices) {
        if (dc.device_name_match.empty()) {
            std::cerr << "[Rig] device without device_name_match ignored\n";
            continue;
        }
        m_devices.push_back(Device{RigSwitchTracker(dc), nullptr});
    }
}

RigInputDevices::~RigInputDevices() {
    for (auto& d : m_devices) Close(d);
}

void RigInputDevices::TryOpen(Device& d) {
    int count = 0;
    SDL_JoystickID* ids = SDL_GetJoysticks(&count);
    if (!ids) {
        return;
    }
    const std::string& match = d.tracker.config().device_name_match;
    for (int i = 0; i < count; ++i) {
        const char* cname = SDL_GetJoystickNameForID(ids[i]);
        const std::string name = cname ? cname : "";
        if (name.find(match) == std::string::npos) {
            continue;
        }
        if (SDL_Joystick* joy = SDL_OpenJoystick(ids[i])) {
            d.joystick = joy;
            d.tracker.Reset();
            std::cout << "[Rig] opened '" << name << "' ("
                      << SDL_GetNumJoystickButtons(joy) << " buttons)\n";
            break;
        }
    }
    SDL_free(ids);
}

void RigInputDevices::Close(Device& d) {
    if (d.joystick) {
        SDL_CloseJoystick(static_cast<SDL_Joystick*>(d.joystick));
        d.joystick = nullptr;
    }
}

RigSample RigInputDevices::Update() {
    RigSample all;
    if (!m_sdl || !m_sdl->ok()) {
        return all;
    }
    auto merge = [&all](const RigSample& s) {
        all.actions.insert(all.actions.end(), s.actions.begin(), s.actions.end());
        all.horn = all.horn || s.horn;
        all.switch_changes.insert(all.switch_changes.end(), s.switch_changes.begin(),
                                  s.switch_changes.end());
    };
    for (auto& d : m_devices) {
        if (d.joystick && !SDL_JoystickConnected(static_cast<SDL_Joystick*>(d.joystick))) {
            std::cout << "[Rig] '" << d.tracker.config().device_name_match
                      << "' disconnected\n";
            Close(d);
            merge(d.tracker.Disconnected());
        }
        if (!d.joystick) {
            TryOpen(d);
            if (!d.joystick) continue;
        }
        SDL_Joystick* joy = static_cast<SDL_Joystick*>(d.joystick);
        const int n_btn   = SDL_GetNumJoystickButtons(joy);
        merge(d.tracker.Sample([joy, n_btn](int b) {
            return b < n_btn && SDL_GetJoystickButton(joy, b);
        }));
    }
    return all;
}

}  // namespace ev1sim
