#include "SdlContext.h"

#include <SDL3/SDL.h>

#include <iostream>

namespace ev1sim {

SdlContext::SdlContext() {
    // Wheel input must keep flowing even when the Irrlicht window doesn't hold
    // OS focus, and SimApp installs its own SIGINT handler — keep SDL out of
    // both of those.
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");

    // SDL3: SDL_Init returns true on success.  No SDL_INIT_VIDEO — we never
    // create a window (Irrlicht owns rendering); only the joystick + haptic
    // subsystems are needed for the wheel and force feedback.
    if (!SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC)) {
        std::cerr << "[SdlContext] SDL_Init(JOYSTICK|HAPTIC) failed: "
                  << SDL_GetError() << " — wheel / force feedback disabled\n";
        m_ok = false;
        return;
    }
    m_ok = true;
}

SdlContext::~SdlContext() {
    if (m_ok) {
        SDL_Quit();
    }
}

void SdlContext::Pump() {
    if (!m_ok) {
        return;
    }
    // Refresh SDL's internal device + axis/button state, then drain the event
    // queue so it stays bounded.  Hot-plug is handled in WheelInputController
    // by rescanning SDL_GetJoysticks() each frame, so we don't need to route
    // individual add/remove events out of here.
    SDL_PumpEvents();
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        // Intentionally discarded — state is polled, not event-driven.
    }
}

}  // namespace ev1sim
