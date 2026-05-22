#include "ForceFeedback.h"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace ev1sim {

double ForceFeedback::CurveUnitForce(const FfbConfig& cfg,
                                     double torque_nm, double speed_mps) {
    double u = torque_nm * cfg.scale_nm_to_unit;
    // Speed taper: ramp the force in between start..full m/s so a parked wheel
    // doesn't fight the driver on tiny residual torque.  full <= start disables.
    if (cfg.speed_taper_full_mps > cfg.speed_taper_start_mps) {
        double t = (speed_mps - cfg.speed_taper_start_mps) /
                   (cfg.speed_taper_full_mps - cfg.speed_taper_start_mps);
        u *= std::clamp(t, 0.0, 1.0);
    }
    if (cfg.invert) {
        u = -u;
    }
    // Sanitize max_clamp to its documented [0,1] range: a negative value would
    // violate std::clamp's lo <= hi precondition (undefined behavior).
    const double cap = std::clamp(cfg.max_clamp, 0.0, 1.0);
    return std::clamp(u, -cap, cap);
}

ForceFeedback::ForceFeedback(void* sdl_joystick, FfbConfig cfg) : m_cfg(cfg) {
    if (!cfg.enabled || sdl_joystick == nullptr) {
        return;
    }
    SDL_Joystick* joy = static_cast<SDL_Joystick*>(sdl_joystick);
    SDL_Haptic* haptic = SDL_OpenHapticFromJoystick(joy);
    if (!haptic) {
        std::cerr << "[ForceFeedback] device has no haptics (" << SDL_GetError()
                  << ") — force feedback disabled\n";
        return;
    }
    if (!(SDL_GetHapticFeatures(haptic) & SDL_HAPTIC_CONSTANT)) {
        std::cerr << "[ForceFeedback] device lacks SDL_HAPTIC_CONSTANT — "
                     "force feedback disabled\n";
        SDL_CloseHaptic(haptic);
        return;
    }

    // The game owns the force directly, so disable the wheel's own centering
    // spring unless explicitly asked for, and apply the configured master gain.
    SDL_SetHapticAutocenter(haptic, cfg.autocenter ? 50 : 0);
    SDL_SetHapticGain(haptic, std::clamp(static_cast<int>(cfg.gain * 100.0 + 0.5), 0, 100));

    SDL_HapticEffect effect{};
    effect.type                      = SDL_HAPTIC_CONSTANT;
    effect.constant.type             = SDL_HAPTIC_CONSTANT;
    effect.constant.direction.type   = SDL_HAPTIC_CARTESIAN;
    effect.constant.direction.dir[0] = 1;                    // X axis = steering
    effect.constant.length           = SDL_HAPTIC_INFINITY;
    effect.constant.level            = 0;

    m_effect_id = SDL_CreateHapticEffect(haptic, &effect);
    if (m_effect_id < 0) {
        std::cerr << "[ForceFeedback] SDL_CreateHapticEffect failed: "
                  << SDL_GetError() << "\n";
        SDL_CloseHaptic(haptic);
        return;
    }
    m_haptic = haptic;
}

ForceFeedback::~ForceFeedback() {
    if (m_haptic) {
        SDL_Haptic* h = static_cast<SDL_Haptic*>(m_haptic);
        if (m_effect_id >= 0) {
            SDL_DestroyHapticEffect(h, m_effect_id);
        }
        SDL_CloseHaptic(h);
    }
}

void ForceFeedback::Update(double torque_nm, double speed_mps, bool active) {
    if (!m_haptic) {
        return;
    }
    SDL_Haptic* h = static_cast<SDL_Haptic*>(m_haptic);
    const double u = active ? CurveUnitForce(m_cfg, torque_nm, speed_mps) : 0.0;

    SDL_HapticEffect effect{};
    effect.type                      = SDL_HAPTIC_CONSTANT;
    effect.constant.type             = SDL_HAPTIC_CONSTANT;
    effect.constant.direction.type   = SDL_HAPTIC_CARTESIAN;
    effect.constant.direction.dir[0] = (u >= 0.0) ? 1 : -1;
    effect.constant.length           = SDL_HAPTIC_INFINITY;
    effect.constant.level =
        static_cast<Sint16>(std::lround(std::min(std::abs(u), 1.0) * 32767.0));

    SDL_UpdateHapticEffect(h, m_effect_id, &effect);
    if (!m_running) {
        SDL_RunHapticEffect(h, m_effect_id, SDL_HAPTIC_INFINITY);
        m_running = true;
    }
}

void ForceFeedback::Stop() {
    if (m_haptic) {
        Update(0.0, 0.0, /*active=*/false);
    }
}

}  // namespace ev1sim
