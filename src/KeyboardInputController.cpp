#include "KeyboardInputController.h"

#include <algorithm>
#include <cmath>

KeyboardInputController::KeyboardInputController(const Rates& rates)
    : m_rates(rates) {
    m_keys.fill(false);
}

// ---------------------------------------------------------------------------
bool KeyboardInputController::OnEvent(const irr::SEvent& event) {
    if (event.EventType == irr::EET_KEY_INPUT_EVENT) {
        m_keys[event.KeyInput.Key] = event.KeyInput.PressedDown;
        return true;
    }
    return false;
}

void KeyboardInputController::SetKeyPressed(irr::EKEY_CODE key, bool pressed) {
    m_keys[key] = pressed;
}

// ---------------------------------------------------------------------------
DriverCommand KeyboardInputController::Update(double dt) {
    // --- Throttle (W) ---
    if (m_keys[irr::KEY_KEY_W]) {
        m_throttle = std::min(1.0, m_throttle + m_rates.throttle_rise * dt);
    } else {
        m_throttle = std::max(0.0, m_throttle - m_rates.throttle_rise * dt);
    }

    // --- Brake (S) ---
    if (m_keys[irr::KEY_KEY_S]) {
        m_brake = std::min(1.0, m_brake + m_rates.brake_rise * dt);
    } else {
        m_brake = std::max(0.0, m_brake - m_rates.brake_rise * dt);
    }

    // --- Steering (A = left / D = right) ---
    // Positive steering = left turn (Chrono convention).
    if (m_keys[irr::KEY_KEY_A]) {
        m_steering = std::min(1.0, m_steering + m_rates.steer_rate * dt);
    } else if (m_keys[irr::KEY_KEY_D]) {
        m_steering = std::max(-1.0, m_steering - m_rates.steer_rate * dt);
    } else {
        // Return to center
        if (m_steering > 0.0)
            m_steering = std::max(0.0, m_steering - m_rates.steer_return_rate * dt);
        else
            m_steering = std::min(0.0, m_steering + m_rates.steer_return_rate * dt);
    }

    // --- Parking brake (Space toggle) ---
    bool space_now = m_keys[irr::KEY_SPACE];
    if (space_now && !m_space_prev)
        m_parking_brake = !m_parking_brake;
    m_space_prev = space_now;

    // --- Reset vehicle (R one-shot) ---
    bool r_now = m_keys[irr::KEY_KEY_R];
    bool do_reset = r_now && !m_r_prev;
    m_r_prev = r_now;

    // --- Camera cycle (C one-shot, consumed via ConsumeCameraCycle) ---
    bool c_now = m_keys[irr::KEY_KEY_C];
    if (c_now && !m_c_prev)
        m_camera_cycle = true;
    m_c_prev = c_now;

    // --- Pause toggle (P one-shot, consumed via ConsumePauseToggle) ---
    bool p_now = m_keys[irr::KEY_KEY_P];
    if (p_now && !m_p_prev)
        m_pause_toggle = true;
    m_p_prev = p_now;

    // --- Quit (Esc) ---
    if (m_keys[irr::KEY_ESCAPE])
        m_quit = true;

    DriverCommand cmd;
    cmd.throttle      = m_throttle;
    cmd.front_brake   = m_brake;
    cmd.rear_brake    = m_brake;
    cmd.steering      = m_steering;
    cmd.parking_brake = m_parking_brake;
    cmd.reset_vehicle = do_reset;

    // Horn — direct key state, no ramping.
    // B = steering-wheel horn (both tones), O = hi only, L = lo only.
    cmd.horn_low  = m_keys[irr::KEY_KEY_B] || m_keys[irr::KEY_KEY_L];
    cmd.horn_high = m_keys[irr::KEY_KEY_B] || m_keys[irr::KEY_KEY_O];

    return cmd;
}

// ---------------------------------------------------------------------------
bool KeyboardInputController::ConsumeCameraCycle() {
    bool v = m_camera_cycle;
    m_camera_cycle = false;
    return v;
}

bool KeyboardInputController::ConsumePauseToggle() {
    bool v = m_pause_toggle;
    m_pause_toggle = false;
    return v;
}

