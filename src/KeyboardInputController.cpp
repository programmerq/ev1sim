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

    // --- Headlight toggle (H one-shot, consumed via ConsumeHeadlightToggle) ---
    bool h_now = m_keys[irr::KEY_KEY_H];
    if (h_now && !m_h_prev)
        m_headlight_toggle = true;
    m_h_prev = h_now;

    // --- Key-on toggle (K one-shot, consumed via ConsumeKeyOnToggle) ---
    bool k_now = m_keys[irr::KEY_KEY_K];
    if (k_now && !m_k_prev)
        m_keyon_toggle = true;
    m_k_prev = k_now;

    // --- PRND up ('.' one-shot, consumed via ConsumePrndUp) ---
    bool period_now = m_keys[irr::KEY_PERIOD];
    if (period_now && !m_period_prev)
        m_prnd_up = true;
    m_period_prev = period_now;

    // --- PRND down (',' one-shot, consumed via ConsumePrndDown) ---
    bool comma_now = m_keys[irr::KEY_COMMA];
    if (comma_now && !m_comma_prev)
        m_prnd_down = true;
    m_comma_prev = comma_now;

    // --- Turn signal left (Q one-shot, consumed via ConsumeTurnSignalLeft) ---
    bool q_now = m_keys[irr::KEY_KEY_Q];
    if (q_now && !m_q_prev)
        m_turn_signal_left = true;
    m_q_prev = q_now;

    // --- Turn signal right (E one-shot, consumed via ConsumeTurnSignalRight) ---
    bool e_now = m_keys[irr::KEY_KEY_E];
    if (e_now && !m_e_prev)
        m_turn_signal_right = true;
    m_e_prev = e_now;

    // --- Hazard toggle (X one-shot, consumed via ConsumeHazardToggle) ---
    // TODO: temporary keybinding; move to floating-UI panel when that lands
    //       per docs/TODO.md.
    bool x_now = m_keys[irr::KEY_KEY_X];
    if (x_now && !m_x_prev)
        m_hazard_toggle = true;
    m_x_prev = x_now;

    // --- Help overlay toggle ('?' = Shift+/, KEY_OEM_2 on many layouts) ---
    // We detect KEY_OEM_2 (the '/' key) regardless of shift, which is fine
    // since '/' is not otherwise bound.
    bool slash_now = m_keys[irr::KEY_OEM_2];
    if (slash_now && !m_slash_prev)
        m_help_toggle = true;
    m_slash_prev = slash_now;

    // --- IPC trip-reset (I one-shot, consumed via ConsumeIpcTripReset) ---
    bool i_now = m_keys[irr::KEY_KEY_I];
    if (i_now && !m_i_prev)
        m_ipc_trip_reset = true;
    m_i_prev = i_now;

    // --- Cruise stalk momentary presses ---
    // G = SET, Y = RESUME, N = CANCEL, +/= = SPEED UP, - = SPEED DOWN.
    bool g_now = m_keys[irr::KEY_KEY_G];
    if (g_now && !m_g_prev)
        m_cruise_set = true;
    m_g_prev = g_now;

    bool y_now = m_keys[irr::KEY_KEY_Y];
    if (y_now && !m_y_prev)
        m_cruise_resume = true;
    m_y_prev = y_now;

    bool n_now = m_keys[irr::KEY_KEY_N];
    if (n_now && !m_n_prev)
        m_cruise_cancel = true;
    m_n_prev = n_now;

    // KEY_PLUS is the '+'/= key on most layouts (numpad or main keyboard).
    bool plus_now = m_keys[irr::KEY_PLUS];
    if (plus_now && !m_plus_prev)
        m_cruise_speed_up = true;
    m_plus_prev = plus_now;

    bool minus_now = m_keys[irr::KEY_MINUS];
    if (minus_now && !m_minus_prev)
        m_cruise_speed_down = true;
    m_minus_prev = minus_now;

    // --- Wiper stalk cycle (V one-shot, consumed via ConsumeWiperCycle) ---
    bool v_now = m_keys[irr::KEY_KEY_V];
    if (v_now && !m_v_prev)
        m_wiper_cycle = true;
    m_v_prev = v_now;

    // --- Wiper wash (M one-shot, consumed via ConsumeWiperWash) ---
    bool m_now = m_keys[irr::KEY_KEY_M];
    if (m_now && !m_m_prev)
        m_wiper_wash = true;
    m_m_prev = m_now;

    // --- Panel toggles (F=hood, T=trunk, [=doorL, ]=doorR) ---
    {
        static const irr::EKEY_CODE panel_keys[4] = {
            irr::KEY_KEY_F, irr::KEY_KEY_T, irr::KEY_OEM_4, irr::KEY_OEM_6
        };
        for (int i = 0; i < 4; ++i) {
            bool now = m_keys[panel_keys[i]];
            if (now && !m_panel_prev[i])
                m_panel_toggle[i] = true;
            m_panel_prev[i] = now;
        }
    }

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

bool KeyboardInputController::ConsumeHeadlightToggle() {
    bool v = m_headlight_toggle;
    m_headlight_toggle = false;
    return v;
}

bool KeyboardInputController::ConsumeKeyOnToggle() {
    bool v = m_keyon_toggle;
    m_keyon_toggle = false;
    return v;
}

bool KeyboardInputController::ConsumePrndUp() {
    bool v = m_prnd_up;
    m_prnd_up = false;
    return v;
}

bool KeyboardInputController::ConsumePrndDown() {
    bool v = m_prnd_down;
    m_prnd_down = false;
    return v;
}

bool KeyboardInputController::ConsumePanelToggle(int index) {
    if (index < 0 || index >= 4) return false;
    bool v = m_panel_toggle[index];
    m_panel_toggle[index] = false;
    return v;
}

bool KeyboardInputController::ConsumeTurnSignalLeft() {
    bool v = m_turn_signal_left;
    m_turn_signal_left = false;
    return v;
}

bool KeyboardInputController::ConsumeTurnSignalRight() {
    bool v = m_turn_signal_right;
    m_turn_signal_right = false;
    return v;
}

bool KeyboardInputController::ConsumeHazardToggle() {
    bool v = m_hazard_toggle;
    m_hazard_toggle = false;
    return v;
}

bool KeyboardInputController::ConsumeHelpToggle() {
    bool v = m_help_toggle;
    m_help_toggle = false;
    return v;
}

bool KeyboardInputController::ConsumeIpcTripReset() {
    bool v = m_ipc_trip_reset;
    m_ipc_trip_reset = false;
    return v;
}

bool KeyboardInputController::ConsumeCruiseSet() {
    bool v = m_cruise_set;
    m_cruise_set = false;
    return v;
}

bool KeyboardInputController::ConsumeCruiseResume() {
    bool v = m_cruise_resume;
    m_cruise_resume = false;
    return v;
}

bool KeyboardInputController::ConsumeCruiseCancel() {
    bool v = m_cruise_cancel;
    m_cruise_cancel = false;
    return v;
}

bool KeyboardInputController::ConsumeCruiseSpeedUp() {
    bool v = m_cruise_speed_up;
    m_cruise_speed_up = false;
    return v;
}

bool KeyboardInputController::ConsumeCruiseSpeedDown() {
    bool v = m_cruise_speed_down;
    m_cruise_speed_down = false;
    return v;
}

bool KeyboardInputController::ConsumeWiperCycle() {
    bool v = m_wiper_cycle;
    m_wiper_cycle = false;
    return v;
}

bool KeyboardInputController::ConsumeWiperWash() {
    bool v = m_wiper_wash;
    m_wiper_wash = false;
    return v;
}

