#pragma once

#include "InputController.h"

#include <irrlicht.h>

#include <array>

// Keyboard-driven input with configurable ramping rates.
// Implements irr::IEventReceiver so it can be registered with
// ChVisualSystemIrrlicht::AddUserEventReceiver().
class KeyboardInputController : public InputController,
                                 public irr::IEventReceiver {
public:
    struct Rates {
        double steer_rate        = 1.8;
        double steer_return_rate = 2.5;
        double throttle_rise     = 1.5;
        double brake_rise        = 2.0;
    };

    explicit KeyboardInputController(const Rates& rates);

    // InputController
    DriverCommand Update(double dt) override;

    // IEventReceiver
    bool OnEvent(const irr::SEvent& event) override;

    // Testing helper — set a key's pressed state without Irrlicht events.
    void SetKeyPressed(irr::EKEY_CODE key, bool pressed);

    // True once after C was tapped (consumed on read).
    bool ConsumeCameraCycle();

    // True once after P was tapped (consumed on read).
    bool ConsumePauseToggle();

    // True once after H was tapped (consumed on read).
    bool ConsumeHeadlightToggle();

    // True once after K was tapped (consumed on read).
    // K = "Key on" — enables propulsion until RSA's vehicle-on signal is wired.
    bool ConsumeKeyOnToggle();

    // True once after '.' was tapped — cycle PRND up (P→R→N→D, clamped at D).
    bool ConsumePrndUp();

    // True once after ',' was tapped — cycle PRND down (D→N→R→P, clamped at P).
    bool ConsumePrndDown();

    // True once after Q was tapped — toggle turn signal LEFT.
    // OFF→LEFT, LEFT→OFF, RIGHT→LEFT.
    bool ConsumeTurnSignalLeft();

    // True once after E was tapped — toggle turn signal RIGHT.
    // OFF→RIGHT, RIGHT→OFF, LEFT→RIGHT.
    bool ConsumeTurnSignalRight();

    // True once after X was tapped — toggle hazard switch on/off.
    // TODO: temporary keybinding; move to floating-UI panel when that lands
    //       per docs/TODO.md.
    bool ConsumeHazardToggle();

    // True once after '?' (Shift+/) was tapped — toggle keyboard help overlay.
    bool ConsumeHelpToggle();

    // Momentary state of the U key — true while held.  Used for the
    // combination-switch flash-to-pass lever (not edge-detected; the lever
    // returns to neutral as soon as the driver releases it).
    bool IsFlashToPassHeld() const { return m_keys[irr::KEY_KEY_U]; }

    // Panel toggles: F=hood, T=trunk, [=left door, ]=right door.
    bool ConsumePanelToggle(int index);

    // True once after I was tapped — IPC trip-reset momentary press.
    bool ConsumeIpcTripReset();

    // Cruise stalk momentary presses:
    //   G = SET, Y = RESUME, N = CANCEL
    //   + (KEY_PLUS / KEY_OEM_PLUS, i.e. '='/'+'key) = SPEED UP
    //   - (KEY_MINUS / KEY_OEM_MINUS) = SPEED DOWN
    bool ConsumeCruiseSet();
    bool ConsumeCruiseResume();
    bool ConsumeCruiseCancel();
    bool ConsumeCruiseSpeedUp();
    bool ConsumeCruiseSpeedDown();

    // True once after V was tapped — cycle wiper stalk position.
    bool ConsumeWiperCycle();

    // True once after M was tapped — wiper wash momentary press.
    bool ConsumeWiperWash();

    // True if Esc was pressed.
    bool QuitRequested() const { return m_quit; }

private:
    Rates  m_rates;

    // Smoothed output values
    double m_throttle = 0.0;
    double m_brake    = 0.0;
    double m_steering = 0.0;
    bool   m_parking_brake = false;

    // Edge-detect helpers
    bool m_space_prev = false;
    bool m_r_prev     = false;
    bool m_c_prev     = false;
    bool m_p_prev     = false;
    bool m_h_prev     = false;
    bool m_k_prev     = false;
    bool m_period_prev = false;
    bool m_comma_prev  = false;
    bool m_q_prev      = false;
    bool m_e_prev      = false;
    bool m_x_prev      = false;
    bool m_slash_prev  = false;   // for '?' (Shift+/)
    bool m_i_prev      = false;   // IPC trip-reset
    bool m_g_prev      = false;   // cruise SET
    bool m_y_prev      = false;   // cruise RESUME
    bool m_n_prev      = false;   // cruise CANCEL
    bool m_plus_prev   = false;   // cruise SPEED UP (= / + key)
    bool m_minus_prev  = false;   // cruise SPEED DOWN
    bool m_v_prev      = false;   // wiper cycle
    bool m_m_prev      = false;   // wiper wash
    bool m_camera_cycle       = false;
    bool m_pause_toggle       = false;
    bool m_headlight_toggle   = false;
    bool m_keyon_toggle       = false;
    bool m_prnd_up            = false;
    bool m_prnd_down          = false;
    bool m_turn_signal_left   = false;
    bool m_turn_signal_right  = false;
    bool m_hazard_toggle      = false;
    bool m_help_toggle        = false;
    bool m_ipc_trip_reset     = false;
    bool m_cruise_set         = false;
    bool m_cruise_resume      = false;
    bool m_cruise_cancel      = false;
    bool m_cruise_speed_up    = false;
    bool m_cruise_speed_down  = false;
    bool m_wiper_cycle        = false;
    bool m_wiper_wash         = false;

    // Panel toggle keys: KEY_KEY_1 through KEY_KEY_4
    bool m_panel_prev[4]   = {};
    bool m_panel_toggle[4] = {};

    bool m_quit             = false;

    std::array<bool, irr::KEY_KEY_CODES_COUNT> m_keys = {};
};
