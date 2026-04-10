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
    bool m_camera_cycle = false;
    bool m_quit         = false;

    std::array<bool, irr::KEY_KEY_CODES_COUNT> m_keys = {};
};
