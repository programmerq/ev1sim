#pragma once

#include <cstdint>
#include <string>

namespace ev1sim {

// Tactile feedback the sim drives on whatever devices the driver has: the
// force-feedback wheel (ForceFeedback) and a rumble controller whose motors can
// sit anywhere, such as on the pedals (ControllerRumble).  These are feel
// settings, not EV1 properties: the car itself pulses its brake pedal.

// When the ABS channel is live.
struct AbsFeedbackConfig {
    bool enabled         = true;
    int  hold_ms         = 250;   // keeps going across the BTCM's re-apply legs
    int  pulse_period_ms = 60;    // wheel sine period; on/off period for rumble motors
};

// A controller whose rumble motors play the feedback channels.
struct ControllerRumbleConfig {
    enum class Motor { Low, High, Both };

    std::string device_name_match;   // substring of the SDL device name; empty = off
    Motor       abs_motor     = Motor::Low;
    double      abs_magnitude = 0.6; // 0..1 motor strength while pulsing
};

struct FeedbackConfig {
    AbsFeedbackConfig      abs;
    ControllerRumbleConfig controller_rumble;
};

// The ABS feedback channel, from the BTCM's front-wheel phases. A HOLD or DUMP
// on either front wheel is the BTCM modulating; its re-apply legs between them
// decode as APPLY, so the channel holds for hold_ms after the last HOLD/DUMP
// instead of chattering with every cycle.
class AbsFeedback {
public:
    explicit AbsFeedback(AbsFeedbackConfig cfg = {}) : m_cfg(cfg) {}

    // dt_s: elapsed time; modulating: a fresh front wheel is in HOLD or DUMP now.
    void Update(double dt_s, bool modulating) {
        if (modulating) {
            if (m_since_s < 0.0) m_phase_s = 0.0;   // a new event starts on a pulse
            m_since_s = 0.0;
        } else if (m_since_s >= 0.0) {
            m_since_s += dt_s;
            if (m_since_s * 1000.0 > m_cfg.hold_ms) m_since_s = -1.0;
        }
        if (m_since_s >= 0.0) m_phase_s += dt_s;
    }
    bool Active() const { return m_cfg.enabled && m_since_s >= 0.0; }
    // Square-wave pulse for on/off motors: on for the first half of each period.
    bool PulseOn() const {
        if (!Active()) return false;
        const double period_s = (m_cfg.pulse_period_ms > 0 ? m_cfg.pulse_period_ms : 1) / 1000.0;
        const double t = m_phase_s - period_s * static_cast<long long>(m_phase_s / period_s);
        return t < 0.5 * period_s;
    }

private:
    AbsFeedbackConfig m_cfg;
    double m_since_s = -1.0;   // time since the last HOLD/DUMP; < 0 = quiet
    double m_phase_s = 0.0;    // time into the current event, for the pulse
};

// SDL rumble levels (0..65535) for the low- and high-frequency motors.
struct RumbleLevels {
    std::uint16_t low  = 0;
    std::uint16_t high = 0;
};

inline RumbleLevels ControllerRumbleLevels(const ControllerRumbleConfig& cfg, bool on) {
    RumbleLevels r;
    if (!on) return r;
    const double m = cfg.abs_magnitude < 0.0 ? 0.0 : (cfg.abs_magnitude > 1.0 ? 1.0 : cfg.abs_magnitude);
    const auto level = static_cast<std::uint16_t>(m * 65535.0 + 0.5);
    using M = ControllerRumbleConfig::Motor;
    if (cfg.abs_motor == M::Low  || cfg.abs_motor == M::Both) r.low  = level;
    if (cfg.abs_motor == M::High || cfg.abs_motor == M::Both) r.high = level;
    return r;
}

}  // namespace ev1sim
