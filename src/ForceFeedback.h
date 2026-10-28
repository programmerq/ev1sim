#pragma once

namespace ev1sim {

// Force-feedback configuration (the "ffb" block of config/input_bindings.json).
struct FfbConfig {
    bool   enabled               = true;
    double scale_nm_to_unit      = 0.04;  // Nm -> unit force (before clamp)
    double max_clamp             = 1.0;   // |unit force| ceiling, 0..1
    double gain                  = 0.8;   // SDL haptic master gain, 0..1
    double speed_taper_start_mps = 0.0;   // below this speed: no force
    double speed_taper_full_mps  = 0.0;   // 0 disables the taper (full force always)
    bool   autocenter            = false; // wheel-driver auto-centering spring
    bool   invert                = false; // flip force direction

    // Rumble while the ABS modulates (see AbsRumble).  A feel setting, not an
    // EV1 property: the car itself pulses the brake pedal, not the wheel.
    bool   abs_rumble            = true;
    double abs_rumble_magnitude  = 0.3;   // sine amplitude, unit force 0..1
    int    abs_rumble_period_ms  = 60;    // sine period
    int    abs_rumble_hold_ms    = 250;   // keeps rumbling across the BTCM's re-apply legs
};

// Whether the wheel should rumble for ABS, from the BTCM's front-wheel phases.
// A HOLD or DUMP on either front wheel is the BTCM modulating; its re-apply
// legs between them decode as APPLY, so the rumble holds for abs_rumble_hold_ms
// after the last HOLD/DUMP instead of chattering with every cycle.
class AbsRumble {
public:
    explicit AbsRumble(FfbConfig cfg = {}) : m_cfg(cfg) {}

    // dt_s: elapsed time; modulating: a fresh front wheel is in HOLD or DUMP now.
    void Update(double dt_s, bool modulating) {
        if (modulating) {
            m_since_s = 0.0;
        } else if (m_since_s >= 0.0) {
            m_since_s += dt_s;
            if (m_since_s * 1000.0 > m_cfg.abs_rumble_hold_ms) m_since_s = -1.0;
        }
    }
    bool Active() const { return m_cfg.abs_rumble && m_since_s >= 0.0; }
    // Unit sine amplitude to play now, 0 when quiet.
    double Magnitude() const {
        if (!Active()) return 0.0;
        return m_cfg.abs_rumble_magnitude < 0.0 ? 0.0
             : (m_cfg.abs_rumble_magnitude > 1.0 ? 1.0 : m_cfg.abs_rumble_magnitude);
    }

private:
    FfbConfig m_cfg;
    double    m_since_s = -1.0;   // time since the last HOLD/DUMP; < 0 = quiet
};

// Constant-force feedback driven by the vehicle's front-axle self-aligning
// torque (VehicleState::steering_torque, also published as chassis signal 4109).
// Holds one persistent SDL constant-force effect on the wheel's haptic device
// and updates its magnitude/direction each frame.  Degrades to a no-op when the
// device lacks haptics or none is open (ok() == false), so the sim runs fine
// without a wheel and on platforms with no FFB backend (e.g. most macOS wheels).
class ForceFeedback {
public:
    // sdl_joystick is an opaque SDL_Joystick* (the open wheel).  A nullptr or a
    // non-haptic device yields ok() == false and makes all calls no-ops.
    ForceFeedback(void* sdl_joystick, FfbConfig cfg);
    ~ForceFeedback();

    ForceFeedback(const ForceFeedback&)            = delete;
    ForceFeedback& operator=(const ForceFeedback&) = delete;

    bool ok() const { return m_haptic != nullptr; }

    // Update the constant force from the front-axle steering torque (Nm) and
    // current vehicle speed (m/s).  When active is false (pause / reset / no
    // device) the force is driven to zero.
    void Update(double steering_torque_nm, double speed_mps, bool active);

    // Play the ABS rumble as a sine on the steering axis at this unit
    // amplitude (0 stops it).  A no-op on a device without SDL_HAPTIC_SINE.
    void SetRumble(double magnitude);

    // Drive the effects to zero (pause / disconnect).
    void Stop();

    // The signed unit force in [-max_clamp, max_clamp] for a torque + speed,
    // applying scale, optional speed taper, invert, and clamp.  Pure function —
    // exposed so the curve can be unit-tested without a haptic device.
    static double CurveUnitForce(const FfbConfig& cfg,
                                 double steering_torque_nm, double speed_mps);

private:
    void* m_haptic    = nullptr;  // SDL_Haptic*
    int   m_effect_id = -1;
    bool  m_running   = false;
    int   m_rumble_id = -1;      // SDL_HAPTIC_SINE effect, -1 when unsupported
    bool  m_rumbling  = false;
    FfbConfig m_cfg;
};

}  // namespace ev1sim
