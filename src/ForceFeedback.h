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

    // Drive the effect to zero (pause / disconnect).
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
    FfbConfig m_cfg;
};

}  // namespace ev1sim
