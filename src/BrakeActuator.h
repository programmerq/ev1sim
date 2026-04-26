#pragma once

#include <algorithm>

/// Models the brake actuator dynamics for the two EV1 axles.
///
/// Front (hydraulic disc) — first-order pressure lag
///   τ_apply  ≈ 50 ms : master-cylinder piston compresses brake fluid
///   τ_release ≈ 80 ms : return spring + fluid bleed-back to reservoir
///
/// Rear (electrically actuated drum) — rate-limited position actuator
///   A small DC motor drives a threaded-rod leadscrew that pushes the brake
///   shoes outward.  At the BTCM's rated drive current the full stroke takes
///   ~300 ms, giving max_rate = 1.0 / 0.300 ≈ 3.33 per second.
///
/// Both actuators clamp their output to [0, 1].
///
/// This struct has no Chrono or Irrlicht dependencies so it can be
/// exercised directly in unit tests.
struct BrakeActuator {
    double front_pressure = 0.0;   ///< Actual hydraulic pressure ratio (0..1)
    double rear_position  = 0.0;   ///< Actual drum shoe position ratio (0..1)

    /// Advance actuator dynamics one time step of length `step` (seconds),
    /// driven by the commanded brake values (both in [0, 1]).
    void Advance(double step, double front_cmd, double rear_cmd) {
        // Front: first-order IIR — bilinear (Tustin) discretisation.
        // α = step / (τ + step) is the pole for this step size.
        constexpr double kTauApply   = 0.050;
        constexpr double kTauRelease = 0.080;
        {
            const double tau   = (front_cmd > front_pressure)
                                     ? kTauApply : kTauRelease;
            const double alpha = step / (tau + step);
            front_pressure += alpha * (front_cmd - front_pressure);
            front_pressure  = std::clamp(front_pressure, 0.0, 1.0);
        }

        // Rear: rate limiter — clamp the per-step change.
        constexpr double kMaxRate = 3.333;   // 1.0 / 0.300 s
        {
            const double delta = std::clamp(rear_cmd - rear_position,
                                            -kMaxRate * step,
                                             kMaxRate * step);
            rear_position = std::clamp(rear_position + delta, 0.0, 1.0);
        }
    }
};
