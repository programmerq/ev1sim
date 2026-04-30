#pragma once

#include <cmath>

namespace ev1sim {

/// Self-energizing drum brake model.
///
/// Computes the wheel-side braking torque produced by an electromechanical
/// rear brake actuator pushing shoes against a rotating drum.  The drum's
/// self-energizing geometry (the friction force itself rotates the shoe
/// further into the drum) amplifies the actuator's clamping force by a
/// factor that depends on rotation:
///
///   T_brake = mu * F_shoe * R_drum * (1 + alpha * smooth_sign(omega))
///
/// where:
///   mu             — drum-on-shoe friction coefficient (typical 0.35-0.40
///                    for phenolic resin lining on cast iron)
///   F_shoe         — actuator clamping force on the shoe pivot (N)
///   R_drum         — drum effective radius (m)
///   alpha          — self-energizing factor (≈ 2.0 for double-leading shoe)
///   smooth_sign(ω) — sign(ω) saturated to ±1 above ω_threshold, ramped
///                    linearly through zero so the model stays well-behaved
///                    as the wheel comes to rest.
///
/// At ω = 0 the smooth_sign is 0, so T_brake = mu * F_shoe * R_drum — the
/// pure clamping torque without any self-energizing assist.  The required
/// actuator force to maintain a given wheel torque is therefore (1 + alpha)
/// times higher at standstill than at speed: ~3× for alpha = 2.  This
/// matches the user's intuition about real drum brakes.
///
/// The returned torque is **always non-negative** — it represents the
/// magnitude of the braking torque.  Direction (opposing wheel rotation)
/// is the caller's responsibility to apply.
///
/// All calculations are stateless and use double precision.  The model is
/// intentionally simple — no thermal effects, no hysteresis, no
/// shoe-position dynamics.  Refinements can come once we have measured
/// EV1 EMB performance data.
class BrakeDrum {
public:
    struct Params {
        /// Drum-on-shoe friction coefficient.  Phenolic-resin lining
        /// against cast iron is typically 0.35–0.40 over the operating
        /// temperature range.
        double mu = 0.38;

        /// Effective drum radius in metres.  EV1 manuals don't quote
        /// this explicitly; ~8" (0.10 m) is typical for a passenger-car
        /// rear drum.
        double drum_radius_m = 0.10;

        /// Self-energizing factor.  Set ≈ 2.0 for a double-leading
        /// (twin-leading) shoe arrangement, ≈ 0 for a trailing-only
        /// shoe.  Real EV1 layout TBD; 2.0 is a conservative starting
        /// value.
        double alpha = 2.0;

        /// Maximum sustained shoe clamping force the EMB can produce
        /// (N).  Back-calculated from EV1 manual hints (motor current
        /// limits, fuse rating) — refine when better numbers land.
        /// 4000 N matches typical caliper clamping force on a similar
        /// passenger-car system.
        double max_shoe_force_n = 4000.0;

        /// Wheel angular velocity (rad/s) above which the smooth_sign
        /// saturates to ±1.  Below this we ramp linearly so the model
        /// is continuous through ω = 0.  0.5 rad/s ≈ slow walking pace
        /// at the wheel.
        double omega_threshold_rad_s = 0.5;
    };

    /// Compute brake torque magnitude (N·m, always >= 0) from
    /// commanded shoe force and current wheel angular velocity.
    ///
    /// @param shoe_force_n   commanded actuator clamping force (N), clamped
    ///                       to [0, max_shoe_force_n] internally.
    /// @param wheel_omega    wheel angular velocity (rad/s), signed.
    /// @param p              parameters (passenger-car drum defaults).
    static double torque_magnitude_nm(double shoe_force_n,
                                      double wheel_omega,
                                      const Params& p) {
        if (shoe_force_n < 0.0) shoe_force_n = 0.0;
        if (shoe_force_n > p.max_shoe_force_n) shoe_force_n = p.max_shoe_force_n;

        const double abs_omega = std::abs(wheel_omega);
        double sign_factor;
        if (abs_omega >= p.omega_threshold_rad_s) {
            sign_factor = 1.0;  // saturated
        } else {
            // Linear ramp from 0 at ω=0 to 1 at ω_threshold.  Magnitude
            // only — direction is the caller's problem.
            sign_factor = abs_omega / p.omega_threshold_rad_s;
        }

        return p.mu * shoe_force_n * p.drum_radius_m *
               (1.0 + p.alpha * sign_factor);
    }

    /// Inverse computation: given a desired braking torque and the
    /// current wheel angular velocity, what shoe force does the EMB
    /// have to produce?  Useful for diagnostics and for the EMB
    /// controller to check whether the requested torque is in reach.
    ///
    /// Returns the required shoe force (N).  Negative-omega and
    /// near-zero-omega cases produce LARGER required forces — exactly
    /// the behavior the user's intuition predicted.
    static double required_shoe_force_n(double target_torque_nm,
                                        double wheel_omega,
                                        const Params& p) {
        const double abs_omega = std::abs(wheel_omega);
        double sign_factor;
        if (abs_omega >= p.omega_threshold_rad_s) {
            sign_factor = 1.0;
        } else {
            sign_factor = abs_omega / p.omega_threshold_rad_s;
        }
        const double effective_mu = p.mu * (1.0 + p.alpha * sign_factor);
        if (effective_mu <= 0.0) return 0.0;  // defensive
        return target_torque_nm / (effective_mu * p.drum_radius_m);
    }
};

}  // namespace ev1sim
