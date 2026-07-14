#pragma once

#include <algorithm>
#include <cmath>

namespace ev1sim {

/// Per-wheel longitudinal slip ratio (Chrono-free).
///
/// Braking convention:
///
///   slip = (v - omega * r) / v        clamped to [-1, +1]
///
/// where:
///   v     — vehicle speed magnitude (m/s)
///   omega — wheel angular speed magnitude (rad/s)
///   r     — tire rolling radius (m)
///
/// so 0 = free rolling, +1 = locked wheel, -1 = wheel spin.
///
/// The quotient is a NUMERICAL-VALIDITY-guarded quantity: it divides by v, so
/// near standstill it divides one sub-noise magnitude by another and rails to
/// the +/-1 clamp on a car that is, for practical purposes, stationary.  The
/// speed floor below defines "too slow to compute honestly"; under it the
/// slip ratio is reported as exactly 0.
///
/// Why 0.5 m/s and not lower: with the TMeasy tire and no true stiction
/// model, a braked-to-a-stop vehicle does not settle to exactly zero speed —
/// residual tire/suspension forces bleed off as a slow creep, measured at
/// ~0.10-0.12 m/s for several seconds in recorded braking-to-stop runs.  A
/// 0.1 m/s floor sat exactly on that creep band, so the reported slip railed
/// to a clamped +/-1 in crisp-edged bursts whenever |v| wandered across the
/// threshold.  The floor must sit safely ABOVE the standstill creep band;
/// 0.5 m/s clears it with margin while keeping real low-speed braking slip
/// (0.5-1.5 m/s) available.
///
/// Deliberately NOT aligned to any wheel-speed sensor's low-speed cutoff:
/// sensor floors belong to the consumer/module layer.  The plant reports
/// honestly-computable physics down to the numerical validity limit, and
/// consumers apply their own (typically higher) cutoffs on top.
///
/// All calculations are stateless and use double precision.
class SlipRatio {
public:
    /// Speed floor (m/s) below which the slip ratio is reported as 0.
    /// Must stay safely above the no-stiction standstill creep band
    /// (~0.12 m/s measured) — see the class comment.
    static constexpr double kSpeedFloorMps = 0.5;

    /// Longitudinal slip ratio for one wheel, braking convention, clamped to
    /// [-1, +1].  Returns 0 when the vehicle speed magnitude is at or below
    /// kSpeedFloorMps (numerical validity floor, not a sensor model).
    static double longitudinal(double vehicle_speed_mps,
                               double wheel_omega_rad_s,
                               double tire_radius_m) {
        const double v = std::abs(vehicle_speed_mps);
        if (v <= kSpeedFloorMps)
            return 0.0;
        const double vw = std::abs(wheel_omega_rad_s) * tire_radius_m;
        return std::clamp((v - vw) / v, -1.0, 1.0);
    }
};

}  // namespace ev1sim
