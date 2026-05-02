// Tests for the BrakeDrum self-energizing model.
//
// Covers torque magnitude vs (shoe force, wheel ω) and the inverse
// "required force for target torque" calculation.  These check the
// user-facing intuition about EMB drum brakes:
//   - At speed, self-energizing assist amplifies clamping force.
//   - At standstill, the assist collapses → much higher actuator force
//     needed to hold the same torque.
//   - The smooth_sign ramp makes the transition continuous through ω=0
//     so the simulator stays well-behaved.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "BrakeDrum.h"

using Catch::Matchers::WithinAbs;
using ev1sim::BrakeDrum;

TEST_CASE("BrakeDrum: zero shoe force → zero torque", "[BrakeDrum]") {
    BrakeDrum::Params p;
    CHECK(BrakeDrum::torque_magnitude_nm(0.0, 10.0, p) == 0.0);
    CHECK(BrakeDrum::torque_magnitude_nm(0.0, 0.0,  p) == 0.0);
    CHECK(BrakeDrum::torque_magnitude_nm(0.0, -50.0, p) == 0.0);
}

TEST_CASE("BrakeDrum: at speed, self-energizing factor is full (1+alpha)",
          "[BrakeDrum]") {
    BrakeDrum::Params p;  // mu=0.38, R=0.10, alpha=2.0, ω_thr=0.5

    // Above threshold ω=10 rad/s (well past the 0.5 ramp end).
    // T = 0.38 × 1000 × 0.10 × (1 + 2.0 × 1) = 0.38 × 100 × 3 = 114 N·m
    const double t = BrakeDrum::torque_magnitude_nm(1000.0, 10.0, p);
    CHECK_THAT(t, WithinAbs(114.0, 1e-9));

    // Same magnitude regardless of sign of ω (drum is symmetric for
    // double-leading shoe).
    CHECK_THAT(BrakeDrum::torque_magnitude_nm(1000.0, -10.0, p),
               WithinAbs(114.0, 1e-9));
}

TEST_CASE("BrakeDrum: at standstill, no self-energizing assist",
          "[BrakeDrum]") {
    BrakeDrum::Params p;
    // ω = 0 → sign_factor = 0 → T = mu × F × R × 1 = 0.38 × 1000 × 0.10 = 38 N·m
    const double t = BrakeDrum::torque_magnitude_nm(1000.0, 0.0, p);
    CHECK_THAT(t, WithinAbs(38.0, 1e-9));

    // Required clamping force at standstill is 3× the at-speed value
    // (1 / (1 + alpha) = 1/3 the effective μ).
    const double t_at_speed = BrakeDrum::torque_magnitude_nm(1000.0, 10.0, p);
    CHECK_THAT(t_at_speed / t, WithinAbs(3.0, 1e-9));
}

TEST_CASE("BrakeDrum: smooth_sign ramps linearly through zero",
          "[BrakeDrum]") {
    BrakeDrum::Params p;  // ω_threshold = 0.5

    // Half-threshold: sign_factor = 0.25/0.5 = 0.5 → assist = 0.5 × 2 = 1.0
    // T = 0.38 × 1000 × 0.10 × (1 + 1.0) = 76 N·m
    const double t_half = BrakeDrum::torque_magnitude_nm(1000.0, 0.25, p);
    CHECK_THAT(t_half, WithinAbs(76.0, 1e-9));

    // Quarter-threshold: sign_factor = 0.125/0.5 = 0.25 → assist = 0.5
    // T = 0.38 × 1000 × 0.10 × 1.5 = 57 N·m
    const double t_quart = BrakeDrum::torque_magnitude_nm(1000.0, 0.125, p);
    CHECK_THAT(t_quart, WithinAbs(57.0, 1e-9));

    // Zero ω → no assist, 38 N·m baseline (consistent with previous test).
    const double t_zero = BrakeDrum::torque_magnitude_nm(1000.0, 0.0, p);
    CHECK_THAT(t_zero, WithinAbs(38.0, 1e-9));
}

TEST_CASE("BrakeDrum: shoe force is clamped to [0, max_shoe_force_n]",
          "[BrakeDrum]") {
    BrakeDrum::Params p;  // max 4000 N

    // Exceed max — should be clamped.
    // T_at_speed_max = 0.38 × 4000 × 0.10 × 3 = 456 N·m
    CHECK_THAT(BrakeDrum::torque_magnitude_nm(8000.0, 10.0, p),
               WithinAbs(456.0, 1e-9));

    // Negative force clamps to zero.
    CHECK(BrakeDrum::torque_magnitude_nm(-100.0, 10.0, p) == 0.0);
}

TEST_CASE("BrakeDrum: required_shoe_force_n inverts torque_magnitude_nm",
          "[BrakeDrum]") {
    BrakeDrum::Params p;

    // At speed, alpha=2, so to hold 100 N·m we need
    // F = 100 / (mu × R × 3) = 100 / (0.38 × 0.10 × 3) = 877.2 N
    CHECK_THAT(BrakeDrum::required_shoe_force_n(100.0, 10.0, p),
               WithinAbs(877.19, 0.01));

    // At standstill (no assist), need 3× that force for the same torque.
    CHECK_THAT(BrakeDrum::required_shoe_force_n(100.0, 0.0, p),
               WithinAbs(2631.58, 0.01));

    // Round-trip: torque(F=required_for(T)) = T.
    for (double target : {10.0, 50.0, 100.0, 200.0}) {
        for (double omega : {10.0, 1.0, 0.4, 0.1}) {
            const double f = BrakeDrum::required_shoe_force_n(target, omega, p);
            const double t = BrakeDrum::torque_magnitude_nm(f, omega, p);
            CHECK_THAT(t, WithinAbs(target, 1e-9));
        }
    }
}

TEST_CASE("BrakeDrum: custom params are honored", "[BrakeDrum]") {
    BrakeDrum::Params p;
    p.mu = 0.30;
    p.drum_radius_m = 0.12;
    p.alpha = 1.5;
    p.max_shoe_force_n = 5000.0;
    p.omega_threshold_rad_s = 1.0;

    // F=2000 at ω=10 (above threshold): T = 0.30 × 2000 × 0.12 × (1+1.5) = 180 N·m
    CHECK_THAT(BrakeDrum::torque_magnitude_nm(2000.0, 10.0, p),
               WithinAbs(180.0, 1e-9));

    // ω = 0.5 (half new threshold): sign_factor = 0.5 → T = 0.30×2000×0.12×1.75 = 126
    CHECK_THAT(BrakeDrum::torque_magnitude_nm(2000.0, 0.5, p),
               WithinAbs(126.0, 1e-9));
}
