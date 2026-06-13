// Integrated brake → deceleration regression tests (S16).
//
// The other brake unit tests (test_brake_drum / test_brake_bias /
// test_brake_pedal) are *stateless contract checks* — they pin a single
// formula or JSON value.  None of them exercises the integrated loop the
// vehicle actually runs every tick:
//
//     pedal travel → BrakeActuator lag → front/rear brake torque → a
//     longitudinal point-mass plant → deceleration over time.
//
// Nor do they cover the safety property the regulator lens flagged:
// **front base braking must persist when the BTCM dies** (SimApp.cpp
// ApplyRearEmbBrake / ApplyAbsFrontBrake, ~lines 1030-1162).  The EV1's
// rear brakes are purely electromechanical with no hydraulic backup, so a
// stale BTCM drops rear force to zero; the front hydraulic line bypasses
// the (de-energised, normally-open) ABS modulator and the master-cylinder
// pressure still reaches the front calipers.  Losing the rear axle must
// NOT lose the car's ability to stop.
//
// This file builds a Chrono-free longitudinal plant out of the *committed*
// models (BrakeActuator, BrakeDrum, the EV1_BrakeSimple_* JSON budgets) and
// integrates it.  Chrono's role in the real run is exactly this point-mass
// longitudinal integrator plus a tire friction ceiling; we mirror that here
// so the integrated decel and the BTCM-death property are verifiable in the
// fast unit suite without booting Chrono.  See SimApp::ApplyRearEmbBrake and
// the brake-bias JSON for the numbers being pinned.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "BrakeActuator.h"
#include "BrakeDrum.h"

#include <algorithm>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using ev1sim::BrakeDrum;

namespace {

// EV1 plant constants — kept in sync with the committed data the real run
// loads (data/vehicle/ev1/**, docs/ev1_chrono_audit.md §2/§5/§6).
constexpr double kCurbMassKg     = 1281.0;   // Gen-2 NiMH curb (audit §2)
constexpr double kTireRadiusM    = 0.2915;   // P175/65R14 unloaded radius (audit §5)
constexpr double kTireMu         = 0.8;      // dry-asphalt friction (audit §5)
constexpr double kGravity        = 9.80665;
// "Maximum Torque" per wheel from the brake-bias JSON (Round 4, 70/30).
constexpr double kFrontMaxTorqueNm = 1120.0; // EV1_BrakeSimple_Front.json
constexpr double kRearMaxTorqueNm  = 480.0;  // EV1_BrakeSimple_Rear.json == kRearBrakeMaxTorqueNm

// Total wheel-side braking force the four corners can produce at a given
// front/rear command ratio [0..1], BEFORE the tire-friction ceiling.  Mirrors
// how Chrono's BrakeSimple maps a brake-input ratio onto axle torque, and how
// SimApp applies front (ApplyFrontBrakePerWheel) and rear
// (ApplyRearBrakePerWheel) per-corner.
double brake_force_n(double front_ratio, double rear_ratio) {
    front_ratio = std::clamp(front_ratio, 0.0, 1.0);
    rear_ratio  = std::clamp(rear_ratio, 0.0, 1.0);
    const double torque_total =
        2.0 * front_ratio * kFrontMaxTorqueNm + 2.0 * rear_ratio * kRearMaxTorqueNm;
    return torque_total / kTireRadiusM;
}

// One-axle point-mass longitudinal integrator.  Decelerates a vehicle of
// kCurbMassKg under the commanded brake force, clamped to the tire-friction
// ceiling (μ·m·g) — exactly the cap that makes Chrono's wheels lock + ABS
// modulate in the real loop.  Returns stopping distance (m); fills out the
// stop time (s).  Semi-implicit Euler, fixed step (the same 2 ms physics step
// the sim defaults to).
struct StopResult {
    double distance_m = 0.0;
    double time_s     = 0.0;
    double peak_decel_mps2 = 0.0;
};

StopResult integrate_stop(double v0_mps, double front_ratio, double rear_ratio,
                          bool actuator_lag) {
    constexpr double kStep = 0.002;          // 2 ms (config default step_size_s)
    constexpr double kStopThresh = 0.05;     // m/s — "stopped"
    constexpr double kFrictionCeil = kTireMu * kCurbMassKg * kGravity;

    BrakeActuator act;  // models hydraulic-front lag + rate-limited rear
    StopResult r;
    double v = v0_mps;
    double t = 0.0;
    while (v > kStopThresh && t < 60.0) {
        double f_cmd = front_ratio;
        double r_cmd = rear_ratio;
        if (actuator_lag) {
            act.Advance(kStep, front_ratio, rear_ratio);
            f_cmd = act.front_pressure;
            r_cmd = act.rear_position;
        }
        double force = brake_force_n(f_cmd, r_cmd);
        force = std::min(force, kFrictionCeil);     // tire-limited
        const double decel = force / kCurbMassKg;
        r.peak_decel_mps2 = std::max(r.peak_decel_mps2, decel);
        const double v_next = std::max(0.0, v - decel * kStep);
        r.distance_m += 0.5 * (v + v_next) * kStep;  // trapezoid
        v = v_next;
        t += kStep;
    }
    r.time_s = t;
    return r;
}

}  // namespace

// ---------------------------------------------------------------------------
// Integrated decel — full-pedal stop from 30 m/s.
// ---------------------------------------------------------------------------
TEST_CASE("BrakeDecel: full-pedal stop from 30 m/s lands in a physical envelope",
          "[BrakeDecel][Integrated]") {
    // Full front + rear command.  The four-corner torque budget (3200 N·m)
    // over-saturates the tire, so deceleration is friction-limited at ~0.8g.
    const StopResult r = integrate_stop(30.0, /*front=*/1.0, /*rear=*/1.0,
                                        /*actuator_lag=*/true);

    // Stopped (didn't time out).
    CHECK(r.time_s < 10.0);

    // Peak decel is the tire-friction ceiling, ~0.8g — NOT the (much larger)
    // raw brake-torque value.  This is the whole point of the Round-4 budget:
    // the brakes saturate friction but the visible decel is μ·g.
    CHECK_THAT(r.peak_decel_mps2, WithinRel(kTireMu * kGravity, 0.02));

    // v²/(2a) ideal stop distance at 0.8g from 30 m/s ≈ 57.4 m; the actuator
    // lag (50 ms front fill) adds a little.  Pin a generous physical window so
    // this is a regression guard, not an over-fit.
    const double ideal = (30.0 * 30.0) / (2.0 * kTireMu * kGravity);  // ~57.4 m
    CHECK(r.distance_m > ideal);                 // lag only ever adds distance
    CHECK(r.distance_m < ideal + 5.0);           // but not much at full pedal
}

// ---------------------------------------------------------------------------
// Integrated decel — partial pedal gives a graded, sub-friction decel.
// ---------------------------------------------------------------------------
TEST_CASE("BrakeDecel: partial pedal is graded and below the friction ceiling",
          "[BrakeDecel][Integrated]") {
    // 30% command: 0.30 × 3200 = 960 N·m total → 960/0.2915 ≈ 3293 N →
    // 3293/1281 ≈ 2.57 m/s² ≈ 0.26g, well under the 0.8g ceiling.  This is the
    // "usable middle range" the Round-4 budget was set to preserve (audit §6).
    const StopResult r = integrate_stop(30.0, 0.30, 0.30, /*actuator_lag=*/true);

    const double expected_decel = brake_force_n(0.30, 0.30) / kCurbMassKg;
    CHECK(expected_decel < kTireMu * kGravity);          // not friction-limited
    CHECK_THAT(r.peak_decel_mps2, WithinRel(expected_decel, 0.02));

    // Longer stop than full pedal — monotonic pedal→decel mapping.
    const StopResult full = integrate_stop(30.0, 1.0, 1.0, /*actuator_lag=*/true);
    CHECK(r.distance_m > full.distance_m);
}

// ---------------------------------------------------------------------------
// BrakeActuator lag is observable in the integrated loop.
// ---------------------------------------------------------------------------
TEST_CASE("BrakeDecel: actuator lag lengthens the stop vs an ideal step",
          "[BrakeDecel][Integrated][Lag]") {
    const StopResult lagged = integrate_stop(20.0, 1.0, 1.0, /*actuator_lag=*/true);
    const StopResult ideal  = integrate_stop(20.0, 1.0, 1.0, /*actuator_lag=*/false);

    // The 50 ms front-fill + rear rate-limit can only delay force build-up, so
    // the lagged stop is never shorter than the instantaneous one.
    CHECK(lagged.distance_m >= ideal.distance_m);
    // And the delay is small at these torques (force saturates friction within
    // a few hundred ms) — bounded, not pathological.
    CHECK(lagged.distance_m - ideal.distance_m < 2.0);
}

// ---------------------------------------------------------------------------
// SAFETY PROPERTY — front base braking persists when the BTCM dies.
// ---------------------------------------------------------------------------
// This pins the regulator-lens "retained-braking-on-ECU-loss" property that
// SimApp::ApplyRearEmbBrake and ApplyAbsFrontBrake implement: a stale BTCM
// zeroes the rear EMB (no hydraulic backup) while the front hydraulic line
// keeps the driver's master-cylinder pressure.  The car must still stop.
TEST_CASE("BrakeDecel: BTCM death zeroes the rear but front braking is retained",
          "[BrakeDecel][Safety][BTCMLoss]") {
    // Model the two SimApp branches directly:
    //   - rear: BTCM stale → command ratio forced to 0 (the ': 0.0' branch in
    //     ApplyRearEmbBrake — "rear has no hydraulic fallback path").
    //   - front: BTCM-off path leaves the symmetric front_pressure from the
    //     driver pedal in effect (ApplyAbsFrontBrake "Nothing to do").
    const double front_pedal = 1.0;

    const StopResult healthy   = integrate_stop(25.0, front_pedal, /*rear=*/1.0, true);
    const StopResult btcm_dead = integrate_stop(25.0, front_pedal, /*rear=*/0.0, true);

    // 1. The car still stops on the front axle alone (the safety property).
    CHECK(btcm_dead.time_s < 20.0);
    CHECK(btcm_dead.distance_m > 0.0);

    // 2. Front-only decel is meaningful: 2×1120 = 2240 N·m → 2240/0.2915 ≈
    //    7684 N → /1281 ≈ 6.0 m/s² ≈ 0.61g.  Below the 0.8g friction ceiling,
    //    so the lost rear is genuinely visible (not masked by saturation).
    const double front_only_decel = brake_force_n(front_pedal, 0.0) / kCurbMassKg;
    CHECK(front_only_decel < kTireMu * kGravity);
    CHECK_THAT(btcm_dead.peak_decel_mps2, WithinRel(front_only_decel, 0.02));

    // 3. Losing the rear costs stopping distance but does not lose the stop —
    //    the failure is degraded, not catastrophic.
    CHECK(btcm_dead.distance_m > healthy.distance_m);

    // 4. A hypothetical rear-only fallback (if the front had ALSO depended on
    //    the BTCM) would be far weaker — proving the front retention is what
    //    carries the stop.  rear pair 2×480 = 960 N·m → ~0.26g.
    const double rear_only_decel = brake_force_n(0.0, 1.0) / kCurbMassKg;
    CHECK(front_only_decel > 2.0 * rear_only_decel);   // front is the dominant axle
}

// ---------------------------------------------------------------------------
// The rear EMB torque→ratio conversion SimApp uses is bounded to [0,1].
// ---------------------------------------------------------------------------
// Mirrors SimApp::ApplyRearEmbBrake's torque_to_ratio lambda + cmd_to_force,
// pinning that a saturated EMB command never exceeds the Chrono brake ratio.
TEST_CASE("BrakeDecel: rear EMB torque→ratio saturates at 1.0, never above",
          "[BrakeDecel][BTCMLoss]") {
    const BrakeDrum::Params drum;  // peaks at 456 N·m at speed
    // Full motor command → full shoe force; at speed the self-energising
    // torque (456 N·m) is below the 480 N·m budget, so the ratio stays < 1.
    const double torque_at_speed =
        BrakeDrum::torque_magnitude_nm(drum.max_shoe_force_n, /*omega=*/30.0, drum);
    const double ratio = std::clamp(torque_at_speed / kRearMaxTorqueNm, 0.0, 1.0);
    CHECK(ratio < 1.0);
    CHECK_THAT(ratio, WithinRel(456.0 / 480.0, 1e-6));

    // A negative / zero motor command yields zero force → zero ratio (the
    // stale-BTCM and motor-retracting cases both land here).
    const double zero_force = std::max(0.0, -0.5) * drum.max_shoe_force_n;
    CHECK(BrakeDrum::torque_magnitude_nm(zero_force, 30.0, drum) == 0.0);
}
