// Rear-EMB fallback under an integrated stop: front base braking persists when
// the BTCM dies.
//
// test_brake_decel.cpp pins the same property with the rear forced to a 0
// ratio. This file drives the rear through the models SimApp actually runs per
// corner (RearEmbClamp -> BrakeDrum -> torque/kRearBrakeMaxTorqueNm) and kills
// the BTCM part-way through the stop, so what is checked is the transition:
// the rear clamp letting go the step the commands go stale, and the front
// carrying on with the driver's pedal.
//
// What the car does when the BTCM is not driving the brakes:
//   - Front: the brake solenoid valves are normally open; the BTCM closes them
//     to isolate the master cylinder from the front calipers when it applies
//     the brakes (@source:manual brakes-308). Unpowered, they stay open, so
//     master-cylinder pressure reaches the front calipers: the front brakes
//     "revert to manual operation" with rear brake apply disabled
//     (@source:manual brakes-222). SimApp::ApplyAbsFrontBrake models this by
//     leaving the driver's symmetric front pressure in effect when stale.
//   - Rear: purely electromechanical, no hydraulic backup (@source:manual
//     brakes-312), so a stale BTCM is zero rear force (RearEmbClamp).
//   - One rear corner can fail on its own: "the BTCM will disable front ABS
//     and left rear brake apply. The front brakes, the right rear brake system
//     and regen will function normally" (@source:manual brakes-254 DTC 147).
//
// The plant is the point-mass longitudinal integrator test_brake_decel.cpp
// uses, with no tyre slip (wheel speed = v / r) and the tyre-friction ceiling.
// No Chrono.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "BrakeActuator.h"
#include "BrakeDrum.h"
#include "RearEmbClamp.h"

#include <algorithm>
#include <cstddef>
#include <vector>

using Catch::Matchers::WithinRel;
using ev1sim::BrakeDrum;
using ev1sim::RearEmbClamp;

namespace {

// Same plant constants as test_brake_decel.cpp (docs/ev1_chrono_audit.md).
constexpr double kCurbMassKg       = 1281.0;
constexpr double kTireRadiusM      = 0.2915;
constexpr double kTireMu           = 0.8;
constexpr double kGravity          = 9.80665;
constexpr double kFrontMaxTorqueNm = 1120.0;  // EV1_BrakeSimple_Front.json
constexpr double kStep             = 0.002;   // config default step_size_s
constexpr double kStopThresh       = 0.05;    // m/s

// When each rear corner's BTCM command goes stale (s); negative = never.
struct Faults {
    double lr_stale_at_s = -1.0;
    double rr_stale_at_s = -1.0;
    double lr_back_at_s  = -1.0;  // fresh again (BTCM recovers)
};

struct Sample {
    double t_s;
    double decel_mps2;
    double front_force_n;
    double rear_lr_shoe_n;
    double rear_rr_shoe_n;
    double rear_force_n;
};

struct Stop {
    double distance_m = 0.0;
    double time_s     = 0.0;
    bool stopped      = false;
    std::vector<Sample> trace;
};

// A full-pedal stop from v0. A live BTCM drives each rear motor in APPLY while
// the pedal is down (+1), as the BTCM does for a non-ABS stop on high grip
// where the rear never needs modulating.
Stop run_stop(double v0_mps, double pedal, const Faults& f) {
    const BrakeDrum::Params drum;
    RearEmbClamp lr;
    RearEmbClamp rr;
    BrakeActuator front;  // only its hydraulic-front lag is used
    const double friction_ceiling_n = kTireMu * kCurbMassKg * kGravity;

    auto live = [](double t, double stale_at, double back_at) {
        if (stale_at < 0.0 || t < stale_at) return true;
        return back_at >= 0.0 && t >= back_at;
    };
    auto rear_ratio = [&](double shoe_n, double omega) {
        const double tq = BrakeDrum::torque_magnitude_nm(shoe_n, omega, drum);
        return std::clamp(tq / ev1sim::kRearBrakeMaxTorqueNm, 0.0, 1.0);
    };

    Stop s;
    double v = v0_mps;
    double t = 0.0;
    while (v > kStopThresh && t < 60.0) {
        const double omega = v / kTireRadiusM;
        const bool lr_live = live(t, f.lr_stale_at_s, f.lr_back_at_s);
        const bool rr_live = live(t, f.rr_stale_at_s, -1.0);
        const float cmd = pedal > 0.0 ? +1.0f : 0.0f;

        // Front: driver pedal through the hydraulic lag, whatever the BTCM.
        front.Advance(kStep, pedal, 0.0);
        const double front_force =
            2.0 * front.front_pressure * kFrontMaxTorqueNm / kTireRadiusM;

        const double lr_shoe = lr.Update(cmd, lr_live, drum, kStep);
        const double rr_shoe = rr.Update(cmd, rr_live, drum, kStep);
        const double rear_force = (rear_ratio(lr_shoe, omega) +
                                   rear_ratio(rr_shoe, omega)) *
                                  ev1sim::kRearBrakeMaxTorqueNm / kTireRadiusM;

        const double force = std::min(front_force + rear_force, friction_ceiling_n);
        const double decel = force / kCurbMassKg;
        s.trace.push_back({t, decel, front_force, lr_shoe, rr_shoe, rear_force});

        const double v_next = std::max(0.0, v - decel * kStep);
        s.distance_m += 0.5 * (v + v_next) * kStep;
        v = v_next;
        t += kStep;
    }
    s.time_s = t;
    s.stopped = v <= kStopThresh;
    return s;
}

const Sample& at(const Stop& s, double t_s) {
    const auto i = static_cast<std::size_t>(t_s / kStep + 0.5);
    REQUIRE(i < s.trace.size());
    return s.trace[i];
}

}  // namespace

// Half pedal keeps the healthy stop below the tyre-friction ceiling, so losing
// the rear shows in the deceleration instead of being hidden by saturation.
constexpr double kPedal = 0.5;
constexpr double kV0    = 25.0;   // m/s
constexpr double kDeath = 1.0;    // s into the stop

TEST_CASE("RearEmbFallback: the rear lets go the step the BTCM dies",
          "[RearEmbFallback][BTCMLoss]") {
    Faults f;
    f.lr_stale_at_s = kDeath;
    f.rr_stale_at_s = kDeath;
    const Stop s = run_stop(kV0, kPedal, f);

    const Sample& before = at(s, kDeath - kStep);
    const Sample& after  = at(s, kDeath);
    CHECK(before.rear_lr_shoe_n > 0.0);
    CHECK(before.rear_rr_shoe_n > 0.0);
    CHECK(after.rear_lr_shoe_n == 0.0);
    CHECK(after.rear_rr_shoe_n == 0.0);
    CHECK(after.rear_force_n == 0.0);
    // And it stays off for the rest of the stop.
    for (const Sample& x : s.trace) {
        if (x.t_s >= kDeath) {
            CHECK(x.rear_force_n == 0.0);
        }
    }
}

TEST_CASE("RearEmbFallback: front base braking persists on BTCM death",
          "[RearEmbFallback][BTCMLoss][Safety]") {
    Faults f;
    f.lr_stale_at_s = kDeath;
    f.rr_stale_at_s = kDeath;
    const Stop dead    = run_stop(kV0, kPedal, f);
    const Stop healthy = run_stop(kV0, kPedal, Faults{});

    // The front force is the driver's pedal either side of the death: the
    // BTCM going away changes nothing at the front.
    const Sample& before = at(dead, kDeath - kStep);
    const Sample& after  = at(dead, kDeath);
    CHECK_THAT(after.front_force_n, WithinRel(before.front_force_n, 1e-3));
    CHECK(after.front_force_n > 0.0);

    // After the death the car decelerates on the front alone, at the rate the
    // pedal asks of the front calipers (the front's share of the brake
    // budget), and less than it did with the rear helping.
    const double pedal_front =
        kPedal * 2.0 * kFrontMaxTorqueNm / kTireRadiusM / kCurbMassKg;
    CHECK_THAT(after.decel_mps2, WithinRel(pedal_front, 1e-3));
    CHECK(after.decel_mps2 < before.decel_mps2);

    // The car stops. Losing the rear costs distance; it does not lose the stop.
    CHECK(dead.stopped);
    CHECK(healthy.stopped);
    CHECK(dead.distance_m > healthy.distance_m);
    // Bounded: no worse than a front-only stop from the start.
    Faults from_start;
    from_start.lr_stale_at_s = 0.0;
    from_start.rr_stale_at_s = 0.0;
    const Stop front_only_stop = run_stop(kV0, kPedal, from_start);
    CHECK(front_only_stop.stopped);
    CHECK(dead.distance_m < front_only_stop.distance_m);
}

TEST_CASE("RearEmbFallback: one rear corner failing leaves the other braking",
          "[RearEmbFallback][BTCMLoss]") {
    Faults f;
    f.lr_stale_at_s = kDeath;  // left rear only (DTC 147-style)
    const Stop s = run_stop(kV0, kPedal, f);
    const Sample& after = at(s, kDeath + 0.1);
    CHECK(after.rear_lr_shoe_n == 0.0);
    CHECK(after.rear_rr_shoe_n > 0.0);
    CHECK(after.front_force_n > 0.0);
    CHECK(s.stopped);
}

TEST_CASE("RearEmbFallback: a recovered BTCM re-applies from home",
          "[RearEmbFallback][BTCMLoss]") {
    // The actuator went home when the BTCM died, so on recovery the motor has
    // to take up the clearance again before the left rear brakes.
    constexpr double kBack = kDeath + 0.5;
    Faults f;
    f.lr_stale_at_s = kDeath;
    f.lr_back_at_s  = kBack;
    const Stop s = run_stop(kV0, kPedal, f);
    const RearEmbClamp::Travel tr;

    CHECK(at(s, kBack).rear_lr_shoe_n == 0.0);
    CHECK(at(s, kBack + tr.clearance_s - 2 * kStep).rear_lr_shoe_n == 0.0);
    CHECK(at(s, kBack + tr.clearance_s + 2 * kStep).rear_lr_shoe_n > 0.0);
    // ...and builds back to the full force after the rise.
    const BrakeDrum::Params drum;
    CHECK_THAT(at(s, kBack + tr.clearance_s + tr.rise_s + 2 * kStep).rear_lr_shoe_n,
               WithinRel(drum.max_shoe_force_n, 1e-9));
}
