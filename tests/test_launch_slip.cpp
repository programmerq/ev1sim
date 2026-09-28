// A healthy dry launch must not ring in wheel slip.
//
// WHY THIS TEST EXISTS
// --------------------
// On a dry, half-pedal launch (the opening of electricsim's VAT case
// safety_pps_triplet_fail, before its fault) the plant oscillated: from
// ~1.7 m/s to ~7 m/s the front slip swung +-20..30 % in ANTI-PHASE left vs
// right, and the undriven rears swung too, with no electricsim fleet
// attached.  In co-sim the BTCM read it as wheel spin and raised a traction
// retard request, and the PIM command dipped 128 -> 90: a false traction-
// control cut on a car that was not slipping.
//
// Cause: Chrono 9's TMeasy is a steady-state slip model applied explicitly
// over the step, and its slip "damping" on the wheel spin grows as 1/speed;
// below a speed proportional to the step it over-corrects every step.  It
// was a numerical instability (halved at 0.5 ms, gone at 0.25 ms), fixed by
// restoring TMeasy's first-order tire dynamics (src/EV1TMeasyTire.h).
//
// The test builds the real plant (ev1sim::BuildEV1Vehicle, the same builder
// the app uses), launches it at half pedal at the app's 1 ms step, and bounds
// two things through the band where it used to ring:
//   * left/right front slip difference — a straight launch on a uniform
//     surface is symmetric; the old mode was anti-phase, 0.79 peak-to-peak;
//   * each wheel's slip about its own 100 ms running mean — catches in-phase
//     ringing and the undriven rears (old: +-0.29 on the rear).
// Slip is computed exactly as the telemetry column the BTCM-facing plots use
// (SlipRatio::longitudinal, 0.2915 m radius).

#include <catch2/catch_test_macros.hpp>

#include "EV1TMeasyTire.h"
#include "EV1Vehicle.h"
#include "SlipRatio.h"

#include "chrono/core/ChGlobal.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace chrono;
using namespace chrono::vehicle;

namespace {

constexpr double kStep = 1e-3;            // the app's step (config simulation.step_size_s)
constexpr double kTireRadius = 0.2915;    // EV1_TMeasyTire.json "Unloaded Radius [m]"
// [s] pedal down at 1.5 s, as the VAT case does.  The car is spawned 0.5 m up
// and its landing rings the tires until ~1.1 s; a launch pressed into that
// landing measures the drop, not the launch.
constexpr double kThrottleOn = 1.5;
constexpr double kPedal = 0.5;            // half pedal, as the VAT case holds
constexpr double kWindowStart = kThrottleOn + 0.5;
constexpr double kWindowEnd = kThrottleOn + 4.0;  // past ~7 m/s, where it used to die out
constexpr int kSampleEvery = 5;           // 5 ms samples

struct Sample {
    double t, speed;
    std::array<double, 4> slip;  // FL FR RL RR
};

std::vector<Sample> RunLaunch() {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);

    auto ev1 = ev1sim::BuildEV1Vehicle(ChCoordsys<>(ChVector3d(0, 0, 0.5), QUNIT), kStep);

    RigidTerrain terrain(ev1->GetSystem());
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);  // dry asphalt, as config/default.json
    mat->SetRestitution(0.01f);
    terrain.AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 400.0, 400.0);
    terrain.Initialize();

    std::vector<Sample> out;
    DriverInputs in{};
    for (int n = 0;; ++n) {
        const double t = ev1->GetSystem()->GetChTime();
        if (t > kWindowEnd)
            break;
        in.m_throttle = (t >= kThrottleOn) ? kPedal : 0.0;
        terrain.Synchronize(t);
        ev1->Synchronize(t, in, terrain);
        terrain.Advance(kStep);
        ev1->Advance(kStep);

        if (n % kSampleEvery == 0) {
            Sample s{t, ev1->GetSpeed(), {}};
            int i = 0;
            for (int a = 0; a < 2; ++a)
                for (auto side : {LEFT, RIGHT})
                    s.slip[i++] = ev1sim::SlipRatio::longitudinal(
                        s.speed, ev1->GetSpindleOmega(a, side), kTireRadius);
            out.push_back(s);
        }
    }
    return out;
}

}  // namespace

TEST_CASE("Launch: a dry half-pedal launch has no sustained wheel-slip oscillation",
          "[plant][tire]") {
    const auto run = RunLaunch();

    double max_lr_front = 0.0;
    std::array<double, 4> max_dev{};
    constexpr int kHalf = 10;  // +-50 ms -> 100 ms running mean
    for (size_t k = kHalf; k + kHalf < run.size(); ++k) {
        if (run[k].t < kWindowStart || run[k].t > kWindowEnd)
            continue;
        max_lr_front = std::max(max_lr_front, std::abs(run[k].slip[0] - run[k].slip[1]));
        for (int w = 0; w < 4; ++w) {
            double mean = 0.0;
            for (size_t j = k - kHalf; j <= k + kHalf; ++j)
                mean += run[j].slip[w];
            mean /= (2 * kHalf + 1);
            max_dev[w] = std::max(max_dev[w], std::abs(run[k].slip[w] - mean));
        }
    }
    INFO("max |slip_FL - slip_FR| = " << max_lr_front);
    INFO("max slip deviation from 100 ms mean FL/FR/RL/RR = " << max_dev[0] << " / "
         << max_dev[1] << " / " << max_dev[2] << " / " << max_dev[3]);
    CHECK(max_lr_front < 0.01);
    for (int w = 0; w < 4; ++w)
        CHECK(max_dev[w] < 0.01);

    // And it is a real launch through the band that used to ring (~1.7-7 m/s):
    // the fix must not have bought quiet by stalling the car.
    INFO("speed at end of window = " << run.back().speed);
    CHECK(run.back().speed > 7.0);
}

TEST_CASE("Launch: the EV1 tire carries its longitudinal tire dynamics",
          "[plant][tire]") {
    SetChronoDataPath(CHRONO_DATA_DIR);
    vehicle::SetDataPath(EV1SIM_VEHICLE_DATA_DIR);
    ev1sim::EV1TMeasyTire tire(GetDataFile("ev1/tire/EV1_TMeasyTire.json"));
    // Without the block the class is plain Chrono 9 TMeasy, which rings.
    CHECK(tire.GetLongitudinalTireStiffness() == 234900.0);
    CHECK(tire.GetLongitudinalTireDamping() == 664.0);
}
