// Unit tests for the Chrono-free body-aerodynamics model (src/Aerodynamics.h).
//
// VehicleWorld feeds these same constants into Chrono's
// ChChassis::SetAerodynamicDrag, which applies F = 0.5·rho·Cd·A·v² internally.
// Pinning the formula + the EV1 constants here makes the applied drag
// verifiable without booting Chrono.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "Aerodynamics.h"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using ev1sim::Aerodynamics;

TEST_CASE("Aerodynamics: zero speed produces zero drag and zero power", "[Aero]") {
    CHECK(Aerodynamics::drag_force_n(0.0) == 0.0);
    CHECK(Aerodynamics::drag_power_w(0.0) == 0.0);
}

TEST_CASE("Aerodynamics: drag scales with the square of speed", "[Aero]") {
    const double f10 = Aerodynamics::drag_force_n(10.0);
    const double f20 = Aerodynamics::drag_force_n(20.0);
    CHECK(f10 > 0.0);
    CHECK_THAT(f20 / f10, WithinRel(4.0, 1e-12));   // 2× speed → 4× force
}

TEST_CASE("Aerodynamics: drag is sign-independent (magnitude only)", "[Aero]") {
    CHECK_THAT(Aerodynamics::drag_force_n(-15.0),
               WithinRel(Aerodynamics::drag_force_n(15.0), 1e-12));
}

TEST_CASE("Aerodynamics: EV1 drag matches the hand-computed value at 100 km/h", "[Aero]") {
    const double v = 100.0 / 3.6;                    // 27.78 m/s
    // 0.5 · 1.225 · 0.19 · 1.89 · v²  ≈  169.7 N
    CHECK_THAT(Aerodynamics::drag_force_n(v), WithinAbs(169.7, 0.5));
    CHECK_THAT(Aerodynamics::cda_m2(),        WithinAbs(0.3591, 1e-3));
    CHECK_THAT(Aerodynamics::drag_power_w(v), WithinAbs(169.7 * v, 20.0));
}

TEST_CASE("Aerodynamics: published EV1 constants are wired", "[Aero]") {
    CHECK(Aerodynamics::kEV1DragCoefficient == 0.19);
    CHECK(Aerodynamics::kEV1FrontalAreaM2  == 1.89);
    CHECK_THAT(Aerodynamics::kAirDensityIsaSeaLevel, WithinAbs(1.225, 1e-12));
}

TEST_CASE("Aerodynamics: custom params override the EV1 defaults", "[Aero]") {
    Aerodynamics::Params p;
    p.cd = 0.30; p.frontal_area = 2.0; p.air_density = 1.0;
    // 0.5 · 1.0 · 0.30 · 2.0 · 10² = 30 N
    CHECK_THAT(Aerodynamics::drag_force_n(10.0, p), WithinAbs(30.0, 1e-9));
    CHECK_THAT(Aerodynamics::cda_m2(p),             WithinAbs(0.60, 1e-12));
}
