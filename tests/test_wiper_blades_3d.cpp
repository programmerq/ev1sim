// WiperBlades3D — the two wipers on the windshield: the fitted glass surface,
// the arm/blade geometry against the manual's parked blade-tip heights, and
// the scene nodes on a null driver.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "WiperBlades3D.h"
#include "WiperRenderer.h"

#include <irrlicht.h>

#include <cmath>

using namespace ev1sim;
using Catch::Matchers::WithinAbs;

namespace {
double Dist(const DashVec3& a, const DashVec3& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}
}  // namespace

TEST_CASE("Windshield fit lands on the mesh's glass edge and roof line", "[wiper3d]") {
    // Bottom edge at the centreline and at y = 0.6, as read off the mesh.
    const DashVec3 c = WindshieldPoint(0.0, 0.0);
    CHECK_THAT(c.x, WithinAbs(1.087, 1e-9));
    CHECK_THAT(c.z, WithinAbs(0.8826, 1e-9));
    const DashVec3 side = WindshieldPoint(0.6, 0.0);
    CHECK_THAT(side.x, WithinAbs(0.943, 0.002));
    CHECK_THAT(side.z, WithinAbs(0.865, 0.002));
    // 0.82 m up the glass is the roof line (x ~0.35, z ~1.24).
    const DashVec3 top = WindshieldPoint(0.0, 0.82);
    CHECK_THAT(top.x, WithinAbs(0.35, 0.02));
    CHECK_THAT(top.z, WithinAbs(1.24, 0.02));
    // Lift moves the point off the glass, forward and up, by the lift.
    const DashVec3 lifted = WindshieldPoint(0.0, 0.3, 0.01);
    CHECK_THAT(Dist(lifted, WindshieldPoint(0.0, 0.3)), WithinAbs(0.01, 1e-4));
    CHECK(lifted.x > WindshieldPoint(0.0, 0.3).x);
    CHECK(lifted.z > WindshieldPoint(0.0, 0.3).z);
}

TEST_CASE("Parked blade tips sit at the manual's heights above the glass edge", "[wiper3d]") {
    // body-126 WIPER ARM installation step 3: driver 83 mm, passenger 67 mm.
    CHECK_THAT(WiperBladeTipS(kDriverWiper, 0.0), WithinAbs(0.083, 0.001));
    CHECK_THAT(WiperBladeTipS(kPassengerWiper, 0.0), WithinAbs(0.067, 0.001));
    // At the far end both blades stand near upright and stay on the glass.
    CHECK(WiperBladeTipS(kDriverWiper, 1.0) > 0.5);
    CHECK(WiperBladeTipS(kDriverWiper, 1.0) < 0.82);
    CHECK(WiperBladeTipS(kPassengerWiper, 1.0) < 0.82);
}

TEST_CASE("Wipers park along the bottom edge and sweep toward the driver's side", "[wiper3d]") {
    const WiperPose park = PlaceWiper(kDriverWiper, 0.0);
    const WiperPose up = PlaceWiper(kDriverWiper, 1.0);
    // Parked: the blade points across the car to the passenger side (-y).
    CHECK(park.blade_out.y < park.pivot.y);
    // Swept: the blade's outer end has risen up the glass (lower x, higher z).
    CHECK(up.blade_out.z > park.blade_out.z + 0.2);
    CHECK(up.blade_out.x < park.blade_out.x);
    // Lengths hold anywhere in the sweep (arm, blade centred on its tip).
    for (double f : {0.0, 0.3, 0.7, 1.0}) {
        const WiperPose w = PlaceWiper(kPassengerWiper, f);
        CHECK_THAT(Dist(w.pivot, w.arm_tip), WithinAbs(kPassengerWiper.arm_m, 0.03));
        CHECK_THAT(Dist(w.blade_in, w.blade_out), WithinAbs(kPassengerWiper.blade_m, 0.04));
    }
    // Out-of-range fractions clamp to the ends.
    CHECK_THAT(PlaceWiper(kDriverWiper, -1.0).blade_out.z, WithinAbs(park.blade_out.z, 1e-12));
    CHECK_THAT(PlaceWiper(kDriverWiper, 2.0).blade_out.z, WithinAbs(up.blade_out.z, 1e-12));
}

TEST_CASE("Wiper scene nodes follow the chassis and the renderer's sweep", "[wiper3d]") {
    irr::IrrlichtDevice* dev = irr::createDevice(irr::video::EDT_NULL,
                                                 irr::core::dimension2d<irr::u32>(320, 240));
    REQUIRE(dev != nullptr);
    {
        WiperBlades3D wipers(dev);
        REQUIRE(wipers.driver_blade() != nullptr);

        VehiclePose pose{};
        pose.px = 10.0;
        pose.py = -4.0;
        pose.pz = 0.5;
        pose.fx = 1.0;
        pose.uz = 1.0;

        // The renderer parked, then the plant at the far end.
        WiperRenderer r;
        wipers.Place(pose, r.GetSweepFraction());
        const DashVec3 park_mid = ChassisToWorld(pose, [] {
            const WiperPose w = PlaceWiper(kDriverWiper, 0.0);
            return DashVec3{(w.blade_in.x + w.blade_out.x) / 2, (w.blade_in.y + w.blade_out.y) / 2,
                            (w.blade_in.z + w.blade_out.z) / 2};
        }());
        const auto p0 = wipers.driver_blade()->getPosition();
        CHECK_THAT(p0.X, WithinAbs(park_mid.x, 1e-4));
        CHECK_THAT(p0.Y, WithinAbs(park_mid.y, 1e-4));
        CHECK_THAT(p0.Z, WithinAbs(park_mid.z, 1e-4));
        // Scaled to the blade's length along its local x.
        CHECK_THAT(wipers.driver_blade()->getScale().X, WithinAbs(kDriverWiper.blade_m, 0.04));

        r.SetPlantSweepPosition(std::uint8_t{50});
        wipers.Place(pose, r.GetSweepFraction());
        CHECK(wipers.driver_blade()->getPosition().Z > p0.Z + 0.1);
    }
    dev->drop();
}
