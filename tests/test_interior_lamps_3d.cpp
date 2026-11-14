// InteriorLamps3D — the two interior lamps: brightness from the lamp's circuit
// and the dimming level, the mounts against the visual mesh, the scene nodes on
// a null driver, and the lights diagram's colour for each lamp.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "InteriorLamps3D.h"
#include "VehicleLights.h"

#include <irrlicht.h>

using namespace ev1sim;
using Catch::Matchers::WithinAbs;

TEST_CASE("Interior lamp brightness: circuit decides lit, level decides how bright",
          "[interior_lamps]") {
    // A dead circuit is dark at any level.
    CHECK(InteriorLampBrightness(false, std::nullopt) == 0.0);
    CHECK(InteriorLampBrightness(false, std::uint8_t{255}) == 0.0);
    // A live circuit follows the module's level.
    CHECK(InteriorLampBrightness(true, std::uint8_t{255}) == 1.0);
    CHECK_THAT(InteriorLampBrightness(true, std::uint8_t{51}), WithinAbs(0.2, 1e-12));
    // A live circuit with no level yet, or level 0, still shows lit.
    CHECK(InteriorLampBrightness(true, std::nullopt) == 1.0);
    CHECK(InteriorLampBrightness(true, std::uint8_t{0}) == 1.0);
}

TEST_CASE("Interior lamp mounts sit inside the cabin under the roof", "[interior_lamps]") {
    // Mirror map lamp below the mirror block (z 1.15-1.22 at x 0.29-0.33 on
    // the visual mesh), four-bar lamp under the headliner (z ~1.26 at
    // x -0.55), both on the centreline.
    CHECK(kMirrorMapLampMount.x > 0.28);
    CHECK(kMirrorMapLampMount.x < 0.34);
    CHECK(kMirrorMapLampMount.z < 1.15);
    CHECK(kFourBarLampMount.x < kMirrorMapLampMount.x);
    CHECK(kFourBarLampMount.z + kFourBarLampMount.thickness_m / 2.0 < 1.26);
    CHECK(kFourBarLampMount.y == 0.0);
    CHECK(kMirrorMapLampMount.y == 0.0);
    CHECK(&MountFor(InteriorLamp::kFourBar) == &kFourBarLampMount);
    CHECK(&MountFor(InteriorLamp::kMirrorMap) == &kMirrorMapLampMount);
}

TEST_CASE("Interior lamp scene nodes follow the chassis and light only when lit",
          "[interior_lamps]") {
    irr::IrrlichtDevice* dev = irr::createDevice(irr::video::EDT_NULL,
                                                 irr::core::dimension2d<irr::u32>(320, 240));
    REQUIRE(dev != nullptr);
    {
        InteriorLamps3D lamps(dev);
        REQUIRE(lamps.lens(InteriorLamp::kFourBar) != nullptr);
        REQUIRE(lamps.light(InteriorLamp::kFourBar) != nullptr);
        REQUIRE(lamps.light(InteriorLamp::kMirrorMap) != nullptr);
        CHECK_FALSE(lamps.light(InteriorLamp::kFourBar)->isVisible());   // created dark

        VehiclePose pose{};
        pose.px = 10.0;
        pose.py = -4.0;
        pose.pz = 0.5;
        pose.fx = 0.0;   // heading +y: the chassis turned 90 degrees left
        pose.fy = 1.0;
        pose.uz = 1.0;

        const double b[kNumInteriorLamps] = {0.5, 0.0};
        lamps.Update(pose, b);

        const DashVec3 bar = ChassisToWorld(
            pose, {kFourBarLampMount.x, kFourBarLampMount.y, kFourBarLampMount.z});
        const auto p = lamps.lens(InteriorLamp::kFourBar)->getPosition();
        CHECK_THAT(p.X, WithinAbs(bar.x, 1e-4));
        CHECK_THAT(p.Y, WithinAbs(bar.y, 1e-4));
        CHECK_THAT(p.Z, WithinAbs(bar.z, 1e-4));
        // The lens turns with the chassis: its length now runs along world y.
        irr::core::vector3df along(1.0f, 0.0f, 0.0f);
        irr::core::matrix4 rot;
        rot.setRotationDegrees(lamps.lens(InteriorLamp::kFourBar)->getRotation());
        rot.rotateVect(along);
        CHECK_THAT(along.X, WithinAbs(0.0, 1e-4));
        CHECK_THAT(along.Y, WithinAbs(1.0, 1e-4));

        // The four-bar lamp lights at half strength; the map lamp stays dark.
        auto* bar_light = lamps.light(InteriorLamp::kFourBar);
        CHECK(bar_light->isVisible());
        CHECK_THAT(bar_light->getLightData().DiffuseColor.r,
                   WithinAbs(0.5 * kInteriorLampColour[0], 1e-6));
        CHECK(bar_light->getPosition().Z < p.Z);   // hangs below its lens
        CHECK_FALSE(lamps.light(InteriorLamp::kMirrorMap)->isVisible());

        const double off[kNumInteriorLamps] = {0.0, 1.0};
        lamps.Update(pose, off);
        CHECK_FALSE(bar_light->isVisible());
        CHECK(lamps.light(InteriorLamp::kMirrorMap)->isVisible());
    }
    dev->drop();
}

TEST_CASE("Lights diagram colours an interior lamp by its brightness", "[interior_lamps]") {
    const auto dark = VehicleLights::InteriorLampHudColour(0.0);
    const auto full = VehicleLights::InteriorLampHudColour(1.0);
    const auto half = VehicleLights::InteriorLampHudColour(0.5);
    CHECK(dark.getRed() == 35u);     // the unlit bulb dot
    CHECK(full.getRed() == 255u);
    CHECK(full.getBlue() == 160u);   // warm white
    CHECK(half.getRed() > dark.getRed());
    CHECK(half.getRed() < full.getRed());
    // Out-of-range input clamps.
    CHECK(VehicleLights::InteriorLampHudColour(-1.0) == dark);
    CHECK(VehicleLights::InteriorLampHudColour(3.0) == full);
}
