// DashScreens3D — the cluster and console as screens in the 3D scene
// (docs/ipc_rsa_display_plan.md Phase 3): chassis-frame placement math, and the
// real scene nodes on the null driver, checked through Irrlicht's own transform.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "DashScreens3D.h"

#include <irrlicht.h>

#include <cmath>

using namespace ev1sim;
using Catch::Approx;

namespace {

void RequireVec(const DashVec3& v, double x, double y, double z) {
    REQUIRE(v.x == Approx(x).margin(1e-9));
    REQUIRE(v.y == Approx(y).margin(1e-9));
    REQUIRE(v.z == Approx(z).margin(1e-9));
}

// A chassis at (10, 20, 0.5) turned 90 degrees left: forward = world +y,
// left = world -x.
VehiclePose TurnedLeft() {
    VehiclePose p;
    p.px = 10; p.py = 20; p.pz = 0.5;
    p.fx = 0; p.fy = 1; p.fz = 0;
    return p;
}

}  // namespace

TEST_CASE("Chassis-frame points follow the chassis pose", "[dash3d]") {
    RequireVec(ChassisToWorld(VehiclePose{}, {1, 2, 3}), 1, 2, 3);
    // forward 1 -> +y, left 2 -> -x, up 3 -> +z.
    RequireVec(ChassisToWorld(TurnedLeft(), {1, 2, 3}), 8, 21, 3.5);
}

TEST_CASE("An upright screen faces the rear with the car's right as its right", "[dash3d]") {
    const DashScreenFrame f = PlaceDashScreen(VehiclePose{}, {1, 0, 1, 0.0, 0.2, 0.2});
    RequireVec(f.center, 1, 0, 1);
    RequireVec(f.right, 0, -1, 0);   // car's right = -y (y is left)
    RequireVec(f.up, 0, 0, 1);
    RequireVec(f.normal, -1, 0, 0);  // toward the driver
}

TEST_CASE("Pitch tilts the screen's top edge forward, face toward the roof at 90", "[dash3d]") {
    const DashScreenFrame flat = PlaceDashScreen(VehiclePose{}, {0, 0, 0, 90.0, 0.2, 0.2});
    RequireVec(flat.up, 1, 0, 0);
    RequireVec(flat.normal, 0, 0, 1);

    const DashScreenFrame tilted = PlaceDashScreen(TurnedLeft(), kConsoleMount);
    // Normal stays perpendicular to the screen and has a rearward and an
    // upward part (the console faces the driver from below).
    REQUIRE(tilted.normal.y < 0.0);
    REQUIRE(tilted.normal.z > 0.0);
    const double dot = tilted.normal.x * tilted.up.x + tilted.normal.y * tilted.up.y +
                       tilted.normal.z * tilted.up.z;
    REQUIRE(dot == Approx(0.0).margin(1e-9));
}

TEST_CASE("Mounts sit ahead of the driver's eye, the console below the cluster", "[dash3d]") {
    REQUIRE(kClusterMount.x > kDriverEye.x);
    REQUIRE(kConsoleMount.x > kDriverEye.x);
    REQUIRE(kConsoleMount.z < kClusterMount.z);
    REQUIRE(kClusterMount.z < kDriverEye.z);
}

TEST_CASE("The cabin view keeps both screens within its field of view", "[dash3d]") {
    // Angle between the look direction and each screen centre, from the eye.
    const DashVec3 look = CabinLookPoint();
    const DashVec3 d{look.x - kDriverEye.x, look.y - kDriverEye.y, look.z - kDriverEye.z};
    auto angle_deg = [&](const DashScreenMount& m) {
        const DashVec3 v{m.x - kDriverEye.x, m.y - kDriverEye.y, m.z - kDriverEye.z};
        const double dot = d.x * v.x + d.y * v.y + d.z * v.z;
        const double n = std::sqrt((d.x * d.x + d.y * d.y + d.z * d.z) *
                                   (v.x * v.x + v.y * v.y + v.z * v.z));
        return std::acos(dot / n) * 180.0 / 3.14159265358979323846;
    };
    // Irrlicht's default camera spans 72 degrees vertically; keep a margin.
    REQUIRE(angle_deg(kClusterMount) < 30.0);
    REQUIRE(angle_deg(kConsoleMount) < 30.0);
}

TEST_CASE("Screens are real scene nodes placed on the chassis (null driver)", "[dash3d]") {
    irr::IrrlichtDevice* dev = irr::createDevice(irr::video::EDT_NULL,
                                                 irr::core::dimension2d<irr::u32>(640, 480));
    REQUIRE(dev != nullptr);
    {
        DashScreens3D screens(dev);
        REQUIRE(screens.cluster_node() != nullptr);
        REQUIRE(screens.console_node() != nullptr);
        // The null driver has no render targets: quads untextured, Refresh no-op.
        REQUIRE_FALSE(screens.HasTextures());
        screens.Refresh(dev->getGUIEnvironment()->getBuiltInFont());

        const VehiclePose pose = TurnedLeft();
        screens.Place(pose);
        auto* node = screens.cluster_node();
        node->updateAbsolutePosition();
        const DashScreenFrame f = PlaceDashScreen(pose, kClusterMount);

        // The quad's top-left corner, through Irrlicht's own node transform,
        // lands at centre - right*w/2 + up*h/2.
        irr::core::vector3df corner(static_cast<irr::f32>(-kClusterMount.width_m / 2),
                                    static_cast<irr::f32>(kClusterMount.height_m / 2), 0);
        node->getAbsoluteTransformation().transformVect(corner);
        const double hw = kClusterMount.width_m / 2, hh = kClusterMount.height_m / 2;
        REQUIRE(corner.X == Approx(f.center.x - f.right.x * hw + f.up.x * hh).margin(1e-4));
        REQUIRE(corner.Y == Approx(f.center.y - f.right.y * hw + f.up.y * hh).margin(1e-4));
        REQUIRE(corner.Z == Approx(f.center.z - f.right.z * hw + f.up.z * hh).margin(1e-4));

        // The panels inside take the snapshot like the HUD ones do.
        DashSnapshot snap;
        snap.rsa = DashRsaConsole{};
        snap.rsa->shift_blocked = true;
        screens.Update(snap);
        REQUIRE(screens.console_panel().view().shift_lock == L"Shift lock: BLOCKED");

        screens.SetVisible(false);
        REQUIRE_FALSE(node->isVisible());
    }
    // Destructor removed both nodes from the scene.
    REQUIRE(dev->getSceneManager()->getRootSceneNode()->getChildren().getSize() == 0u);
    dev->drop();
}

TEST_CASE("Screens tolerate a missing device", "[dash3d]") {
    DashScreens3D screens(nullptr);
    REQUIRE(screens.cluster_node() == nullptr);
    screens.Place(VehiclePose{});
    screens.Refresh(nullptr);
    screens.SetVisible(true);
}
