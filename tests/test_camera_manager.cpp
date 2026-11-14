// CameraManager — mode cycling and names, the config string, zoom limits,
// where each mode puts a real camera node (null driver), and the mouse:
// wheel zoom in every mode, free-look orbit drag, and UI-mode pass-through.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "CameraManager.h"
#include "DashScreens3D.h"

#include <irrlicht.h>

#include <cmath>

using Catch::Matchers::WithinAbs;

namespace {

struct Rig {
    irr::IrrlichtDevice* dev = nullptr;
    irr::scene::ICameraSceneNode* cam = nullptr;
    Rig() {
        dev = irr::createDevice(irr::video::EDT_NULL, irr::core::dimension2d<irr::u32>(320, 240));
        if (dev) cam = dev->getSceneManager()->addCameraSceneNode();
    }
    ~Rig() {
        if (dev) dev->drop();
    }
};

void CheckVec(const irr::core::vector3df& v, double x, double y, double z) {
    CHECK_THAT(v.X, WithinAbs(x, 1e-4));
    CHECK_THAT(v.Y, WithinAbs(y, 1e-4));
    CHECK_THAT(v.Z, WithinAbs(z, 1e-4));
}

irr::SEvent Mouse(irr::EMOUSE_INPUT_EVENT e, int x = 0, int y = 0, float wheel = 0.0f) {
    irr::SEvent ev{};
    ev.EventType = irr::EET_MOUSE_INPUT_EVENT;
    ev.MouseInput.Event = e;
    ev.MouseInput.X = x;
    ev.MouseInput.Y = y;
    ev.MouseInput.Wheel = wheel;
    return ev;
}

// A car at (10, 20, 0.5) heading +y (turned 90 degrees left), Z up.
VehiclePose Pose() {
    VehiclePose p;
    p.px = 10.0;
    p.py = 20.0;
    p.pz = 0.5;
    p.fx = 0.0;
    p.fy = 1.0;
    return p;
}

}  // namespace

TEST_CASE("Camera modes cycle in order and wrap", "[camera]") {
    CameraManager m(nullptr, 6.0, 2.0);
    const char* names[] = {"Chase", "Hood", "Rear", "Top-Down", "Free-Look", "Cabin"};
    for (const char* name : names) {
        CHECK(m.GetModeName() == name);
        m.CycleMode();
    }
    CHECK(m.GetMode() == CameraMode::Chase);
    m.Update(Pose());   // no camera node: a no-op
}

TEST_CASE("Camera mode from a config string; unknown keeps the mode", "[camera]") {
    CameraManager m(nullptr, 6.0, 2.0);
    m.SetModeFromString("cabin");
    CHECK(m.GetMode() == CameraMode::Cabin);
    m.SetModeFromString("topdown");
    CHECK(m.GetMode() == CameraMode::TopDown);
    m.SetModeFromString("orbit");
    CHECK(m.GetMode() == CameraMode::TopDown);
}

TEST_CASE("Each camera mode places the camera relative to the car", "[camera]") {
    Rig rig;
    REQUIRE(rig.cam != nullptr);
    CameraManager m(rig.cam, 6.0, 2.0);
    const VehiclePose p = Pose();

    m.Update(p);   // chase: 6 m behind, 2 m up, looking at the car
    CheckVec(rig.cam->getPosition(), 10.0, 14.0, 2.5);
    CheckVec(rig.cam->getTarget(), 10.0, 20.0, 0.5);

    m.SetModeFromString("hood");   // on the hood, looking 20 m ahead
    m.Update(p);
    CheckVec(rig.cam->getPosition(), 10.0, 21.2, 1.3);
    CheckVec(rig.cam->getTarget(), 10.0, 40.0, 0.5);

    m.SetModeFromString("rear");   // behind the car, looking backward
    m.Update(p);
    CheckVec(rig.cam->getPosition(), 10.0, 18.5, 1.5);
    CheckVec(rig.cam->getTarget(), 10.0, 0.0, 0.5);

    m.SetModeFromString("topdown");   // 30 m overhead, car's forward as screen up
    m.Update(p);
    CheckVec(rig.cam->getPosition(), 10.0, 20.0, 30.5);
    CheckVec(rig.cam->getUpVector(), 0.0, 1.0, 0.0);

    m.SetModeFromString("freelook");   // orbit 15 m at yaw 0, pitch 25 degrees
    m.Update(p);
    const double c = std::cos(25.0 * 3.14159265358979 / 180.0);
    const double s = std::sin(25.0 * 3.14159265358979 / 180.0);
    CheckVec(rig.cam->getPosition(), 10.0 + 15.0 * c, 20.0, 0.5 + 15.0 * s);
    CheckVec(rig.cam->getUpVector(), 0.0, 0.0, 1.0);

    m.SetModeFromString("cabin");   // the driver's eye, looking at the dash screens
    m.Update(p);
    const ev1sim::DashVec3 eye = ev1sim::ChassisToWorld(p, ev1sim::kDriverEye);
    CheckVec(rig.cam->getPosition(), eye.x, eye.y, eye.z);
}

TEST_CASE("Zoom scales the chase distance and is clamped", "[camera]") {
    Rig rig;
    REQUIRE(rig.cam != nullptr);
    CameraManager m(rig.cam, 6.0, 2.0);
    m.Zoom(1.0);   // 2x
    m.Update(Pose());
    CheckVec(rig.cam->getPosition(), 10.0, 8.0, 4.5);
    m.Zoom(100.0);   // clamps at 20x
    m.Update(Pose());
    CheckVec(rig.cam->getPosition(), 10.0, 20.0 - 120.0, 40.5);
    m.Zoom(-100.0);   // clamps at 0.2x
    m.Update(Pose());
    CheckVec(rig.cam->getPosition(), 10.0, 20.0 - 1.2, 0.9);
}

TEST_CASE("Mouse wheel zooms in every mode; free-look changes orbit distance", "[camera]") {
    Rig rig;
    REQUIRE(rig.cam != nullptr);
    CameraManager m(rig.cam, 6.0, 2.0);
    CHECK(m.OnEvent(Mouse(irr::EMIE_MOUSE_WHEEL, 0, 0, -1.0f)));   // zoom out 0.3
    m.Update(Pose());
    CheckVec(rig.cam->getPosition(), 10.0, 20.0 - 7.8, 0.5 + 2.6);

    CameraManager f(rig.cam, 6.0, 2.0);
    f.SetModeFromString("freelook");
    CHECK(f.OnEvent(Mouse(irr::EMIE_MOUSE_WHEEL, 0, 0, 2.5f)));   // 15 - 5 = 10 m
    f.Update(Pose());
    CHECK_THAT(rig.cam->getPosition().getDistanceFrom(rig.cam->getTarget()), WithinAbs(10.0, 1e-3));
    for (int i = 0; i < 100; ++i) f.OnEvent(Mouse(irr::EMIE_MOUSE_WHEEL, 0, 0, 1.0f));
    f.Update(Pose());   // clamps at 2 m
    CHECK_THAT(rig.cam->getPosition().getDistanceFrom(rig.cam->getTarget()), WithinAbs(2.0, 1e-3));
}

TEST_CASE("Free-look drag orbits the camera; pitch stops short of vertical", "[camera]") {
    Rig rig;
    REQUIRE(rig.cam != nullptr);
    CameraManager m(rig.cam, 6.0, 2.0);
    // Outside free-look, clicks and moves are not the camera's.
    CHECK_FALSE(m.OnEvent(Mouse(irr::EMIE_LMOUSE_PRESSED_DOWN, 100, 100)));

    m.SetModeFromString("freelook");
    CHECK(m.OnEvent(Mouse(irr::EMIE_LMOUSE_PRESSED_DOWN, 100, 100)));
    CHECK(m.OnEvent(Mouse(irr::EMIE_MOUSE_MOVED, 400, 100)));   // +300 px: yaw +90 degrees
    m.Update(Pose());
    const auto pos = rig.cam->getPosition();
    CHECK_THAT(pos.X, WithinAbs(10.0, 1e-3));   // now off the car's +y side
    CHECK(pos.Y > 20.0);

    CHECK(m.OnEvent(Mouse(irr::EMIE_MOUSE_MOVED, 400, -10000)));   // drag far up
    m.Update(Pose());
    // Pitch clamps at 89 degrees: still a hair off vertical.
    const auto top = rig.cam->getPosition();
    CHECK(top.Z < 0.5 + 15.0);
    CHECK(top.Z > 0.5 + 15.0 * 0.999);

    CHECK(m.OnEvent(Mouse(irr::EMIE_LMOUSE_LEFT_UP, 400, -10000)));
    CHECK_FALSE(m.OnEvent(Mouse(irr::EMIE_MOUSE_MOVED, 0, 0)));   // released: no orbit
    m.Update(Pose());
    CheckVec(rig.cam->getPosition(), top.X, top.Y, top.Z);
}

TEST_CASE("In UI mode the camera passes every mouse event through", "[camera]") {
    CameraManager m(nullptr, 6.0, 2.0);
    m.SetGrabbingMouse(false);
    CHECK_FALSE(m.OnEvent(Mouse(irr::EMIE_MOUSE_WHEEL, 0, 0, 1.0f)));
    m.SetGrabbingMouse(true);
    CHECK(m.OnEvent(Mouse(irr::EMIE_MOUSE_WHEEL, 0, 0, 1.0f)));
    irr::SEvent key{};
    key.EventType = irr::EET_KEY_INPUT_EVENT;
    CHECK_FALSE(m.OnEvent(key));   // keys are never the camera's
}
