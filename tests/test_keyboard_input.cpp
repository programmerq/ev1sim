#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "KeyboardInputController.h"

using Catch::Matchers::WithinAbs;

static KeyboardInputController MakeController() {
    KeyboardInputController::Rates r;
    r.steer_rate        = 2.0;   // reaches full lock in 0.5 s
    r.steer_return_rate = 4.0;   // returns to centre in 0.25 s
    r.throttle_rise     = 5.0;   // full throttle in 0.2 s
    r.brake_rise        = 5.0;
    return KeyboardInputController(r);
}

// -----------------------------------------------------------------------
TEST_CASE("Keyboard default output is all zeros", "[Keyboard]") {
    auto ctrl = MakeController();
    auto cmd = ctrl.Update(0.016);

    CHECK_THAT(cmd.throttle,    WithinAbs(0.0, 1e-9));
    CHECK_THAT(cmd.front_brake, WithinAbs(0.0, 1e-9));
    CHECK_THAT(cmd.steering,    WithinAbs(0.0, 1e-9));
    CHECK(cmd.parking_brake  == false);
    CHECK(cmd.reset_vehicle  == false);
}

// -----------------------------------------------------------------------
TEST_CASE("Throttle ramps up while W is held", "[Keyboard]") {
    auto ctrl = MakeController();

    ctrl.SetKeyPressed(irr::KEY_KEY_W, true);
    auto cmd = ctrl.Update(0.1);  // 5.0 * 0.1 = 0.5
    CHECK_THAT(cmd.throttle, WithinAbs(0.5, 1e-9));

    cmd = ctrl.Update(0.1);  // 0.5 + 0.5 = 1.0
    CHECK_THAT(cmd.throttle, WithinAbs(1.0, 1e-9));

    cmd = ctrl.Update(0.1);  // clamped at 1.0
    CHECK_THAT(cmd.throttle, WithinAbs(1.0, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Throttle decays when W is released", "[Keyboard]") {
    auto ctrl = MakeController();

    ctrl.SetKeyPressed(irr::KEY_KEY_W, true);
    ctrl.Update(0.1);  // 0.5
    ctrl.SetKeyPressed(irr::KEY_KEY_W, false);

    auto cmd = ctrl.Update(0.1);  // 0.5 - 0.5 = 0.0
    CHECK_THAT(cmd.throttle, WithinAbs(0.0, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Steering ramps left with A, returns to centre on release", "[Keyboard]") {
    auto ctrl = MakeController();

    // Hold A (left = positive steering in Chrono convention).
    ctrl.SetKeyPressed(irr::KEY_KEY_A, true);
    auto cmd = ctrl.Update(0.25);  // 2.0 * 0.25 = 0.5
    CHECK_THAT(cmd.steering, WithinAbs(0.5, 1e-9));

    // Release — should return toward 0.
    ctrl.SetKeyPressed(irr::KEY_KEY_A, false);
    cmd = ctrl.Update(0.1);  // 0.5 - 4.0*0.1 = 0.1
    CHECK_THAT(cmd.steering, WithinAbs(0.1, 1e-9));

    cmd = ctrl.Update(0.1);  // 0.1 - 0.4 = clamped to 0
    CHECK_THAT(cmd.steering, WithinAbs(0.0, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Steering ramps right with D (negative)", "[Keyboard]") {
    auto ctrl = MakeController();

    ctrl.SetKeyPressed(irr::KEY_KEY_D, true);
    auto cmd = ctrl.Update(0.25);
    CHECK_THAT(cmd.steering, WithinAbs(-0.5, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Brake ramps with S and sets both front and rear", "[Keyboard]") {
    auto ctrl = MakeController();

    ctrl.SetKeyPressed(irr::KEY_KEY_S, true);
    auto cmd = ctrl.Update(0.1);

    CHECK_THAT(cmd.front_brake, WithinAbs(0.5, 1e-9));
    CHECK_THAT(cmd.rear_brake,  WithinAbs(0.5, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("Parking brake toggles on Space press", "[Keyboard]") {
    auto ctrl = MakeController();

    // Press space.
    ctrl.SetKeyPressed(irr::KEY_SPACE, true);
    auto cmd = ctrl.Update(0.016);
    CHECK(cmd.parking_brake == true);

    // Still held — no re-toggle.
    cmd = ctrl.Update(0.016);
    CHECK(cmd.parking_brake == true);

    // Release then press again — toggles off.
    ctrl.SetKeyPressed(irr::KEY_SPACE, false);
    ctrl.Update(0.016);
    ctrl.SetKeyPressed(irr::KEY_SPACE, true);
    cmd = ctrl.Update(0.016);
    CHECK(cmd.parking_brake == false);
}

// -----------------------------------------------------------------------
TEST_CASE("Reset is a one-shot on R press", "[Keyboard]") {
    auto ctrl = MakeController();

    ctrl.SetKeyPressed(irr::KEY_KEY_R, true);
    auto cmd = ctrl.Update(0.016);
    CHECK(cmd.reset_vehicle == true);

    // Still held — should not fire again.
    cmd = ctrl.Update(0.016);
    CHECK(cmd.reset_vehicle == false);
}

// -----------------------------------------------------------------------
TEST_CASE("Camera cycle is consumable one-shot", "[Keyboard]") {
    auto ctrl = MakeController();

    ctrl.SetKeyPressed(irr::KEY_KEY_C, true);
    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeCameraCycle() == true);
    CHECK(ctrl.ConsumeCameraCycle() == false);  // consumed
}
