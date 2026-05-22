#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ForceFeedback.h"
#include "InputActions.h"
#include "SdlContext.h"
#include "WheelInputController.h"

#include <SDL3/SDL.h>

using namespace ev1sim;
using Catch::Matchers::WithinAbs;

namespace {
WheelConfig make_test_config() {
    WheelConfig cfg;
    cfg.enabled           = true;
    cfg.device_name_match = "ev1sim virtual wheel";  // match only our virtual device
    cfg.axes = {
        {0, WheelAxisBinding::Target::Steering,   -32768.0, 32767.0, false, 0.02},
        {1, WheelAxisBinding::Target::Throttle,   -32768.0, 32767.0, false, 0.0},
        {2, WheelAxisBinding::Target::FrontBrake, -32768.0, 32767.0, false, 0.0},
    };
    cfg.buttons = {
        {0, InputAction::Horn},
        {4, InputAction::TurnSignalLeft},
        {5, InputAction::CameraCycle},
    };
    return cfg;
}
}  // namespace

TEST_CASE("WheelInputController maps a virtual joystick's axes + buttons", "[wheel]") {
    SdlContext sdl;
    if (!sdl.ok()) {
        WARN("SDL joystick/haptic init unavailable — skipping virtual-wheel test");
        return;
    }

    SDL_VirtualJoystickDesc desc;
    SDL_INIT_INTERFACE(&desc);  // zeroes the struct and sets desc.version
    desc.type     = static_cast<Uint16>(SDL_JOYSTICK_TYPE_WHEEL);
    desc.naxes    = 3;
    desc.nbuttons = 6;
    desc.name     = "ev1sim virtual wheel";
    SDL_JoystickID id = SDL_AttachVirtualJoystick(&desc);
    REQUIRE(id != 0);
    SDL_UpdateJoysticks();  // make the new device visible to enumeration

    SDL_Joystick* vjoy = SDL_OpenJoystick(id);
    REQUIRE(vjoy != nullptr);

    WheelInputController wheel(&sdl, make_test_config());
    REQUIRE(wheel.HasDevice());

    SECTION("neutral axes -> neutral command, no actions") {
        SDL_SetJoystickVirtualAxis(vjoy, 0, 0);        // steering centered
        SDL_SetJoystickVirtualAxis(vjoy, 1, 32767);    // throttle floored (in_max)
        SDL_SetJoystickVirtualAxis(vjoy, 2, -32768);   // brake released (in_min)
        SDL_UpdateJoysticks();

        DriverCommand cmd = wheel.Update(0.016);
        CHECK_THAT(cmd.steering,    WithinAbs(0.0, 0.05));  // inside deadzone
        CHECK_THAT(cmd.throttle,    WithinAbs(1.0, 0.01));
        CHECK_THAT(cmd.front_brake, WithinAbs(0.0, 0.01));
        CHECK_FALSE(cmd.horn_low);
        CHECK(wheel.PendingActions().empty());
    }

    SECTION("steer + held horn + momentary turn-signal edge") {
        // Prime previous button state with a neutral poll first.
        SDL_UpdateJoysticks();
        wheel.Update(0.016);
        wheel.ClearPendingActions();

        SDL_SetJoystickVirtualAxis(vjoy, 0, 32767);     // full right (+1)
        SDL_SetJoystickVirtualButton(vjoy, 0, true);    // horn (held)
        SDL_SetJoystickVirtualButton(vjoy, 4, true);    // turn-left (rising edge)
        SDL_UpdateJoysticks();

        DriverCommand cmd = wheel.Update(0.016);
        CHECK_THAT(cmd.steering, WithinAbs(1.0, 0.01));
        CHECK(cmd.horn_low);
        CHECK(cmd.horn_high);
        REQUIRE(wheel.PendingActions().size() == 1);
        CHECK(wheel.PendingActions()[0] == InputAction::TurnSignalLeft);

        // Holding the same button must NOT re-fire (edge, not level).
        wheel.ClearPendingActions();
        SDL_UpdateJoysticks();
        wheel.Update(0.016);
        CHECK(wheel.PendingActions().empty());
    }

    SDL_CloseJoystick(vjoy);
    SDL_DetachVirtualJoystick(id);
}

TEST_CASE("ForceFeedback curve: scale, sign, clamp, invert, speed taper", "[ffb]") {
    FfbConfig cfg;
    cfg.scale_nm_to_unit = 0.1;
    cfg.max_clamp        = 1.0;

    CHECK_THAT(ForceFeedback::CurveUnitForce(cfg, 5.0, 30.0), WithinAbs(0.5, 1e-9));
    CHECK(ForceFeedback::CurveUnitForce(cfg, -5.0, 30.0) < 0.0);          // sign follows torque
    CHECK_THAT(ForceFeedback::CurveUnitForce(cfg, 100.0, 30.0), WithinAbs(1.0, 1e-9)); // clamp

    FfbConfig inv = cfg;
    inv.invert = true;
    CHECK_THAT(ForceFeedback::CurveUnitForce(inv, 5.0, 30.0), WithinAbs(-0.5, 1e-9));

    FfbConfig tap = cfg;
    tap.speed_taper_start_mps = 5.0;
    tap.speed_taper_full_mps  = 15.0;
    CHECK_THAT(ForceFeedback::CurveUnitForce(tap, 5.0, 0.0),  WithinAbs(0.0, 1e-9));   // below start
    CHECK_THAT(ForceFeedback::CurveUnitForce(tap, 5.0, 10.0), WithinAbs(0.25, 1e-9));  // halfway
    CHECK_THAT(ForceFeedback::CurveUnitForce(tap, 5.0, 20.0), WithinAbs(0.5, 1e-9));   // above full
}
