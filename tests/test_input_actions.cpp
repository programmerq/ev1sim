#include <catch2/catch_test_macros.hpp>

#include "InputActions.h"

#include <set>
#include <string>

using namespace ev1sim;

TEST_CASE("InputAction registry round-trips every named action", "[input]") {
    std::set<std::string> seen;
    for (std::size_t i = 0; i < kInputActionCount; ++i) {
        const auto& e = detail::kInputActionNames[i];
        auto a = InputActionFromString(e.name);
        REQUIRE(a.has_value());
        CHECK(*a == e.action);                                   // name -> action
        CHECK(InputActionToString(e.action) == e.name);          // action -> name
        CHECK(seen.insert(std::string(e.name)).second);          // names are unique
    }
}

TEST_CASE("InputAction unknown / empty names return nullopt", "[input]") {
    CHECK_FALSE(InputActionFromString("not_an_action").has_value());
    CHECK_FALSE(InputActionFromString("").has_value());
    CHECK(InputActionToString(InputAction::None) == "none");
}

TEST_CASE("InputAction: only Horn is a held/level action", "[input]") {
    CHECK(InputActionIsHeld(InputAction::Horn));
    CHECK_FALSE(InputActionIsHeld(InputAction::TurnSignalLeft));
    CHECK_FALSE(InputActionIsHeld(InputAction::CameraCycle));
    CHECK_FALSE(InputActionIsHeld(InputAction::None));
}

TEST_CASE("InputAction: representative names resolve to expected enumerators", "[input]") {
    CHECK(InputActionFromString("horn")          == InputAction::Horn);
    CHECK(InputActionFromString("turn_left")     == InputAction::TurnSignalLeft);
    CHECK(InputActionFromString("turn_right")    == InputAction::TurnSignalRight);
    CHECK(InputActionFromString("wiper_cycle")   == InputAction::WiperCycle);
    CHECK(InputActionFromString("cruise_set")    == InputAction::CruiseSet);
    CHECK(InputActionFromString("camera_cycle")  == InputAction::CameraCycle);
    CHECK(InputActionFromString("headlight_cycle") == InputAction::HeadlightCycle);
}
