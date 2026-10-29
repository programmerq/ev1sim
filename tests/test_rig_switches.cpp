// RigSwitchTracker and ApplyRigSwitch: rig button boxes as momentary buttons
// and maintained switches driving PhysicalWorld inputs.

#include <catch2/catch_test_macros.hpp>

#include "PhysicalWorld.h"
#include "RigSwitches.h"

#include <set>

using namespace ev1sim;

namespace {
// Button levels for one poll: the set of pressed SDL button indices.
auto pressed(std::set<int> down) {
    return [down = std::move(down)](int b) { return down.count(b) > 0; };
}

RigDeviceConfig box() {
    RigDeviceConfig c;
    c.device_name_match = "Box";
    c.buttons  = {{0, InputAction::HazardToggle}, {1, InputAction::Horn}};
    c.switches = {{2, RigSwitchTarget::SeatbeltDriver, false},
                  {3, RigSwitchTarget::WindowDriverUp, false},
                  {4, RigSwitchTarget::AirConditioning, true}};
    return c;
}
}  // namespace

TEST_CASE("RigSwitchTracker: first poll reports every switch level and fires no button edges", "[rig]") {
    RigSwitchTracker t(box());
    const RigSample s = t.Sample(pressed({0, 2}));   // hazard already held at open
    CHECK(s.actions.empty());
    REQUIRE(s.switch_changes.size() == 3);
    CHECK(s.switch_changes[0] == std::pair{RigSwitchTarget::SeatbeltDriver, true});
    CHECK(s.switch_changes[1] == std::pair{RigSwitchTarget::WindowDriverUp, false});
    CHECK(s.switch_changes[2] == std::pair{RigSwitchTarget::AirConditioning, true});   // inverted, open
}

TEST_CASE("RigSwitchTracker: momentary buttons fire on the press only; horn is a level", "[rig]") {
    RigSwitchTracker t(box());
    t.Sample(pressed({}));
    RigSample s = t.Sample(pressed({0, 1}));
    REQUIRE(s.actions.size() == 1);
    CHECK(s.actions[0] == InputAction::HazardToggle);
    CHECK(s.horn);
    s = t.Sample(pressed({0, 1}));
    CHECK(s.actions.empty());
    CHECK(s.horn);
    s = t.Sample(pressed({}));
    CHECK_FALSE(s.horn);
}

TEST_CASE("RigSwitchTracker: a switch is reported only when its level changes", "[rig]") {
    RigSwitchTracker t(box());
    t.Sample(pressed({}));
    CHECK(t.Sample(pressed({})).switch_changes.empty());
    const RigSample s = t.Sample(pressed({2}));
    REQUIRE(s.switch_changes.size() == 1);
    CHECK(s.switch_changes[0] == std::pair{RigSwitchTarget::SeatbeltDriver, true});
    CHECK(t.Sample(pressed({2})).switch_changes.empty());
}

TEST_CASE("RigSwitchTracker: disconnect releases held window switches, keeps latched ones", "[rig]") {
    RigSwitchTracker t(box());
    t.Sample(pressed({2, 3}));
    const RigSample s = t.Disconnected();
    REQUIRE(s.switch_changes.size() == 1);
    CHECK(s.switch_changes[0] == std::pair{RigSwitchTarget::WindowDriverUp, false});
    // Reconnecting reports every level again.
    CHECK(t.Sample(pressed({2})).switch_changes.size() == 3);
}

TEST_CASE("ApplyRigSwitch drives the PhysicalWorld inputs", "[rig]") {
    PhysicalWorld w;
    ApplyRigSwitch(w, RigSwitchTarget::SeatbeltDriver, true);
    CHECK(w.seatbelts().driver_buckled());
    ApplyRigSwitch(w, RigSwitchTarget::SeatbeltDriver, false);
    CHECK_FALSE(w.seatbelts().driver_buckled());

    ApplyRigSwitch(w, RigSwitchTarget::ChargeCoupler, true);
    CHECK(w.charge_coupler().present());

    ApplyRigSwitch(w, RigSwitchTarget::WindowPassengerDown, true);
    CHECK(w.power_windows().passenger_down());
    ApplyRigSwitch(w, RigSwitchTarget::WindowPassengerDown, false);
    CHECK_FALSE(w.power_windows().passenger_down());

    ApplyRigSwitch(w, RigSwitchTarget::AirConditioning, true);
    CHECK(w.hvac_controls().ac_on());
    ApplyRigSwitch(w, RigSwitchTarget::Defrost, true);
    CHECK(w.hvac_controls().defrost_on());
}

TEST_CASE("ApplyRigSwitch: a rotary position selects on close and ignores its open", "[rig]") {
    PhysicalWorld w;
    ApplyRigSwitch(w, RigSwitchTarget::HvacFanHigh, true);
    CHECK(w.hvac_controls().fan() == HvacControls::Fan::HIGH);
    ApplyRigSwitch(w, RigSwitchTarget::HvacFanMed, true);
    ApplyRigSwitch(w, RigSwitchTarget::HvacFanHigh, false);
    CHECK(w.hvac_controls().fan() == HvacControls::Fan::MED);
    ApplyRigSwitch(w, RigSwitchTarget::HvacModeFeet, true);
    CHECK(w.hvac_controls().mode() == HvacControls::Mode::FEET);
}

TEST_CASE("RigSwitchTargetFromString round-trips every switch name", "[rig]") {
    for (const auto& e : detail::kRigSwitchNames) {
        CHECK(RigSwitchTargetFromString(e.name) == e.target);
    }
    CHECK_FALSE(RigSwitchTargetFromString("ignition").has_value());
}
