#include <catch2/catch_test_macros.hpp>

// FloatingUiPanel pure label helpers — testable without Irrlicht window.
#include "FloatingUiPanel.h"
#include "PhysicalWorld.h"
#include "KeyboardInputController.h"

// ---------------------------------------------------------------------------
// Label format tests (pure, no Irrlicht)
// ---------------------------------------------------------------------------

TEST_CASE("FormatHazardLabel: OFF when hazard not active", "[FloatingUI]") {
    CHECK(FormatHazardLabel(false) == L"Hazard: OFF");
}

TEST_CASE("FormatHazardLabel: ON when hazard active", "[FloatingUI]") {
    CHECK(FormatHazardLabel(true) == L"Hazard: ON");
}

TEST_CASE("FormatLockAllLabel: 'Lock All' when none locked", "[FloatingUI]") {
    CHECK(FormatLockAllLabel(false) == L"Lock All");
}

TEST_CASE("FormatLockAllLabel: 'Unlock All' when any locked", "[FloatingUI]") {
    CHECK(FormatLockAllLabel(true) == L"Unlock All");
}

TEST_CASE("FormatDoorLabel: unlocked state", "[FloatingUI]") {
    CHECK(FormatDoorLabel(L"Driver",    false) == L"Driver: unlocked");
    CHECK(FormatDoorLabel(L"Passenger", false) == L"Passenger: unlocked");
    CHECK(FormatDoorLabel(L"Trunk",     false) == L"Trunk: unlocked");
}

TEST_CASE("FormatDoorLabel: locked state", "[FloatingUI]") {
    CHECK(FormatDoorLabel(L"Driver",    true) == L"Driver: locked");
    CHECK(FormatDoorLabel(L"Passenger", true) == L"Passenger: locked");
    CHECK(FormatDoorLabel(L"Trunk",     true) == L"Trunk: locked");
}

TEST_CASE("FormatCouplerLabel: UNPLUGGED when not present", "[FloatingUI]") {
    CHECK(FormatCouplerLabel(false) == L"Coupler: UNPLUGGED");
}

TEST_CASE("FormatCouplerLabel: PLUGGED when present", "[FloatingUI]") {
    CHECK(FormatCouplerLabel(true) == L"Coupler: PLUGGED");
}

TEST_CASE("FormatExtKeypadButtonLabel: all five buttons", "[FloatingUI]") {
    CHECK(FormatExtKeypadButtonLabel(0) == L"[1/2]");
    CHECK(FormatExtKeypadButtonLabel(1) == L"[3/4]");
    CHECK(FormatExtKeypadButtonLabel(2) == L"[5/6]");
    CHECK(FormatExtKeypadButtonLabel(3) == L"[7/8]");
    CHECK(FormatExtKeypadButtonLabel(4) == L"[9/0]");
}

TEST_CASE("FormatExtKeypadButtonLabel: out-of-range returns fallback", "[FloatingUI]") {
    CHECK(FormatExtKeypadButtonLabel(-1) == L"[?]");
    CHECK(FormatExtKeypadButtonLabel(5)  == L"[?]");
}

TEST_CASE("FormatDoorHandleLabel: driver and passenger", "[FloatingUI]") {
    CHECK(FormatDoorHandleLabel(L"Driver")    == L"Handle: Driver");
    CHECK(FormatDoorHandleLabel(L"Passenger") == L"Handle: Passenger");
}

// ---------------------------------------------------------------------------
// Callback invocation tests — use real PhysicalWorld objects
// ---------------------------------------------------------------------------

TEST_CASE("FloatingUI callback: hazard toggle inverts state", "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    REQUIRE_FALSE(world.hazard_switch().on());

    // Simulate the callback that SimApp would wire.
    auto cb = [&]() { world.hazard_switch().toggle(); };

    cb();
    CHECK(world.hazard_switch().on());
    CHECK(FormatHazardLabel(world.hazard_switch().on()) == L"Hazard: ON");

    cb();
    CHECK_FALSE(world.hazard_switch().on());
    CHECK(FormatHazardLabel(world.hazard_switch().on()) == L"Hazard: OFF");
}

TEST_CASE("FloatingUI callback: lock_all / unlock_all via label-driven logic",
          "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    // Starting state: all unlocked → label says "Lock All".
    REQUIRE_FALSE(world.door_locks().any_locked());
    CHECK(FormatLockAllLabel(world.door_locks().any_locked()) == L"Lock All");

    // Simulate the "Lock All / Unlock All" button callback.
    auto cb = [&]() {
        if (world.door_locks().any_locked())
            world.door_locks().unlock_all();
        else
            world.door_locks().lock_all();
    };

    cb();  // locks all
    CHECK(world.door_locks().any_locked());
    CHECK(FormatLockAllLabel(world.door_locks().any_locked()) == L"Unlock All");

    cb();  // unlocks all
    CHECK_FALSE(world.door_locks().any_locked());
    CHECK(FormatLockAllLabel(world.door_locks().any_locked()) == L"Lock All");
}

TEST_CASE("FloatingUI callback: toggle_driver changes only driver",
          "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    using S = ev1sim::DoorLocks::State;

    auto cb = [&]() { world.door_locks().toggle_driver(); };
    cb();
    CHECK(world.door_locks().driver()    == S::LOCKED);
    CHECK(world.door_locks().passenger() == S::UNLOCKED);
    CHECK(world.door_locks().trunk()     == S::UNLOCKED);
    CHECK(FormatDoorLabel(L"Driver", world.door_locks().driver() == S::LOCKED)
          == L"Driver: locked");
}

TEST_CASE("FloatingUI callback: charge coupler toggle flips present flag",
          "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    REQUIRE_FALSE(world.charge_coupler().present());

    auto cb = [&]() {
        world.charge_coupler().set_present(!world.charge_coupler().present());
    };

    cb();
    CHECK(world.charge_coupler().present());
    CHECK(FormatCouplerLabel(world.charge_coupler().present()) == L"Coupler: PLUGGED");

    cb();
    CHECK_FALSE(world.charge_coupler().present());
    CHECK(FormatCouplerLabel(world.charge_coupler().present()) == L"Coupler: UNPLUGGED");
}

// ---------------------------------------------------------------------------
// Wave 2 label helpers
// ---------------------------------------------------------------------------

TEST_CASE("FormatWiperLabel: all four positions", "[FloatingUI]") {
    CHECK(FormatWiperLabel(0) == L"Wiper: OFF");
    CHECK(FormatWiperLabel(1) == L"Wiper: INT");
    CHECK(FormatWiperLabel(2) == L"Wiper: LOW");
    CHECK(FormatWiperLabel(3) == L"Wiper: HIGH");
}

TEST_CASE("FormatWiperLabel: out-of-range returns fallback", "[FloatingUI]") {
    CHECK(FormatWiperLabel(-1) == L"Wiper: ?");
    CHECK(FormatWiperLabel(4)  == L"Wiper: ?");
}

TEST_CASE("FormatWashLabel: static label", "[FloatingUI]") {
    CHECK(FormatWashLabel() == L"Wash");
}

TEST_CASE("FormatCruiseLabel: action appended", "[FloatingUI]") {
    CHECK(FormatCruiseLabel(L"SET")    == L"Cruise: SET");
    CHECK(FormatCruiseLabel(L"RES")    == L"Cruise: RES");
    CHECK(FormatCruiseLabel(L"CANCEL") == L"Cruise: CANCEL");
    CHECK(FormatCruiseLabel(L"+")      == L"Cruise: +");
    CHECK(FormatCruiseLabel(L"-")      == L"Cruise: -");
}

TEST_CASE("FormatTripResetLabel: static label", "[FloatingUI]") {
    CHECK(FormatTripResetLabel() == L"Trip Reset");
}

TEST_CASE("FormatSeatbeltLabel: driver buckled / unbuckled", "[FloatingUI]") {
    CHECK(FormatSeatbeltLabel(L"D", true)  == L"Seatbelt D: BUCKLED");
    CHECK(FormatSeatbeltLabel(L"D", false) == L"Seatbelt D: UNBUCKLED");
}

TEST_CASE("FormatSeatbeltLabel: passenger buckled / unbuckled", "[FloatingUI]") {
    CHECK(FormatSeatbeltLabel(L"P", true)  == L"Seatbelt P: BUCKLED");
    CHECK(FormatSeatbeltLabel(L"P", false) == L"Seatbelt P: UNBUCKLED");
}

TEST_CASE("FloatingUI callback: wiper cycle updates label", "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    using P = ev1sim::WiperStalk::Position;
    REQUIRE(world.wiper_stalk().position() == P::OFF);

    auto cb = [&]() { world.wiper_stalk().cycle_position(); };

    cb();
    CHECK(world.wiper_stalk().position() == P::INT);
    CHECK(FormatWiperLabel(static_cast<int>(world.wiper_stalk().position()))
          == L"Wiper: INT");

    cb(); cb(); cb();  // LOW → HIGH → OFF
    CHECK(world.wiper_stalk().position() == P::OFF);
    CHECK(FormatWiperLabel(static_cast<int>(world.wiper_stalk().position()))
          == L"Wiper: OFF");
}

TEST_CASE("FloatingUI callback: seatbelt driver toggle", "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    REQUIRE(world.seatbelts().driver_buckled());

    auto cb = [&]() { world.seatbelts().toggle_driver(); };

    cb();
    CHECK_FALSE(world.seatbelts().driver_buckled());
    CHECK(FormatSeatbeltLabel(L"D", world.seatbelts().driver_buckled())
          == L"Seatbelt D: UNBUCKLED");

    cb();
    CHECK(world.seatbelts().driver_buckled());
    CHECK(FormatSeatbeltLabel(L"D", world.seatbelts().driver_buckled())
          == L"Seatbelt D: BUCKLED");
}

TEST_CASE("FloatingUI callback: seatbelt passenger toggle leaves driver unchanged",
          "[FloatingUI][Callbacks]") {
    ev1sim::PhysicalWorld world;
    REQUIRE(world.seatbelts().passenger_buckled());

    world.seatbelts().toggle_passenger();
    CHECK_FALSE(world.seatbelts().passenger_buckled());
    CHECK(world.seatbelts().driver_buckled());  // driver unaffected
}

// ---------------------------------------------------------------------------
// UI mode toggle state machine in KeyboardInputController
// ---------------------------------------------------------------------------

TEST_CASE("KeyboardInputController: TAB produces ConsumeUiModeToggle one-shot",
          "[FloatingUI][Keyboard]") {
    KeyboardInputController::Rates r;
    KeyboardInputController ctrl(r);

    // Initially no toggle pending.
    CHECK_FALSE(ctrl.ConsumeUiModeToggle());

    // Press TAB.
    ctrl.SetKeyPressed(irr::KEY_TAB, true);
    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == true);
    CHECK(ctrl.ConsumeUiModeToggle() == false);  // consumed

    // Release and re-press — fires again.
    ctrl.SetKeyPressed(irr::KEY_TAB, false);
    ctrl.Update(0.016);
    ctrl.SetKeyPressed(irr::KEY_TAB, true);
    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == true);
    CHECK(ctrl.ConsumeUiModeToggle() == false);
}

TEST_CASE("KeyboardInputController: holding TAB does not repeat toggle",
          "[FloatingUI][Keyboard]") {
    KeyboardInputController::Rates r;
    KeyboardInputController ctrl(r);

    ctrl.SetKeyPressed(irr::KEY_TAB, true);
    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == true);   // edge on first frame

    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == false);  // still held — no repeat

    ctrl.Update(0.016);
    CHECK(ctrl.ConsumeUiModeToggle() == false);
}

// ---------------------------------------------------------------------------
// HVAC status label helpers (FormatHvacBlowerLabel, FormatDefrostGridLabel)
// ---------------------------------------------------------------------------

TEST_CASE("FormatHvacBlowerLabel: all four HTCM levels", "[FloatingUI][HVAC]") {
    CHECK(FormatHvacBlowerLabel(0u) == L"Blower: OFF");
    CHECK(FormatHvacBlowerLabel(1u) == L"Blower: LOW");
    CHECK(FormatHvacBlowerLabel(2u) == L"Blower: MED");
    CHECK(FormatHvacBlowerLabel(3u) == L"Blower: HIGH");
}

TEST_CASE("FormatHvacBlowerLabel: never-received sentinel (0xFF) shows placeholder",
          "[FloatingUI][HVAC]") {
    CHECK(FormatHvacBlowerLabel(0xFFu) == L"Blower: ---");
}

TEST_CASE("FormatHvacBlowerLabel: unknown value shows fallback", "[FloatingUI][HVAC]") {
    CHECK(FormatHvacBlowerLabel(4u)   == L"Blower: ?");
    CHECK(FormatHvacBlowerLabel(200u) == L"Blower: ?");
}

TEST_CASE("FormatDefrostGridLabel: OFF when not active", "[FloatingUI][HVAC]") {
    CHECK(FormatDefrostGridLabel(false) == L"Defrost: OFF");
}

TEST_CASE("FormatDefrostGridLabel: ON when active", "[FloatingUI][HVAC]") {
    CHECK(FormatDefrostGridLabel(true) == L"Defrost: ON");
}
