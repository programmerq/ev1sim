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
