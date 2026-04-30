#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "PhysicalWorld.h"

using namespace ev1sim;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Drive the turn-signal state machine through a series of (steering, dt)
/// steps and return true if an auto-cancel event fires during those steps.
static bool run_steps(TurnSignalStalk& stalk,
                      double steering, double dt_s, int steps)
{
    for (int i = 0; i < steps; ++i) {
        stalk.update_for_steering(steering, dt_s);
        if (stalk.consume_auto_cancel_event())
            return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Basic toggle behaviour (unchanged semantics)
// ---------------------------------------------------------------------------

TEST_CASE("TurnSignalStalk: toggle_left cycles correctly", "[PhysicalWorld]") {
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;

    CHECK(stalk.position() == P::OFF);

    stalk.toggle_left();
    CHECK(stalk.position() == P::LEFT);
    CHECK(stalk.active_left());
    CHECK_FALSE(stalk.active_right());

    stalk.toggle_left();
    CHECK(stalk.position() == P::OFF);
}

TEST_CASE("TurnSignalStalk: toggle_right cycles correctly", "[PhysicalWorld]") {
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;

    stalk.toggle_right();
    CHECK(stalk.position() == P::RIGHT);
    CHECK(stalk.active_right());
    CHECK_FALSE(stalk.active_left());

    stalk.toggle_right();
    CHECK(stalk.position() == P::OFF);
}

TEST_CASE("TurnSignalStalk: RIGHT → toggle_left → LEFT", "[PhysicalWorld]") {
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;

    stalk.toggle_right();
    CHECK(stalk.position() == P::RIGHT);
    stalk.toggle_left();
    CHECK(stalk.position() == P::LEFT);
}

// ---------------------------------------------------------------------------
// Auto-cancel: normal (travel then return)
// ---------------------------------------------------------------------------

TEST_CASE("TurnSignalStalk: LEFT active, steer left past threshold then return to center -> auto-cancel",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    stalk.toggle_left();
    REQUIRE(stalk.position() == P::LEFT);

    // Phase 1: ramp steering left past travel threshold (>0.33).
    // Use dt=0.01s per step; opposite-cancel won't fire with leftward steering.
    bool cancelled = false;
    stalk.update_for_steering(0.5, 0.01);  // well past kTravelThreshold=0.33
    cancelled = stalk.consume_auto_cancel_event();
    REQUIRE_FALSE(cancelled);  // travel registered but not yet returned

    // Phase 2: bring steering back past center (active < kReturnThreshold=0.05).
    stalk.update_for_steering(0.0, 0.01);
    cancelled = stalk.consume_auto_cancel_event();
    CHECK(cancelled);
    CHECK(stalk.position() == P::OFF);
}

TEST_CASE("TurnSignalStalk: LEFT active, steer left but NOT past threshold -> no auto-cancel",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    stalk.toggle_left();

    // Steer left but only to 0.2 — below kTravelThreshold=0.33.
    bool cancelled = run_steps(stalk, 0.2, 0.01, 10);
    REQUIRE_FALSE(cancelled);

    // Return to center — still no cancel because travel was never registered.
    cancelled = run_steps(stalk, 0.0, 0.01, 5);
    CHECK_FALSE(cancelled);
    CHECK(stalk.position() == P::LEFT);
}

TEST_CASE("TurnSignalStalk: RIGHT active, steer right past threshold then return -> auto-cancel",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    stalk.toggle_right();
    REQUIRE(stalk.position() == P::RIGHT);

    // Steer right: convention is negative steering_normalized for right.
    stalk.update_for_steering(-0.5, 0.01);
    bool cancelled = stalk.consume_auto_cancel_event();
    REQUIRE_FALSE(cancelled);

    // Return past center.
    stalk.update_for_steering(0.0, 0.01);
    cancelled = stalk.consume_auto_cancel_event();
    CHECK(cancelled);
    CHECK(stalk.position() == P::OFF);
}

TEST_CASE("TurnSignalStalk: steering must cross kReturnThreshold before cancel fires",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    stalk.toggle_left();

    // Register travel.
    stalk.update_for_steering(0.5, 0.01);
    REQUIRE_FALSE(stalk.consume_auto_cancel_event());

    // Return to just above threshold (0.06 > kReturnThreshold=0.05) — no cancel yet.
    stalk.update_for_steering(0.06, 0.01);
    CHECK_FALSE(stalk.consume_auto_cancel_event());
    CHECK(stalk.position() == P::LEFT);

    // Now cross it.
    stalk.update_for_steering(0.04, 0.01);
    CHECK(stalk.consume_auto_cancel_event());
    CHECK(stalk.position() == P::OFF);
}

// ---------------------------------------------------------------------------
// Auto-cancel: opposite-direction sustained cancel
// ---------------------------------------------------------------------------

TEST_CASE("TurnSignalStalk: LEFT active, steer RIGHT past opp threshold for >500ms -> auto-cancel",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    stalk.toggle_left();

    // Steer right (negative) at -0.6, well past kOppositeCancelThreshold=0.50.
    // Use dt=0.1s per step -> need >=5 steps for 500ms.
    bool cancelled = false;
    for (int i = 0; i < 4 && !cancelled; ++i) {
        stalk.update_for_steering(-0.6, 0.1);
        cancelled = stalk.consume_auto_cancel_event();
    }
    REQUIRE_FALSE(cancelled);  // not yet at 500ms (only 400ms)

    // One more step tips past the 500ms threshold.
    stalk.update_for_steering(-0.6, 0.1);
    cancelled = stalk.consume_auto_cancel_event();
    CHECK(cancelled);
    CHECK(stalk.position() == P::OFF);
}

TEST_CASE("TurnSignalStalk: opposite-cancel timer resets when steering returns",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    stalk.toggle_left();

    // Build up 400ms of opposite-direction counter (below 500ms threshold).
    bool cancelled = run_steps(stalk, -0.6, 0.1, 4);
    REQUIRE_FALSE(cancelled);

    // Steer back to neutral — resets the accumulator.
    stalk.update_for_steering(0.0, 0.1);
    REQUIRE_FALSE(stalk.consume_auto_cancel_event());

    // Try again for another 400ms — still no cancel (accumulator was reset).
    cancelled = run_steps(stalk, -0.6, 0.1, 4);
    CHECK_FALSE(cancelled);
    CHECK(stalk.position() == P::LEFT);
}

TEST_CASE("TurnSignalStalk: opposite-cancel below threshold -> no cancel even with long duration",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    stalk.toggle_left();

    // Steer right but only at 0.4 — below kOppositeCancelThreshold=0.50.
    bool cancelled = run_steps(stalk, -0.4, 0.1, 20);  // 2 full seconds
    CHECK_FALSE(cancelled);
    CHECK(stalk.position() == P::LEFT);
}

// ---------------------------------------------------------------------------
// Manual cancel resets state machine
// ---------------------------------------------------------------------------

TEST_CASE("TurnSignalStalk: manual toggle-off resets auto-cancel state",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    stalk.toggle_left();

    // Register travel so state machine is in WaitingForReturn.
    stalk.update_for_steering(0.5, 0.01);
    REQUIRE_FALSE(stalk.consume_auto_cancel_event());

    // Manual toggle off.
    stalk.toggle_left();
    REQUIRE(stalk.position() == P::OFF);

    // No spurious event.
    CHECK_FALSE(stalk.consume_auto_cancel_event());

    // Steering returns to center — should NOT produce any event since stalk is OFF.
    stalk.update_for_steering(0.0, 0.01);
    CHECK_FALSE(stalk.consume_auto_cancel_event());
    CHECK(stalk.position() == P::OFF);
}

TEST_CASE("TurnSignalStalk: toggling to opposite side resets state machine",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    stalk.toggle_left();

    // Register left-travel.
    stalk.update_for_steering(0.5, 0.01);
    REQUIRE_FALSE(stalk.consume_auto_cancel_event());

    // User manually switches to RIGHT — state machine resets; now tracking RIGHT.
    stalk.toggle_right();
    REQUIRE(stalk.position() == P::RIGHT);

    // Return past center (active=right direction) — no event yet since we haven't
    // seen right-travel yet.
    stalk.update_for_steering(0.0, 0.01);
    CHECK_FALSE(stalk.consume_auto_cancel_event());
    CHECK(stalk.position() == P::RIGHT);
}

// ---------------------------------------------------------------------------
// No event from inactive stalk
// ---------------------------------------------------------------------------

TEST_CASE("TurnSignalStalk: OFF stalk never produces auto-cancel event",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    using P = TurnSignalStalk::Position;
    REQUIRE(stalk.position() == P::OFF);

    // Drive heavy steering both ways for many ticks.
    bool cancelled = false;
    for (int i = 0; i < 20; ++i) {
        stalk.update_for_steering(0.8, 0.1);
        cancelled = cancelled || stalk.consume_auto_cancel_event();
        stalk.update_for_steering(-0.8, 0.1);
        cancelled = cancelled || stalk.consume_auto_cancel_event();
    }
    CHECK_FALSE(cancelled);
    CHECK(stalk.position() == P::OFF);
}

// ---------------------------------------------------------------------------
// consume_auto_cancel_event() clears after first call
// ---------------------------------------------------------------------------

TEST_CASE("TurnSignalStalk: consume_auto_cancel_event clears on first read",
          "[PhysicalWorld][AutoCancel]")
{
    TurnSignalStalk stalk;
    stalk.toggle_left();

    stalk.update_for_steering(0.5, 0.01);
    stalk.update_for_steering(0.0, 0.01);

    // First call should be true.
    CHECK(stalk.consume_auto_cancel_event());
    // Second call must be false — flag was cleared.
    CHECK_FALSE(stalk.consume_auto_cancel_event());
}

// ---------------------------------------------------------------------------
// IpcTripResetButton
// ---------------------------------------------------------------------------

TEST_CASE("IpcTripResetButton: consume returns false at construction", "[PhysicalWorld]") {
    IpcTripResetButton btn;
    CHECK_FALSE(btn.consume_press_event());
}

TEST_CASE("IpcTripResetButton: press then consume returns true once then false", "[PhysicalWorld]") {
    IpcTripResetButton btn;
    btn.press();
    CHECK(btn.consume_press_event());   // first consume: event was pending
    CHECK_FALSE(btn.consume_press_event()); // second: flag was cleared
}

TEST_CASE("IpcTripResetButton: multiple press calls accumulate into a single event", "[PhysicalWorld]") {
    IpcTripResetButton btn;
    btn.press();
    btn.press();
    CHECK(btn.consume_press_event());
    CHECK_FALSE(btn.consume_press_event());
}

// ---------------------------------------------------------------------------
// CruiseStalk
// ---------------------------------------------------------------------------

TEST_CASE("CruiseStalk: all consume methods return false at construction", "[PhysicalWorld]") {
    CruiseStalk stalk;
    CHECK_FALSE(stalk.consume_set());
    CHECK_FALSE(stalk.consume_resume());
    CHECK_FALSE(stalk.consume_cancel());
    CHECK_FALSE(stalk.consume_speed_up());
    CHECK_FALSE(stalk.consume_speed_down());
}

TEST_CASE("CruiseStalk: press_set → consume_set returns true once", "[PhysicalWorld]") {
    CruiseStalk stalk;
    stalk.press_set();
    CHECK(stalk.consume_set());
    CHECK_FALSE(stalk.consume_set());
}

TEST_CASE("CruiseStalk: each button is independent", "[PhysicalWorld]") {
    CruiseStalk stalk;
    stalk.press_resume();
    stalk.press_cancel();

    // Only the pressed buttons return true.
    CHECK_FALSE(stalk.consume_set());
    CHECK(stalk.consume_resume());
    CHECK(stalk.consume_cancel());
    CHECK_FALSE(stalk.consume_speed_up());
    CHECK_FALSE(stalk.consume_speed_down());

    // All cleared after first consume.
    CHECK_FALSE(stalk.consume_resume());
    CHECK_FALSE(stalk.consume_cancel());
}

TEST_CASE("CruiseStalk: speed_up and speed_down are independent", "[PhysicalWorld]") {
    CruiseStalk stalk;
    stalk.press_speed_up();
    CHECK(stalk.consume_speed_up());
    CHECK_FALSE(stalk.consume_speed_up());
    CHECK_FALSE(stalk.consume_speed_down()); // never pressed

    stalk.press_speed_down();
    CHECK(stalk.consume_speed_down());
    CHECK_FALSE(stalk.consume_speed_down());
}

// ---------------------------------------------------------------------------
// WiperStalk
// ---------------------------------------------------------------------------

TEST_CASE("WiperStalk: default position is OFF", "[PhysicalWorld]") {
    WiperStalk ws;
    CHECK(ws.position() == WiperStalk::Position::OFF);
    CHECK_FALSE(ws.consume_wash());
}

TEST_CASE("WiperStalk: cycle_position cycles OFF → INT → LOW → HIGH → OFF", "[PhysicalWorld]") {
    WiperStalk ws;
    using P = WiperStalk::Position;

    CHECK(ws.position() == P::OFF);
    ws.cycle_position();
    CHECK(ws.position() == P::INT);
    ws.cycle_position();
    CHECK(ws.position() == P::LOW);
    ws.cycle_position();
    CHECK(ws.position() == P::HIGH);
    ws.cycle_position();
    CHECK(ws.position() == P::OFF);   // wraps back
}

TEST_CASE("WiperStalk: wash press → consume_wash one-shot", "[PhysicalWorld]") {
    WiperStalk ws;
    ws.press_wash();
    CHECK(ws.consume_wash());
    CHECK_FALSE(ws.consume_wash());   // cleared on first consume
}

TEST_CASE("WiperStalk: position unaffected by wash press", "[PhysicalWorld]") {
    WiperStalk ws;
    ws.cycle_position(); // OFF → INT
    ws.press_wash();
    CHECK(ws.position() == WiperStalk::Position::INT);
    CHECK(ws.consume_wash());
    CHECK(ws.position() == WiperStalk::Position::INT); // unchanged
}

// ---------------------------------------------------------------------------
// DoorLocks
// ---------------------------------------------------------------------------

TEST_CASE("DoorLocks: default state is all unlocked", "[PhysicalWorld][DoorLocks]") {
    DoorLocks locks;
    using S = DoorLocks::State;
    CHECK(locks.driver()    == S::UNLOCKED);
    CHECK(locks.passenger() == S::UNLOCKED);
    CHECK(locks.trunk()     == S::UNLOCKED);
    CHECK_FALSE(locks.any_locked());
}

TEST_CASE("DoorLocks: lock_all sets all three to LOCKED", "[PhysicalWorld][DoorLocks]") {
    DoorLocks locks;
    using S = DoorLocks::State;
    locks.lock_all();
    CHECK(locks.driver()    == S::LOCKED);
    CHECK(locks.passenger() == S::LOCKED);
    CHECK(locks.trunk()     == S::LOCKED);
    CHECK(locks.any_locked());
}

TEST_CASE("DoorLocks: unlock_all resets all three to UNLOCKED", "[PhysicalWorld][DoorLocks]") {
    DoorLocks locks;
    using S = DoorLocks::State;
    locks.lock_all();
    locks.unlock_all();
    CHECK(locks.driver()    == S::UNLOCKED);
    CHECK(locks.passenger() == S::UNLOCKED);
    CHECK(locks.trunk()     == S::UNLOCKED);
    CHECK_FALSE(locks.any_locked());
}

TEST_CASE("DoorLocks: any_locked is true if exactly one door is locked", "[PhysicalWorld][DoorLocks]") {
    {
        DoorLocks locks;
        locks.toggle_driver();
        CHECK(locks.any_locked());
        CHECK(locks.driver() == DoorLocks::State::LOCKED);
        CHECK(locks.passenger() == DoorLocks::State::UNLOCKED);
        CHECK(locks.trunk()     == DoorLocks::State::UNLOCKED);
    }
    {
        DoorLocks locks;
        locks.toggle_passenger();
        CHECK(locks.any_locked());
        CHECK(locks.driver()    == DoorLocks::State::UNLOCKED);
        CHECK(locks.passenger() == DoorLocks::State::LOCKED);
        CHECK(locks.trunk()     == DoorLocks::State::UNLOCKED);
    }
    {
        DoorLocks locks;
        locks.toggle_trunk();
        CHECK(locks.any_locked());
        CHECK(locks.driver()    == DoorLocks::State::UNLOCKED);
        CHECK(locks.passenger() == DoorLocks::State::UNLOCKED);
        CHECK(locks.trunk()     == DoorLocks::State::LOCKED);
    }
}

TEST_CASE("DoorLocks: independent toggle — only the toggled door changes", "[PhysicalWorld][DoorLocks]") {
    DoorLocks locks;
    using S = DoorLocks::State;

    // Toggle driver twice: UNLOCKED → LOCKED → UNLOCKED.
    locks.toggle_driver();
    CHECK(locks.driver()    == S::LOCKED);
    CHECK(locks.passenger() == S::UNLOCKED);  // unchanged
    CHECK(locks.trunk()     == S::UNLOCKED);  // unchanged

    locks.toggle_driver();
    CHECK(locks.driver()    == S::UNLOCKED);  // back to unlocked
    CHECK_FALSE(locks.any_locked());
}

TEST_CASE("DoorLocks: toggle_passenger leaves other doors unaffected", "[PhysicalWorld][DoorLocks]") {
    DoorLocks locks;
    using S = DoorLocks::State;

    locks.lock_all();
    locks.toggle_passenger();  // LOCKED → UNLOCKED
    CHECK(locks.driver()    == S::LOCKED);   // unchanged
    CHECK(locks.passenger() == S::UNLOCKED); // toggled
    CHECK(locks.trunk()     == S::LOCKED);   // unchanged
    CHECK(locks.any_locked());  // driver and trunk still locked
}

TEST_CASE("DoorLocks: toggle_trunk leaves other doors unaffected", "[PhysicalWorld][DoorLocks]") {
    DoorLocks locks;
    using S = DoorLocks::State;

    locks.toggle_trunk();
    CHECK(locks.driver()    == S::UNLOCKED);
    CHECK(locks.passenger() == S::UNLOCKED);
    CHECK(locks.trunk()     == S::LOCKED);
    CHECK(locks.any_locked());

    locks.toggle_trunk();
    CHECK(locks.trunk() == S::UNLOCKED);
    CHECK_FALSE(locks.any_locked());
}

// ---------------------------------------------------------------------------
// BrakeSwitch basic tests (already in PhysicalWorld.h/.cpp, verify compile)
// ---------------------------------------------------------------------------

TEST_CASE("BrakeSwitch: default off, hysteresis", "[PhysicalWorld]") {
    BrakeSwitch sw;
    CHECK_FALSE(sw.pressed());

    CHECK(sw.update(0.05));   // at ON threshold => pressed
    CHECK(sw.pressed());

    CHECK(sw.update(0.04));   // above OFF threshold (0.03) => still pressed
    CHECK(sw.pressed());

    CHECK_FALSE(sw.update(0.02));  // below OFF threshold => released
    CHECK_FALSE(sw.pressed());
}
