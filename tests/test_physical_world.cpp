#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "PhysicalWorld.h"

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

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

// The stalk is a faithful three-contact model (chassis cavities 4047-4049).
// It does NOT decode tap vs. hold — it only opens/closes the raw contacts; PIM
// does the decoding.  These tests assert the contact mechanics: keyboard held
// state passes straight through, the ON/OFF master latch auto-arms on activity
// and opens on CANCEL, and UI presses synthesize a brief (tap) or sustained
// (hold) close.

TEST_CASE("CruiseStalk: all contacts open at construction", "[PhysicalWorld]") {
    CruiseStalk stalk;
    CHECK_FALSE(stalk.set_coast_contact());
    CHECK_FALSE(stalk.resume_accel_contact());
    CHECK_FALSE(stalk.on_off_contact());
}

TEST_CASE("CruiseStalk: idle update keeps every contact open", "[PhysicalWorld]") {
    CruiseStalk stalk;
    for (int i = 0; i < 100; ++i) stalk.update(0.02, false, false);
    CHECK_FALSE(stalk.set_coast_contact());
    CHECK_FALSE(stalk.resume_accel_contact());
    CHECK_FALSE(stalk.on_off_contact());
}

TEST_CASE("CruiseStalk: keyboard SET/COAST held closes the contact and auto-arms ON/OFF",
          "[PhysicalWorld]") {
    CruiseStalk stalk;
    stalk.update(0.016, /*set_coast*/true, /*resume_accel*/false);
    CHECK(stalk.set_coast_contact());
    CHECK_FALSE(stalk.resume_accel_contact());
    CHECK(stalk.on_off_contact());          // any SET/RESUME activity arms ON/OFF

    // Release: the SET/COAST contact opens, but the master latch stays armed
    // (only CANCEL opens it) — mirrors a real ON/OFF switch left on.
    stalk.update(0.016, false, false);
    CHECK_FALSE(stalk.set_coast_contact());
    CHECK(stalk.on_off_contact());
}

TEST_CASE("CruiseStalk: keyboard RESUME/ACCEL held closes only that contact",
          "[PhysicalWorld]") {
    CruiseStalk stalk;
    stalk.update(0.016, /*set_coast*/false, /*resume_accel*/true);
    CHECK(stalk.resume_accel_contact());
    CHECK_FALSE(stalk.set_coast_contact());
    CHECK(stalk.on_off_contact());
}

TEST_CASE("CruiseStalk: CANCEL opens the ON/OFF latch; later activity re-arms it",
          "[PhysicalWorld]") {
    CruiseStalk stalk;
    stalk.update(0.016, true, false);       // arm via SET/COAST activity
    CHECK(stalk.on_off_contact());

    stalk.press_cancel();                   // N key / UI CANCEL
    stalk.update(0.016, false, false);
    CHECK_FALSE(stalk.on_off_contact());    // master latch opened → PIM CANCELs

    // A fresh SET/RESUME activity re-arms the latch.
    stalk.update(0.016, true, false);
    CHECK(stalk.on_off_contact());
}

TEST_CASE("CruiseStalk: UI press_set synthesizes a brief tap close", "[PhysicalWorld]") {
    CruiseStalk stalk;
    stalk.press_set();
    stalk.update(0.016, false, false);
    CHECK(stalk.set_coast_contact());       // closed on the frame after the press
    CHECK(stalk.on_off_contact());          // and arms the latch

    // A tap lasts ~50 ms; advancing well past that re-opens the contact.
    for (int i = 0; i < 10; ++i) stalk.update(0.016, false, false);
    CHECK_FALSE(stalk.set_coast_contact());
    CHECK(stalk.on_off_contact());          // latch survives the tap
}

TEST_CASE("CruiseStalk: UI SPEED close is sustained well beyond a tap", "[PhysicalWorld]") {
    CruiseStalk up;
    up.press_speed_up();                    // sustained RESUME/ACCEL close (~600 ms)
    for (int i = 0; i < 6; ++i) up.update(0.016, false, false);   // ~96 ms in
    CHECK(up.resume_accel_contact());       // a 50 ms tap would already be open
    for (int i = 0; i < 40; ++i) up.update(0.016, false, false);  // well past 600 ms
    CHECK_FALSE(up.resume_accel_contact());

    CruiseStalk down;
    down.press_speed_down();                // sustained SET/COAST close
    for (int i = 0; i < 6; ++i) down.update(0.016, false, false);
    CHECK(down.set_coast_contact());
    CHECK_FALSE(down.resume_accel_contact());
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
// AmbientTempSensor — diurnal sinusoid model
// ---------------------------------------------------------------------------

TEST_CASE("test_ambient_hour_of_day_comes_from_the_scenario_not_the_host",
          "[PhysicalWorld][AmbientTemp][determinism]") {
    using Catch::Matchers::WithinAbs;
    // The hour is the environment preset's, advanced by sim time — so the same
    // scenario publishes the same ambient temperature whenever it is run and
    // on whatever machine.  It used to be localtime_r(system_clock::now()),
    // which made the value a property of the host's clock and timezone.
    CHECK_THAT(AmbientTempSensor::hour_of_day("day", 0.0),   WithinAbs(12.0, 1e-9));
    CHECK_THAT(AmbientTempSensor::hour_of_day("dusk", 0.0),  WithinAbs(18.0, 1e-9));
    CHECK_THAT(AmbientTempSensor::hour_of_day("night", 0.0), WithinAbs(1.0, 1e-9));
    // An unknown preset reads as day rather than throwing the run off.
    CHECK_THAT(AmbientTempSensor::hour_of_day("banana", 0.0), WithinAbs(12.0, 1e-9));

    // Sim time advances it: an hour of sim time is an hour of the day.
    CHECK_THAT(AmbientTempSensor::hour_of_day("day", 3600.0), WithinAbs(13.0, 1e-9));

    // And it wraps rather than walking off the end of the diurnal model.
    CHECK_THAT(AmbientTempSensor::hour_of_day("night", 24.0 * 3600.0),
               WithinAbs(1.0, 1e-6));
    const double wrapped = AmbientTempSensor::hour_of_day("dusk", 10.0 * 3600.0);
    CHECK(wrapped >= 0.0);
    CHECK(wrapped < 24.0);
    CHECK_THAT(wrapped, WithinAbs(4.0, 1e-6));   // 18:00 + 10 h -> 04:00

    // Same scenario inputs, same published values.
    AmbientTempSensor a, b;
    a.update(AmbientTempSensor::hour_of_day("day", 7.5));
    b.update(AmbientTempSensor::hour_of_day("day", 7.5));
    CHECK(a.temp_c() == b.temp_c());
    CHECK(a.humidity_pct() == b.humidity_pct());
}

TEST_CASE("test_ambient_temp_default_config", "[PhysicalWorld][AmbientTemp]") {
    using Catch::Matchers::WithinAbs;
    AmbientTempSensor sensor;

    // Default config: mean=18, amp=8, phase_offset=14h.
    // At noon (12h): angle = 2π*(12-14)/24 = -π/6
    //   cos(-π/6) ≈ 0.866  →  temp ≈ 18 + 8*0.866 ≈ 24.93°C  (above mean)
    sensor.update(12.0);
    CHECK(sensor.temp_c() > 18.0);                           // above mean at noon
    CHECK_THAT(sensor.temp_c(), WithinAbs(24.93, 0.5));

    // At midnight (0h): angle = 2π*(0-14)/24 = -7π/6
    //   cos(-7π/6) ≈ -0.866  →  temp ≈ 18 - 8*0.866 ≈ 11.07°C  (below mean)
    sensor.update(0.0);
    CHECK(sensor.temp_c() < 18.0);                           // below mean at midnight
    CHECK_THAT(sensor.temp_c(), WithinAbs(11.07, 0.5));
}

TEST_CASE("test_ambient_temp_humidity_inverse_correlation", "[PhysicalWorld][AmbientTemp]") {
    using Catch::Matchers::WithinAbs;
    AmbientTempSensor sensor;

    // Peak temperature occurs at phase_offset_hours=14 (2 pm).
    // At that hour, cos(0) = 1 → temp_c = mean + amp = 18 + 8 = 26°C (maximum).
    // Humidity at peak temp: mean_humidity - diurnal_humidity_amp = 55 - 15 = 40%.
    sensor.update(14.0);
    CHECK_THAT(sensor.temp_c(),       WithinAbs(26.0, 0.5));  // max temp
    CHECK_THAT(sensor.humidity_pct(), WithinAbs(40.0, 0.5));  // min humidity

    // Trough temperature occurs 12 h from peak, at 2h (14-12=2).
    // cos(π) = -1 → temp_c = 18 - 8 = 10°C (minimum).
    // Humidity at trough temp: 55 - 15*(-1) = 70%.
    sensor.update(2.0);
    CHECK_THAT(sensor.temp_c(),       WithinAbs(10.0, 0.5));  // min temp
    CHECK_THAT(sensor.humidity_pct(), WithinAbs(70.0, 0.5));  // max humidity

    // Verify inverse: when temp is highest, humidity is lowest, and vice versa.
    double temp_at_peak, humidity_at_peak;
    double temp_at_trough, humidity_at_trough;
    sensor.update(14.0);
    temp_at_peak    = sensor.temp_c();
    humidity_at_peak = sensor.humidity_pct();
    sensor.update(2.0);
    temp_at_trough    = sensor.temp_c();
    humidity_at_trough = sensor.humidity_pct();
    CHECK(temp_at_peak > temp_at_trough);
    CHECK(humidity_at_peak < humidity_at_trough);
}

TEST_CASE("test_ambient_temp_set_config_changes_output", "[PhysicalWorld][AmbientTemp]") {
    using Catch::Matchers::WithinAbs;
    AmbientTempSensor sensor;

    // Override mean_temp_c = 30; amp=8, phase=14h.
    // At peak (14h): temp = 30 + 8 = 38°C.
    // At trough (2h): temp = 30 - 8 = 22°C.
    AmbientTempSensor::Config cfg;
    cfg.mean_temp_c = 30.0;
    sensor.set_config(cfg);

    sensor.update(14.0);  // peak
    CHECK_THAT(sensor.temp_c(), WithinAbs(38.0, 0.5));

    sensor.update(2.0);   // trough
    CHECK_THAT(sensor.temp_c(), WithinAbs(22.0, 0.5));

    // At noon: centered on 30, not 18.
    sensor.update(12.0);
    CHECK(sensor.temp_c() > 24.0);  // well above old default mean of 18
    CHECK_THAT(sensor.temp_c(), WithinAbs(30.0 + 8.0 * 0.866, 0.5));
}

TEST_CASE("test_ambient_temp_phase_offset", "[PhysicalWorld][AmbientTemp]") {
    using Catch::Matchers::WithinAbs;
    AmbientTempSensor sensor;

    // Shift peak to 10h (10am) instead of default 14h.
    AmbientTempSensor::Config cfg;
    cfg.phase_offset_hours = 10.0;
    sensor.set_config(cfg);

    // Peak should now be at 10h.
    sensor.update(10.0);
    double temp_at_10h = sensor.temp_c();
    CHECK_THAT(temp_at_10h, WithinAbs(26.0, 0.5));  // mean + amp = 18 + 8

    // At old peak (14h) with new offset: angle = 2π*(14-10)/24 = π/3
    // cos(π/3) = 0.5  →  temp = 18 + 8*0.5 = 22°C  (not peak)
    sensor.update(14.0);
    double temp_at_14h = sensor.temp_c();
    CHECK(temp_at_14h < temp_at_10h);  // old peak time is no longer the peak
    CHECK_THAT(temp_at_14h, WithinAbs(22.0, 0.5));
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

// ---------------------------------------------------------------------------
// PowerWindows
// ---------------------------------------------------------------------------

TEST_CASE("test_power_windows_default_state", "[PhysicalWorld][PowerWindows]") {
    PowerWindows pw;
    using W = PowerWindows::Window;
    using D = PowerWindows::Direction;

    CHECK(pw.state(W::DRIVER)    == D::NONE);
    CHECK(pw.state(W::PASSENGER) == D::NONE);
    CHECK_FALSE(pw.driver_up());
    CHECK_FALSE(pw.driver_down());
    CHECK_FALSE(pw.passenger_up());
    CHECK_FALSE(pw.passenger_down());
}

TEST_CASE("test_power_windows_press_release_driver_up", "[PhysicalWorld][PowerWindows]") {
    PowerWindows pw;
    using W = PowerWindows::Window;
    using D = PowerWindows::Direction;

    pw.press(W::DRIVER, D::UP);
    CHECK(pw.driver_up());
    CHECK_FALSE(pw.driver_down());
    CHECK(pw.state(W::DRIVER) == D::UP);

    pw.release(W::DRIVER);
    CHECK_FALSE(pw.driver_up());
    CHECK(pw.state(W::DRIVER) == D::NONE);
}

TEST_CASE("test_power_windows_press_overrides_existing", "[PhysicalWorld][PowerWindows]") {
    PowerWindows pw;
    using W = PowerWindows::Window;
    using D = PowerWindows::Direction;

    pw.press(W::DRIVER, D::UP);
    CHECK(pw.driver_up());
    CHECK_FALSE(pw.driver_down());

    // Press DOWN while already UP — state becomes DOWN.
    pw.press(W::DRIVER, D::DOWN);
    CHECK_FALSE(pw.driver_up());
    CHECK(pw.driver_down());
    CHECK(pw.state(W::DRIVER) == D::DOWN);
}

TEST_CASE("test_power_windows_set_held_asserts_and_releases", "[PhysicalWorld][PowerWindows]") {
    PowerWindows pw;
    using W = PowerWindows::Window;
    using D = PowerWindows::Direction;

    // Held → asserted; released → back to NONE.
    pw.set_held(W::DRIVER, D::UP, true);
    CHECK(pw.driver_up());
    pw.set_held(W::DRIVER, D::UP, false);
    CHECK(pw.state(W::DRIVER) == D::NONE);
}

TEST_CASE("test_power_windows_set_held_does_not_clobber_opposite", "[PhysicalWorld][PowerWindows]") {
    PowerWindows pw;
    using W = PowerWindows::Window;
    using D = PowerWindows::Direction;

    // DOWN is held; the (un-held) UP control must not release the switch.
    pw.set_held(W::DRIVER, D::DOWN, true);
    CHECK(pw.driver_down());

    pw.set_held(W::DRIVER, D::UP, false);   // UP button not pressed this frame
    CHECK(pw.driver_down());                 // DOWN survives
    CHECK(pw.state(W::DRIVER) == D::DOWN);

    // Releasing the DOWN control (the active one) does return to NONE.
    pw.set_held(W::DRIVER, D::DOWN, false);
    CHECK(pw.state(W::DRIVER) == D::NONE);
}

TEST_CASE("test_power_windows_independent_per_window", "[PhysicalWorld][PowerWindows]") {
    PowerWindows pw;
    using W = PowerWindows::Window;
    using D = PowerWindows::Direction;

    pw.press(W::DRIVER,    D::UP);
    pw.press(W::PASSENGER, D::DOWN);

    CHECK(pw.driver_up());
    CHECK_FALSE(pw.driver_down());
    CHECK_FALSE(pw.passenger_up());
    CHECK(pw.passenger_down());

    CHECK(pw.state(W::DRIVER)    == D::UP);
    CHECK(pw.state(W::PASSENGER) == D::DOWN);
}

// ---------------------------------------------------------------------------
// RsaKeypadDriver — interior keypad + mode-button state machine (C2).
// ---------------------------------------------------------------------------
//
// Regression coverage for the boot-keying redesign
// (the external sim's boot-keying-race supplement note):
// the faithful detent order is OFF → ACC → RUN → OFF and a cycle must NEVER
// downgrade RUN→ACC.  The old OFF→RUN→ACC→OFF order silently failed the
// "key_on_cycle x2" boot recipe (two cycles landed in ACC).  These tests run
// the actual RsaKeypadDriver (not the SimApp mock used in test_scenario.cpp),
// draining the scheduler tick-by-tick and recording what reaches the bus.

namespace {

// Drives the keypad scheduler for `ticks` updates of `dt`, collecting the
// ordered list of mode-button pulses (1=OFF, 2=ACC, 3=RUN) and the total digit
// pulses emitted.  Mirrors the SimApp publish path: update() then
// consume_fires_now() each tick.
struct KeypadTrace {
    std::vector<std::uint8_t> mode_presses;  // in emission order
    int digit_pulses = 0;
};

KeypadTrace drain_keypad(RsaKeypadDriver& kp, double dt, int ticks) {
    KeypadTrace tr;
    for (int i = 0; i < ticks; ++i) {
        kp.update(dt);
        auto fires = kp.consume_fires_now();
        for (int b = 0; b < 5; ++b)
            if (fires.button_value[b] != 0) ++tr.digit_pulses;
        if (fires.mode_button != 0) tr.mode_presses.push_back(fires.mode_button);
    }
    return tr;
}

}  // namespace

TEST_CASE("test_rsa_keypad_cold_start_reaches_RUN_in_two_cycles",
          "[PhysicalWorld][RsaKeypadDriver][C2]") {
    RsaKeypadDriver kp;
    CHECK(kp.expected_state() == RsaKeypadDriver::ExpectedState::OFF);

    // Cycle 1: OFF → ACC (faithful first detent).
    kp.cycle_k();
    CHECK(kp.expected_state() == RsaKeypadDriver::ExpectedState::ACC);

    // Cycle 2: ACC → RUN.  This is the second of the "x2" boot recipe.
    kp.cycle_k();
    CHECK(kp.expected_state() == RsaKeypadDriver::ExpectedState::RUN);

    // Drain the scheduler (100 ms ticks; plenty for 6 digits + 2 mode presses).
    const auto tr = drain_keypad(kp, 0.10, 20);

    // The default code "111111" is six taps of button 0 → six digit pulses,
    // entered exactly once (on the OFF→ACC transition, opening the auth window).
    CHECK(tr.digit_pulses == 6);

    // Mode presses, in order: ACC (2) then RUN (3) — never an ACC press AFTER
    // RUN, and never a spurious OFF.
    REQUIRE(tr.mode_presses.size() == 2);
    CHECK(tr.mode_presses[0] == 2);  // ACC
    CHECK(tr.mode_presses[1] == 3);  // RUN
}

TEST_CASE("test_rsa_keypad_RUN_press_follows_the_code_not_races_it",
          "[PhysicalWorld][RsaKeypadDriver][C2]") {
    // The co-sim recipe cycles ~300 ms apart, faster than the 600 ms code
    // emission.  RUN must still be pressed AFTER all six digits — otherwise the
    // external sim RSA rejects it (RUN is gated on the auth window being open).
    RsaKeypadDriver kp;
    KeypadTrace tr;

    // t=0: cycle 1 (OFF→ACC), schedules the digits.
    kp.cycle_k();
    // Three 100 ms ticks of code emission (3 digits sent so far)...
    for (int i = 0; i < 3; ++i) {
        kp.update(0.10);
        auto f = kp.consume_fires_now();
        for (int b = 0; b < 5; ++b) if (f.button_value[b]) ++tr.digit_pulses;
        if (f.mode_button) tr.mode_presses.push_back(f.mode_button);
    }
    // ...then cycle 2 (ACC→RUN) arrives mid-sequence (~t=300 ms).
    kp.cycle_k();
    CHECK(kp.expected_state() == RsaKeypadDriver::ExpectedState::RUN);

    // Drain the rest.
    auto rest = drain_keypad(kp, 0.10, 20);
    tr.digit_pulses += rest.digit_pulses;
    for (auto m : rest.mode_presses) tr.mode_presses.push_back(m);

    // All six digits emitted; ACC then RUN, in order; RUN never arrives before
    // the code completes (the FIFO serialises it behind the in-flight digits).
    CHECK(tr.digit_pulses == 6);
    REQUIRE(tr.mode_presses.size() == 2);
    CHECK(tr.mode_presses[0] == 2);  // ACC
    CHECK(tr.mode_presses[1] == 3);  // RUN
}

TEST_CASE("test_rsa_keypad_cycle_never_downgrades_RUN_to_ACC",
          "[PhysicalWorld][RsaKeypadDriver][C2]") {
    RsaKeypadDriver kp;
    kp.cycle_k();  // OFF → ACC
    kp.cycle_k();  // ACC → RUN
    drain_keypad(kp, 0.10, 20);  // flush ACC + RUN presses

    // From RUN, the next cycle must turn the car OFF — NOT step back to ACC.
    kp.cycle_k();
    CHECK(kp.expected_state() == RsaKeypadDriver::ExpectedState::OFF);
    const auto tr = drain_keypad(kp, 0.10, 5);
    REQUIRE(tr.mode_presses.size() == 1);
    CHECK(tr.mode_presses[0] == 1);  // OFF
    CHECK(tr.digit_pulses == 0);     // no re-entry of the code going to OFF
}

TEST_CASE("test_rsa_keypad_full_loop_OFF_ACC_RUN_OFF",
          "[PhysicalWorld][RsaKeypadDriver][C2]") {
    RsaKeypadDriver kp;
    using S = RsaKeypadDriver::ExpectedState;

    // Walk the full detent loop the way it is actually driven — one cycle, then
    // let the scheduler drain before the next (keyboard K presses and scenario
    // key_on_cycle events are always ≥100 ms apart).
    kp.cycle_k(); CHECK(kp.expected_state() == S::ACC);
    auto t1 = drain_keypad(kp, 0.10, 20);
    CHECK(t1.digit_pulses == 6);              // code entered, then ACC
    REQUIRE(t1.mode_presses.size() == 1);
    CHECK(t1.mode_presses[0] == 2);           // ACC

    kp.cycle_k(); CHECK(kp.expected_state() == S::RUN);
    auto t2 = drain_keypad(kp, 0.10, 5);
    CHECK(t2.digit_pulses == 0);              // no re-entry going ACC→RUN
    REQUIRE(t2.mode_presses.size() == 1);
    CHECK(t2.mode_presses[0] == 3);           // RUN

    kp.cycle_k(); CHECK(kp.expected_state() == S::OFF);
    auto t3 = drain_keypad(kp, 0.10, 5);
    CHECK(t3.digit_pulses == 0);              // no re-entry going RUN→OFF
    REQUIRE(t3.mode_presses.size() == 1);
    CHECK(t3.mode_presses[0] == 1);           // OFF

    // Wraps: OFF → ACC again, re-entering the code.
    kp.cycle_k(); CHECK(kp.expected_state() == S::ACC);
    auto t4 = drain_keypad(kp, 0.10, 20);
    CHECK(t4.digit_pulses == 6);              // code re-entered on the wrap
    REQUIRE(t4.mode_presses.size() == 1);
    CHECK(t4.mode_presses[0] == 2);           // ACC
}

TEST_CASE("test_rsa_keypad_custom_code_is_emitted",
          "[PhysicalWorld][RsaKeypadDriver][C2]") {
    RsaKeypadDriver kp;
    // "246802" mixes taps and long-presses across several buttons.
    kp.set_code_string("246802");
    kp.cycle_k();  // OFF → ACC: schedules these digits.

    // Count tap vs long-press pulses across the drain.
    int taps = 0, longs = 0;
    for (int i = 0; i < 20; ++i) {
        kp.update(0.10);
        auto f = kp.consume_fires_now();
        for (int b = 0; b < 5; ++b) {
            if (f.button_value[b] == 1) ++taps;
            else if (f.button_value[b] == 2) ++longs;
        }
    }
    // "246802": 2(long) 4(long) 6(long) 8(long) 0(long) 2(long) → all long-press
    // digits.  Six digit pulses total, all long.
    CHECK(taps == 0);
    CHECK(longs == 6);
}

// ---------------------------------------------------------------------------
// RsaExteriorKeypad
// ---------------------------------------------------------------------------

TEST_CASE("test_rsa_exterior_keypad_button_press", "[PhysicalWorld][RsaExteriorKeypad]") {
    RsaExteriorKeypad kp;

    // Default: all idle.
    for (int i = 0; i < 5; ++i) {
        CHECK(kp.button_value(i) == 0);
        CHECK_FALSE(kp.button_tap(i));
        CHECK_FALSE(kp.button_long(i));
    }

    // Press button 0 (tap).
    kp.press_button(0, /*long_press=*/false);
    CHECK(kp.button_value(0) == 1);
    CHECK(kp.button_tap(0));
    CHECK_FALSE(kp.button_long(0));
    // Other buttons unaffected.
    for (int i = 1; i < 5; ++i)
        CHECK(kp.button_value(i) == 0);

    // clear_oneshots resets all to idle.
    kp.clear_oneshots();
    for (int i = 0; i < 5; ++i)
        CHECK(kp.button_value(i) == 0);
}

TEST_CASE("test_rsa_exterior_keypad_long_press", "[PhysicalWorld][RsaExteriorKeypad]") {
    RsaExteriorKeypad kp;

    kp.press_button(2, /*long_press=*/true);
    CHECK(kp.button_value(2) == 2);
    CHECK_FALSE(kp.button_tap(2));
    CHECK(kp.button_long(2));

    kp.clear_oneshots();
    CHECK(kp.button_value(2) == 0);
}

TEST_CASE("test_rsa_exterior_keypad_enter_code_sequence", "[PhysicalWorld][RsaExteriorKeypad]") {
    RsaExteriorKeypad kp;

    // "111111" = six taps of button 0.
    kp.enter_code_sequence("111111");
    CHECK(kp.sequence_in_progress());

    // First fire: timer=0, should fire immediately.
    kp.update(0.0);
    bool fired = kp.consume_sequence_fire();
    CHECK(fired);
    // button 0 should be tap (value=1).
    CHECK(kp.button_value(0) == 1);
    kp.clear_oneshots();

    // Sequence still in progress (5 more digits).
    CHECK(kp.sequence_in_progress());

    // Advance through remaining 5 digits.
    for (int i = 0; i < 5; ++i) {
        // Advance time past 100ms interval.
        kp.update(0.11);
        bool f = kp.consume_sequence_fire();
        CHECK(f);
        kp.clear_oneshots();
    }

    // All 6 digits consumed — sequence done.
    CHECK_FALSE(kp.sequence_in_progress());
}

TEST_CASE("test_rsa_exterior_keypad_out_of_range_button_ignored", "[PhysicalWorld][RsaExteriorKeypad]") {
    RsaExteriorKeypad kp;
    // button_idx out of range — should not crash, returns 0.
    kp.press_button(-1, false);
    kp.press_button(5, false);
    for (int i = 0; i < 5; ++i)
        CHECK(kp.button_value(i) == 0);
    CHECK(kp.button_value(-1) == 0);
    CHECK(kp.button_value(5) == 0);
}

// ---------------------------------------------------------------------------
// DoorHandles
// ---------------------------------------------------------------------------

TEST_CASE("test_door_handles_default_no_attempt", "[PhysicalWorld][DoorHandles]") {
    DoorHandles dh;
    CHECK_FALSE(dh.driver_attempt());
    CHECK_FALSE(dh.passenger_attempt());
}

TEST_CASE("test_door_handles_driver_attempt_one_shot", "[PhysicalWorld][DoorHandles]") {
    DoorHandles dh;
    dh.attempt_driver();
    CHECK(dh.driver_attempt());
    CHECK_FALSE(dh.passenger_attempt());  // unaffected

    dh.clear_oneshots();
    CHECK_FALSE(dh.driver_attempt());
    CHECK_FALSE(dh.passenger_attempt());
}

TEST_CASE("test_door_handles_passenger_attempt_one_shot", "[PhysicalWorld][DoorHandles]") {
    DoorHandles dh;
    dh.attempt_passenger();
    CHECK_FALSE(dh.driver_attempt());
    CHECK(dh.passenger_attempt());

    dh.clear_oneshots();
    CHECK_FALSE(dh.driver_attempt());
    CHECK_FALSE(dh.passenger_attempt());
}

TEST_CASE("test_door_handles_attempt_when_locked_does_not_change_panel",
          "[PhysicalWorld][DoorHandles]") {
    // Model: door locked → pull attempt sets the flag (for signal publish)
    // but the caller (SimApp) does NOT open the panel.
    // We verify DoorHandles itself doesn't touch panel state — that logic is in SimApp.
    DoorHandles dh;
    DoorLocks   locks;
    locks.lock_all();

    dh.attempt_driver();
    // DoorHandles::driver_attempt() is true regardless of lock state;
    // it is SimApp's job to check lock before toggling panel.
    CHECK(dh.driver_attempt());
    CHECK(locks.driver() == DoorLocks::State::LOCKED);
    // Panel state would not change — but we have no VehiclePanels here.
    // Just confirm DoorHandles doesn't mutate lock state.
    CHECK(locks.driver() == DoorLocks::State::LOCKED);
}

TEST_CASE("test_door_handles_attempt_when_unlocked_opens_panel",
          "[PhysicalWorld][DoorHandles]") {
    // Model: door unlocked → pull attempt sets the flag and SimApp opens the door.
    // We confirm DoorHandles + DoorLocks interaction at the model level.
    DoorHandles dh;
    DoorLocks   locks;  // default: all unlocked

    dh.attempt_driver();
    CHECK(dh.driver_attempt());
    CHECK(locks.driver() == DoorLocks::State::UNLOCKED);
    // In the real SimApp path: since driver == UNLOCKED, SimApp would call
    // m_panels->Toggle(PanelID::DOOR_LEFT).  We just confirm the flag and state.
    dh.clear_oneshots();
    CHECK_FALSE(dh.driver_attempt());
    // Lock state should be unchanged (handle pull doesn't auto-unlock).
    CHECK(locks.driver() == DoorLocks::State::UNLOCKED);
}

// ---------------------------------------------------------------------------
// Seatbelts
// ---------------------------------------------------------------------------

TEST_CASE("Seatbelts: default state is both buckled", "[PhysicalWorld][Seatbelts]") {
    Seatbelts sb;
    CHECK(sb.driver_buckled());
    CHECK(sb.passenger_buckled());
}

TEST_CASE("Seatbelts: set_driver / set_passenger are independent", "[PhysicalWorld][Seatbelts]") {
    Seatbelts sb;
    sb.set_driver(false);
    CHECK_FALSE(sb.driver_buckled());
    CHECK(sb.passenger_buckled());  // passenger unchanged

    sb.set_passenger(false);
    CHECK_FALSE(sb.driver_buckled());
    CHECK_FALSE(sb.passenger_buckled());

    sb.set_driver(true);
    CHECK(sb.driver_buckled());
    CHECK_FALSE(sb.passenger_buckled());  // passenger still unbuckled
}

TEST_CASE("Seatbelts: toggle_driver flips driver only", "[PhysicalWorld][Seatbelts]") {
    Seatbelts sb;
    // Default: both buckled.
    sb.toggle_driver();
    CHECK_FALSE(sb.driver_buckled());
    CHECK(sb.passenger_buckled());  // unchanged

    sb.toggle_driver();
    CHECK(sb.driver_buckled());
    CHECK(sb.passenger_buckled());
}

TEST_CASE("Seatbelts: toggle_passenger flips passenger only", "[PhysicalWorld][Seatbelts]") {
    Seatbelts sb;
    // Default: both buckled.
    sb.toggle_passenger();
    CHECK(sb.driver_buckled());      // unchanged
    CHECK_FALSE(sb.passenger_buckled());

    sb.toggle_passenger();
    CHECK(sb.driver_buckled());
    CHECK(sb.passenger_buckled());
}

// ---------------------------------------------------------------------------
// DoorLockMotor — RHJB dual-H-bridge motor + mechanical lock stroke.
// ---------------------------------------------------------------------------

TEST_CASE("DoorLockMotor: defaults to UNLOCKED, not moving", "[PhysicalWorld][DoorLockMotor]") {
    DoorLockMotor m;
    CHECK(m.stroke() == DoorLockMotor::Stroke::UNLOCKED);
    CHECK(m.position() == 0.0);
    CHECK_FALSE(m.moving());
    CHECK(std::string(m.stroke_name()) == "UNLOCKED");
}

TEST_CASE("DoorLockMotor: both drives low or both high → motor off", "[PhysicalWorld][DoorLockMotor]") {
    DoorLockMotor m;
    m.update(0.1, false, false);   // neither leg energised
    CHECK(m.position() == 0.0);
    m.update(0.1, true, true);     // illegal shoot-through → treated as off
    CHECK(m.position() == 0.0);
    CHECK_FALSE(m.moving());
}

TEST_CASE("DoorLockMotor: lock drive advances toward LOCKED and clamps", "[PhysicalWorld][DoorLockMotor]") {
    using Catch::Matchers::WithinAbs;
    DoorLockMotor m;                       // default 0.5 s traverse
    m.update(0.25, true, false);           // half the stroke
    CHECK_THAT(m.position(), WithinAbs(0.5, 1e-9));
    CHECK(m.stroke() == DoorLockMotor::Stroke::MID_STROKE);
    CHECK(m.moving());
    m.update(0.40, true, false);           // overshoot — clamps at end-of-travel
    CHECK(m.position() == 1.0);
    CHECK(m.stroke() == DoorLockMotor::Stroke::LOCKED);
    CHECK_FALSE(m.moving());
}

TEST_CASE("DoorLockMotor: unlock drive returns toward UNLOCKED", "[PhysicalWorld][DoorLockMotor]") {
    DoorLockMotor m;
    m.set_position(1.0);                   // start LOCKED
    m.update(0.6, false, true);            // unlock for longer than the stroke
    CHECK(m.position() == 0.0);
    CHECK(m.stroke() == DoorLockMotor::Stroke::UNLOCKED);
}

TEST_CASE("DoorLockMotor: 600 ms drive pulse reaches the latch", "[PhysicalWorld][DoorLockMotor]") {
    // Default 0.5 s traverse sits inside the RHJB DLM's 600 ms pulse, so a
    // single pulse drives the latch fully home.
    DoorLockMotor m;
    m.update(0.6, true, false);
    CHECK(m.stroke() == DoorLockMotor::Stroke::LOCKED);
}

TEST_CASE("DoorLockMotor: a winding is live only when the legs differ",
          "[PhysicalWorld][DoorLockMotor]") {
    // Two SPDT relay commons across one reversible motor: both legs closed puts
    // both brushes on the same rail, so neither winding has a differential.
    // This is what keeps the energised columns from being a copy of the relay
    // commands — they disagree exactly here.
    DoorLockMotor m;
    m.update(0.05, false, false);
    CHECK_FALSE(m.lock_energised());
    CHECK_FALSE(m.unlock_energised());

    m.update(0.05, true, false);
    CHECK(m.lock_energised());
    CHECK_FALSE(m.unlock_energised());
    CHECK(m.ran_last_update());

    m.update(0.05, false, true);
    CHECK_FALSE(m.lock_energised());
    CHECK(m.unlock_energised());

    m.update(0.05, true, true);          // both relays closed
    CHECK_FALSE(m.lock_energised());
    CHECK_FALSE(m.unlock_energised());
    CHECK_FALSE(m.ran_last_update());    // and the armature did not turn
}

TEST_CASE("DoorLockMotor: stalls against the end stop with the winding still live",
          "[PhysicalWorld][DoorLockMotor]") {
    // The real motor keeps drawing current after the pawl is home; that is why
    // the drive pulse has to be momentary. stalled() separates "still driving"
    // from "still moving".
    DoorLockMotor m;
    m.update(0.25, true, false);         // mid-stroke
    CHECK(m.ran_last_update());
    CHECK_FALSE(m.stalled());

    m.update(0.40, true, false);         // reaches the locked stop this tick
    CHECK(m.position() == 1.0);
    CHECK(m.ran_last_update());          // it did move, then clamped
    CHECK(m.stalled());

    m.update(0.10, true, false);         // held against the stop
    CHECK(m.lock_energised());
    CHECK_FALSE(m.ran_last_update());
    CHECK(m.stalled());

    m.update(0.10, false, false);        // pulse released
    CHECK_FALSE(m.stalled());
}

// ---------------------------------------------------------------------------
// StepDoorLockPlant — the leg -> stroke -> latch chain, end to end.
//
// This lives on PhysicalWorld rather than inside SimApp's tick precisely so it
// can be driven here: SimApp.cpp is not compiled into the Chrono-free test
// target, so while the chain lived there, handing the plant four zeroes instead
// of the real legs changed no test result.
// ---------------------------------------------------------------------------

TEST_CASE("StepDoorLockPlant: a lock pulse strokes both motors and latches both doors",
          "[PhysicalWorld][DoorLockPlant]") {
    using S = DoorLocks::State;
    PhysicalWorld w;
    REQUIRE(w.door_locks().driver()    == S::UNLOCKED);
    REQUIRE(w.door_locks().passenger() == S::UNLOCKED);

    // The junction block drives both LOCK legs in lockstep for a 600 ms pulse.
    // Nothing latches while the pawls are still travelling.
    for (int i = 0; i < 4; ++i) {                       // 4 x 50 ms = 0.2 s
        const auto step = w.StepDoorLockPlant(0.05, true, false, true, false);
        CHECK_FALSE(step.latched[0]);
        CHECK_FALSE(step.latched[1]);
    }
    CHECK(w.door_locks().driver()    == S::UNLOCKED);
    CHECK(w.door_lock_motor_lh().stroke() == DoorLockMotor::Stroke::MID_STROKE);

    // Reaching the stop is the event that moves the latch, once.
    bool saw_latch = false;
    for (int i = 0; i < 8; ++i) {
        const auto step = w.StepDoorLockPlant(0.05, true, false, true, false);
        if (step.latched[0]) {
            CHECK(step.latched[1]);                     // both doors, in lockstep
            CHECK(step.now_locked[0]);
            CHECK(step.now_locked[1]);
            CHECK_FALSE(saw_latch);                     // reported exactly once
            saw_latch = true;
        }
    }
    CHECK(saw_latch);
    CHECK(w.door_locks().driver()    == S::LOCKED);
    CHECK(w.door_locks().passenger() == S::LOCKED);
    CHECK(w.door_lock_motor_lh().position() == 1.0);
    CHECK(w.door_lock_motor_lh().stalled());            // still driven, at the stop

    // The pulse releases: nothing further latches and nothing unwinds.
    for (int i = 0; i < 4; ++i) {
        const auto step = w.StepDoorLockPlant(0.05, false, false, false, false);
        CHECK_FALSE(step.latched[0]);
    }
    CHECK(w.door_locks().driver() == S::LOCKED);
    CHECK_FALSE(w.door_lock_motor_lh().stalled());

    // An UNLOCK pulse runs it back and latches the other way.
    saw_latch = false;
    for (int i = 0; i < 12; ++i) {
        const auto step = w.StepDoorLockPlant(0.05, false, true, false, true);
        if (step.latched[0]) {
            CHECK_FALSE(step.now_locked[0]);
            saw_latch = true;
        }
    }
    CHECK(saw_latch);
    CHECK(w.door_locks().driver()    == S::UNLOCKED);
    CHECK(w.door_locks().passenger() == S::UNLOCKED);
}

TEST_CASE("StepDoorLockPlant: legs are not crossed between the two doors",
          "[PhysicalWorld][DoorLockPlant]") {
    // Four distinct leg patterns, so a swap of any pair is visible.  A test that
    // drove both LOCK legs together could not tell LH from RH at all.
    PhysicalWorld w;
    w.StepDoorLockPlant(0.10, /*lh_lock=*/true, false, /*rh_lock=*/false, false);
    CHECK(w.door_lock_motor_lh().position() > 0.0);
    CHECK(w.door_lock_motor_rh().position() == 0.0);

    w.door_lock_motor_lh().set_position(0.0);
    w.StepDoorLockPlant(0.10, false, false, /*rh_lock=*/true, false);
    CHECK(w.door_lock_motor_lh().position() == 0.0);
    CHECK(w.door_lock_motor_rh().position() > 0.0);

    // Unlock legs, same test in reverse.
    w.door_lock_motor_lh().set_position(1.0);
    w.door_lock_motor_rh().set_position(1.0);
    w.StepDoorLockPlant(0.10, false, /*lh_unlock=*/true, false, false);
    CHECK(w.door_lock_motor_lh().position() < 1.0);
    CHECK(w.door_lock_motor_rh().position() == 1.0);

    w.door_lock_motor_lh().set_position(1.0);
    w.StepDoorLockPlant(0.10, false, false, false, /*rh_unlock=*/true);
    CHECK(w.door_lock_motor_lh().position() == 1.0);
    CHECK(w.door_lock_motor_rh().position() < 1.0);
}

TEST_CASE("StepDoorLockPlant: both relays closed moves nothing",
          "[PhysicalWorld][DoorLockPlant]") {
    // Shoot-through: both commons on one rail, no differential, no torque.  The
    // acceptance rules see this on the raw leg columns; the plant sees a motor
    // that does not turn.
    PhysicalWorld w;
    for (int i = 0; i < 20; ++i) w.StepDoorLockPlant(0.05, true, true, true, true);
    CHECK(w.door_lock_motor_lh().position() == 0.0);
    CHECK(w.door_lock_motor_rh().position() == 0.0);
    CHECK(w.door_locks().driver() == DoorLocks::State::UNLOCKED);
    CHECK_FALSE(w.door_lock_motor_lh().lock_energised());
    CHECK_FALSE(w.door_lock_motor_lh().unlock_energised());
}

TEST_CASE("StepDoorLockSwitches ages both rockers",
          "[PhysicalWorld][DoorLockSwitch]") {
    PhysicalWorld w;
    w.door_lock_switch_lh().press_lock();
    w.door_lock_switch_rh().press_unlock();
    w.StepDoorLockSwitches(0.05);
    CHECK(w.door_lock_switch_lh().lock_contact());
    CHECK(w.door_lock_switch_rh().unlock_contact());
    w.StepDoorLockSwitches(0.50);
    CHECK_FALSE(w.door_lock_switch_lh().any_contact());
    CHECK_FALSE(w.door_lock_switch_rh().any_contact());
}

// ---------------------------------------------------------------------------
// DoorLockSwitch — the door-mounted momentary rocker.
// ---------------------------------------------------------------------------

TEST_CASE("DoorLockSwitch: both contacts open at construction",
          "[PhysicalWorld][DoorLockSwitch]") {
    DoorLockSwitch sw;
    CHECK_FALSE(sw.lock_contact());
    CHECK_FALSE(sw.unlock_contact());
    CHECK_FALSE(sw.any_contact());
    sw.update(1.0);                       // ageing an unpressed switch is a no-op
    CHECK_FALSE(sw.any_contact());
}

TEST_CASE("DoorLockSwitch: a press closes one contact then opens it",
          "[PhysicalWorld][DoorLockSwitch]") {
    DoorLockSwitch sw;                    // default 0.25 s press
    sw.press_lock();
    CHECK(sw.lock_contact());
    CHECK_FALSE(sw.unlock_contact());     // never both — one rocker, two throws

    sw.update(0.05);
    CHECK(sw.lock_contact());
    sw.update(0.30);                      // past the press duration
    CHECK_FALSE(sw.lock_contact());
    CHECK_FALSE(sw.any_contact());

    sw.press_unlock();
    CHECK(sw.unlock_contact());
    CHECK_FALSE(sw.lock_contact());
    sw.update(0.05);
    sw.update(0.30);
    CHECK_FALSE(sw.any_contact());
}

TEST_CASE("DoorLockSwitch: a press survives a tick longer than the press itself",
          "[PhysicalWorld][DoorLockSwitch]") {
    // The junction block one-shots on the rising edge, so a press that opened
    // and closed inside a single tick would leave it with no edge to see and
    // the whole door-lock chain would read as a healthy quiescent car.  A press
    // is always observable for at least one update().
    DoorLockSwitch sw;
    sw.press_lock();
    sw.update(5.0);                       // one absurdly coarse tick
    CHECK(sw.lock_contact());             // still closed — the edge got out
    sw.update(5.0);
    CHECK_FALSE(sw.lock_contact());       // and released on the next one
}

TEST_CASE("DoorLockMotor: a full stroke lands exactly on the stop",
          "[PhysicalWorld][DoorLockMotor]") {
    // Summing dt/traverse over a stroke can land a few 1e-16 short of the
    // limit, which reports MID_STROKE with the pawl physically home and holds
    // the latched door state one sample late.  Ten 0.05 s steps of a 0.5 s
    // traverse is one of the sums that does it.
    DoorLockMotor m;
    m.set_position(1.0);
    for (int i = 0; i < 10; ++i) m.update(0.05, false, true);
    CHECK(m.position() == 0.0);
    CHECK(m.stroke() == DoorLockMotor::Stroke::UNLOCKED);

    for (int i = 0; i < 10; ++i) m.update(0.05, true, false);
    CHECK(m.position() == 1.0);
    CHECK(m.stroke() == DoorLockMotor::Stroke::LOCKED);
}

TEST_CASE("DoorLockSwitch: release opens the contact immediately",
          "[PhysicalWorld][DoorLockSwitch]") {
    DoorLockSwitch sw;
    sw.press_lock();
    REQUIRE(sw.lock_contact());
    sw.release();
    CHECK_FALSE(sw.any_contact());
}

// ---------------------------------------------------------------------------
// Sounder — LHJB flasher piezo "click".
// ---------------------------------------------------------------------------

TEST_CASE("Sounder: silent at construction", "[PhysicalWorld][Sounder]") {
    Sounder s;
    CHECK_FALSE(s.sounding());
    CHECK(s.click_count() == 0);
}

TEST_CASE("Sounder: counts one click per rising edge", "[PhysicalWorld][Sounder]") {
    Sounder s;
    s.update(true);                 // rising edge → click 1
    CHECK(s.sounding());
    CHECK(s.click_count() == 1);
    s.update(true);                 // still high → no new click
    CHECK(s.click_count() == 1);
    s.update(false);                // falling edge — silent, count unchanged
    CHECK_FALSE(s.sounding());
    CHECK(s.click_count() == 1);
    s.update(true);                 // next rising edge → click 2
    CHECK(s.click_count() == 2);
}

// ---------------------------------------------------------------------------
// PowerSteeringPumpMotor — PSCM-driven pump (minimum-viable plant).
// ---------------------------------------------------------------------------

TEST_CASE("PowerSteeringPumpMotor: present (interlock closed) by default", "[PhysicalWorld][SteeringPump]") {
    PowerSteeringPumpMotor p;
    CHECK(p.interlock_closed());
    CHECK(p.commanded_speed() == 0.0);
    CHECK(p.actual_speed() == 0.0);
}

TEST_CASE("PowerSteeringPumpMotor: actual speed lags then converges to command", "[PhysicalWorld][SteeringPump]") {
    PowerSteeringPumpMotor p;            // default tau 0.15 s
    p.update(0.01, 1.0);
    CHECK(p.commanded_speed() == 1.0);
    CHECK(p.actual_speed() > 0.0);       // started moving
    CHECK(p.actual_speed() < 1.0);       // but lags the command
    for (int i = 0; i < 200; ++i) p.update(0.01, 1.0);   // 2 s
    CHECK(p.actual_speed() > 0.99);      // converged
}

TEST_CASE("PowerSteeringPumpMotor: command is clamped to [0,1]", "[PhysicalWorld][SteeringPump]") {
    PowerSteeringPumpMotor p;
    p.update(0.01, 5.0);
    CHECK(p.commanded_speed() == 1.0);
    p.update(0.01, -3.0);
    CHECK(p.commanded_speed() == 0.0);
}

TEST_CASE("PowerSteeringPumpMotor: zero tau is an instantaneous plant", "[PhysicalWorld][SteeringPump]") {
    using Catch::Matchers::WithinAbs;
    PowerSteeringPumpMotor::Config cfg;
    cfg.response_tau_s = 0.0;
    PowerSteeringPumpMotor p(cfg);
    p.update(0.01, 0.7);
    CHECK_THAT(p.actual_speed(), WithinAbs(0.7, 1e-12));
}

TEST_CASE("PowerSteeringPumpMotor: unplugging opens the HV interlock loop", "[PhysicalWorld][SteeringPump]") {
    PowerSteeringPumpMotor p;
    CHECK(p.interlock_closed());
    p.set_present(false);
    CHECK_FALSE(p.interlock_closed());
    p.set_present(true);
    CHECK(p.interlock_closed());
}

// ---------------------------------------------------------------------------
// HvacControls — driver climate HMI (publishes requests to HTCM).
// ---------------------------------------------------------------------------

TEST_CASE("HvacControls: sane defaults", "[PhysicalWorld][Hvac]") {
    using Catch::Matchers::WithinAbs;
    HvacControls h;
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(21.0, 1e-9));
    CHECK(h.fan()  == HvacControls::Fan::OFF);
    CHECK(h.mode() == HvacControls::Mode::FACE);
    CHECK(h.fan_u8()  == 0);
    CHECK(h.mode_u8() == 0);
    CHECK_FALSE(h.ac_on());
    CHECK_FALSE(h.defrost_on());
}

TEST_CASE("HvacControls: setpoint clamps to the cabin range", "[PhysicalWorld][Hvac]") {
    using Catch::Matchers::WithinAbs;
    HvacControls h;
    h.set_temp_setpoint_c(25.0);
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(25.0, 1e-9));
    h.set_temp_setpoint_c(99.0);
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(HvacControls::kMaxSetpointC, 1e-9));
    h.set_temp_setpoint_c(-40.0);
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(HvacControls::kMinSetpointC, 1e-9));
    // adjust_setpoint_c nudges and clamps.
    h.set_temp_setpoint_c(21.0);
    h.adjust_setpoint_c(2.5);
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(23.5, 1e-9));
    h.adjust_setpoint_c(-100.0);
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(HvacControls::kMinSetpointC, 1e-9));
}

TEST_CASE("HvacControls: fan cycles OFF -> LOW -> MED -> HIGH -> OFF", "[PhysicalWorld][Hvac]") {
    HvacControls h;
    h.cycle_fan(); CHECK(h.fan() == HvacControls::Fan::LOW);  CHECK(h.fan_u8() == 1);
    h.cycle_fan(); CHECK(h.fan() == HvacControls::Fan::MED);  CHECK(h.fan_u8() == 2);
    h.cycle_fan(); CHECK(h.fan() == HvacControls::Fan::HIGH); CHECK(h.fan_u8() == 3);
    h.cycle_fan(); CHECK(h.fan() == HvacControls::Fan::OFF);
}

TEST_CASE("HvacControls: mode cycles FACE -> BILEVEL -> FEET -> DEFROST -> FACE", "[PhysicalWorld][Hvac]") {
    HvacControls h;
    h.cycle_mode(); CHECK(h.mode() == HvacControls::Mode::BILEVEL); CHECK(h.mode_u8() == 1);
    h.cycle_mode(); CHECK(h.mode() == HvacControls::Mode::FEET);    CHECK(h.mode_u8() == 2);
    h.cycle_mode(); CHECK(h.mode() == HvacControls::Mode::DEFROST); CHECK(h.mode_u8() == 3);
    h.cycle_mode(); CHECK(h.mode() == HvacControls::Mode::FACE);
}

TEST_CASE("HvacControls: AC and defrost toggle independently", "[PhysicalWorld][Hvac]") {
    HvacControls h;
    h.toggle_ac();      CHECK(h.ac_on());      CHECK_FALSE(h.defrost_on());
    h.toggle_defrost(); CHECK(h.ac_on());      CHECK(h.defrost_on());
    h.set_ac(false);    CHECK_FALSE(h.ac_on()); CHECK(h.defrost_on());
    h.set_defrost(false); CHECK_FALSE(h.defrost_on());
}

TEST_CASE("HvacControls: non-finite setpoint falls back to the default", "[PhysicalWorld][Hvac]") {
    using Catch::Matchers::WithinAbs;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    HvacControls h;
    h.set_temp_setpoint_c(25.0);               // move off the default first
    h.set_temp_setpoint_c(nan);
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(HvacControls::kDefaultSetpointC, 1e-9));
    h.set_temp_setpoint_c(inf);
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(HvacControls::kDefaultSetpointC, 1e-9));
    h.set_temp_setpoint_c(-inf);
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(HvacControls::kDefaultSetpointC, 1e-9));
    // A non-finite *delta* (current + NaN) must not leak NaN either.
    h.set_temp_setpoint_c(21.0);
    h.adjust_setpoint_c(nan);
    CHECK_THAT(h.temp_setpoint_c(), WithinAbs(HvacControls::kDefaultSetpointC, 1e-9));
}
