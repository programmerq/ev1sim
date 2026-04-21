#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ScriptedDriver.h"

using Catch::Matchers::WithinAbs;

// Helper: feed a state into the driver and return the resulting command.
static DriverCommand Tick(ScriptedDriver& d, double sim_time, double speed_mps) {
    VehicleState s;
    s.sim_time  = sim_time;
    s.speed_mps = speed_mps;
    return d.Update(s);
}

// --------------------------------------------------------------------------
TEST_CASE("ScriptedDriver starts in Accel with full throttle",
          "[ScriptedDriver]") {
    ScriptedDriver::Params p;
    p.target_speed_mps = 10.0;
    ScriptedDriver d(p);

    REQUIRE(d.CurrentPhase() == ScriptedDriver::Phase::Accel);

    DriverCommand cmd = Tick(d, 0.0, 0.0);
    CHECK(cmd.throttle    == 1.0);
    CHECK(cmd.front_brake == 0.0);
    CHECK(cmd.rear_brake  == 0.0);
    CHECK(d.CurrentPhase() == ScriptedDriver::Phase::Accel);
}

// --------------------------------------------------------------------------
TEST_CASE("ScriptedDriver transitions Accel -> Hold when target reached",
          "[ScriptedDriver]") {
    ScriptedDriver::Params p;
    p.target_speed_mps = 10.0;
    p.hold_time_s      = 1.0;
    ScriptedDriver d(p);

    // Below target: still accelerating.
    Tick(d, 0.5, 9.0);
    CHECK(d.CurrentPhase() == ScriptedDriver::Phase::Accel);

    // At target: transitions to Hold on this tick.
    DriverCommand cmd = Tick(d, 1.0, 10.0);
    CHECK(d.CurrentPhase() == ScriptedDriver::Phase::Hold);
    // The tick that transitions still emits the Accel command (transition
    // happens after the command is chosen); next tick is the Hold command.
    (void)cmd;
}

// --------------------------------------------------------------------------
TEST_CASE("ScriptedDriver Hold modulates throttle around target",
          "[ScriptedDriver]") {
    ScriptedDriver::Params p;
    p.target_speed_mps = 10.0;
    p.hold_time_s      = 1.0;
    ScriptedDriver d(p);

    // Enter Hold.
    Tick(d, 1.0, 10.0);
    REQUIRE(d.CurrentPhase() == ScriptedDriver::Phase::Hold);

    // Below target during Hold -> some throttle.
    DriverCommand slow = Tick(d, 1.1, 9.5);
    CHECK(slow.throttle > 0.0);
    CHECK(slow.front_brake == 0.0);
    CHECK(slow.rear_brake  == 0.0);

    // At/above target during Hold -> no throttle.
    DriverCommand fast = Tick(d, 1.2, 10.5);
    CHECK(fast.throttle == 0.0);
}

// --------------------------------------------------------------------------
TEST_CASE("ScriptedDriver transitions Hold -> Brake after hold_time_s",
          "[ScriptedDriver]") {
    ScriptedDriver::Params p;
    p.target_speed_mps = 10.0;
    p.hold_time_s      = 0.5;
    ScriptedDriver d(p);

    // Enter Hold at t=1.0.
    Tick(d, 1.0, 10.0);
    REQUIRE(d.CurrentPhase() == ScriptedDriver::Phase::Hold);

    // Still within hold window.
    Tick(d, 1.3, 10.0);
    CHECK(d.CurrentPhase() == ScriptedDriver::Phase::Hold);

    // Past hold window -> Brake.
    Tick(d, 1.6, 10.0);
    CHECK(d.CurrentPhase() == ScriptedDriver::Phase::Brake);
}

// --------------------------------------------------------------------------
TEST_CASE("ScriptedDriver Brake commands full brake and transitions to Done",
          "[ScriptedDriver]") {
    ScriptedDriver::Params p;
    p.target_speed_mps   = 10.0;
    p.hold_time_s        = 0.1;
    p.stop_threshold_mps = 0.2;
    ScriptedDriver d(p);

    // March through Accel and Hold.
    Tick(d, 0.0, 10.0);   // -> Hold
    Tick(d, 0.2, 10.0);   // -> Brake

    REQUIRE(d.CurrentPhase() == ScriptedDriver::Phase::Brake);

    // Still moving: full brake.
    DriverCommand cmd = Tick(d, 0.3, 5.0);
    CHECK(cmd.throttle    == 0.0);
    CHECK(cmd.front_brake == 1.0);
    CHECK(cmd.rear_brake  == 1.0);
    CHECK(d.CurrentPhase() == ScriptedDriver::Phase::Brake);

    // Below threshold: transitions to Done.
    Tick(d, 1.0, 0.1);
    CHECK(d.CurrentPhase() == ScriptedDriver::Phase::Done);
    CHECK(d.IsDone() == true);
}

// --------------------------------------------------------------------------
TEST_CASE("ScriptedDriver Done is a sink (all zeros)", "[ScriptedDriver]") {
    ScriptedDriver::Params p;
    p.target_speed_mps   = 10.0;
    p.hold_time_s        = 0.1;
    p.stop_threshold_mps = 0.5;
    ScriptedDriver d(p);

    Tick(d, 0.0, 10.0);   // -> Hold
    Tick(d, 0.2, 10.0);   // -> Brake
    Tick(d, 0.3, 0.0);    // -> Done
    REQUIRE(d.IsDone());

    // Subsequent ticks stay in Done with zero command, regardless of state.
    DriverCommand cmd = Tick(d, 5.0, 20.0);
    CHECK(cmd.throttle    == 0.0);
    CHECK(cmd.front_brake == 0.0);
    CHECK(cmd.rear_brake  == 0.0);
    CHECK(d.IsDone() == true);
}

// --------------------------------------------------------------------------
TEST_CASE("ScriptedDriver full run: Accel -> Hold -> Brake -> Done",
          "[ScriptedDriver][integration]") {
    ScriptedDriver::Params p;
    p.target_speed_mps   = 8.0;
    p.hold_time_s        = 0.5;
    p.stop_threshold_mps = 0.2;
    ScriptedDriver d(p);

    // Ramp up speed.
    for (double t = 0.0, v = 0.0; v < 8.0; t += 0.1, v += 1.0) {
        Tick(d, t, v);
    }
    // One more tick at/above target to trigger transition.
    Tick(d, 1.0, 8.0);
    REQUIRE(d.CurrentPhase() == ScriptedDriver::Phase::Hold);

    // Hold for the required duration.
    Tick(d, 1.6, 8.0);
    REQUIRE(d.CurrentPhase() == ScriptedDriver::Phase::Brake);

    // Decel to stop.
    for (double v = 8.0; v > 0.1; v -= 2.0) {
        Tick(d, 2.0, v);
    }
    Tick(d, 5.0, 0.0);
    CHECK(d.IsDone());
}
