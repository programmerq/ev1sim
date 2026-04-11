#include <catch2/catch_test_macros.hpp>

#include "DriverCommand.h"

TEST_CASE("DriverCommand default-initialises to safe values", "[DriverCommand]") {
    DriverCommand cmd;

    CHECK(cmd.throttle    == 0.0);
    CHECK(cmd.front_brake == 0.0);
    CHECK(cmd.rear_brake  == 0.0);
    CHECK(cmd.steering    == 0.0);
    CHECK(cmd.parking_brake  == false);
    CHECK(cmd.reset_vehicle  == false);
    CHECK(cmd.horn_low   == false);
    CHECK(cmd.horn_high  == false);
}

TEST_CASE("DriverCommand fields are independently assignable", "[DriverCommand]") {
    DriverCommand cmd;
    cmd.throttle    = 0.75;
    cmd.front_brake = 0.5;
    cmd.rear_brake  = 0.3;
    cmd.steering    = -0.4;
    cmd.parking_brake = true;

    CHECK(cmd.throttle    == 0.75);
    CHECK(cmd.front_brake == 0.5);
    CHECK(cmd.rear_brake  == 0.3);
    CHECK(cmd.steering    == -0.4);
    CHECK(cmd.parking_brake == true);
    CHECK(cmd.reset_vehicle == false);  // untouched
}
