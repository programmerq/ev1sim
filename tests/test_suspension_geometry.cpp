// Verification of the shipped EV1 double-wishbone suspension geometry (Round 4).
//
// "Verification" here = executable regression guards on the hand-tuned
// hardpoints, masses, and spring/damper rates (rationale in
// docs/vehicle-dynamics.md).  No Chrono boot — these read the same suspension
// JSON that Chrono::Vehicle loads.  Real EV1 track: front 1.496 m, rear
// 1.281 m (deliberately narrower at the rear for aero).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using Catch::Matchers::WithinAbs;

#ifndef EV1SIM_SOURCE_DIR
#define EV1SIM_SOURCE_DIR "."
#endif

static json ReadJson(const std::string& relative_path) {
    std::string path = std::string(EV1SIM_SOURCE_DIR) + "/" + relative_path;
    std::ifstream f(path);
    REQUIRE(f.is_open());
    return json::parse(f);
}

static constexpr const char* kFront =
    "data/vehicle/ev1/suspension/EV1_FrontDoubleWishbone.json";
static constexpr const char* kRear =
    "data/vehicle/ev1/suspension/EV1_RearDoubleWishbone.json";

// Half-track = the spindle COM's Y coordinate (lateral offset from centerline).
static double HalfTrack(const json& susp) {
    return susp.at("Spindle").at("COM")[1].get<double>();
}

TEST_CASE("Suspension: front/rear track widths match the real EV1", "[Suspension]") {
    CHECK_THAT(2.0 * HalfTrack(ReadJson(kFront)), WithinAbs(1.496, 0.005));
    CHECK_THAT(2.0 * HalfTrack(ReadJson(kRear)),  WithinAbs(1.281, 0.005));
}

TEST_CASE("Suspension: rear track is narrower than the front (aero intent)", "[Suspension]") {
    CHECK(HalfTrack(ReadJson(kRear)) < HalfTrack(ReadJson(kFront)));
}

TEST_CASE("Suspension: every body mass and inertia component is physically positive",
          "[Suspension]") {
    for (const char* file : {kFront, kRear}) {
        const auto s = ReadJson(file);
        CAPTURE(file);
        for (const char* part : {"Spindle", "Upright", "Upper Control Arm", "Lower Control Arm"}) {
            CHECK(s.at(part).at("Mass").get<double>() > 0.0);
        }
        // Spindle stores a diagonal "Inertia" vec3; the arms/upright use
        // "Moments of Inertia".  All principal moments must be > 0.
        for (double i : s.at("Spindle").at("Inertia")) CHECK(i > 0.0);
        for (const char* part : {"Upright", "Upper Control Arm", "Lower Control Arm"}) {
            for (double i : s.at(part).at("Moments of Inertia")) CHECK(i > 0.0);
        }
    }
}

TEST_CASE("Suspension: spring/shock rates and free length are positive", "[Suspension]") {
    for (const char* file : {kFront, kRear}) {
        const auto s = ReadJson(file);
        CAPTURE(file);
        CHECK(s.at("Spring").at("Spring Coefficient").get<double>() > 0.0);
        CHECK(s.at("Spring").at("Free Length").get<double>()        > 0.0);
        CHECK(s.at("Shock").at("Damping Coefficient").get<double>() > 0.0);
        CHECK(s.at("Axle").at("Inertia").get<double>()             > 0.0);
    }
}

TEST_CASE("Suspension: control arms angle outward (upright end past the chassis mounts)",
          "[Suspension]") {
    // Geometric invariant: each control arm's upright (outboard) hardpoint sits
    // further from the centerline than both of its chassis (inboard) mounts.
    // A sign flip here would mean an arm pointing the wrong way.
    for (const char* file : {kFront, kRear}) {
        const auto s = ReadJson(file);
        CAPTURE(file);
        for (const char* arm : {"Upper Control Arm", "Lower Control Arm"}) {
            const double y_upright = s.at(arm).at("Location Upright")[1].get<double>();
            const double y_front   = s.at(arm).at("Location Chassis Front")[1].get<double>();
            const double y_back    = s.at(arm).at("Location Chassis Back")[1].get<double>();
            CAPTURE(arm);
            CHECK(y_upright > y_front);
            CHECK(y_upright > y_back);
        }
    }
}

TEST_CASE("Suspension: camber/toe alignment angles are present and sane", "[Suspension]") {
    for (const char* file : {kFront, kRear}) {
        const auto s = ReadJson(file);
        CAPTURE(file);
        // Static alignment should be small (within ±3°); a large value usually
        // means a units mistake (radians-for-degrees, etc.).
        CHECK(std::abs(s.at("Camber Angle (deg)").get<double>()) <= 3.0);
        CHECK(std::abs(s.at("Toe Angle (deg)").get<double>())    <= 3.0);
    }
}
