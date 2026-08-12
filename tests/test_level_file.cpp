#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using Catch::Matchers::WithinAbs;

// Tests validate the shape/values of the shipped level JSON files.
// They do not boot Chrono — VehicleWorld::LoadLevelFile reads the same fields,
// so a well-formed JSON here guarantees a consistent runtime load.
//
// SPAWN-TO-BOUNDARY IS THE LOAD-BEARING NUMBER on every transition level.
// Each of these levels exists so a scenario can finish its LAUNCH — reach
// entry speed and then settle off the throttle — before the vehicle reaches
// the surface it is there to test.  When the runway is shorter than that, the
// scenario silently stops testing what it says it tests: abs_mu_jump crossed
// onto ice at full throttle and braked 34 m past the boundary, so its mu-jump
// never happened at all, and abs_split_mu straddled the seam under power and
// arrived at its brake event already 3.4° off heading — on the one test whose
// verdict IS the yaw.  Nothing in the old assertions could see either: they
// checked that the spawn was on the asphalt patch and that the patch was
// "≥ 100 m", both of which a 40 m runway satisfies.
//
// So each transition case below pins spawn-to-boundary against a measured
// launch budget, with the measurement and its scenario named.  These are the
// assertions that fail against the pre-2026-08-11 geometry.

#ifndef EV1SIM_SOURCE_DIR
#define EV1SIM_SOURCE_DIR "."
#endif

static json ReadLevel(const std::string& relative_path) {
    std::string path = std::string(EV1SIM_SOURCE_DIR) + "/" + relative_path;
    std::ifstream f(path);
    REQUIRE(f.is_open());
    return json::parse(f);
}

// -----------------------------------------------------------------------
TEST_CASE("flat_ice_transition level has asphalt then ice patches", "[Level]") {
    auto j = ReadLevel("level/flat_ice_transition.json");

    REQUIRE(j.contains("spawn"));
    CHECK_THAT(j["spawn"]["y"].get<double>(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(j["spawn"]["yaw_deg"].get<double>(), WithinAbs(0.0, 1e-9));
    // Spawn must sit on the asphalt side of the transition with room to
    // get up to speed AND settle before crossing onto the ice at x=0.
    //
    // abs_mu_jump.json's ramped launch to 15 m/s needs 44.3 m (measured),
    // its settle to the brake event a further 63.3 m, and the brake goes on
    // ~4.9 m before the boundary — 112 m of runway.  110 m is that budget
    // less its rounding.  The pre-2026-08-11 spawn was -40, which is where
    // the car crossed onto the ice at full throttle.
    const double spawn_x = j["spawn"]["x"].get<double>();
    CHECK(spawn_x <= -110.0);

    REQUIRE(j.contains("patches"));
    const auto& patches = j["patches"];
    REQUIRE(patches.size() == 2);

    // Asphalt patch comes first, sits on the -X side, high friction.
    const auto& asphalt = patches[0];
    CHECK(asphalt["type"]    == "plane");
    CHECK(asphalt["surface"] == "asphalt");
    CHECK_THAT(asphalt["friction"].get<double>(), WithinAbs(0.9, 1e-9));
    CHECK(asphalt["center"][0].get<double>() < 0.0);
    // Asphalt run-up needs real length and a wide body so the driver has
    // room to accelerate and settle before the mu transition.
    CHECK(asphalt["size"][0].get<double>() >= 150.0);
    CHECK(asphalt["size"][1].get<double>() >= 40.0);
    // No gap at the transition: the asphalt patch has to reach x=0.
    CHECK(asphalt["center"][0].get<double>()
          + asphalt["size"][0].get<double>() / 2.0 >= 0.0);

    // Ice patch sits on the +X side, very low friction.
    const auto& ice = patches[1];
    CHECK(ice["type"]    == "plane");
    CHECK(ice["surface"] == "ice");
    CHECK(ice["friction"].get<double>() < 0.2);
    CHECK(ice["center"][0].get<double>() > 0.0);
    // Ice field has to be WAY longer than the asphalt so the vehicle
    // can slide to a stop and steer without running off the patch.
    CHECK(ice["size"][0].get<double>() >= 400.0);
    CHECK(ice["size"][1].get<double>() >= 40.0);

    // Spawn sits on the asphalt patch (x inside [center-L/2, center+L/2]).
    const double ax  = asphalt["center"][0].get<double>();
    const double al  = asphalt["size"][0].get<double>();
    CHECK(spawn_x >= ax - al / 2.0);
    CHECK(spawn_x <= ax + al / 2.0);
}

// -----------------------------------------------------------------------
TEST_CASE("flat_low_mu_transition level has asphalt then packed-snow patches",
          "[Level]") {
    auto j = ReadLevel("level/flat_low_mu_transition.json");

    REQUIRE(j.contains("spawn"));
    CHECK_THAT(j["spawn"]["y"].get<double>(), WithinAbs(0.0, 1e-9));
    CHECK_THAT(j["spawn"]["yaw_deg"].get<double>(), WithinAbs(0.0, 1e-9));
    // Spawn must sit on the asphalt side, with real room to reach entry
    // speed and settle before the snow crossing (abs_low_mu_stop.json
    // needs ~41 m to reach 15 m/s; the spawn-to-boundary distance below
    // gives it ~80 m).
    const double spawn_x = j["spawn"]["x"].get<double>();
    CHECK(spawn_x < 0.0);
    CHECK(spawn_x <= -60.0);

    REQUIRE(j.contains("patches"));
    const auto& patches = j["patches"];
    REQUIRE(patches.size() == 2);

    // Asphalt patch comes first, sits on the -X side, high friction.
    const auto& asphalt = patches[0];
    CHECK(asphalt["type"]    == "plane");
    CHECK(asphalt["surface"] == "asphalt");
    CHECK_THAT(asphalt["friction"].get<double>(), WithinAbs(0.9, 1e-9));
    CHECK(asphalt["center"][0].get<double>() < 0.0);
    CHECK(asphalt["size"][0].get<double>() >= 100.0);
    CHECK(asphalt["size"][1].get<double>() >= 40.0);

    // Snow patch sits on the +X side.  Friction is hard-packed snow
    // (J.Y. Wong, Theory of Ground Vehicles, 2nd ed. 1993, p. 26: 0.20
    // peak), not the 0.08 "ice" used by the other transition scenarios
    // (flat_ice_transition.json, flat_split_mu.json, flat_diagonal_mu.json)
    // — deliberately a different, less slick surface for a different test.
    const auto& snow = patches[1];
    CHECK(snow["type"]    == "plane");
    CHECK(snow["surface"] == "snow");
    CHECK_THAT(snow["friction"].get<double>(), WithinAbs(0.20, 1e-9));
    CHECK(snow["center"][0].get<double>() > 0.0);
    CHECK(snow["size"][0].get<double>() >= 400.0);
    CHECK(snow["size"][1].get<double>() >= 40.0);

    // Spawn sits on the asphalt patch (x inside [center-L/2, center+L/2]).
    const double ax  = asphalt["center"][0].get<double>();
    const double al  = asphalt["size"][0].get<double>();
    CHECK(spawn_x >= ax - al / 2.0);
    CHECK(spawn_x <= ax + al / 2.0);

    // Boundary (x=0) must sit inside the snow patch too, with the asphalt
    // patch's +X edge reaching it (no gap at the transition).
    CHECK(ax + al / 2.0 >= 0.0);
    const double sx = snow["center"][0].get<double>();
    const double sl = snow["size"][0].get<double>();
    CHECK(sx - sl / 2.0 <= 0.0);
}

// -----------------------------------------------------------------------
TEST_CASE("flat_split_mu level splits friction along Y axis", "[Level]") {
    auto j = ReadLevel("level/flat_split_mu.json");

    REQUIRE(j.contains("spawn"));
    // Vehicle spawns with the chassis straddling y=0 so wheels end up on
    // both sides of the split once it crosses into the split-mu zone.
    CHECK_THAT(j["spawn"]["y"].get<double>(), WithinAbs(0.0, 1e-9));
    const double spawn_x = j["spawn"]["x"].get<double>();

    REQUIRE(j.contains("patches"));
    const auto& patches = j["patches"];
    // 1 full-asphalt runway + 2 split-mu lanes.
    REQUIRE(patches.size() == 3);

    // Collect patches by their role: the runway straddles y=0 and is
    // pure asphalt; the split lanes sit on +Y (asphalt) and -Y (ice).
    const nlohmann::json* runway      = nullptr;
    const nlohmann::json* split_asph  = nullptr;
    const nlohmann::json* split_ice   = nullptr;
    for (const auto& p : patches) {
        const double cy      = p["center"][1].get<double>();
        const std::string s  = p["surface"].get<std::string>();
        if (s == "asphalt" && std::abs(cy) < 1e-9)     runway     = &p;
        else if (s == "asphalt" && cy > 0.0)           split_asph = &p;
        else if (s == "ice"     && cy < 0.0)           split_ice  = &p;
    }
    REQUIRE(runway     != nullptr);
    REQUIRE(split_asph != nullptr);
    REQUIRE(split_ice  != nullptr);

    // Runway gives the car room to get up to speed on pure asphalt AND to
    // settle off the throttle before the split starts.  Spawn must sit on it.
    //
    // abs_split_mu.json's ramped launch to 20 m/s needs 79.0 m (measured) and
    // its on-asphalt settle a further 52.0 m — 130 m of runway.  128 m is that
    // budget less its rounding.  The pre-2026-08-11 spawn was -60, which put
    // the seam 27 m BEFORE entry speed: the car straddled it under full
    // throttle and yawed 3.4° before the brake event began.
    CHECK((*runway)["friction"].get<double>() == 0.9);
    const double rx = (*runway)["center"][0].get<double>();
    const double rl = (*runway)["size"  ][0].get<double>();
    CHECK(spawn_x >= rx - rl / 2.0);
    CHECK(spawn_x <= rx + rl / 2.0);
    CHECK(spawn_x <= -128.0);
    // Runway ends exactly where the split zone begins — no gap, no overlap.
    CHECK_THAT(rx + rl / 2.0, WithinAbs(0.0, 1e-9));

    // Split-mu zone: long, so the yaw disturbance can play out.
    CHECK_THAT((*split_asph)["friction"].get<double>(), WithinAbs(0.9, 1e-9));
    CHECK((*split_ice)["friction"].get<double>() < 0.2);
    CHECK((*split_asph)["size"][0].get<double>() >= 400.0);
    CHECK((*split_ice )["size"][0].get<double>() >= 400.0);
}

// -----------------------------------------------------------------------
TEST_CASE("flat_diagonal_mu runway holds the whole launch", "[Level]") {
    auto j = ReadLevel("level/flat_diagonal_mu.json");

    REQUIRE(j.contains("spawn"));
    CHECK_THAT(j["spawn"]["y"].get<double>(), WithinAbs(0.0, 1e-9));
    const double spawn_x = j["spawn"]["x"].get<double>();

    REQUIRE(j.contains("patches"));
    const auto& patches = j["patches"];
    // 1 runway + N stripe pairs + 1 tail.
    REQUIRE(patches.size() > 3);

    // The runway is patch 0 by construction (scripts/gen_diagonal_mu_level.py
    // emits it first) and is the only full-width patch on the -X side.
    const auto& runway = patches[0];
    CHECK(runway["surface"] == "asphalt");
    CHECK_THAT(runway["friction"].get<double>(), WithinAbs(0.9, 1e-9));
    const double rx = runway["center"][0].get<double>();
    const double rl = runway["size"  ][0].get<double>();

    // The stripe field starts where the runway ends.
    const double boundary_x = rx + rl / 2.0;
    CHECK_THAT(boundary_x, WithinAbs(-10.0, 1e-9));

    // abs_diagonal_mu.json's ramped launch to 18 m/s needs 63.5 m (measured)
    // and its settle a further ~44 m before the stripes.  107 m is that budget
    // less its rounding.  The pre-2026-08-11 spawn was -75, i.e. 65 m — the
    // launch finished 1.5 m short of the boundary, and the brake fired on the
    // same tick the wait_for_speed barrier released.
    CHECK(boundary_x - spawn_x >= 107.0);
    CHECK(spawn_x >= rx - rl / 2.0);

    // Stripe geometry: pairs of half-width patches one wheelbase long,
    // alternating asphalt/ice across y — the diagonal pattern itself.  If this
    // stops holding, the level is no longer a diagonal-mu surface however long
    // its runway is.
    const auto& s0_top = patches[1];
    const auto& s0_bot = patches[2];
    CHECK_THAT(s0_top["size"][0].get<double>(), WithinAbs(2.512, 1e-9));
    CHECK_THAT(s0_top["size"][1].get<double>(), WithinAbs(15.0, 1e-9));
    CHECK(s0_top["center"][1].get<double>() > 0.0);
    CHECK(s0_bot["center"][1].get<double>() < 0.0);
    CHECK(s0_top["surface"].get<std::string>()
          != s0_bot["surface"].get<std::string>());
    // ...and the next pair flips sides.
    const auto& s1_top = patches[3];
    CHECK(s1_top["surface"].get<std::string>()
          != s0_top["surface"].get<std::string>());
    CHECK_THAT(s1_top["center"][0].get<double>()
               - s0_top["center"][0].get<double>(), WithinAbs(2.512, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("milford level keeps mesh patches (regression)", "[Level]") {
    auto j = ReadLevel("level/milford.json");
    REQUIRE(j.contains("patches"));
    const auto& patches = j["patches"];
    REQUIRE(patches.size() == 2);

    // type field is optional and defaults to "mesh" in the loader; the shipped
    // milford.json omits it entirely.  Verify the mesh field is still present.
    for (const auto& p : patches) {
        CHECK(p.contains("mesh"));
        CHECK_FALSE(p["mesh"].get<std::string>().empty());
    }
}
