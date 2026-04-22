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
    // get up to speed before crossing onto the ice at x=0.
    const double spawn_x = j["spawn"]["x"].get<double>();
    CHECK(spawn_x < 0.0);

    REQUIRE(j.contains("patches"));
    const auto& patches = j["patches"];
    REQUIRE(patches.size() == 2);

    // Asphalt patch comes first, sits on the -X side, high friction.
    const auto& asphalt = patches[0];
    CHECK(asphalt["type"]    == "plane");
    CHECK(asphalt["surface"] == "asphalt");
    CHECK_THAT(asphalt["friction"].get<double>(), WithinAbs(0.9, 1e-9));
    CHECK(asphalt["center"][0].get<double>() < 0.0);
    // Asphalt run-up needs real length (≥ 100 m) and a wide body so the
    // driver has room to accelerate and settle before the mu transition.
    CHECK(asphalt["size"][0].get<double>() >= 100.0);
    CHECK(asphalt["size"][1].get<double>() >= 40.0);

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

    // Runway gives the car room to get up to speed on pure asphalt
    // before the split starts.  Spawn must sit on it.
    CHECK((*runway)["friction"].get<double>() == 0.9);
    const double rx = (*runway)["center"][0].get<double>();
    const double rl = (*runway)["size"  ][0].get<double>();
    CHECK(spawn_x >= rx - rl / 2.0);
    CHECK(spawn_x <= rx + rl / 2.0);
    // Runway ends at/before the split zone begins.
    CHECK(rx + rl / 2.0 <= 0.0 + 1e-9);

    // Split-mu zone: long, so the yaw disturbance can play out.
    CHECK_THAT((*split_asph)["friction"].get<double>(), WithinAbs(0.9, 1e-9));
    CHECK((*split_ice)["friction"].get<double>() < 0.2);
    CHECK((*split_asph)["size"][0].get<double>() >= 400.0);
    CHECK((*split_ice )["size"][0].get<double>() >= 400.0);
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
