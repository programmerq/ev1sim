#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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
    CHECK_THAT(j["spawn"]["x"].get<double>(), WithinAbs(-40.0, 1e-9));
    CHECK_THAT(j["spawn"]["y"].get<double>(), WithinAbs(  0.0, 1e-9));
    CHECK_THAT(j["spawn"]["yaw_deg"].get<double>(), WithinAbs(0.0, 1e-9));

    REQUIRE(j.contains("patches"));
    const auto& patches = j["patches"];
    REQUIRE(patches.size() == 2);

    // Asphalt patch comes first, sits on the -X side, high friction.
    const auto& asphalt = patches[0];
    CHECK(asphalt["type"]    == "plane");
    CHECK(asphalt["surface"] == "asphalt");
    CHECK_THAT(asphalt["friction"].get<double>(), WithinAbs(0.9, 1e-9));
    CHECK_THAT(asphalt["center"][0].get<double>(), WithinAbs(-50.0, 1e-9));
    CHECK_THAT(asphalt["size"][0].get<double>(),   WithinAbs(100.0, 1e-9));
    CHECK_THAT(asphalt["size"][1].get<double>(),   WithinAbs( 20.0, 1e-9));

    // Ice patch sits on the +X side, very low friction.
    const auto& ice = patches[1];
    CHECK(ice["type"]    == "plane");
    CHECK(ice["surface"] == "ice");
    CHECK(ice["friction"].get<double>() < 0.2);
    CHECK_THAT(ice["center"][0].get<double>(), WithinAbs(50.0, 1e-9));
}

// -----------------------------------------------------------------------
TEST_CASE("flat_split_mu level splits friction along Y axis", "[Level]") {
    auto j = ReadLevel("level/flat_split_mu.json");

    REQUIRE(j.contains("spawn"));
    // Vehicle spawns with the chassis straddling y=0.
    CHECK_THAT(j["spawn"]["y"].get<double>(), WithinAbs(0.0, 1e-9));

    REQUIRE(j.contains("patches"));
    const auto& patches = j["patches"];
    REQUIRE(patches.size() == 2);

    // Asphalt on +Y side.
    const auto& asphalt = patches[0];
    CHECK(asphalt["surface"] == "asphalt");
    CHECK_THAT(asphalt["center"][1].get<double>(), WithinAbs( 5.0, 1e-9));
    CHECK_THAT(asphalt["friction"].get<double>(),  WithinAbs(0.9, 1e-9));

    // Ice on -Y side.
    const auto& ice = patches[1];
    CHECK(ice["surface"] == "ice");
    CHECK_THAT(ice["center"][1].get<double>(), WithinAbs(-5.0, 1e-9));
    CHECK(ice["friction"].get<double>() < 0.2);

    // Both patches cover a long straight (200 m).
    CHECK_THAT(asphalt["size"][0].get<double>(), WithinAbs(200.0, 1e-9));
    CHECK_THAT(ice     ["size"][0].get<double>(), WithinAbs(200.0, 1e-9));
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
