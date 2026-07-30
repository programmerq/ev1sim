// Checks on the shipped EV1 suspension JSON.  Two kinds of number live in those
// files and only one kind came off a vehicle:
//
//   * Track, camber and toe are printed in the EV1 chassis manual (page 62 for
//     the exterior dimensions, page 61 for the alignment specs).  Those are
//     pinned here to the printed values.
//   * Every hardpoint coordinate is Chrono's stock `sedan` sample rescaled to
//     the printed track.  Nothing there measures an EV1, so what gets pinned is
//     the rescale itself: each lateral coordinate must equal the sedan value
//     times the scale the file declares in "//hardpoint_scale_y".
//
// That second check is the one that catches the defect these files shipped
// with.  A 2026-04-30 commit set the declared front track to an unsourced
// 1.496 m by adding a flat +13 mm to four outboard Y coordinates and leaving
// the other ten at their 1.470-derived values, so the spindle claimed one track
// while the control arms, spring mounts and body COMs encoded another.  A
// uniform-scale check fails on that; the spindle-only check that shipped
// alongside it passed, and made the wrong number look deliberate.
//
// No Chrono boot — these read the same JSON that Chrono::Vehicle loads.

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

// EV1 chassis manual, printed page 62, "Exterior Dimensions":
//   Track Front 1470 mm (57.9 in) / Track Rear 1244 mm (49.0 in).
// The same table's Overall Height is 1281 mm, two rows above Track Rear — the
// value that was once transcribed into these files as a rear track.
static constexpr double kPrintedTrackFrontM = 1.470;
static constexpr double kPrintedTrackRearM  = 1.244;

// EV1 chassis manual, printed page 61, "ALIGNMENT SPECIFICATIONS" nominals:
//   FRONT Camber -0.10 (range -0.90..0.70), Toe at each wheel -0.05.
//   REAR  Camber -0.50 (range -0.90..-0.10), Toe at each wheel 0.00.
static constexpr double kPrintedCamberFrontDeg = -0.10;
static constexpr double kPrintedToeFrontDeg    = -0.05;
static constexpr double kPrintedCamberRearDeg  = -0.50;
static constexpr double kPrintedToeRearDeg     =  0.00;

// Lateral (Y) coordinates of Chrono 9.0.1's stock sedan double-wishbone sample,
// <chrono>/data/vehicle/sedan/suspension/Sedan_DoubleWishbone.json.  Copied in
// rather than read from the Chrono install so this stays a Chrono-free test,
// and so "these are sedan coordinates" is something the suite checks instead of
// something a comment asserts.
struct SedanHardpoint {
    const char* body;
    const char* key;
    double      y;
};
static const SedanHardpoint kSedanHardpoints[] = {
    {"Spindle",           "COM",                    0.7979},
    {"Upright",           "COM",                    0.7470},
    {"Upper Control Arm", "COM",                    0.5972},
    {"Upper Control Arm", "Location Chassis Front", 0.4700},
    {"Upper Control Arm", "Location Chassis Back",  0.5100},
    {"Upper Control Arm", "Location Upright",       0.6950},
    {"Lower Control Arm", "COM",                    0.6112},
    {"Lower Control Arm", "Location Chassis Front", 0.4200},
    {"Lower Control Arm", "Location Chassis Back",  0.4700},
    {"Lower Control Arm", "Location Upright",       0.7700},
    {"Tierod",            "Location Chassis",       0.4200},
    {"Tierod",            "Location Upright",       0.7700},
    {"Spring",            "Location Chassis",       0.5200},
    {"Spring",            "Location Arm",           0.6200},
};
// The sedan sample's own track, from its spindle COM.
static constexpr double kSedanTrackM = 2.0 * 0.7979;

// Half-track = the spindle COM's Y coordinate (lateral offset from centerline).
static double HalfTrack(const json& susp) {
    return susp.at("Spindle").at("COM")[1].get<double>();
}

TEST_CASE("Suspension: track widths are the chassis-manual printed values",
          "[Suspension]") {
    CHECK_THAT(2.0 * HalfTrack(ReadJson(kFront)), WithinAbs(kPrintedTrackFrontM, 1e-6));
    CHECK_THAT(2.0 * HalfTrack(ReadJson(kRear)),  WithinAbs(kPrintedTrackRearM,  1e-6));
}

TEST_CASE("Suspension: the printed rear track is narrower than the printed front",
          "[Suspension]") {
    // 1244 mm rear against 1470 mm front, both off page 62 — an ordering the
    // geometry has to keep, whatever reason GM had for it.
    CHECK(HalfTrack(ReadJson(kRear)) < HalfTrack(ReadJson(kFront)));
}

TEST_CASE("Suspension: each file declares the track its hardpoints were scaled to",
          "[Suspension]") {
    struct Case { const char* file; double printed_track_m; };
    for (const Case& c : {Case{kFront, kPrintedTrackFrontM}, Case{kRear, kPrintedTrackRearM}}) {
        const auto s = ReadJson(c.file);
        CAPTURE(c.file);

        // The declared track is the printed one...
        const double declared_track_m = s.at("//track_mm").get<double>() / 1000.0;
        CHECK_THAT(declared_track_m, WithinAbs(c.printed_track_m, 1e-9));

        // ...it is where the spindle actually sits...
        CHECK_THAT(2.0 * HalfTrack(s), WithinAbs(declared_track_m, 1e-6));

        // ...and the declared sedan-template scale is derived from it.
        CHECK_THAT(s.at("//hardpoint_scale_y").get<double>(),
                   WithinAbs(declared_track_m / kSedanTrackM, 1e-6));
    }
}

TEST_CASE("Suspension: every lateral hardpoint is the sedan template at the declared scale",
          "[Suspension]") {
    // The hardpoints are not EV1 measurements — they are Chrono's sedan sample
    // scaled to the printed EV1 track.  Hold them to exactly that, so a future
    // track edit has to move all of them or fail here.  Tolerance 4 mm: the
    // shipped values are rounded to 3 decimal places, and four front points
    // were hand-rounded up to 3 mm off the exact scale.
    constexpr double kToleranceM = 0.004;
    for (const char* file : {kFront, kRear}) {
        const auto s = ReadJson(file);
        const double scale = s.at("//hardpoint_scale_y").get<double>();
        CAPTURE(file);
        for (const SedanHardpoint& hp : kSedanHardpoints) {
            const double actual   = s.at(hp.body).at(hp.key)[1].get<double>();
            const double expected = hp.y * scale;
            CAPTURE(hp.body, hp.key, expected, actual);
            CHECK_THAT(actual, WithinAbs(expected, kToleranceM));
        }
    }
}

TEST_CASE("Suspension: the provenance note stays with the numbers", "[Suspension]") {
    // Strip the note and the template coordinates read as EV1 measurements
    // again, which is how they read for the three months this file existed.
    for (const char* file : {kFront, kRear}) {
        const auto s = ReadJson(file);
        CAPTURE(file);
        REQUIRE(s.contains("//PROVENANCE"));
        std::string note;
        for (const auto& line : s.at("//PROVENANCE")) note += line.get<std::string>() + "\n";
        CHECK(note.find("Sedan_DoubleWishbone.json") != std::string::npos);
        CHECK(note.find("chassis manual") != std::string::npos);
    }
}

TEST_CASE("Suspension: camber and toe are the chassis-manual printed nominals",
          "[Suspension]") {
    // These four are the only vehicle-derived angles in the files.  Page 61
    // prints them, so pin them — the previous check bounded them to ±3°, which
    // any plausible number passes and which said nothing about their source.
    const auto front = ReadJson(kFront);
    CHECK_THAT(front.at("Camber Angle (deg)").get<double>(),
               WithinAbs(kPrintedCamberFrontDeg, 1e-9));
    CHECK_THAT(front.at("Toe Angle (deg)").get<double>(),
               WithinAbs(kPrintedToeFrontDeg, 1e-9));

    const auto rear = ReadJson(kRear);
    CHECK_THAT(rear.at("Camber Angle (deg)").get<double>(),
               WithinAbs(kPrintedCamberRearDeg, 1e-9));
    CHECK_THAT(rear.at("Toe Angle (deg)").get<double>(),
               WithinAbs(kPrintedToeRearDeg, 1e-9));
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
