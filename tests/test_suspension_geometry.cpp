// Checks on the shipped EV1 suspension JSON.  The front is Chrono's
// DoubleWishbone template; the rear is a GenericWheeledSuspension beam axle on
// five links, the layout chassis manual p.271 prints.  In the front file two
// kinds of number live side by side and only one kind came off a vehicle:
//
//   * Track, camber and toe are printed in the EV1 chassis manual (page 62 for
//     the exterior dimensions, page 61 for the alignment specs).  Those values
//     are NOT pinned here: per the owner's 2026-10-08 ruling
//     (ev1-canon:ci-checks) a blocking test checks structure and behaviour,
//     never a number read off a manual page, since a better reading replaces
//     it.  What is checked is that the files stay self-consistent (declared
//     track == spindle position == declared scale) and keep the rear track
//     narrower than the front.
//   * Every front hardpoint coordinate is Chrono's stock `sedan` sample rescaled to
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
// The rear file's hardpoints come from the ev1 repo's suspension baseline and
// carry their own table ("//hardpoints_ev1_mm", in the body repair manual's
// datum frame) and frame map ("//frame").  The rear checks recompute every
// Chrono coordinate from that table, so a hand edit to either side fails, and
// check the layout p.271 describes: which links lead, which trail, the track
// bar crossing the car, the spindles on one rigid axle.
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
    "data/vehicle/ev1/suspension/EV1_RearFiveLink.json";
static constexpr const char* kChassis =
    "data/vehicle/ev1/chassis/EV1_Chassis.json";
static constexpr const char* kVehicle =
    "data/vehicle/ev1/vehicle/EV1_Vehicle.json";

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

TEST_CASE("Suspension: the rear track is narrower than the front",
          "[Suspension]") {
    // The chassis manual (page 62) prints a rear track well under the front —
    // an ordering the geometry has to keep, whatever reason GM had for it.
    CHECK(HalfTrack(ReadJson(kRear)) < HalfTrack(ReadJson(kFront)));
}

TEST_CASE("Suspension: the front declares the track its hardpoints were scaled to",
          "[Suspension]") {
    for (const char* file : {kFront}) {
        const auto s = ReadJson(file);
        CAPTURE(file);

        // The file declares a positive track...
        const double declared_track_m = s.at("//track_mm").get<double>() / 1000.0;
        CHECK(declared_track_m > 0.0);

        // ...it is where the spindle actually sits...
        CHECK_THAT(2.0 * HalfTrack(s), WithinAbs(declared_track_m, 1e-6));

        // ...and the declared sedan-template scale is derived from it.
        CHECK_THAT(s.at("//hardpoint_scale_y").get<double>(),
                   WithinAbs(declared_track_m / kSedanTrackM, 1e-6));
    }
}

TEST_CASE("Suspension: every front lateral hardpoint is the sedan template at the declared scale",
          "[Suspension]") {
    // The hardpoints are not EV1 measurements — they are Chrono's sedan sample
    // scaled to the printed EV1 track.  Hold them to exactly that, so a future
    // track edit has to move all of them or fail here.  Tolerance 4 mm: the
    // shipped values are rounded to 3 decimal places, and four front points
    // were hand-rounded up to 3 mm off the exact scale.
    constexpr double kToleranceM = 0.004;
    for (const char* file : {kFront}) {
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

static std::string Provenance(const json& s) {
    REQUIRE(s.contains("//PROVENANCE"));
    std::string note;
    for (const auto& line : s.at("//PROVENANCE")) note += line.get<std::string>() + "\n";
    return note;
}

TEST_CASE("Suspension: the provenance note stays with the numbers", "[Suspension]") {
    // Strip the note and the template coordinates read as EV1 measurements
    // again, which is how they read for the three months the front file existed.
    const std::string front = Provenance(ReadJson(kFront));
    CHECK(front.find("Sedan_DoubleWishbone.json") != std::string::npos);
    CHECK(front.find("chassis manual") != std::string::npos);

    // The rear names its layout source and the exact baseline commit its
    // hardpoints were copied from, so "which numbers?" has one answer.
    const std::string rear = Provenance(ReadJson(kRear));
    CHECK(rear.find("chassis manual, printed page 271") != std::string::npos);
    CHECK(rear.find("ev1_suspension_hardpoints.csv") != std::string::npos);
    CHECK(rear.find("a6ab7146d91dcdf4536ec2abfc72f979deb1c7d6") != std::string::npos);
}

TEST_CASE("Suspension: camber and toe are present as numbers", "[Suspension]") {
    // The values come from the chassis manual (page 61) and are not pinned —
    // only that Chrono::Vehicle will find a numeric angle under each key.
    for (const char* file : {kFront, kRear}) {
        const auto s = ReadJson(file);
        CAPTURE(file);
        REQUIRE(s.contains("Camber Angle (deg)"));
        REQUIRE(s.contains("Toe Angle (deg)"));
        CHECK(s.at("Camber Angle (deg)").is_number());
        CHECK(s.at("Toe Angle (deg)").is_number());
    }
}

TEST_CASE("Suspension: every front body mass and inertia component is physically positive",
          "[Suspension]") {
    for (const char* file : {kFront}) {
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

TEST_CASE("Suspension: front spring/shock rates and free length are positive", "[Suspension]") {
    for (const char* file : {kFront}) {
        const auto s = ReadJson(file);
        CAPTURE(file);
        CHECK(s.at("Spring").at("Spring Coefficient").get<double>() > 0.0);
        CHECK(s.at("Spring").at("Free Length").get<double>()        > 0.0);
        CHECK(s.at("Shock").at("Damping Coefficient").get<double>() > 0.0);
        CHECK(s.at("Axle").at("Inertia").get<double>()             > 0.0);
    }
}

TEST_CASE("Suspension: front control arms angle outward (upright end past the chassis mounts)",
          "[Suspension]") {
    // Geometric invariant: each control arm's upright (outboard) hardpoint sits
    // further from the centerline than both of its chassis (inboard) mounts.
    // A sign flip here would mean an arm pointing the wrong way.
    for (const char* file : {kFront}) {
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

// ---------------------------------------------------------------------------
// Rear: beam axle on five links (GenericWheeledSuspension)
// ---------------------------------------------------------------------------

namespace {

struct Vec3 { double x, y, z; };

Vec3 V(const json& a) { return {a[0].get<double>(), a[1].get<double>(), a[2].get<double>()}; }

double Dist(const Vec3& a, const Vec3& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}

// The file's own frame map: baseline frame (mm; X rearward, Y right, Z up)
// to Chrono's suspension frame (m; X forward, Y left, Z up), anchored on the
// rear wheel centre.  `mirrored` parts are given for Chrono's left side by the
// baseline's right-side value; a part that crosses the car keeps its side.
Vec3 ToChrono(const json& rear, const Vec3& p_mm, bool mirrored) {
    const Vec3 w = V(rear.at("//frame").at("ev1_rear_wheel_centre_mm"));
    const Vec3 s = V(rear.at("//frame").at("chrono_spindle_m"));
    return {s.x - (p_mm.x - w.x) / 1000.0,
            mirrored ? p_mm.y / 1000.0 : -p_mm.y / 1000.0,
            s.z + (p_mm.z - w.z) / 1000.0};
}

Vec3 Hardpoint(const json& rear, const std::string& name, bool mirrored) {
    return ToChrono(rear, V(rear.at("//hardpoints_ev1_mm").at(name).at("xyz")), mirrored);
}

const json& Named(const json& list, const std::string& name) {
    for (const auto& item : list)
        if (item.at("Name").get<std::string>() == name) return item;
    FAIL("no entry named " << name);
    return list;  // unreachable
}

}  // namespace

TEST_CASE("Rear suspension: a rigid, unsteered axle carries both spindles",
          "[Suspension][Rear]") {
    const auto r = ReadJson(kRear);
    CHECK(r.at("Template").get<std::string>() == "GenericWheeledSuspension");
    CHECK_FALSE(r.at("Steerable").get<bool>());
    CHECK_FALSE(r.at("Independent").get<bool>());

    // Both spindles turn on the one axle body, which is a single part.
    CHECK(r.at("Spindle").at("Attachment Body").get<std::string>() == "Axle Tube");
    CHECK_FALSE(Named(r.at("Bodies"), "Axle Tube").at("Mirrored").get<bool>());

    // Declared track is where the spindle sits.
    CHECK_THAT(2.0 * HalfTrack(r), WithinAbs(r.at("//track_mm").get<double>() / 1000.0, 1e-6));
}

TEST_CASE("Rear suspension: the five links p.271 names, and nothing else",
          "[Suspension][Rear]") {
    const auto r = ReadJson(kRear);
    // Two upper links and two lower links are one mirrored definition each;
    // the track bar is one part across the car.
    CHECK(Named(r.at("Bodies"), "Upper Link").at("Mirrored").get<bool>());
    CHECK(Named(r.at("Bodies"), "Lower Link").at("Mirrored").get<bool>());
    CHECK_FALSE(Named(r.at("Bodies"), "Track Bar").at("Mirrored").get<bool>());
    CHECK(r.at("Bodies").size() == 4);

    // Every link runs from the chassis to the axle tube, and only the axle
    // ends are bushings.
    for (const char* link : {"Lower Link", "Upper Link", "Track Bar"}) {
        CAPTURE(link);
        const auto& body_end = Named(r.at("Joints"), std::string(link) + " Body");
        const auto& axle_end = Named(r.at("Joints"), std::string(link) + " Axle");
        CHECK(body_end.at("Body1").get<std::string>() == "Chassis");
        CHECK(body_end.at("Body2").get<std::string>() == link);
        CHECK(axle_end.at("Body1").get<std::string>() == link);
        CHECK(axle_end.at("Body2").get<std::string>() == "Axle Tube");
        CHECK_FALSE(body_end.contains("Bushing Data"));
        CHECK(axle_end.contains("Bushing Data"));
    }
    CHECK(r.at("Joints").size() == 6);
}

TEST_CASE("Rear suspension: every coordinate is the baseline table through the frame map",
          "[Suspension][Rear]") {
    // The hardpoint table is what carries provenance; the Chrono numbers are
    // a pure function of it.  1 mm: the file rounds to 0.1 mm.
    constexpr double kTolM = 0.001;
    const auto r = ReadJson(kRear);

    for (const auto& j : r.at("Joints")) {
        const std::string name = j.at("Name").get<std::string>();
        CAPTURE(name);
        REQUIRE(j.contains("//hardpoint"));
        const Vec3 want = Hardpoint(r, j.at("//hardpoint").get<std::string>(),
                                    j.at("Mirrored").get<bool>());
        const Vec3 got = V(j.at("Position"));
        CHECK(Dist(want, got) < kTolM);
    }
    for (const auto& t : r.at("TSDAs")) {
        const std::string name = t.at("Name").get<std::string>();
        CAPTURE(name);
        const bool mirrored = t.at("Mirrored").get<bool>();
        const auto& hp = t.at("//hardpoints");
        CHECK(Dist(Hardpoint(r, hp[0].get<std::string>(), mirrored), V(t.at("Point1"))) < kTolM);
        CHECK(Dist(Hardpoint(r, hp[1].get<std::string>(), mirrored), V(t.at("Point2"))) < kTolM);
    }

    // Each link body sits at the midpoint of its two joints.
    for (const char* link : {"Lower Link", "Upper Link", "Track Bar"}) {
        CAPTURE(link);
        const Vec3 a = V(Named(r.at("Joints"), std::string(link) + " Body").at("Position"));
        const Vec3 b = V(Named(r.at("Joints"), std::string(link) + " Axle").at("Position"));
        const Vec3 mid{(a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2};
        CHECK(Dist(mid, V(Named(r.at("Bodies"), link).at("Position"))) < kTolM);
    }

    // The anchor itself: the baseline's wheel centre lands on the spindle.
    const Vec3 wheel = ToChrono(r, V(r.at("//frame").at("ev1_rear_wheel_centre_mm")), true);
    CHECK(Dist(wheel, V(r.at("Spindle").at("COM"))) < 1e-9);
}

TEST_CASE("Rear suspension: upper links lead, lower links trail, the track bar crosses",
          "[Suspension][Rear]") {
    // Chrono X points forward.  A trailing link's body pivot is AHEAD of its
    // axle end; a leading link's is BEHIND it (p.271: "two upper suspension
    // leading links, two lower suspension trailing links").
    const auto r = ReadJson(kRear);
    const auto& J = r.at("Joints");
    const Vec3 lb = V(Named(J, "Lower Link Body").at("Position"));
    const Vec3 la = V(Named(J, "Lower Link Axle").at("Position"));
    const Vec3 ub = V(Named(J, "Upper Link Body").at("Position"));
    const Vec3 ua = V(Named(J, "Upper Link Axle").at("Position"));
    CHECK(lb.x > la.x);
    CHECK(ub.x < ua.x);

    // The upper pair sits above the lower pair at the axle, so together they
    // react the torque of the axle-mounted brakes.
    CHECK(ua.z > la.z);

    // One track bar end on each side of the centreline.
    const Vec3 tb = V(Named(J, "Track Bar Body").at("Position"));
    const Vec3 ta = V(Named(J, "Track Bar Axle").at("Position"));
    CHECK(tb.y * ta.y < 0.0);

    // Springs and dampers act between the chassis and the axle tube.
    for (const char* name : {"Spring", "Shock"}) {
        CAPTURE(name);
        const auto& t = Named(r.at("TSDAs"), name);
        CHECK(t.at("Body1").get<std::string>() == "Chassis");
        CHECK(t.at("Body2").get<std::string>() == "Axle Tube");
        CHECK(V(t.at("Point1")).z > V(t.at("Point2")).z);
    }
}

TEST_CASE("Rear suspension: the spring carries the chassis at the design position",
          "[Suspension][Rear]") {
    // The free length is derived, not chosen: installed length plus the
    // static load over the rate, with the load from the chassis file's mass
    // and COM.  So a mass or COM edit that forgets the spring fails here
    // instead of silently changing the car's ride height.
    const auto r = ReadJson(kRear);
    const auto c = ReadJson(kChassis);
    const auto v = ReadJson(kVehicle);

    const auto& comp = c.at("Components")[0];
    const double m_chassis = comp.at("Mass").get<double>();
    const double x_com = comp.at("Centroidal Frame").at("Location")[0].get<double>();
    const double x_front = v.at("Axles")[0].at("Suspension Location")[0].get<double>();
    const double x_rear  = v.at("Axles")[1].at("Suspension Location")[0].get<double>();
    REQUIRE(v.at("Axles")[1].at("Suspension Input File").get<std::string>() ==
            "ev1/suspension/EV1_RearFiveLink.json");

    constexpr double g = 9.81;
    const double per_spring_n = m_chassis * g * (x_front - x_com) / (x_front - x_rear) / 2.0;

    const auto& spring = Named(r.at("TSDAs"), "Spring");
    const double k = spring.at("Spring Coefficient").get<double>();
    const double installed = Dist(V(spring.at("Point1")), V(spring.at("Point2")));
    const double free_len = spring.at("Free Length").get<double>();
    CHECK(k > 0.0);
    CHECK(free_len > installed);  // compressed at rest
    // 1 mm of free length is ~25 N at this rate, under 1 % of the load.
    CHECK_THAT(free_len, WithinAbs(installed + per_spring_n / k, 0.001));
}

TEST_CASE("Rear suspension: masses, inertias and damping are physically positive",
          "[Suspension][Rear]") {
    const auto r = ReadJson(kRear);
    for (const auto& b : r.at("Bodies")) {
        CAPTURE(b.at("Name").get<std::string>());
        CHECK(b.at("Mass").get<double>() > 0.0);
        for (double i : b.at("Moments of Inertia")) CHECK(i > 0.0);
    }
    CHECK(r.at("Spindle").at("Mass").get<double>() > 0.0);
    for (double i : r.at("Spindle").at("Inertia")) CHECK(i > 0.0);
    CHECK(Named(r.at("TSDAs"), "Shock").at("Damping Coefficient").get<double>() > 0.0);
    CHECK(r.at("Axle").at("Inertia").get<double>() > 0.0);
}
