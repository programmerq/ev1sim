// Checks on the shipped EV1 suspension JSON.  The front is Chrono's
// DoubleWishbone template, the short/long-arm layout chassis manual p.251
// prints; the rear is a GenericWheeledSuspension beam axle on five links, the
// layout chassis manual p.271 prints.
//
// Track, camber and toe are printed in the EV1 chassis manual (page 62 for the
// exterior dimensions, page 61 for the alignment specs).  Those values are NOT
// pinned here: per the owner's 2026-10-08 ruling (ev1-canon:ci-checks) a
// blocking test checks structure and behaviour, never a number read off a
// manual page, since a better reading replaces it.  What is checked is that
// the files stay self-consistent (declared track == spindle position) and keep
// the rear track narrower than the front.
//
// Both files' hardpoints come from the ev1 repo's suspension baseline and
// carry their own table ("//hardpoints_ev1_mm", in the body repair manual's
// datum frame) and frame map ("//frame").  The checks recompute every Chrono
// coordinate from that table, so a hand edit to either side fails, and check
// the layout each manual page describes: on the front a short upper arm over a
// long lower one with the coil-over on the lower arm; on the rear which links
// lead, which trail, the track bar crossing the car, the spindles on one rigid
// axle.  Each spring's free length is checked against the chassis load it has
// to carry, so a mass or COM edit that forgets the springs fails here.
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

TEST_CASE("Suspension: each file declares the track its spindle sits at",
          "[Suspension]") {
    for (const char* file : {kFront, kRear}) {
        const auto s = ReadJson(file);
        CAPTURE(file);
        const double declared_track_m = s.at("//track_mm").get<double>() / 1000.0;
        CHECK(declared_track_m > 0.0);
        CHECK_THAT(2.0 * HalfTrack(s), WithinAbs(declared_track_m, 1e-6));
    }
}

static std::string Provenance(const json& s) {
    REQUIRE(s.contains("//PROVENANCE"));
    std::string note;
    for (const auto& line : s.at("//PROVENANCE")) note += line.get<std::string>() + "\n";
    return note;
}

TEST_CASE("Suspension: the provenance note stays with the numbers", "[Suspension]") {
    // Each file names its layout source and the exact baseline commit its
    // hardpoints were copied from, so "which numbers?" has one answer.
    const std::string front = Provenance(ReadJson(kFront));
    CHECK(front.find("chassis manual, printed page 251") != std::string::npos);
    CHECK(front.find("ev1_suspension_hardpoints.csv") != std::string::npos);
    CHECK(front.find("a6ab7146d91dcdf4536ec2abfc72f979deb1c7d6") != std::string::npos);

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

namespace {

struct Vec3 { double x, y, z; };

Vec3 V(const json& a) { return {a[0].get<double>(), a[1].get<double>(), a[2].get<double>()}; }

double Dist(const Vec3& a, const Vec3& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}

// The baseline wheel centre a file's frame map is anchored on: the one
// "//frame" key ending in "_wheel_centre_mm".
Vec3 WheelCentreMm(const json& susp) {
    for (const auto& [key, value] : susp.at("//frame").items())
        if (key.size() > 16 && key.compare(key.size() - 16, 16, "_wheel_centre_mm") == 0)
            return V(value);
    FAIL("no *_wheel_centre_mm key in //frame");
    return {};
}

// The file's own frame map: baseline frame (mm; X rearward, Y right, Z up)
// to Chrono's suspension frame (m; X forward, Y left, Z up), anchored on the
// axle's wheel centre.  `mirrored` parts are given for Chrono's left side by
// the baseline's right-side value; a part that crosses the car keeps its side.
Vec3 ToChrono(const json& susp, const Vec3& p_mm, bool mirrored) {
    const Vec3 w = WheelCentreMm(susp);
    const Vec3 s = V(susp.at("//frame").at("chrono_spindle_m"));
    return {s.x - (p_mm.x - w.x) / 1000.0,
            mirrored ? p_mm.y / 1000.0 : -p_mm.y / 1000.0,
            s.z + (p_mm.z - w.z) / 1000.0};
}

Vec3 Hardpoint(const json& susp, const std::string& name, bool mirrored) {
    return ToChrono(susp, V(susp.at("//hardpoints_ev1_mm").at(name).at("xyz")), mirrored);
}

// The chassis body's static load per side on the axle at `axle_index`, from
// the chassis file's mass and COM and the vehicle file's axle positions.
double ChassisLoadPerSideN(int axle_index) {
    const auto c = ReadJson(kChassis);
    const auto v = ReadJson(kVehicle);
    const auto& comp = c.at("Components")[0];
    const double m_chassis = comp.at("Mass").get<double>();
    const double x_com = comp.at("Centroidal Frame").at("Location")[0].get<double>();
    const double x_front = v.at("Axles")[0].at("Suspension Location")[0].get<double>();
    const double x_rear  = v.at("Axles")[1].at("Suspension Location")[0].get<double>();
    constexpr double g = 9.81;
    const double share = axle_index == 0 ? (x_com - x_rear) : (x_front - x_com);
    return m_chassis * g * share / (x_front - x_rear) / 2.0;
}

const json& Named(const json& list, const std::string& name) {
    for (const auto& item : list)
        if (item.at("Name").get<std::string>() == name) return item;
    FAIL("no entry named " << name);
    return list;  // unreachable
}

}  // namespace

// ---------------------------------------------------------------------------
// Rear: beam axle on five links (GenericWheeledSuspension)
// ---------------------------------------------------------------------------

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

TEST_CASE("Rear suspension: the five links p.271 names",
          "[Suspension][Rear]") {
    const auto r = ReadJson(kRear);
    // The lower links are one mirrored body; the track bar is one part
    // across the car.  Both hang from the chassis and hold the axle tube.
    CHECK(Named(r.at("Bodies"), "Lower Link").at("Mirrored").get<bool>());
    CHECK_FALSE(Named(r.at("Bodies"), "Track Bar").at("Mirrored").get<bool>());
    for (const char* link : {"Lower Link", "Track Bar"}) {
        CAPTURE(link);
        const auto& body_end = Named(r.at("Joints"), std::string(link) + " Body");
        const auto& axle_end = Named(r.at("Joints"), std::string(link) + " Axle");
        CHECK(body_end.at("Body1").get<std::string>() == "Chassis");
        CHECK(body_end.at("Body2").get<std::string>() == link);
        CHECK(axle_end.at("Body1").get<std::string>() == link);
        CHECK(axle_end.at("Body2").get<std::string>() == "Axle Tube");
    }

    // The upper links are one chassis-to-axle spring-damper on the
    // centreline, unloaded at the design position, with both links' rates:
    // in side view it is the pair, and in roll it adds no stiffness.
    const auto& upper = Named(r.at("TSDAs"), "Upper Links");
    CHECK_FALSE(upper.at("Mirrored").get<bool>());
    CHECK(upper.at("Body1").get<std::string>() == "Chassis");
    CHECK(upper.at("Body2").get<std::string>() == "Axle Tube");
    CHECK(V(upper.at("Point1")).y == 0.0);
    CHECK(V(upper.at("Point2")).y == 0.0);
    const auto& rate = r.at("//upper_link_rate");
    CHECK(upper.at("Spring Coefficient").get<double>() ==
          2.0 * rate.at("k_link_N_per_m").get<double>());
    CHECK(upper.at("Damping Coefficient").get<double>() ==
          2.0 * rate.at("c_link_Ns_per_m").get<double>());
    CHECK(upper.at("Spring Coefficient").get<double>() >
          50.0 * Named(r.at("TSDAs"), "Spring").at("Spring Coefficient").get<double>());
    CHECK_THAT(upper.at("Free Length").get<double>(),
               WithinAbs(Dist(V(upper.at("Point1")), V(upper.at("Point2"))), 0.001));

    // Chrono's default Barzilai-Borwein solver aborts on any stiffness
    // matrix, which is what joint bushing data creates.
    for (const auto& j : r.at("Joints")) {
        CAPTURE(j.at("Name").get<std::string>());
        CHECK_FALSE(j.contains("Bushing Data"));
    }
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
        Vec3 p1 = Hardpoint(r, hp[0].get<std::string>(), mirrored);
        Vec3 p2 = Hardpoint(r, hp[1].get<std::string>(), mirrored);
        if (name == "Upper Links")  // moved to the centreline, per "//links"
            p1.y = p2.y = 0.0;
        CHECK(Dist(p1, V(t.at("Point1"))) < kTolM);
        CHECK(Dist(p2, V(t.at("Point2"))) < kTolM);
    }

    // Each link body sits at the midpoint of its two joints.
    for (const char* link : {"Lower Link", "Track Bar"}) {
        CAPTURE(link);
        const Vec3 a = V(Named(r.at("Joints"), std::string(link) + " Body").at("Position"));
        const Vec3 b = V(Named(r.at("Joints"), std::string(link) + " Axle").at("Position"));
        const Vec3 mid{(a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2};
        CHECK(Dist(mid, V(Named(r.at("Bodies"), link).at("Position"))) < kTolM);
    }

    // The anchor itself: the baseline's wheel centre lands on the spindle.
    const Vec3 wheel = ToChrono(r, WheelCentreMm(r), true);
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
    const Vec3 ub = Hardpoint(r, "upper_link_body", true);
    const Vec3 ua = Hardpoint(r, "upper_link_axle", true);
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
    REQUIRE(ReadJson(kVehicle).at("Axles")[1].at("Suspension Input File").get<std::string>() ==
            "ev1/suspension/EV1_RearFiveLink.json");
    const double per_spring_n = ChassisLoadPerSideN(1);

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

// ---------------------------------------------------------------------------
// Front: short/long arm (DoubleWishbone)
// ---------------------------------------------------------------------------

namespace {

Vec3 Sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 Add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 Scale(const Vec3& a, double k) { return {a.x * k, a.y * k, a.z * k}; }
double Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// `p` turned by `th` radians about the axis through `a` and `b`.
Vec3 RotateAbout(const Vec3& p, const Vec3& a, const Vec3& b, double th) {
    const Vec3 k = Scale(Sub(b, a), 1.0 / Dist(a, b));
    const Vec3 v = Sub(p, a);
    const Vec3 r = Add(Add(Scale(v, std::cos(th)), Scale(Cross(k, v), std::sin(th))),
                       Scale(k, Dot(k, v) * (1.0 - std::cos(th))));
    return Add(a, r);
}

Vec3 Mid(const Vec3& a, const Vec3& b) { return Scale(Add(a, b), 0.5); }

Vec3 At(const json& s, const char* body, const char* key) { return V(s.at(body).at(key)); }

}  // namespace

TEST_CASE("Front suspension: every coordinate is the baseline table through the frame map",
          "[Suspension][Front]") {
    // As on the rear: the table carries provenance, the Chrono numbers are a
    // pure function of it.  1 mm: the file rounds to 0.1 mm.
    constexpr double kTolM = 0.001;
    const auto f = ReadJson(kFront);
    const auto& map = f.at("//coordinates");

    // Every Location in the file is mapped, and every mapping holds.
    for (const auto& [body, value] : f.items()) {
        if (!value.is_object()) continue;
        for (const auto& [key, unused] : value.items()) {
            if (key.rfind("Location", 0) != 0) continue;
            const std::string where = body + "/" + key;
            CAPTURE(where);
            REQUIRE(map.contains(where));
            const Vec3 want = Hardpoint(f, map.at(where).get<std::string>(), true);
            CHECK(Dist(want, V(value.at(key))) < kTolM);
        }
    }

    // Body COMs sit at the centroid of the points that hold them.
    auto centroid = [](const Vec3& a, const Vec3& b, const Vec3& c) {
        return Scale(Add(Add(a, b), c), 1.0 / 3.0);
    };
    for (const char* arm : {"Upper Control Arm", "Lower Control Arm"}) {
        CAPTURE(arm);
        const Vec3 want = centroid(At(f, arm, "Location Chassis Front"),
                                   At(f, arm, "Location Chassis Back"),
                                   At(f, arm, "Location Upright"));
        CHECK(Dist(want, At(f, arm, "COM")) < kTolM);
    }
    const Vec3 upright = centroid(At(f, "Upper Control Arm", "Location Upright"),
                                  At(f, "Lower Control Arm", "Location Upright"),
                                  At(f, "Spindle", "COM"));
    CHECK(Dist(upright, At(f, "Upright", "COM")) < kTolM);

    // The anchor itself: the baseline's wheel centre lands on the spindle.
    CHECK(Dist(ToChrono(f, WheelCentreMm(f), true), At(f, "Spindle", "COM")) < 1e-9);
}

TEST_CASE("Front suspension: a short upper arm over a long lower arm, coil-over on the lower",
          "[Suspension][Front]") {
    // p.251: short/long arm.  In front view the upper arm is the shorter one,
    // which is what makes the wheel gain negative camber in bump.
    const auto f = ReadJson(kFront);
    auto front_view_len = [&](const char* arm) {
        const Vec3 in = Mid(At(f, arm, "Location Chassis Front"), At(f, arm, "Location Chassis Back"));
        const Vec3 out = At(f, arm, "Location Upright");
        return std::hypot(out.y - in.y, out.z - in.z);
    };
    CHECK(front_view_len("Upper Control Arm") < front_view_len("Lower Control Arm"));
    CHECK(At(f, "Upper Control Arm", "Location Upright").z >
          At(f, "Lower Control Arm", "Location Upright").z);

    // The coil-over runs from the body down to the lower arm, landing between
    // the arm's inner pivots and its ball joint.
    for (const char* name : {"Spring", "Shock"}) {
        CAPTURE(name);
        const Vec3 top = At(f, name, "Location Chassis");
        const Vec3 arm = At(f, name, "Location Arm");
        CHECK(top.z > arm.z);
        CHECK(arm.y > At(f, "Lower Control Arm", "Location Chassis Front").y);
        CHECK(arm.y < At(f, "Lower Control Arm", "Location Upright").y);
    }
}

TEST_CASE("Front suspension: the spring carries the chassis at the design position",
          "[Suspension][Front]") {
    // As on the rear, the free length is derived: installed length plus the
    // static spring force over the rate.  Here the spring sits partway out the
    // lower arm, so the force is the corner load over the motion ratio, which
    // comes from the file's own geometry: spring length change per unit of
    // lower ball joint rise as the arm turns on its pivots, carried out to the
    // tyre by the front-view instant centre of the two arms.
    const auto f = ReadJson(kFront);
    REQUIRE(ReadJson(kVehicle).at("Axles")[0].at("Suspension Input File").get<std::string>() ==
            "ev1/suspension/EV1_FrontDoubleWishbone.json");

    const Vec3 pf = At(f, "Lower Control Arm", "Location Chassis Front");
    const Vec3 pb = At(f, "Lower Control Arm", "Location Chassis Back");
    const Vec3 lbj = At(f, "Lower Control Arm", "Location Upright");
    const Vec3 ubj = At(f, "Upper Control Arm", "Location Upright");
    const Vec3 top = At(f, "Spring", "Location Chassis");
    const Vec3 seat = At(f, "Spring", "Location Arm");

    constexpr double th = 1e-5;
    const double installed = Dist(top, seat);
    const double d_spring = Dist(top, RotateAbout(seat, pf, pb, th)) - installed;
    const double d_lbj = RotateAbout(lbj, pf, pb, th).z - lbj.z;
    const double ratio_at_lbj = std::abs(d_spring / d_lbj);

    // Front-view instant centre: where the two arm lines cross.
    const Vec3 ui = Mid(At(f, "Upper Control Arm", "Location Chassis Front"),
                        At(f, "Upper Control Arm", "Location Chassis Back"));
    const Vec3 li = Mid(pf, pb);
    const double mu = (ubj.z - ui.z) / (ubj.y - ui.y);
    const double ml = (lbj.z - li.z) / (lbj.y - li.y);
    const double ic_y = (li.z - ui.z + mu * ui.y - ml * li.y) / (mu - ml);
    const double wheel_y = HalfTrack(f);
    const double motion_ratio = ratio_at_lbj * (lbj.y - ic_y) / (wheel_y - ic_y);
    CHECK(motion_ratio > 0.5);
    CHECK(motion_ratio < 1.0);

    const auto& spring = f.at("Spring");
    const double k = spring.at("Spring Coefficient").get<double>();
    const double free_len = spring.at("Free Length").get<double>();
    const double per_spring_n = ChassisLoadPerSideN(0) / motion_ratio;
    CHECK(k > 0.0);
    CHECK(free_len > installed);  // compressed at rest
    CHECK_THAT(free_len, WithinAbs(installed + per_spring_n / k, 0.001));

    // Coil-over: one line, so the damper shares the spring's two points.
    CHECK(Dist(top, At(f, "Shock", "Location Chassis")) < 1e-9);
    CHECK(Dist(seat, At(f, "Shock", "Location Arm")) < 1e-9);
}
