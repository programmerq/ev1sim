// Guards the EV1 traction drive's SPEED CEILING and its torque-vs-speed shape.
//
// WHY THIS TEST EXISTS
// --------------------
// In the low-mu ABS scenario a spinning front wheel reached ~435 rad/s — about
// 127 m/s of tread speed (~284 mph) on a 0.2915 m tire.  Through the 10.946:1
// reduction that is ~4761 rad/s at the motor, ~45 500 RPM: three and a half
// times the RPM the car sees at its own top speed.  A real EV1 cannot do that
// at any throttle on any surface.
//
// The cause was not a missing clamp — it was a clamp that only looked like one.
// Chrono's simple-map engine does:
//
//     m_motor_speed = ChClamp(motorshaft_speed, 0.0, GetMaxEngineSpeed());
//         — ChEngineSimpleMap.cpp:39
//
// and ChFunctionInterp holds the endpoint value outside the table, because
// extrapolation is off by default:
//
//     if (x >= m_table.rbegin()->first)
//         return m_table.rbegin()->second + GetDer(x) * (x - ...);
//     // ... GetDer() returns 0.0 when !m_extrapolate
//         — ChFunctionInterp.cpp:49-52, 92-100
//
// Together those mean the map's LAST TORQUE VALUE is delivered at every higher
// speed, forever.  The map ended at 13 000 RPM / 74.9 N·m, so the model handed
// out 74.9 N·m at 45 500 RPM — 357 kW from a ~102 kW drive.  Nothing bounded
// the shaft, so an unloaded wheel accelerated without limit.
//
// The fix is structural, and this test pins the structure: the last point of
// BOTH maps must be ZERO torque, and "Maximal Engine Speed RPM" must land on
// them.  Then the endpoint hold delivers nothing above the ceiling instead of a
// fictitious constant.  Both maps, because the coast map's endpoint is held
// forever too — a residual -40 N·m is ~202 kW of phantom regen at the ~48 000
// RPM the pre-fix model reached — and because a full-throttle-only sweep can
// never see the coast map, which (1 - throttle) multiplies out to zero.
//
// TWO LIMITS, DELIBERATELY SEPARATE
// ---------------------------------
//   * The DRIVE-SYSTEM ceiling — 16 000 RPM — belongs here, in the drive's
//     physics.  @source:manual propulsion p328, DTC 007: "a pulse rate greater
//     than 34,000 Hz ... corresponds to a shaft speed of 16,000 RPM", at which
//     "the SERVICE NOW telltale is illuminated, the DTC is stored and
//     propulsion is disabled".  Deliberately not called a HARDWARE ceiling:
//     the threshold is a pulse rate and the reaction is a PCM software action.
//     The manual gives no rotor-burst or bearing speed anywhere.  What 16 000
//     RPM is, exactly, is the speed above which the propulsion system makes no
//     torque — which is all a torque map can represent, and enough.
//   * The ~80 mph top speed is a SOFTWARE calibration and does NOT belong here.
//     @source:manual propulsion p250: "The PIM will limit vehicle speed in the
//     forward direction to 129 km/h (80 mph) and in the reverse direction to
//     48 km/h (30 mph)", enforced by decreasing the torque current.  That is
//     the propulsion controller's job.  (The same page also has the PIM
//     decreasing torque current "to limit drive motor shaft speed" — so
//     shaft-speed limiting is a PIM function on the real car.  This map is not
//     that limiter; it is the envelope the limiter would act inside.)
// So the ceiling asserted below must sit ABOVE the 80 mph equivalent — if it
// ever equals it, the calibration has been baked back into the physics.
//
// Chrono-free by design (Chrono is not in CI): this reads the same JSON the
// Chrono vehicle loads and re-implements the two lookup rules quoted above.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

#ifndef EV1SIM_SOURCE_DIR
#define EV1SIM_SOURCE_DIR "."
#endif

namespace {

constexpr const char* kEngine = "data/vehicle/ev1/powertrain/EV1_EngineSimpleMap.json";
constexpr const char* kTrans  = "data/vehicle/ev1/powertrain/EV1_AutomaticTransmissionSimpleMap.json";
constexpr const char* kDrive  = "data/vehicle/ev1/driveline/EV1_Driveline2WD.json";
constexpr const char* kTire   = "data/vehicle/ev1/tire/EV1_TMeasyTire.json";

constexpr double kPi = 3.14159265358979323846;

// The operating point the owner observed, straight off the VAT run.
constexpr double kObservedSpinWheelRadS = 435.0;

// The drive's rated peak power.  The shipped map is a ~102 kW envelope; the
// service manual (propulsion p250) states 103 kW.  120 kW is a deliberately
// slack bound — the point is to catch "hundreds of kW", not to re-pin the
// rating, which is a separate calibration question (see docs/TODO.md).
constexpr double kDriveRatedPowerW  = 102000.0;
constexpr double kPowerSanityBoundW = 120000.0;

json ReadJson(const std::string& relative_path) {
    std::string path = std::string(EV1SIM_SOURCE_DIR) + "/" + relative_path;
    std::ifstream f(path);
    REQUIRE(f.is_open());
    return json::parse(f);
}

// A torque map keyed by RPM, in the order Chrono stores it (std::map).
using TorqueMap = std::map<double, double>;

TorqueMap ReadMap(const json& engine, const char* key) {
    TorqueMap m;
    for (const auto& pt : engine.at(key))
        m[pt[0].get<double>()] = pt[1].get<double>();
    return m;
}

// Re-implements chrono::ChFunctionInterp::GetVal with extrapolation OFF:
// piecewise-linear inside the table, endpoint value held outside it.
double InterpHoldingEndpoints(const TorqueMap& table, double x) {
    REQUIRE_FALSE(table.empty());
    if (x <= table.begin()->first)  return table.begin()->second;
    if (x >= table.rbegin()->first) return table.rbegin()->second;

    auto hi = table.upper_bound(x);
    auto lo = std::prev(hi);
    const double t = (x - lo->first) / (hi->first - lo->first);
    return lo->second + t * (hi->second - lo->second);
}

// Re-implements chrono::vehicle::ChEngineSimpleMap::Synchronize: clamp the
// shaft speed into the map's domain, then blend the two maps by throttle.
double DeliveredTorqueNm(const TorqueMap& zero_throttle,
                         const TorqueMap& full_throttle,
                         double max_engine_rpm,
                         double shaft_rpm,
                         double throttle) {
    const double looked_up = std::min(std::max(shaft_rpm, 0.0), max_engine_rpm);
    const double t_zero = InterpHoldingEndpoints(zero_throttle, looked_up);
    const double t_full = InterpHoldingEndpoints(full_throttle, looked_up);
    return t_zero * (1.0 - throttle) + t_full * throttle;
}

double RpmToRadS(double rpm) { return rpm * 2.0 * kPi / 60.0; }
double RadSToRpm(double w)   { return w * 60.0 / (2.0 * kPi); }
double MpsToMph(double v)    { return v * 2.2369362920544; }

// Reduction ratio, read from the driveline rather than hard-coded, so a change
// there cannot silently invalidate the road-speed assertions below.
double ReductionRatio() {
    const double conical = ReadJson(kDrive).at("Gear Ratio").at("Conical Gear").get<double>();
    REQUIRE(conical > 0.0);
    return 1.0 / conical;
}

double TireRadiusM() {
    return ReadJson(kTire).at("Design").at("Unloaded Radius [m]").get<double>();
}

}  // namespace

// ---------------------------------------------------------------------------
// Self-test: the lookup helpers must actually reproduce Chrono's two rules.
// Without this the rest of the file could be asserting against a helper that
// silently returns something harmless, and every check below would pass for
// the wrong reason.
// ---------------------------------------------------------------------------
// What this self-test can and cannot do: it pins the helper against the two
// Chrono rules as they read in 9.0.1, so the helper cannot silently drift into
// something that no longer reproduces the defect.  It cannot catch CHRONO
// drifting away from the helper — if a future Chrono turned endpoint
// extrapolation on by default, this file would keep passing while the real
// vehicle changed behaviour.  Guarding that needs a Chrono-linked test, which
// this suite deliberately is not (Chrono is not in CI).
TEST_CASE("Motor ceiling: the map-lookup helper matches Chrono's semantics",
          "[Motor][Ceiling][Selftest]") {
    const TorqueMap t{{1000.0, 100.0}, {2000.0, 50.0}};

    // Interior: linear between bracketing points.
    CHECK_THAT(InterpHoldingEndpoints(t, 1500.0), WithinAbs(75.0, 1e-9));
    // Below the table: hold the first value (extrapolation off).
    CHECK_THAT(InterpHoldingEndpoints(t,    0.0), WithinAbs(100.0, 1e-9));
    // Above the table: hold the LAST value.  This is the rule that turned a
    // finite map into unbounded power — if this ever became 0.0, the test
    // would stop being able to see the defect it was written for.
    CHECK_THAT(InterpHoldingEndpoints(t, 9e9), WithinAbs(50.0, 1e-9));

    // And the engine's clamp is on the LOOKUP, not on the shaft: a shaft far
    // above max_engine_rpm still yields the torque defined AT max.
    const TorqueMap zero{{0.0, 0.0}, {2000.0, 0.0}};
    CHECK_THAT(DeliveredTorqueNm(zero, t, /*max_rpm=*/2000.0, /*shaft=*/1e6, 1.0),
               WithinAbs(50.0, 1e-9));
}

// ---------------------------------------------------------------------------
// The structural fix.
// ---------------------------------------------------------------------------
TEST_CASE("Motor ceiling: BOTH maps end at zero torque and the clamp lands there",
          "[Motor][Ceiling]") {
    const auto engine = ReadJson(kEngine);
    const auto full   = ReadMap(engine, "Map Full Throttle");
    const auto zero   = ReadMap(engine, "Map Zero Throttle");
    const double max_rpm = engine.at("Maximal Engine Speed RPM").get<double>();

    // Both maps' last points must be zero torque.  Chrono holds each map's
    // endpoint at every higher speed, so a non-zero last point is not a
    // ceiling — it is a constant torque applied to infinity.  That applies to
    // the COAST map just as much: a residual -40 N·m held forever is ~197 kW
    // of phantom regen at the ~48 000 RPM the pre-fix model actually reached,
    // from a drive rated ~102 kW.  Above the ceiling propulsion is disabled,
    // so neither sign of torque survives.
    CHECK_THAT(full.rbegin()->second, WithinAbs(0.0, 1e-9));
    CHECK_THAT(zero.rbegin()->second, WithinAbs(0.0, 1e-9));

    // ...and the lookup clamp must land exactly on those zero points.  If
    // max_rpm sat below them, the clamp would pin each lookup at a lower,
    // non-zero torque and the zero points would be unreachable.
    CHECK_THAT(max_rpm, WithinAbs(full.rbegin()->first, 1e-9));
    CHECK_THAT(max_rpm, WithinAbs(zero.rbegin()->first, 1e-9));

    // Torque must be strictly positive right below the ceiling — the ramp to
    // zero has to be a ramp, not the whole high-speed region zeroed out...
    CHECK(InterpHoldingEndpoints(full, max_rpm - 1000.0) > 0.0);
    // ...and coast drag must still be present there, for the same reason.
    CHECK(InterpHoldingEndpoints(zero, max_rpm - 1000.0) < 0.0);
}

TEST_CASE("Motor ceiling: no torque above the ceiling at ANY throttle position",
          "[Motor][Ceiling]") {
    const auto engine = ReadJson(kEngine);
    const auto full   = ReadMap(engine, "Map Full Throttle");
    const auto zero   = ReadMap(engine, "Map Zero Throttle");
    const double max_rpm = engine.at("Maximal Engine Speed RPM").get<double>();

    // Sweeping only full throttle would leave the coast map unguarded: at
    // throttle 1.0 it is multiplied by (1 - throttle) = 0, so ANY value could
    // sit there — including a wildly wrong one — and a full-throttle-only
    // check would stay green.  Sweep the pedal as well as the speed.
    for (double throttle = 0.0; throttle <= 1.0 + 1e-9; throttle += 0.125) {
        for (double rpm = max_rpm; rpm <= 200000.0; rpm += 10000.0) {
            const double t = DeliveredTorqueNm(zero, full, max_rpm, rpm, throttle);
            INFO("throttle = " << throttle << ", shaft RPM = " << rpm);
            CHECK_THAT(t, WithinAbs(0.0, 1e-9));
        }
    }
}

TEST_CASE("Motor ceiling: 16 000 RPM, and it is not the 80 mph software calibration",
          "[Motor][Ceiling]") {
    const auto engine = ReadJson(kEngine);
    const double max_rpm = engine.at("Maximal Engine Speed RPM").get<double>();

    // @source:manual propulsion p328 (DTC 007): 34 000 Hz on the speed/direction
    // input "corresponds to a shaft speed of 16,000 RPM", above which propulsion
    // is disabled.  This is the speed at which the propulsion system stops
    // making torque — NOT a rotor-burst or bearing speed, which the manual
    // states nowhere.  The map represents torque, so that is the right ceiling
    // for it, but the distinction is why nothing here says "hardware limit".
    CHECK_THAT(max_rpm, WithinAbs(16000.0, 1e-9));

    const double ratio  = ReductionRatio();
    const double radius = TireRadiusM();

    // The ~80 mph software cap (@source:manual propulsion p250) expressed as
    // motor RPM.  The ceiling must sit clearly ABOVE it: the moment the two
    // coincide, a software calibration has been re-encoded as motor physics.
    const double cap_mps = 80.0 / 2.2369362920544;
    const double cap_rpm = RadSToRpm(cap_mps / radius * ratio);
    CHECK(max_rpm > cap_rpm * 1.10);

    // ...but "capable of more than 80 mph, not significantly beyond".  The
    // sourced ceiling is ~1.25x the cap; anything past 1.6x would mean the
    // ceiling had drifted into fantasy.
    CHECK(max_rpm < cap_rpm * 1.60);

    // Sanity: state the ceiling as a road speed so the number is legible.
    // NOTE this uses the UNLOADED tyre radius, as every road-speed figure in
    // this file does; the loaded rolling radius is a few percent smaller, so
    // these mph numbers run correspondingly optimistic.  That is fine for a
    // bounds check and would not be fine for a calibration.
    const double ceiling_mph = MpsToMph(RpmToRadS(max_rpm) / ratio * radius);
    CHECK(ceiling_mph > 90.0);
    CHECK(ceiling_mph < 110.0);
}

// ---------------------------------------------------------------------------
// The receipt: the observed runaway, evaluated the way Chrono evaluates it.
// This is the assertion that fails on the pre-fix map.
// ---------------------------------------------------------------------------
TEST_CASE("Motor ceiling: the observed spinning wheel cannot draw unbounded power",
          "[Motor][Ceiling]") {
    const auto engine = ReadJson(kEngine);
    const auto full   = ReadMap(engine, "Map Full Throttle");
    const auto zero   = ReadMap(engine, "Map Zero Throttle");
    const double max_rpm = engine.at("Maximal Engine Speed RPM").get<double>();

    const double ratio = ReductionRatio();

    // Three shaft speeds, one verdict each — the values that make this a
    // receipt rather than a tautology.
    struct Point { const char* what; double shaft_rpm; };
    const double observed_rpm = RadSToRpm(kObservedSpinWheelRadS * ratio);

    // 1. Inside the calibrated envelope: real power, near the rating.
    const double t_mid = DeliveredTorqueNm(zero, full, max_rpm, 9000.0, 1.0);
    const double p_mid = t_mid * RpmToRadS(9000.0);
    CHECK(p_mid > 0.90 * kDriveRatedPowerW);
    CHECK(p_mid < kPowerSanityBoundW);

    // 2. At the ceiling: no propulsion torque at all.
    const double t_ceil = DeliveredTorqueNm(zero, full, max_rpm, max_rpm, 1.0);
    CHECK_THAT(t_ceil, WithinAbs(0.0, 1e-9));

    // 3. At the speed actually observed (~45 500 RPM): still nothing.  Before
    // the fix this returned 74.9 N·m — 357 kW — which is what let the wheel
    // keep accelerating.
    CHECK(observed_rpm > 40000.0);   // the arithmetic still describes the bug
    const double t_obs = DeliveredTorqueNm(zero, full, max_rpm, observed_rpm, 1.0);
    const double p_obs = t_obs * RpmToRadS(observed_rpm);
    CHECK_THAT(t_obs, WithinAbs(0.0, 1e-9));
    CHECK(p_obs < kPowerSanityBoundW);

    // Stated as the observation was: the drive can put no torque into a wheel
    // turning fast enough to make ~284 mph of tread speed.
    const double tread_mph = MpsToMph(kObservedSpinWheelRadS * TireRadiusM());
    CHECK(tread_mph > 250.0);
}

TEST_CASE("Motor ceiling: full throttle delivers no torque above the ceiling, at any speed",
          "[Motor][Ceiling]") {
    const auto engine = ReadJson(kEngine);
    const auto full   = ReadMap(engine, "Map Full Throttle");
    const auto zero   = ReadMap(engine, "Map Zero Throttle");
    const double max_rpm = engine.at("Maximal Engine Speed RPM").get<double>();

    // Sweep well past the ceiling.  The endpoint hold means one value decides
    // all of these, but sweeping is what makes the guard readable as "no
    // speed escapes".
    for (double rpm = max_rpm; rpm <= 200000.0; rpm += 5000.0) {
        const double t = DeliveredTorqueNm(zero, full, max_rpm, rpm, 1.0);
        INFO("shaft RPM = " << rpm);
        CHECK(t <= 1e-9);
    }
}

// ---------------------------------------------------------------------------
// Regression guard: normal driving must not move.
// ---------------------------------------------------------------------------
TEST_CASE("Motor ceiling: the calibrated envelope below 13 000 RPM is untouched",
          "[Motor][Ceiling][Regression]") {
    const auto engine = ReadJson(kEngine);
    const auto full   = ReadMap(engine, "Map Full Throttle");
    const auto zero   = ReadMap(engine, "Map Zero Throttle");

    // Every point of the pre-fix map, verbatim.  The ceiling was added by
    // EXTENDING the table, never by editing it, so acceleration and coastdown
    // trajectories below 13 000 RPM (~81 mph) are identical by construction —
    // no VAT baseline moves.  A future edit that retunes the envelope has to
    // come here and say so.
    const TorqueMap kFullBefore{
        {-100.0, 150.0}, {0.0, 150.0}, {1000.0, 150.0}, {2000.0, 150.0},
        {3000.0, 150.0}, {4000.0, 150.0}, {5000.0, 150.0}, {6000.0, 150.0},
        {6500.0, 150.0}, {7000.0, 143.0}, {7500.0, 130.0}, {8000.0, 121.7},
        {9000.0, 108.2}, {10000.0, 97.4}, {11000.0, 88.5}, {12000.0, 81.1},
        {13000.0, 74.9}};
    const TorqueMap kZeroBefore{
        {-100.0, 0.0}, {0.0, 0.0}, {1000.0, -5.0}, {2000.0, -5.0},
        {3000.0, -5.0}, {4000.0, -5.0}, {5000.0, -5.0}, {6000.0, -8.0},
        {7000.0, -10.0}, {8000.0, -12.0}, {10000.0, -15.0}, {12000.0, -20.0},
        {13000.0, -25.0}};

    for (const auto& [rpm, torque] : kFullBefore) {
        INFO("full-throttle map at " << rpm << " RPM");
        REQUIRE(full.count(rpm) == 1);
        CHECK_THAT(full.at(rpm), WithinAbs(torque, 1e-9));
    }
    for (const auto& [rpm, torque] : kZeroBefore) {
        INFO("zero-throttle map at " << rpm << " RPM");
        REQUIRE(zero.count(rpm) == 1);
        CHECK_THAT(zero.at(rpm), WithinAbs(torque, 1e-9));
    }
}

TEST_CASE("Motor ceiling: constant torque then constant power, up to the ramp",
          "[Motor][Ceiling]") {
    const auto engine = ReadJson(kEngine);
    const auto full   = ReadMap(engine, "Map Full Throttle");

    // Constant-torque region: flat to the 6500 RPM corner.
    for (double rpm = 0.0; rpm <= 6500.0; rpm += 500.0) {
        INFO("constant-torque region at " << rpm << " RPM");
        CHECK_THAT(InterpHoldingEndpoints(full, rpm), WithinAbs(150.0, 1e-9));
    }

    // Constant-power region: torque falls as 1/omega, so power is flat.  This
    // now runs past the old 13 000 RPM end of the table — the extension
    // continues the same hyperbola rather than inventing a new shape.
    //
    // The sweep starts at 7500, not 7000, and that is a real exclusion rather
    // than a tidy round number: the shipped 7000 RPM point is 143.0 N·m =
    // 104.8 kW, 2.8 % above the envelope, so it would fail this 2 % band.  It
    // is the corner rounding between the constant-torque plateau and the
    // hyperbola, it predates this change, and it is inside the region left
    // untouched deliberately.  It goes away with the corner-point correction
    // tracked in docs/TODO.md; until then, excluding it is honest and
    // narrowing the band to include it would just re-tune a number this
    // change promised not to touch.
    for (double rpm = 7500.0; rpm <= 15500.0; rpm += 500.0) {
        const double p = InterpHoldingEndpoints(full, rpm) * RpmToRadS(rpm);
        INFO("constant-power region at " << rpm << " RPM: " << p << " W");
        CHECK_THAT(p, WithinRel(kDriveRatedPowerW, 0.02));
    }

    // Torque must fall monotonically across the whole speed range.  A drive
    // whose torque rises with speed anywhere is not an induction machine.
    double prev = InterpHoldingEndpoints(full, 6500.0);
    for (double rpm = 6500.0; rpm <= 16000.0; rpm += 250.0) {
        const double t = InterpHoldingEndpoints(full, rpm);
        INFO("monotonic falloff at " << rpm << " RPM");
        CHECK(t <= prev + 1e-9);
        prev = t;
    }
}

// ---------------------------------------------------------------------------
// Keep the second copy of the ceiling honest.
// ---------------------------------------------------------------------------
TEST_CASE("Motor ceiling: the transmission's shift ceiling agrees with the engine's",
          "[Motor][Ceiling]") {
    const double max_rpm =
        ReadJson(kEngine).at("Maximal Engine Speed RPM").get<double>();

    const auto shift = ReadJson(kTrans).at("Gear Box").at("Shift Points Map RPM");
    REQUIRE(shift.size() >= 1);
    const double upshift_rpm = shift[0][1].get<double>();

    // Inert with one forward gear, but it is a second written copy of the
    // drive ceiling and copies drift.  Pin them together.
    //
    // This pin is correct ONLY while there is one forward gear.  Add a second
    // ratio and it becomes actively wrong — an upshift point at the drive
    // ceiling means the box would never upshift.  Whoever adds a second gear
    // must delete this assertion, not satisfy it.
    CHECK_THAT(upshift_rpm, WithinAbs(max_rpm, 1e-9));

    // Single-speed: exactly one forward ratio, and it is direct.
    const auto ratios = ReadJson(kTrans).at("Gear Box").at("Forward Gear Ratios");
    REQUIRE(ratios.size() == 1);
    CHECK_THAT(ratios[0].get<double>(), WithinAbs(1.0, 1e-9));
}
