// ClimateReadout — decoding and formatting the climate controller's and the
// APM's readings, the readout's sections, its CSV columns, and the panel on a
// null driver.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ClimateReadout.h"

#include <irrlicht.h>

#include <set>
#include <string>

using namespace ev1sim;
using Catch::Matchers::WithinAbs;

TEST_CASE("Climate temperatures decode from int16 Q8 bit patterns", "[climate_readout]") {
    CHECK_THAT(DegCFromQ8Cell(0u), WithinAbs(0.0, 1e-12));
    CHECK_THAT(DegCFromQ8Cell(25u * 256u + 128u), WithinAbs(25.5, 1e-12));
    CHECK_THAT(DegCFromQ8Cell(0xF600u), WithinAbs(-10.0, 1e-12));   // -10 C as int16 Q8
}

TEST_CASE("Climate readings format in their own units", "[climate_readout]") {
    CHECK(FormatClimateReading(ClimateReading::kOutsideAirTemp, -10.0) == "Outside air: -10.0 C");
    CHECK(FormatClimateReading(ClimateReading::kRefrigHighKpa, 1520.0) ==
          "Refrigerant high side: 1520 kPa");
    CHECK(FormatClimateReading(ClimateReading::kWindshieldRailV, 47.12) == "Output rail: 47.1 V");
    CHECK(FormatClimateReading(ClimateReading::kCompressorDutyPct, 45.0) ==
          "Speed command duty: 45 %");
    CHECK(FormatClimateReading(ClimateReading::kCompressorFreqHz, 70.0) ==
          "Target drive frequency: 70 Hz");
    CHECK(FormatClimateReading(ClimateReading::kReversingValve, 1.0) ==
          "Heat-pump reversing valve: ON");
    CHECK(FormatClimateReading(ClimateReading::kCoolantLevelLow, 0.0) == "Coolant level low: OFF");
    CHECK(FormatClimateReading(ClimateReading::kModeDoor, 128.0) == "Mode door: 128 of 255");
    CHECK(FormatClimateReading(ClimateReading::kPackTemp, std::nullopt) == "Battery pack: ---");
}

TEST_CASE("Climate readout lists every reading under its section", "[climate_readout]") {
    ClimateReadings r{};
    r[static_cast<int>(ClimateReading::kRefrigLowKpa)] = 310.0;
    const auto lines = ClimateReadoutLines(r);
    // Four section headers plus one line per reading.
    CHECK(lines.size() == static_cast<std::size_t>(kNumClimateReadings + 4));
    CHECK(lines.front() == "SENSORS");
    int headers = 0;
    for (const auto& l : lines) headers += (l[0] != ' ') ? 1 : 0;
    CHECK(headers == 4);
    bool saw = false;
    for (const auto& l : lines) saw = saw || l == "  Refrigerant low side: 310 kPa";
    CHECK(saw);
}

TEST_CASE("Climate CSV columns are distinct and map back to their reading",
          "[climate_readout]") {
    std::set<std::string> seen;
    for (int i = 0; i < kNumClimateReadings; ++i) {
        const std::string col = std::string("climate_") + kClimateReadingInfo[i].column;
        CHECK(seen.insert(col).second);
        CHECK(ClimateReadingForColumn(col) == static_cast<ClimateReading>(i));
    }
    CHECK_FALSE(ClimateReadingForColumn("refrig_low_kpa").has_value());   // needs the prefix
    CHECK_FALSE(ClimateReadingForColumn("climate_nope").has_value());
}

TEST_CASE("Climate readout panel draws on a null driver", "[climate_readout]") {
    DrawClimateReadout(nullptr, ClimateReadings{});   // null device is a no-op
    irr::IrrlichtDevice* dev = irr::createDevice(irr::video::EDT_NULL,
                                                 irr::core::dimension2d<irr::u32>(640, 480));
    REQUIRE(dev != nullptr);
    ClimateReadings r{};
    r[0] = 18.0;
    dev->getVideoDriver()->beginScene();
    DrawClimateReadout(dev, r);
    dev->getVideoDriver()->endScene();
    dev->drop();
}
