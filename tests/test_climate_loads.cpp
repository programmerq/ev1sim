// ClimateLoads — column names and the status-panel rows for the climate
// system's loads.

#include <catch2/catch_test_macros.hpp>

#include "ClimateLoads.h"

#include <set>
#include <string>

using namespace ev1sim;

TEST_CASE("Climate load columns are distinct and map back to their load", "[climate_loads]") {
    std::set<std::string> seen;
    for (int i = 0; i < kNumClimateLoads; ++i) {
        const auto load = static_cast<ClimateLoad>(i);
        const std::string col = ClimateLoadColumn(load);
        CHECK_FALSE(col.empty());
        CHECK(seen.insert(col).second);
        CHECK(ClimateLoadForColumn(col) == load);
    }
    CHECK_FALSE(ClimateLoadForColumn("hvac_blower_level").has_value());
    CHECK_FALSE(ClimateLoadForColumn("").has_value());
}

TEST_CASE("A climate load's row shows ON, OFF, or no data yet", "[climate_loads]") {
    CHECK(FormatClimateLoadLabel(L"Compressor", true) == L"Compressor: ON");
    CHECK(FormatClimateLoadLabel(L"Compressor", false) == L"Compressor: OFF");
    CHECK(FormatClimateLoadLabel(L"Radiator fan", std::nullopt) == L"Radiator fan: ---");
}

TEST_CASE("The rear defog row counts its two grids", "[climate_loads]") {
    CHECK(FormatRearDefogGridsLabel(true, true) == L"Rear defog grids: BOTH");
    CHECK(FormatRearDefogGridsLabel(true, false) == L"Rear defog grids: ONE");
    CHECK(FormatRearDefogGridsLabel(std::nullopt, true) == L"Rear defog grids: ONE");
    CHECK(FormatRearDefogGridsLabel(false, false) == L"Rear defog grids: OFF");
    CHECK(FormatRearDefogGridsLabel(std::nullopt, std::nullopt) == L"Rear defog grids: ---");
}
