// DriveReadout and ReadoutPanel — the shared formatters, the APM's encoded
// readings, the readout's sections, its CSV columns, the J page cycle and the
// panel on a null driver.

#include <catch2/catch_test_macros.hpp>

#include "DriveReadout.h"
#include "ClimateReadout.h"

#include <irrlicht.h>

#include <set>
#include <string>

using namespace ev1sim;

TEST_CASE("Readout formatters show each unit", "[drive_readout]") {
    namespace f = readout_format;
    CHECK(f::RadPerSec(42.04) == "42.0 rad/s");
    CHECK(f::KphFromMps(10.0) == "36.0 km/h");
    CHECK(f::Kohm(606.0) == "606 kohm");
    CHECK(f::Count(3.0) == "3");
    CHECK(f::Volts(312.04) == "312.0 V");
    CHECK(f::OnOff(1.0) == "ON");
    CHECK(f::OnOff(0.0) == "OFF");
}

TEST_CASE("APM run mode, setpoint and bitmasks decode to names", "[drive_readout]") {
    CHECK(FormatApmRunMode(0.0) == "OFF");
    CHECK(FormatApmRunMode(2.0) == "RUN");
    CHECK(FormatApmRunMode(4.0) == "LOCK");
    CHECK(FormatApmRunMode(9.0) == "MODE 9");
    CHECK(FormatApmSetpoint(1.0) == "HIGH (14.5 V)");
    CHECK(FormatApmSetpoint(0.0) == "LOW");
    CHECK(FormatApmSubsystems(0.0) == "none");
    CHECK(FormatApmSubsystems(0x05) == "REGULATING, HV PRESENT");
    CHECK(FormatApmSubsystems(0x07) == "REGULATING, LOAD SHED, HV PRESENT");
    CHECK(FormatApmSubsystems(0x10) == "BIT 4");
    CHECK(FormatApmLampRequests(0x03) == "SERVICE SOON, SERVICE NOW");
    CHECK(FormatApmLampRequests(0x04) == "TEMP");
    CHECK(FormatApmLampRequests(0x84) == "TEMP, BIT 7");
}

TEST_CASE("Drive readings format with their labels", "[drive_readout]") {
    CHECK(FormatDriveReading(DriveReading::kWheelSpeedLf, 42.0) ==
          "Left front wheel: 42.0 rad/s");
    CHECK(FormatDriveReading(DriveReading::kReferenceSpeed, 25.0) == "Reference speed: 90.0 km/h");
    CHECK(FormatDriveReading(DriveReading::kIsolationKohm, 606.0) ==
          "Isolation to chassis: 606 kohm");
    CHECK(FormatDriveReading(DriveReading::kApmRunMode, 3.0) == "Run mode: CHARGE");
    CHECK(FormatDriveReading(DriveReading::kApmModuleTemp, -5.25) == "Module temp: -5.2 C");
    CHECK(FormatDriveReading(DriveReading::kIpcActiveCodes, std::nullopt) ==
          "Cluster active trouble codes: ---");
}

TEST_CASE("Drive readout lists every reading under its section", "[drive_readout]") {
    DriveReadings r{};
    r[static_cast<int>(DriveReading::kPackVolts)] = 312.0;
    const auto lines = DriveReadoutLines(r);
    CHECK(lines.size() == static_cast<std::size_t>(kNumDriveReadings + 4));   // four sections
    CHECK(lines.front() == "BRAKE CONTROLLER");
    std::set<std::string> headers;
    for (const auto& l : lines)
        if (l[0] != ' ') headers.insert(l);
    CHECK(headers == std::set<std::string>{"BRAKE CONTROLLER", "HIGH VOLTAGE",
                                           "ACCESSORY POWER MODULE", "OTHER MODULES"});
    bool saw = false;
    for (const auto& l : lines) saw = saw || l == "  Pack at the contactors: 312.0 V";
    CHECK(saw);
}

TEST_CASE("Drive CSV columns are distinct and map back to their reading", "[drive_readout]") {
    std::set<std::string> seen;
    for (int i = 0; i < kNumDriveReadings; ++i) {
        const std::string col = std::string("drive_") + kDriveReadingInfo[i].column;
        CHECK(seen.insert(col).second);
        CHECK(DriveReadingForColumn(col) == static_cast<DriveReading>(i));
        // No drive column is also a climate column.
        CHECK_FALSE(ClimateReadingForColumn(col).has_value());
    }
    CHECK_FALSE(DriveReadingForColumn("pack_v").has_value());   // needs the prefix
    CHECK_FALSE(DriveReadingForColumn("drive_nope").has_value());
    CHECK_FALSE(DriveReadingForColumn("climate_pack_temp_c").has_value());
}

TEST_CASE("J steps the readout pages: climate, drive and power, none", "[drive_readout]") {
    CHECK(NextReadoutPage(ReadoutPage::kNone) == ReadoutPage::kClimate);
    CHECK(NextReadoutPage(ReadoutPage::kClimate) == ReadoutPage::kDrive);
    CHECK(NextReadoutPage(ReadoutPage::kDrive) == ReadoutPage::kNone);
}

TEST_CASE("Drive readout panel draws on a null driver", "[drive_readout]") {
    DrawDriveReadout(nullptr, DriveReadings{});   // null device is a no-op
    irr::IrrlichtDevice* dev = irr::createDevice(irr::video::EDT_NULL,
                                                 irr::core::dimension2d<irr::u32>(640, 480));
    REQUIRE(dev != nullptr);
    DriveReadings r{};
    r[0] = 12.0;
    dev->getVideoDriver()->beginScene();
    DrawDriveReadout(dev, r);
    dev->getVideoDriver()->endScene();
    dev->drop();
}
