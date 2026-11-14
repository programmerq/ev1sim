#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// ClimateReadout — what the climate controller (HTCM) senses and commands, and
// what the APM's compressor and heated-windshield drives are doing, as
// electricsim publishes them for scan tools and visualisers.
//
// Each reading is one CHASSIS_HTCM_* / CHASSIS_APM_* cell, decoded to a
// double in the unit kClimateReadingInfo gives (temperatures from int16 Q8
// deg C, pressures in kPa, the windshield rail from mV to V, on/off as 1/0).
// nullopt until electricsim has written the cell. WireTruthChassis::
// climate_readings() holds the cell for each reading.
// ---------------------------------------------------------------------------

namespace irr { class IrrlichtDevice; }

namespace ev1sim {

enum class ClimateReading : int {
    // HTCM sensors
    kOutsideAirTemp = 0,
    kInletTemp,
    kPackTemp,
    kPimTemp,
    kCoolantTemp,
    kCoolantPumpTemp,
    kDriveMotorCoolantTemp,
    kCoolantLevelLow,
    kRefrigLowKpa,
    kRefrigHighKpa,
    kRefrigTemp,
    kRefrigDischargeTemp,
    // HTCM outputs
    kModeDoor,
    kOutsideAirValve,
    kReversingValve,
    kThreeWayCoolantValve,
    kCoolantRecoveryValve,
    kHeaterValveFeed,
    kCoolantPumpFeed,
    // APM compressor drive
    kCompressorDutyPct,
    kCompressorFreqHz,
    kCompressorDriveEnabled,
    kCompressorInterlockClosed,
    kCompressorInverterFault,
    // APM heated-windshield drive
    kWindshieldHeatCommanded,
    kWindshieldHeatDriving,
    kWindshieldRailV,
};
inline constexpr int kNumClimateReadings = 27;

enum class ClimateUnit { kDegC, kKpa, kVolts, kPercent, kHz, kOnOff, kPosition };

struct ClimateReadingInfo {
    const char* column;  // scenario-CSV column, prefixed "climate_"
    const char* label;   // readout label
    ClimateUnit unit;
    const char* group;   // readout section the reading is listed under
};

/// Column, label, unit and section per reading, indexed by ClimateReading.
extern const std::array<ClimateReadingInfo, kNumClimateReadings> kClimateReadingInfo;

using ClimateReadings = std::array<std::optional<double>, kNumClimateReadings>;

/// int16 Q8 deg C carried as a uint16 bit pattern, to deg C. Pure.
double DegCFromQ8Cell(unsigned bits);

/// One reading as readout text: "Refrigerant high: 1520 kPa", "---" for the
/// value before it is written. Pure.
std::string FormatClimateReading(ClimateReading r, std::optional<double> value);

/// The whole readout: a header line per section, then one line per reading.
/// Pure.
std::vector<std::string> ClimateReadoutLines(const ClimateReadings& readings);

/// The reading a scenario-CSV column names ("climate_refrig_high_kpa"), or
/// nullopt for any other column. Pure.
std::optional<ClimateReading> ClimateReadingForColumn(const std::string& column);

/// Draw the readout as a panel on the right of the screen, below the
/// snapshot overlay (toggled with J). Call between BeginScene/EndScene.
/// Null-safe.
void DrawClimateReadout(irr::IrrlichtDevice* device, const ClimateReadings& readings);

}  // namespace ev1sim
