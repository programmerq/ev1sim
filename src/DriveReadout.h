#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "ReadoutPanel.h"

// ---------------------------------------------------------------------------
// DriveReadout — what the brake controller (BTCM), the high-voltage side (AD
// and PIM), the accessory power module (APM) and two other modules report,
// as electricsim publishes them for scan tools and visualisers.
//
// Each reading is one CHASSIS_* cell, decoded to a double in the unit its
// formatter shows: wheel speeds rad/s, the reference speed m/s (shown as
// km/h), voltages V (from mV), the APM temperature deg C (from signed
// hundredths), isolation kohm, on/off and the APM setpoint choice as 1/0,
// run mode and the two APM bitmasks as their raw value. nullopt until
// electricsim has written the cell. WireTruthChassis::drive_readings() holds
// the cell for each reading; the table, lines and panel are ReadoutPanel.h's.
// ---------------------------------------------------------------------------

namespace ev1sim {

enum class DriveReading : int {
    // BTCM
    kWheelSpeedLf = 0,
    kWheelSpeedRf,
    kWheelSpeedLr,
    kWheelSpeedRr,
    kReferenceSpeed,
    kAntilockLamp,
    kBrakeLamp,
    kLowTractionLamp,
    kParkBrakeLamp,
    // AD and PIM
    kPackVolts,
    kInverterCapVolts,
    kIsolationKohm,
    kTractionInverterDelivering,
    // APM
    kApmKeyOn,
    kApmRunMode,
    kApmModuleTemp,
    kApm12vInBand,
    kApmSetpointHigh,
    kApmSetpointStale,
    kApmSubsystems,
    kApmLampRequests,
    kApmActiveFaults,
    kApmWindshieldFeedbackVolts,
    kApmLoadShedCount,
    // Other modules
    kHtcmWakeUp,
    kIpcActiveCodes,
};
inline constexpr int kNumDriveReadings = 26;

/// Column, label, formatter and section per reading, indexed by DriveReading.
extern const std::array<ReadoutRow, kNumDriveReadings> kDriveReadingInfo;

using DriveReadings = std::array<std::optional<double>, kNumDriveReadings>;

// APM scan encodings, as electricsim's ev1/apm/apm_scan.h declares them.
namespace apm_scan {
inline constexpr unsigned kSubsysRegulating = 0x01u;
inline constexpr unsigned kSubsysLoadShedActive = 0x02u;
inline constexpr unsigned kSubsysHvPresent = 0x04u;
inline constexpr unsigned kLampServiceSoon = 0x01u;
inline constexpr unsigned kLampServiceNow = 0x02u;
inline constexpr unsigned kLampTemp = 0x04u;
}  // namespace apm_scan

// Formatters for the APM's encoded readings. Pure.
/// "OFF", "ACCESSORY", "RUN", "CHARGE", "LOCK", or "MODE <n>".
std::string FormatApmRunMode(double v);
/// "HIGH (14.5 V)" for 1, "LOW" for 0.
std::string FormatApmSetpoint(double v);
/// "REGULATING, LOAD SHED, HV PRESENT", "BIT <n>" for unknown bits, or "none".
std::string FormatApmSubsystems(double v);
/// "SERVICE SOON, SERVICE NOW, TEMP", "BIT <n>" for unknown bits, or "none".
std::string FormatApmLampRequests(double v);

/// One reading as readout text: "Left front wheel: 42.0 rad/s", "---" before
/// it is written. Pure.
std::string FormatDriveReading(DriveReading r, std::optional<double> value);

/// The whole readout: a header line per section, then one line per reading.
/// Pure.
std::vector<std::string> DriveReadoutLines(const DriveReadings& readings);

/// The reading a scenario-CSV column names ("drive_pack_v"), or nullopt for
/// any other column. Pure.
std::optional<DriveReading> DriveReadingForColumn(const std::string& column);

/// Draw the readout as a panel on the right of the screen (the second page J
/// shows). Call between BeginScene/EndScene. Null-safe.
void DrawDriveReadout(irr::IrrlichtDevice* device, const DriveReadings& readings);

}  // namespace ev1sim
