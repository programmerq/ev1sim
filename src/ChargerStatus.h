#pragma once

#include <cstdint>
#include <optional>
#include <string>

// ---------------------------------------------------------------------------
// ChargerStatus — what the off-board charger reports while the paddle is in
// the car's inlet, decoded from the eight CHASSIS_CHARGER_* cells electricsim's
// charger publishes (the state it reports to the BPM over the Class 2 data
// line, plus the coupler's temperature sensor).
//
// Each field is nullopt until the charger has written its cell. Encodings, as
// electricsim's topology declares them:
//   CHASSIS_CHARGER_SM_STATE           uint32 enum, ChargerState below
//   CHASSIS_CHARGER_ENABLED            bit, 1 = delivering (CHARGING or TAPER)
//   CHASSIS_CHARGER_ACTUAL_CURRENT_DA  int32 in a uint32, deciamps, charging
//                                      negative
//   CHASSIS_CHARGER_ACTUAL_VOLTAGE_DV  uint32 decivolts
//   CHASSIS_CHARGER_TEMP_C             int32 in a uint32, deg C
//   CHASSIS_CHARGER_COUPLER_TEMP_C     int32 in a uint32, deg C
//   CHASSIS_CHARGER_AC_INPUT_MV        uint32 millivolts
//   CHASSIS_CHARGER_FAULT_BITS         uint32 bitmap, ChargerFault below
// ---------------------------------------------------------------------------

namespace ev1sim {

// electricsim ev1/charger/charger_logic.hpp ChargerSmState.
enum class ChargerState : std::uint32_t {
    kWaitForCoupler = 0,
    kIdle = 1,
    kCharging = 2,
    kTaper = 3,
    kComplete = 4,
    kFault = 5,
};

// electricsim ev1/charger/charger_logic.hpp ChargerFaultBits. Bit 2 (coupler
// loss) is reserved there and never raised.
namespace ChargerFault {
inline constexpr std::uint32_t kOvertemp = 1u << 0;
inline constexpr std::uint32_t kOvercurrent = 1u << 1;
inline constexpr std::uint32_t kCommLoss = 1u << 3;
inline constexpr std::uint32_t kAcInputBrownout = 1u << 4;
inline constexpr std::uint32_t kCouplerOvertemp = 1u << 5;
inline constexpr std::uint32_t kInternalAbort = 1u << 6;
}  // namespace ChargerFault

struct ChargerStatus {
    std::optional<std::uint32_t> state;   // ChargerState value, raw so an unknown one shows
    std::optional<bool> delivering;       // CHASSIS_CHARGER_ENABLED
    std::optional<double> current_a;      // charging negative
    std::optional<double> voltage_v;
    std::optional<int> charger_temp_c;
    std::optional<int> coupler_temp_c;
    std::optional<double> ac_input_v;
    std::optional<std::uint32_t> fault_bits;
};

/// A signed cell carried as the uint32 bit pattern of an int32. Pure.
inline std::int32_t SignedFromCell(std::uint32_t bits) {
    return static_cast<std::int32_t>(bits);
}

/// "WAIT FOR PADDLE", "IDLE", "CHARGING", "TAPER", "COMPLETE", "FAULT", or
/// "STATE <n>" for a value this build does not know. Pure.
std::string ChargerStateName(std::uint32_t state);

/// The faults set in `bits`, comma-separated ("OVERTEMP, COMM LOSS"), or
/// "none". Unknown bits show as "BIT <n>". Pure.
std::string ChargerFaultNames(std::uint32_t bits);

// Status-panel rows. Each shows "---" for a value not yet written. Pure.
/// "Charger: CHARGING"
std::wstring FormatChargerStateLabel(const ChargerStatus& s);
/// "Charge: 12.5 A at 352.0 V" (current shown as a positive charge rate)
std::wstring FormatChargerOutputLabel(const ChargerStatus& s);
/// "Charger temp 41 C, paddle 30 C"
std::wstring FormatChargerTempsLabel(const ChargerStatus& s);
/// "Mains: 240 V"
std::wstring FormatChargerMainsLabel(const ChargerStatus& s);
/// "Charger faults: none"
std::wstring FormatChargerFaultsLabel(const ChargerStatus& s);

}  // namespace ev1sim
