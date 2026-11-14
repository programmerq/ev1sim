#pragma once

#include <optional>
#include <string>

// ---------------------------------------------------------------------------
// ClimateLoads — the climate system's power loads as electricsim's solver sees
// them: is current flowing through each one.
//
// Each is a conductor cell the solver publishes from the circuit, true only
// while both sides of the load are live. These are what is actually running,
// as opposed to what a module asked for (the HTCM's blower level and defrost
// command, 4082 / 4083).
//
//   kCompressor          heat-pump compressor motor, across the APM's 902 / 903
//                        phase legs        (CHASSIS_APM_CMC_MOTOR_ENERGISED)
//   kWindshieldHeat      heated-windshield grid, 365A hot / 364A return at the
//                        APM               (CHASSIS_APM_HWS_GRID_ENERGISED)
//   kRearDefogPrimary    rear defog, primary grid, circuit 292 at TJB J2.G
//                                          (CHASSIS_TJB_PRI_DEFOG_LO)
//   kRearDefogSecondary  rear defog, secondary grid, circuit 681 at TJB J2.H
//                                          (CHASSIS_TJB_SEC_DEFOG_LO)
//   kRadiatorFan         radiator fan feed from the HTCM
//                                          (CHASSIS_HTCM_RADIATOR_FAN_FEED)
// ---------------------------------------------------------------------------

namespace ev1sim {

enum class ClimateLoad : int {
    kCompressor = 0,
    kWindshieldHeat,
    kRearDefogPrimary,
    kRearDefogSecondary,
    kRadiatorFan,
};
inline constexpr int kNumClimateLoads = 5;

/// Short scenario-CSV column name for a load ("compressor_energised", ...).
const char* ClimateLoadColumn(ClimateLoad load);

/// The load a scenario-CSV column names, or nullopt for any other column.
std::optional<ClimateLoad> ClimateLoadForColumn(const std::string& column);

/// Status-panel row for a single load: "<name>: ON", "<name>: OFF", or
/// "<name>: ---" before the solver has written it. Pure.
std::wstring FormatClimateLoadLabel(const wchar_t* name, std::optional<bool> on);

/// Status-panel row for the rear defog, which has two grids: "Rear defog
/// grids: BOTH", "ONE", "OFF", or "---" until either is written. Pure.
std::wstring FormatRearDefogGridsLabel(std::optional<bool> primary,
                                       std::optional<bool> secondary);

}  // namespace ev1sim
