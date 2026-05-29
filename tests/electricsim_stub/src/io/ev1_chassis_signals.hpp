#pragma once
// CI STUB — minimal stand-in for electricsim's src/io/ev1_chassis_signals.hpp.
// Canonical chassis-bus IDs, pinned to ev1sim's own values so the connector's
// drift-guard static_asserts compile.  NOTE: this is self-referential — it
// proves the guarded region COMPILES (the horn-bug class), not that ev1sim is
// in sync with the real electricsim (that needs the App-token integrated build).
// Regenerate from the EV1SIM_CHASSIS_ID_MATCHES pairs if you add a guarded ID.
namespace electricsim { namespace io {
constexpr std::uint32_t kSigChassisMotorRpm = 4070U;
constexpr std::uint32_t kSigChassisMotorTorqueNm = 4071U;
constexpr std::uint32_t kSigChassisSimTimeNs = 4075U;
constexpr std::uint32_t kSigChassisThrottleCmdQ8 = 4073U;
constexpr std::uint32_t kSigChassisBrakeMasterPressureKpa = 4074U;
constexpr std::uint32_t kSigChassisWiperMotorCommand = 4080U;
constexpr std::uint32_t kSigChassisWasherPumpCommand = 4081U;
constexpr std::uint32_t kSigChassisHvacBlowerLevel = 4082U;
constexpr std::uint32_t kSigChassisDefrostGridActive = 4083U;
constexpr std::uint32_t kSigChassisDoorLockCmdDriver = 4084U;
constexpr std::uint32_t kSigChassisDoorLockCmdPassenger = 4085U;
constexpr std::uint32_t kSigChassisPowerWindowMotorDriver = 4086U;
constexpr std::uint32_t kSigChassisPowerWindowMotorPassenger = 4087U;
constexpr std::uint32_t kSigChassisRsaShiftBlocked = 4088U;
constexpr std::uint32_t kSigChassisAmbientTempC = 4090U;
constexpr std::uint32_t kSigChassisAmbientHumidityPct = 4091U;
constexpr std::uint32_t kSigChassisIpcSeatbeltTelltaleDriver = 4130U;
constexpr std::uint32_t kSigChassisIpcSeatbeltTelltalePassenger = 4131U;
constexpr std::uint32_t kSigChassisIpcTripDistanceM = 4132U;
constexpr std::uint32_t kSigChassisIpcBrakeTelltale = 4134U;
constexpr std::uint32_t kSigChassisIpcParkBrakeTelltale = 4135U;
constexpr std::uint32_t kSigChassisIpcAntilockTelltale = 4136U;
constexpr std::uint32_t kSigChassisIpcLowTracTelltale = 4137U;
constexpr std::uint32_t kSigChassisIpcAirBagTelltale = 4138U;
constexpr std::uint32_t kSigChassisBpmPackVoltageMv = 4139U;
constexpr std::uint32_t kSigChassisIpcServiceNowTelltale = 4140U;
constexpr std::uint32_t kSigChassisIpcCheckMessagesTelltale = 4141U;
constexpr std::uint32_t kSigChassisIpcTempTelltale = 4142U;
constexpr std::uint32_t kSigChassisIpcBatteryLifeTelltale = 4143U;
constexpr std::uint32_t kSigChassisIpcReducedPerfTelltale = 4144U;
constexpr std::uint32_t kSigChassisIpcCheckTirePressTelltale = 4145U;
constexpr std::uint32_t kSigChassisBtcmIsoCloseFL = 4147U;
constexpr std::uint32_t kSigChassisBtcmIsoCloseFR = 4148U;
constexpr std::uint32_t kSigChassisBtcmDumpOpenFL = 4149U;
constexpr std::uint32_t kSigChassisBtcmDumpOpenFR = 4150U;
constexpr std::uint32_t kSigChassisBtcmEmbMotorCmdLR = 4151U;
constexpr std::uint32_t kSigChassisBtcmEmbMotorCmdRR = 4152U;
constexpr std::uint32_t kSigChassisBtcmCylPressureFL_kPa = 4153U;
constexpr std::uint32_t kSigChassisBtcmCylPressureFR_kPa = 4154U;
constexpr std::uint32_t kSigTurnHazSw_RightTurnOut = 4043U;
constexpr std::uint32_t kSigTurnHazSw_LeftTurnOut = 4044U;
constexpr std::uint32_t kSigTurnHazSw_HazardOut = 4045U;
constexpr std::uint32_t kSigTurnHazSw_HornOut = 4046U;
constexpr std::uint32_t kSigWiperSw_DelayOut = 4054U;
constexpr std::uint32_t kSigWiperSw_RequestOut = 4055U;
constexpr std::uint32_t kSigWiperSw_HiOut = 4056U;
constexpr std::uint32_t kSigWiperSw_WasherSwitchOut = 4057U;
constexpr std::uint32_t kSigChassisSpeedMps = 4100U;
}}
