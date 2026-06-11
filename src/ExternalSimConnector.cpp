#include "ExternalSimConnector.h"

#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

#if EV1SIM_HAVE_EXTERNAL_SIM
#  include "protocol.hpp"
#  include "shm_transport.hpp"
#  include "ev1_chassis_signals.hpp"  // canonical chassis IDs — drift guard below
#  include <chrono>
#endif

namespace {

// ---------------------------------------------------------------------------
// Signal ID layout
//
// Bulb feed-line IDs (4000..4018) mirror the external electrical sim's LightIdx
// enum so both sides agree on which physical bulb a given ID drives.  The
// first 17 slots match the electric sim's catalog 1:1; the final two slots
// (LRTL/RRTL) model the dual-filament tail-lamp elements separately — the
// electric sim doesn't currently publish those and simply leaves them unused.
//
//   4000  BACKUP_LEFT                -> LBL
//   4001  BACKUP_RIGHT               -> RBL
//   4002  HEADLAMP_HI_LEFT           -> LHBH
//   4003  HEADLAMP_HI_RIGHT          -> RHBH
//   4004  HEADLAMP_LO_LEFT           -> LLBH
//   4005  HEADLAMP_LO_RIGHT          -> RLBH
//   4006  SIDE_MARKER_REAR_LEFT      -> LRSM
//   4007  SIDE_MARKER_REAR_RIGHT     -> RRSM
//   4008  SIGNAL_FRONT_MARKER_LEFT   -> LFML
//   4009  SIGNAL_FRONT_MARKER_RIGHT  -> RFML
//   4010  SIGNAL_FRONT_TURN_LEFT     -> LFTS
//   4011  SIGNAL_FRONT_TURN_RIGHT    -> RFTS
//   4012  SIGNAL_REAR_LEFT           -> LRTS  (rear turn signal)
//   4013  SIGNAL_REAR_RIGHT          -> RRTS
//   4014  STOPLAMP_LEFT              -> LRSL
//   4015  CHMSL                      -> CHMSL
//   4016  STOPLAMP_RIGHT             -> RRSL
//   4017  (ev1sim-only)              -> LRTL  (left rear tail filament)
//   4018  (ev1sim-only)              -> RRTL
//
//   4020  horn_low_drive_line                        (input to ev1sim)
//   4021  horn_high_drive_line                       (input to ev1sim)
//   4030..4033  panel ajar switches (HOOD/TRUNK/DL/DR)    (output from ev1sim)
//
// Vehicle dynamics signals (all float32 IEEE 754 LE, output from ev1sim):
//   4100  vehicle.dynamics.speed_mps           forward speed (m/s)
//   4101  vehicle.dynamics.accel_long          longitudinal accel (m/s^2, chassis frame)
//   4102  vehicle.dynamics.accel_lat           lateral accel (m/s^2, chassis frame)
//   4103  vehicle.dynamics.yaw_rate            yaw rate (rad/s)
//   4104  vehicle.dynamics.applied_throttle      0..1  (commanded)
//   4105  vehicle.dynamics.applied_front_brake   0..1  (commanded)
//   4106  vehicle.dynamics.applied_rear_brake    0..1  (commanded)
//   4107  vehicle.dynamics.front_brake_pressure  0..1  (actual, after hydraulic lag)
//   4108  vehicle.dynamics.rear_brake_position   0..1  (actual, after rate-limit)
//   4109  vehicle.dynamics.steering_torque       Nm    (front-axle Mz sum, FFB feed)
//   4110  vehicle.dynamics.wheel_omega_fl        rad/s (front-left)
//   4111  vehicle.dynamics.wheel_omega_fr      rad/s (front-right)
//   4112  vehicle.dynamics.wheel_omega_rl      rad/s (rear-left)
//   4113  vehicle.dynamics.wheel_omega_rr      rad/s (rear-right)
//   4120  vehicle.dynamics.slip_ratio_fl       0=free rolling, +1=locked, -1=spinning
//   4121  vehicle.dynamics.slip_ratio_fr
//   4122  vehicle.dynamics.slip_ratio_rl
//   4123  vehicle.dynamics.slip_ratio_rr
// ---------------------------------------------------------------------------
constexpr std::uint32_t kBulbCmdBase    = 4000;
constexpr std::uint32_t kHornLowCmd     = 4020;
constexpr std::uint32_t kHornHighCmd    = 4021;
constexpr std::uint32_t kPanelBase      = 4030;

// Combination switch outputs (ev1sim → electricsim, chassis segment).
// 6-way blue connector 12084699; 3 output pins meaningfully published.
//   4040  combination_switch.low_beam_out         (pin C, YEL 525B)
//   4041  combination_switch.flash_to_pass_out    (pin B, PPL 524B)
//   4042  combination_switch.park_headlamp_out    (pin F, LTBLU 74)
constexpr std::uint32_t kCombSwLowBeamOutId      = 4040;
constexpr std::uint32_t kCombSwFlashToPassOutId  = 4041;
constexpr std::uint32_t kCombSwParkHeadlampOutId = 4042;
constexpr int           kNumCombSw               = 3;

// Turn/hazard combination switch outputs (ev1sim → electricsim, chassis segment).
// Steering-column combination switch, connector 12092237 (DARK GRAY).
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigTurnHazSw_* = 4043-4046.  Horn is a single command (circuit 28); there is
// no hi/lo driver choice — LHJB drives both sounders (4020/4021) together.
//   4043  vehicle.body.turn_haz_switch.right_turn_out   (pin B, DK BLU/WHT 1415)
//   4044  vehicle.body.turn_haz_switch.left_turn_out    (pin C, LT BLU/WHT 1414)
//   4045  vehicle.body.turn_haz_switch.hazard_out       (derived; cavity D 640H)
//   4046  vehicle.body.turn_haz_switch.horn_out         (pin H, BLK 28)
constexpr std::uint32_t kTurnHazSwRightTurnOutId = 4043;
constexpr std::uint32_t kTurnHazSwLeftTurnOutId  = 4044;
constexpr std::uint32_t kTurnHazSwHazardOutId    = 4045;
constexpr std::uint32_t kTurnHazSwHornOutId      = 4046;
// (The endpoint-registry entries for these cavities live once in the
//  driver-input section as vehicle.driver.*_contact; these IDs are still used
//  by the chassis-bus publish block + the drift guard below.)

// Charge coupler presence (ev1sim → electricsim, chassis segment).
//   4060  vehicle.body.charge_coupler.present
//         True when the J1772/Avcon paddle is mated.  Stubbed false for now.
constexpr std::uint32_t kChargeCouplerPresentId = 4060;

// PRND selector lines (ev1sim → electricsim, chassis segment).
// Four PIM prnd_a/b/c/d cavities — physical wires from floor lever to PIM.
// Encoding (Gray-coded with parity, propulsion manual p. 343):
//   PARK    A=0 B=1 C=1 D=0
//   REVERSE A=0 B=0 C=1 D=1
//   NEUTRAL A=1 B=0 C=1 D=0
//   DRIVE   A=1 B=0 C=0 D=1
//   4050  vehicle.driver.prnd_selector_a
//   4051  vehicle.driver.prnd_selector_b
//   4052  vehicle.driver.prnd_selector_c
//   4053  vehicle.driver.prnd_selector_d  (even-parity bit)
constexpr std::uint32_t kPrndSelectorAId = 4050;
constexpr std::uint32_t kPrndSelectorBId = 4051;
constexpr std::uint32_t kPrndSelectorCId = 4052;
constexpr std::uint32_t kPrndSelectorDId = 4053;
constexpr int           kNumPrndSelector = 4;

// Wiper/washer switch outputs (ev1sim → electricsim, chassis segment).
// Steering-column wiper/washer switch, connector 12092254 (BLACK).
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigWiperSw_* = 4054-4057.  HI layers on the LOW "request" wire.
//   4054  vehicle.body.wiper_washer_switch.delay_out         (pin B, GRA 112)
//   4055  vehicle.body.wiper_washer_switch.request_out       (pin C, DK GRN 113)
//   4056  vehicle.body.wiper_washer_switch.hi_out            (pin D, PPL 92)
//   4057  vehicle.body.wiper_washer_switch.washer_switch_out (pin E, PNK 228C)
constexpr std::uint32_t kWiperSwDelayOutId        = 4054;
constexpr std::uint32_t kWiperSwRequestOutId      = 4055;
constexpr std::uint32_t kWiperSwHiOutId           = 4056;
constexpr std::uint32_t kWiperSwWasherSwitchOutId = 4057;
// (Registered once in the driver-input section as vehicle.driver.*_contact;
//  these IDs remain in use by the chassis-bus publish block + drift guard.)

constexpr std::uint32_t kDynamicsBase   = 4100;

// Driver input signal IDs on the main harness segment (electricsim_ev1_bus).
// Encoding per electricsim/src/io/ev1_driver_inputs.hpp.
constexpr std::uint32_t kSigDriverBrakePedalQ8    = 6900U;
constexpr std::uint32_t kSigDriverSteeringDegQ8   = 6901U;
constexpr std::uint32_t kSigDriverGearSelector    = 6902U;
constexpr std::uint32_t kSigDriverThrottleQ8      = 6903U;
// Brake switch (discrete bool, 0/1) — locked in lockstep with electricsim
// ev1_driver_inputs.hpp kSigDriverBrakeSwitch = 6904.
constexpr std::uint32_t kSigDriverBrakeSwitch     = 6904U;
// Driver seatbelt buckle — locked in lockstep with kSigDriverSeatbeltBuckled = 6964.
constexpr std::uint32_t kSigDriverSeatbeltBuckled = 6964U;
// Passenger seatbelt buckle — locked in lockstep with
// kSigDriverSeatbeltBuckledPassenger = 6965 (mirrors driver).
// TODO(consumer): IPC seatbelt-light telltale — deferred.
constexpr std::uint32_t kSigDriverSeatbeltBuckledPassenger = 6965U;
// Turn/hazard combination-switch CHASSIS cavities (chassis segment), locked in
// lockstep with electricsim kSigTurnHazSw_* = 4043-4046 (connector 12092237).
// LHJB consumes these directly (turn left/right + hazard + the single horn
// command on circuit 28).
constexpr std::uint32_t kSigTurnHazSw_RightTurnOut = 4043U;
constexpr std::uint32_t kSigTurnHazSw_LeftTurnOut  = 4044U;
constexpr std::uint32_t kSigTurnHazSw_HazardOut    = 4045U;
constexpr std::uint32_t kSigTurnHazSw_HornOut      = 4046U;  // single horn cmd (ckt 28)
// RSA per-digit keypad button signals (momentary 1-tick bool, 0=idle, 1=pressed).
// Locked in lockstep with electricsim kSigDriverRsaKeypadButton[1..5] = 6975-6979.
// (Slot 6970 was kSigDriverRsaKeypadCodeOk — now reserved, not published here.)
constexpr std::uint32_t kSigDriverRsaKeypadButton1 = 6975U;  // "1/2" button (tap=1)
constexpr std::uint32_t kSigDriverRsaKeypadButton2 = 6976U;  // "3/4" button (tap=3)
constexpr std::uint32_t kSigDriverRsaKeypadButton3 = 6977U;  // "5/6" button (tap=5)
constexpr std::uint32_t kSigDriverRsaKeypadButton4 = 6978U;  // "7/8" button (tap=7)
constexpr std::uint32_t kSigDriverRsaKeypadButton5 = 6979U;  // "9/0" button (tap=9)
// RSA mode button press — momentary 1-tick uint8 enum.
// 0=NONE, 1=OFF, 2=ACC, 3=RUN, 4=START.
// Locked in lockstep with electricsim kSigDriverRsaModeButton = 6971.
constexpr std::uint32_t kSigDriverRsaModeButton    = 6971U;
// IPC trip-reset (ID 6952), locked in lockstep with electricsim
// kSigDriverIpcTripResetButton = 6952.  (The cruise stalk formerly published
// pre-decoded pulses 6953-6957 here; it now publishes the raw chassis cavities
// kSigCruiseSw_* (4047-4049) below — the same edge model as wiper/turn-hazard.)
constexpr std::uint32_t kSigDriverIpcTripResetButton = 6952U;
// Wiper/washer switch CHASSIS cavities, locked in lockstep with electricsim
// kSigWiperSw_* = 4054-4057. ev1sim computes these raw contacts from the
// detent enum (driver_wiper_switch) and publishes them on the chassis segment;
// RHJB's WSW decoder turns them back into OFF/INT/LOW/HIGH (ESM p.511).
constexpr std::uint32_t kSigWiperSw_DelayOut          = 4054U;  // INT detent
constexpr std::uint32_t kSigWiperSw_RequestOut        = 4055U;  // any on-state
constexpr std::uint32_t kSigWiperSw_HiOut             = 4056U;  // HIGH detent
constexpr std::uint32_t kSigWiperSw_WasherSwitchOut   = 4057U;  // wash button
// Cruise-control switch CHASSIS cavities, locked in lockstep with electricsim
// kSigCruiseSw_{SetCoast,ResumeAccel,OnOff}Out = 4047/4048/4049 (circuits
// 84/87/397 -> PIM J1).  ev1sim opens/closes these raw contacts from the
// keyboard/UI cruise stalk; PIM's tap/hold decoder (pim_cruise_input) turns a
// brief SET/COAST close into SET and a sustained close into SPEED_DOWN
// (RESUME/ACCEL -> RESUME / SPEED_UP) and treats ON/OFF as the master arm
// (falling edge = CANCEL).  ev1sim does NOT pre-decode — held duration alone
// selects tap vs. hold downstream.  Replaced the old 6953-6957 pulse path.
constexpr std::uint32_t kSigCruiseSw_SetCoastOut    = 4047U;  // ckt 84,  PIM J1 pin 9
constexpr std::uint32_t kSigCruiseSw_ResumeAccelOut = 4048U;  // ckt 87,  PIM J1 pin 25
constexpr std::uint32_t kSigCruiseSw_OnOffOut       = 4049U;  // ckt 397, PIM J1 pin 10
constexpr int           kNumNewDriverInputs         = 1;  // ipc trip-reset (6952) only
// Power window switch signals (momentary bool, held while pressed).
// Locked in lockstep with electricsim kSigDriverPowerWindow* = 6980-6983.
// consumer = RSA (window-motor logic), future round.
constexpr std::uint32_t kSigDriverPowerWindowDriverUp      = 6980U;
constexpr std::uint32_t kSigDriverPowerWindowDriverDown    = 6981U;
constexpr std::uint32_t kSigDriverPowerWindowPassengerUp   = 6982U;
constexpr std::uint32_t kSigDriverPowerWindowPassengerDown = 6983U;
constexpr int           kNumPowerWindowInputs              = 4;

// RSA exterior pillar keypad signals (momentary uint8, 0=idle/1=tap/2=long).
// Locked in lockstep with electricsim kSigDriverRsaExteriorKeypad[1..5] = 6985-6989.
// Same Option A encoding as interior keypad (6975-6979).
// consumer = RSA exterior keypad logic, future round.
constexpr std::uint32_t kSigDriverRsaExteriorKeypad1 = 6985U;
constexpr std::uint32_t kSigDriverRsaExteriorKeypad2 = 6986U;
constexpr std::uint32_t kSigDriverRsaExteriorKeypad3 = 6987U;
constexpr std::uint32_t kSigDriverRsaExteriorKeypad4 = 6988U;
constexpr std::uint32_t kSigDriverRsaExteriorKeypad5 = 6989U;
// Door handle pull attempt signals (momentary bool 0=idle, 1=pulled this tick).
// Locked in lockstep with electricsim kSigDriverDoorHandleAttempt{Driver,Passenger}
// = 6990-6991.  consumer = RSA (decides if door unlocks).
constexpr std::uint32_t kSigDriverDoorHandleAttemptDriver    = 6990U;
constexpr std::uint32_t kSigDriverDoorHandleAttemptPassenger = 6991U;
constexpr int           kNumExteriorKeypadInputs             = 7;  // 5 buttons + 2 handles

// Number of driver-input endpoints on the main harness segment.
// 6900, 6901, 6902, 6903, 6904, 6964, 6965,
// 6971, 6975, 6976, 6977, 6978, 6979,
// 6952,
// 6980, 6981, 6982, 6983,
// 6985, 6986, 6987, 6988, 6989, 6990, 6991 = 25 total.
// (6970 is reserved; not registered as an endpoint.  Turn/hazard 6944/6948/6949,
//  wiper 6958/6959, and cruise 6953-6957 moved to chassis switch cavities.)
constexpr int kNumPassengerSeatbelt = 1;  // passenger seatbelt (6965)
constexpr int kNumDriverInputs = 12 + kNumNewDriverInputs + kNumPowerWindowInputs + kNumExteriorKeypadInputs + kNumPassengerSeatbelt;  // 12 base: turn/hazard (6944/6948/6949) + cruise (6953-6957) moved to chassis cavities

// Motor state signals on the chassis segment (ev1sim → electricsim, float32 LE).
//   4070  vehicle.dynamics.motor_rpm          motor shaft RPM
//   4071  vehicle.dynamics.motor_torque_nm    motor shaft torque (Nm, signed)
//   4072  vehicle.dynamics.motor_current_a    DC pack current (A, signed: + discharge)
constexpr std::uint32_t kSigMotorRpm       = 4070U;
constexpr std::uint32_t kSigMotorTorqueNm  = 4071U;
constexpr std::uint32_t kSigMotorCurrentA  = 4072U;
constexpr int           kNumMotorSignals   = 3;
// NOTE: 4072 (motor_current_a) is allocated ev1sim-side first — electricsim has
// no kSigChassisMotorCurrentA counterpart yet, so it is intentionally absent
// from the compile-time drift guard below (a static_assert for an ID
// electricsim doesn't define would break the integrated build).  Move it into
// the guard once electricsim adds the canonical constant.

// Sim-time master clock (ev1sim → electricsim, chassis segment, uint64 LE ns).
//   4075  vehicle.dynamics.sim_time_ns   monotonic Chrono sim-time, nanoseconds
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisSimTimeNs = 4075.  (NOT 4070 — that is motor RPM.)  Published
// every physics step so electricsim's SimClock can switch from wall-clock to
// sim-time-master mode (BTCM/PIM/IPC subscribe).  Must increase monotonically;
// electricsim ignores non-increasing samples.
constexpr std::uint32_t kSigSimTimeNs      = 4075U;
constexpr int           kNumSimTimeSignals = 1;

// Ambient environment sensors (ev1sim → electricsim, chassis segment, float32 LE).
//   4090  vehicle.environment.ambient_temp_c          ambient air temperature (°C)
//   4091  vehicle.environment.ambient_humidity_pct    ambient relative humidity (%)
// Publisher: ev1sim AmbientTempSensor (naive diurnal sinusoid; no live weather API).
// Consumers: HTCM (heat-pump), BPM (battery thermal) — wiring deferred to future round.
constexpr std::uint32_t kSigAmbientTempC       = 4090U;
constexpr std::uint32_t kSigAmbientHumidityPct = 4091U;
constexpr int           kNumAmbientSignals      = 2;

// Brake master cylinder pressure (ev1sim → electricsim, chassis segment).
//   4074  vehicle.brake.master_cylinder_pressure_kpa   float32 LE, kPa
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisBrakeMasterPressureKpa = 4074.  Computed by BrakePedal's
// two-stage curve from normalized pedal travel each tick.
constexpr std::uint32_t kSigBrakeMasterPressureKpa = 4074U;
constexpr int           kNumBrakeSignals           = 1;

// Throttle command (electricsim/PIM → ev1sim, chassis segment).
//   4073  vehicle.dynamics.throttle_cmd_q8   uint8 q8: 0=zero, 255=full
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisThrottleCmdQ8 = 4073.  Subscribed when running in
// "electronics" drive mode; ignored in "local" mode.
constexpr std::uint32_t kSigThrottleCmdQ8     = 4073U;
constexpr int           kNumThrottleCmdSignals = 1;

// Steering command (electricsim → ev1sim, chassis segment).
//   4076  vehicle.dynamics.steering_cmd   float32 LE, normalized -1..+1
//         (positive = left, matching Chrono / DriverCommand.steering).
// The symmetric counterpart of the throttle bus path: when running in
// "electronics" drive mode ev1sim subscribes and overrides the local steering
// when the value is fresh, falling back to the local input otherwise.  Allocated
// ev1sim-side first — there is no electricsim steering producer today; this is a
// closed-loop / physical-rig steering input, NOT the PSCM power-steering pump
// (that is the separate power_steering_pump_motor peripheral).  See the
// "pending electricsim adoption" note in the drift guard below.
constexpr std::uint32_t kSigSteeringCmd        = 4076U;
constexpr int           kNumSteeringCmdSignals = 1;

// Wiper motor command (electricsim/RHJB → ev1sim, chassis segment).
//   4080  vehicle.body.wiper_motor.command  uint8 enum: 0=OFF, 1=INT, 2=LOW, 3=HIGH
//   4081  vehicle.body.washer_pump.command  uint8 bool: 0=idle, 1=pump active
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisWiperMotorCommand = 4080, kSigChassisWasherPumpCommand = 4081.
constexpr std::uint32_t kSigWiperMotorCommand  = 4080U;
constexpr std::uint32_t kSigWasherPumpCommand  = 4081U;
constexpr int           kNumWiperSignals       = 2;

// HVAC blower level + defrost grid (electricsim/HTCM → ev1sim, chassis segment).
//   4082  vehicle.hvac.blower_level   uint8 enum: 0=OFF, 1=LOW, 2=MED, 3=HIGH
//   4083  vehicle.hvac.defrost_grid   uint8 bool: 0=inactive, 1=active
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisHvacBlowerLevel = 4082, kSigChassisDefrostGridActive = 4083.
// Encoding confirmed from electricsim/ev1/htcm/htcm_supervisor.h:
//   hvac_blower_level: 0=OFF, 1=LOW, 2=MED, 3=HIGH (clamped 0-3)
//   defrost_grid_active: bool
constexpr std::uint32_t kSigHvacBlowerLevel  = 4082U;
constexpr std::uint32_t kSigDefrostGridActive = 4083U;
constexpr int           kNumHvacSignals       = 2;

// IPC LCD seatbelt telltale outputs (electricsim/IPC → ev1sim, chassis segment).
//   4130  vehicle.ipc.seatbelt_telltale_driver    uint8 bool: 0=lamp off, 1=lamp on
//   4131  vehicle.ipc.seatbelt_telltale_passenger uint8 bool: 0=lamp off, 1=lamp on
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisIpcSeatbeltTelltaleDriver = 4130,
// kSigChassisIpcSeatbeltTelltalePassenger = 4131.
// Lit when the corresponding seat is unbuckled AND vehicle speed > ~8 km/h.
constexpr std::uint32_t kSigIpcSeatbeltTelltaleDriver    = 4130U;
constexpr std::uint32_t kSigIpcSeatbeltTelltalePassenger = 4131U;
constexpr int           kNumIpcTelltaleSignals           = 2;

// IPC trip distance (electricsim/IPC → ev1sim, chassis segment).
//   4132  vehicle.ipc.trip_distance_m   float32 LE, metres
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisIpcTripDistanceM = 4132.
// Published by IPC controller on change (epsilon ~0.5 m); resets to 0.0 on
// trip-reset button press.
constexpr std::uint32_t kSigIpcTripDistanceM    = 4132U;
constexpr int           kNumIpcTripDistSignals  = 1;

// IPC BTCM / airbag telltale outputs (electricsim/IPC → ev1sim, chassis segment).
//   4134  vehicle.ipc.brake_telltale         uint8 bool: 0=off, 1=on (DTC 42, BTCM brake_ind)
//   4135  vehicle.ipc.park_brake_telltale    uint8 bool: 0=off, 1=on (DTC 44, BTCM park_brake_ind)
//   4136  vehicle.ipc.antilock_telltale      uint8 bool: 0=off, 1=on (DTC 41, BTCM antilock_ind)
//   4137  vehicle.ipc.low_trac_telltale      uint8 bool: 0=off, 1=on (DTC 43, BTCM low_trac_ind)
//   4138  vehicle.ipc.air_bag_telltale       uint8 bool: 0=off, 1=on (DTC 40, set_airbag_input)
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisIpcBrakeTelltale=4134 .. kSigChassisIpcAirBagTelltale=4138.
constexpr std::uint32_t kSigIpcBrakeTelltale     = 4134U;
constexpr std::uint32_t kSigIpcParkBrakeTelltale = 4135U;
constexpr std::uint32_t kSigIpcAntilockTelltale  = 4136U;
constexpr std::uint32_t kSigIpcLowTracTelltale   = 4137U;
constexpr std::uint32_t kSigIpcAirBagTelltale    = 4138U;
constexpr int           kNumIpcBtcmTelltaleSignals = 5;

// IPC extra LCD telltale outputs (electricsim/IPC → ev1sim, chassis segment).
//   4140  vehicle.ipc.service_now_telltale      uint8 bool: 0=off, 1=on (DTCs 31/33/35 via HTCM/PCM/BPM)
//   4141  vehicle.ipc.check_messages_telltale   uint8 bool: 0=off, 1=on (aggregate: any comm-loss/req/BTCM)
//   4142  vehicle.ipc.temp_telltale             uint8 bool: 0=off, 1=on (DTCs 32/34/36 via HTCM/PCM/BPM)
//   4143  vehicle.ipc.battery_life_telltale     uint8 bool: 0=off, 1=on (DTC 38 via IPC_REQ_BATTERY_LIFE_FROM_BPM)
//   4144  vehicle.ipc.reduced_perf_telltale     uint8 bool: 0=off, 1=on (DTC 37 via IPC_REQ_REDUCED_PERF_FROM_PCM)
//   4145  vehicle.ipc.check_tire_press_telltale uint8 bool: 0=off, 1=on (DTC 39 via IPC_REQ_CHECK_TIRE_PRESS_FROM_RSA)
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisIpcServiceNowTelltale=4140 .. kSigChassisIpcCheckTirePressTelltale=4145.
constexpr std::uint32_t kSigIpcServiceNowTelltale     = 4140U;
constexpr std::uint32_t kSigIpcCheckMessagesTelltale  = 4141U;
constexpr std::uint32_t kSigIpcTempTelltale           = 4142U;
constexpr std::uint32_t kSigIpcBatteryLifeTelltale    = 4143U;
constexpr std::uint32_t kSigIpcReducedPerfTelltale    = 4144U;
constexpr std::uint32_t kSigIpcCheckTirePressTelltale = 4145U;
constexpr int           kNumIpcExtraTelltaleSignals   = 6;

// BPM pack voltage (electricsim/BPM → ev1sim, chassis segment).
//   4139  vehicle.bpm.pack_voltage_mv   uint32 LE, millivolts.
// BPM publishes on change (epsilon ~50 mV) after each supervisor tick while
// key-on.  Derived from pack_hi_v_q8 (primary pack-voltage sense lead, Q8 V).
// ev1sim subscribes to surface the live pack voltage in the floating-UI panel.
// 0 = sentinel "never received"; valid range 0..~360 000 mV (0..360 V).
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisBpmPackVoltageMv = 4139.
constexpr std::uint32_t kSigBpmPackVoltageMv     = 4139U;
constexpr int           kNumBpmPackVoltageSignals = 1;

// Door lock commands (electricsim/RSA → ev1sim, chassis segment).
//   4084  vehicle.body.door_lock_cmd.driver    uint8: 0=unlocked, 1=locked
//   4085  vehicle.body.door_lock_cmd.passenger uint8: 0=unlocked, 1=locked
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisDoorLockCmdDriver = 4084, kSigChassisDoorLockCmdPassenger = 4085.
constexpr std::uint32_t kSigDoorLockCmdDriver    = 4084U;
constexpr std::uint32_t kSigDoorLockCmdPassenger = 4085U;

// Power window motor commands (electricsim/RSA → ev1sim, chassis segment).
//   4086  vehicle.body.power_window_motor.driver    uint8: 0=stop, 1=up, 2=down
//   4087  vehicle.body.power_window_motor.passenger uint8: 0=stop, 1=up, 2=down
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisPowerWindowMotorDriver = 4086, kSigChassisPowerWindowMotorPassenger = 4087.
constexpr std::uint32_t kSigPowerWindowMotorDriver    = 4086U;
constexpr std::uint32_t kSigPowerWindowMotorPassenger = 4087U;
constexpr int           kNumDoorLockPwSignals         = 4;  // 4084+4085+4086+4087

// Door lock STATE feedback (ev1sim → electricsim, chassis segment).
// The resulting per-door latched state of ev1sim::DoorLocks, after the
// door_lock_motor (4092-4095) reaches end-of-travel or the RSA cmd (4084/4085)
// mirror is applied.  Closes the central-locking loop so RSA/IPC can confirm
// the actuated state.
// MOVED 4155-4157 → 4165-4167: the old block had been allocated ev1sim-side
// only and collided with the HV bus rail IDs in the canonical chassis-signal
// contract (rail consumers read 4155-4157 as millivolts / hv-present /
// milliamps). The trio is now adopted in the contract at 4165-4167 (contract
// 1.3.0) and guarded by the compile-time drift checks below.
//   4165  vehicle.body.door_lock_state.driver     uint8: 0=unlocked, 1=locked
//   4166  vehicle.body.door_lock_state.passenger  uint8: 0=unlocked, 1=locked
//   4167  vehicle.body.door_lock_state.trunk      uint8: 0=unlocked, 1=locked
constexpr std::uint32_t kSigDoorLockStateDriver    = 4165U;
constexpr std::uint32_t kSigDoorLockStatePassenger = 4166U;
constexpr std::uint32_t kSigDoorLockStateTrunk     = 4167U;
constexpr int           kNumDoorLockStateSignals   = 3;

// RSA shift-blocked cue (electricsim/RSA → ev1sim, chassis segment).
//   4088  vehicle.body.rsa.shift_blocked    uint8 bool: 0=no block, 1=blocked this tick
// Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp
// kSigChassisRsaShiftBlocked = 4088.
// Published as 1 when a P→non-P shift is refused (brake switch not pressed).
// Level signal: 1 while the PRND selector holds a non-PARK request without brake; 0 otherwise.
constexpr std::uint32_t kSigRsaShiftBlocked = 4088U;
constexpr int           kNumRsaShiftBlockedSignals = 1;

// Door-lock motor leg drives (electricsim/RHJB → ev1sim, chassis segment).
// RHJB's rhjb_door_lock behavioral model is a 4-output dual-H-bridge driving
// BOTH door-lock motors (LH driver + RH passenger) in lockstep.  Each motor has
// a LOCK leg and an UNLOCK leg; the leg whose drive is high decides the motor's
// direction, both-high/both-low → motor off.  ev1sim's door_lock_motor
// peripheral (ev1sim::DoorLockMotor ×2) consumes these and models the
// mechanical lock stroke.  Wire-level mapping (cavity → chassis-bus signal),
// authoritative for the electricsim router — see docs/peripherals.md:
//   4092  vehicle.body.door_lock_motor.lh_lock_drive    ckt 294A, RHJB J9.C5 → LH motor LOCK
//   4093  vehicle.body.door_lock_motor.lh_unlock_drive  ckt 295A, RHJB J9.C6 → LH motor UNLOCK
//   4094  vehicle.body.door_lock_motor.rh_lock_drive    ckt 294C, RHJB J3.A6 → RH motor LOCK
//   4095  vehicle.body.door_lock_motor.rh_unlock_drive  ckt 295C, RHJB J3.A7 → RH motor UNLOCK
// uint8/bool wire level: 1 = leg energised.  Allocated ev1sim-side first; see
// the "pending electricsim adoption" note in the drift-guard block below.
constexpr std::uint32_t kSigDoorLockMotorLhLockDrive   = 4092U;
constexpr std::uint32_t kSigDoorLockMotorLhUnlockDrive = 4093U;
constexpr std::uint32_t kSigDoorLockMotorRhLockDrive   = 4094U;
constexpr std::uint32_t kSigDoorLockMotorRhUnlockDrive = 4095U;
constexpr int           kNumDoorLockMotorSignals       = 4;

// Sounder / piezo drive (electricsim/LHJB → ev1sim, chassis segment).
//   4096  vehicle.body.sounder.piezo_drive   uint8 bool: 1 = piezo energised
// The LHJB turn/hazard flasher produces a piezo square-wave (the TURN/HAZ
// "click" in a real GM vehicle).  ev1sim's sounder peripheral
// (ev1sim::Sounder) consumes the boolean drive and exposes an
// audible-output signal for the 3D-sim audio contract; each rising edge is one
// click.  The piezo is a real LHJB-internal component the printed schematics
// are silent on (no first-class component_id) — see docs/peripherals.md.
constexpr std::uint32_t kSigSounderPiezoDrive = 4096U;
constexpr int           kNumSounderSignals    = 1;

// Power-steering pump motor (PSCM ↔ ev1sim, chassis segment).
// The PSCM is an HV inverter on batt-731 whose molex 3-phase outputs (molex.A/
// B/C) drive a steering pump motor; the motor body returns the HV interlock
// loop on molex.D/E.  ev1sim's power_steering_pump_motor peripheral
// (ev1sim::PowerSteeringPumpMotor) is the minimum-viable plant: it
// consumes a single commanded pump speed and closes the HV interlock loop.
// Per-phase BLDC commutation is future work.  Wire-level mapping:
//   4097  vehicle.steering.pump_motor.speed_cmd_q8     PSCM molex.A/B/C → pump  (uint8 q8: 0=stopped, 255=full)  [in]
//   4098  vehicle.steering.pump_motor.interlock_closed pump molex.D/E → PSCM    (uint8 bool: 1=loop closed)      [out]
constexpr std::uint32_t kSigPscmPumpSpeedCmdQ8      = 4097U;
constexpr std::uint32_t kSigPscmPumpInterlockClosed = 4098U;
constexpr int           kNumSteeringPumpSignals     = 2;  // 4097 (in) + 4098 (out)

// HVAC driver controls (ev1sim → electricsim/HTCM, chassis segment).
// The driver's climate-panel requests, modelled by ev1sim::HvacControls.
// HTCM owns the plant and feeds back the actual blower level (4082) + rear
// defrost grid (4083); these five are the upstream driver inputs.  Allocated
// ev1sim-side first (see the "pending electricsim adoption" note below).
//   4124  vehicle.hvac.temp_setpoint_c   float32 LE, °C (clamped 16..30)
//   4125  vehicle.hvac.fan_request       uint8: 0=OFF, 1=LOW, 2=MED, 3=HIGH
//   4126  vehicle.hvac.mode_request      uint8: 0=FACE, 1=BILEVEL, 2=FEET, 3=DEFROST
//   4127  vehicle.hvac.ac_request        uint8 bool: 0=off, 1=on
//   4128  vehicle.hvac.defrost_request   uint8 bool: 0=off, 1=on
constexpr std::uint32_t kSigHvacTempSetpointC   = 4124U;
constexpr std::uint32_t kSigHvacFanRequest      = 4125U;
constexpr std::uint32_t kSigHvacModeRequest     = 4126U;
constexpr std::uint32_t kSigHvacAcRequest       = 4127U;
constexpr std::uint32_t kSigHvacDefrostRequest  = 4128U;
constexpr int           kNumHvacControlSignals  = 5;
// Sanitization bounds for the HVAC setters (mirror ev1sim::HvacControls);
// keep out-of-range inputs from violating the wire contract or breaking the
// publish-on-change sentinels.
constexpr float        kHvacSetpointMinC     = 16.0f;
constexpr float        kHvacSetpointMaxC     = 30.0f;
constexpr float        kHvacSetpointDefaultC = 21.0f;   // NaN/Inf fallback
constexpr std::uint8_t kHvacFanMax           = 3u;      // OFF/LOW/MED/HIGH
constexpr std::uint8_t kHvacModeMax          = 3u;      // FACE/BILEVEL/FEET/DEFROST

// RSA run-mode broadcast — published by RSA on the main harness segment.
// ev1sim subscribes to this (input_to_sim = true for subscription, but we
// don't register it as an endpoint we publish — only receive).
// Locked in lockstep with electricsim/ev1/rsa/rsa_signals.hpp kSigRunModeBroadcast = 5711.
constexpr std::uint32_t kSigRunModeBroadcast = 5711U;

// PIM cruise-control state — published by PIM on the main harness segment.
// ev1sim subscribes (input direction); not registered as published endpoints.
//   kSigPimCruiseActive      = 5360  (bool, uint8: 0=off/standby, 1=engaged)
//   kSigPimCruiseSetpointMps = 5361  (float32 LE, target speed in m/s)
// MOVED 5860/5861 → 5360/5361: the PIM controller relocated its cruise
// signals into its own 5300-block (the 58xx range belongs to the
// steering-pump module, which now publishes pump telemetry there) — this
// mirror had silently kept the stale IDs, so cruise state was never
// received and pump telemetry would have been misread as cruise state the
// moment a steering-pump host joined the bus.
constexpr std::uint32_t kSigPimCruiseActive      = 5360U;
constexpr std::uint32_t kSigPimCruiseSetpointMps = 5361U;

// Auto Disconnect (AD) HV status — published by the AD controller on the
// main harness segment, on change.  ev1sim subscribes (input direction) so
// acceptance scenarios can log the HV power-up sequence (precharge → main
// contactor closed) and its failure modes (precharge timeout latch).
//   kSigAdMainContactor  = 5224  (bool: 1 = main HV contactor closed)
//   kSigAdPrechargeRelay = 5225  (bool: 1 = precharge relay closed)
//   kSigAdStateEnum      = 5230  (uint32 LE state enum: 0=OK,
//                                 6=precharging, 7=precharge failed)
constexpr std::uint32_t kSigAdMainContactor  = 5224U;
constexpr std::uint32_t kSigAdPrechargeRelay = 5225U;
constexpr std::uint32_t kSigAdStateEnum      = 5230U;

// BTCM front ABS solenoid signals — published by BTCM on the main harness segment.
// ev1sim subscribes (input_to_sim direction); not registered as published endpoints.
// Locked in lockstep with electricsim/ev1/btcm/btcm_signals.hpp:
//   kSigSolFL_ISO = 5010, kSigSolFL_DMP = 5011
//   kSigSolFR_ISO = 5012, kSigSolFR_DMP = 5013
constexpr std::uint32_t kSigSolFL_ISO = 5010U;
constexpr std::uint32_t kSigSolFL_DMP = 5011U;
constexpr std::uint32_t kSigSolFR_ISO = 5012U;
constexpr std::uint32_t kSigSolFR_DMP = 5013U;

// BTCM canonical-frame heartbeat — supervisor broadcasts a 16-byte
// status frame at 5 Hz regardless of ABS modulation state.  Used
// as the "BTCM alive" liveness signal, separate from the iso/dump-pin
// state (which encodes commanded valve position, not liveness).
// Locked in lockstep with electricsim/ev1/btcm/btcm_signals.hpp:
//   kSigBtcmUartFrame = 5050
constexpr std::uint32_t kSigBtcmUartFrame = 5050U;

// BTCM rear EMB motor commands — published by BTCM on the main harness segment.
// Float in [-1, +1]: +1=apply, 0=hold/idle, -1=release.  ev1sim consumes
// these to drive the BrakeDrum self-energizing model and apply per-wheel
// rear brake torque to Chrono.  Not registered as a published endpoint.
//   kSigRearMotorLR = 5014, kSigRearMotorRR = 5015
constexpr std::uint32_t kSigRearMotorLR = 5014U;
constexpr std::uint32_t kSigRearMotorRR = 5015U;

// 3D-sim integration contract — physics telemetry + cabin sensors published
// on the main harness segment.  Locked to electricsim's docs/3d_sim_contract.md
// reservations 6920-6939 (physics telemetry), 6940-6959 (buttons), and
// 6960-6979 (discrete sensors).  electricsim does not yet subscribe to most
// of these — publishing them locks the wire format on the public side so
// the eventual subscriber doesn't have to guess.  Cross-references the
// existing chassis-bus equivalents:
//   6920 vs 4100 (speed),  6921 vs 4101 (long accel),  6922 vs 4102 (lat accel)
//   6960-6963 vs 4030-4033 (panel ajar sensors — same data, different segment)
// Pose (6930-6932) is new; ev1sim does not publish it on the chassis bus.
constexpr std::uint32_t kSigExtBodyVelocityMps        = 6920U;
constexpr std::uint32_t kSigExtAccelLongitudinalMps2  = 6921U;
constexpr std::uint32_t kSigExtAccelLateralMps2       = 6922U;
constexpr std::uint32_t kSigExtVehiclePoseX           = 6930U;
constexpr std::uint32_t kSigExtVehiclePoseY           = 6931U;
constexpr std::uint32_t kSigExtVehiclePoseYawRad      = 6932U;
// Buttons / discrete driver inputs — see docs/3d_sim_contract.md §1.3.
// 6940/6941 (horn high/low request) REMOVED — the driver horn is a single
// physical contact (circuit 28), now published as chassis horn cavity 4046.
// IDs left reserved (do not reuse) per docs/3d_sim_contract.md §1.3.
constexpr std::uint32_t kSigDriverHeadlightSwitch     = 6942U;
constexpr std::uint32_t kSigDriverHeadlightDimRequest = 6943U;
constexpr std::uint32_t kSigDriverTelltaleTestRequest = 6945U;
constexpr std::uint32_t kSigDriverParkBrakeSetRequest = 6946U;
constexpr std::uint32_t kSigDriverParkBrakeReleaseRequest = 6947U;
// Discrete sensors — see docs/3d_sim_contract.md §1.4.
constexpr std::uint32_t kSigSensorDoorOpenDriver      = 6960U;
constexpr std::uint32_t kSigSensorDoorOpenPassenger   = 6961U;
constexpr std::uint32_t kSigSensorHoodOpen            = 6962U;
constexpr std::uint32_t kSigSensorTrunkOpen           = 6963U;
constexpr std::uint32_t kSigSensorKeyPosition         = 6966U;
constexpr int kNumExtContractSignals = 16;  // was 18; horn 6940/6941 removed (now chassis cavity 4046)

// BTCM per-wheel actuator state published on the chassis bus (IDs 4147-4154).
// These mirror the legacy main-harness publications above but live on the
// public chassis-bus segment that is ev1sim's primary input fabric.  Both
// sources stream in parallel; the consumers in this file prefer chassis-bus
// values whenever they have ever been observed for a given wheel, and fall
// back to the main-harness equivalents otherwise.  Registered as input
// endpoints so the standard chassis-bus drain picks them up via FindEndpoint.
constexpr std::uint32_t kSigChassisBtcmIsoCloseFL        = 4147U;
constexpr std::uint32_t kSigChassisBtcmIsoCloseFR        = 4148U;
constexpr std::uint32_t kSigChassisBtcmDumpOpenFL        = 4149U;
constexpr std::uint32_t kSigChassisBtcmDumpOpenFR        = 4150U;
constexpr std::uint32_t kSigChassisBtcmEmbMotorCmdLR     = 4151U;
constexpr std::uint32_t kSigChassisBtcmEmbMotorCmdRR     = 4152U;
constexpr std::uint32_t kSigChassisBtcmCylPressureFL_kPa = 4153U;
constexpr std::uint32_t kSigChassisBtcmCylPressureFR_kPa = 4154U;
constexpr int kNumBtcmActuatorChassisSignals = 8;

// Mapping from signal slot (kBulbCmdBase + slot) to LightID.  Order must stay
// locked to the electric sim's LightIdx enum for the first 17 entries.
constexpr LightID kBulbOrder[NUM_LIGHTS] = {
    LightID::LBL,    LightID::RBL,
    LightID::LHBH,   LightID::RHBH,
    LightID::LLBH,   LightID::RLBH,
    LightID::LRSM,   LightID::RRSM,
    LightID::LFML,   LightID::RFML,
    LightID::LFTS,   LightID::RFTS,
    LightID::LRTS,   LightID::RRTS,
    LightID::LRSL,   LightID::CHMSL,  LightID::RRSL,
    LightID::LRTL,   LightID::RRTL,   // ev1sim-only dual-filament tail elements
};

// Short / qualified names indexed by LightID enum value (not by signal slot).
// Keeps the EV1-manual abbreviation stable regardless of wire-level ordering.
constexpr const char* kBulbShort[NUM_LIGHTS] = {
    "lhbh_bulb_feed_line",  "llbh_bulb_feed_line",
    "rhbh_bulb_feed_line",  "rlbh_bulb_feed_line",
    "lfts_bulb_feed_line",  "rfts_bulb_feed_line",
    "lfml_bulb_feed_line",  "rfml_bulb_feed_line",
    "lrsl_bulb_feed_line",  "rrsl_bulb_feed_line",
    "lrtl_bulb_feed_line",  "rrtl_bulb_feed_line",
    "lrts_bulb_feed_line",  "rrts_bulb_feed_line",
    "lrsm_bulb_feed_line",  "rrsm_bulb_feed_line",
    "chmsl_bulb_feed_line",
    "lbl_bulb_feed_line",   "rbl_bulb_feed_line",
};

constexpr const char* kBulbQualified[NUM_LIGHTS] = {
    "vehicle.body.lhbh.bulb_feed_line",  "vehicle.body.llbh.bulb_feed_line",
    "vehicle.body.rhbh.bulb_feed_line",  "vehicle.body.rlbh.bulb_feed_line",
    "vehicle.body.lfts.bulb_feed_line",  "vehicle.body.rfts.bulb_feed_line",
    "vehicle.body.lfml.bulb_feed_line",  "vehicle.body.rfml.bulb_feed_line",
    "vehicle.body.lrsl.bulb_feed_line",  "vehicle.body.rrsl.bulb_feed_line",
    "vehicle.body.lrtl.bulb_feed_line",  "vehicle.body.rrtl.bulb_feed_line",
    "vehicle.body.lrts.bulb_feed_line",  "vehicle.body.rrts.bulb_feed_line",
    "vehicle.body.lrsm.bulb_feed_line",  "vehicle.body.rrsm.bulb_feed_line",
    "vehicle.body.chmsl.bulb_feed_line",
    "vehicle.body.lbl.bulb_feed_line",   "vehicle.body.rbl.bulb_feed_line",
};

// Look up the LightID that corresponds to an inbound bulb signal_id, or -1
// if the signal is outside our bulb command range.
int LightIdForBulbSignal(std::uint32_t signal_id) {
    if (signal_id < kBulbCmdBase || signal_id >= kBulbCmdBase + NUM_LIGHTS)
        return -1;
    return static_cast<int>(kBulbOrder[signal_id - kBulbCmdBase]);
}

struct PanelNames { const char* qualified; const char* shortname; };
constexpr PanelNames kPanelNames[] = {
    {"vehicle.body.hood.ajar_switch",       "hood_ajar"},
    {"vehicle.body.trunk.ajar_switch",      "trunk_ajar"},
    {"vehicle.body.door_left.ajar_switch",  "door_left_ajar"},
    {"vehicle.body.door_right.ajar_switch", "door_right_ajar"},
};
static_assert(sizeof(kPanelNames) / sizeof(kPanelNames[0]) ==
                  VehiclePanels::NUM_PANELS,
              "kPanelNames must cover every PanelID");

// ---------------------------------------------------------------------------
// Vehicle dynamics endpoint names.
// Ordered to match kDynamicsBase + index; gaps in the signal ID space (e.g.
// 4114-4119, 4124+) are skipped by using explicit offsets in BuildEndpoints().
// ---------------------------------------------------------------------------
struct DynNames { std::uint32_t offset; const char* qualified; const char* shortname; };
constexpr DynNames kDynamicsNames[] = {
    {0,  "vehicle.dynamics.speed_mps",            "speed_mps"},
    {1,  "vehicle.dynamics.accel_long",           "accel_long"},
    {2,  "vehicle.dynamics.accel_lat",            "accel_lat"},
    {3,  "vehicle.dynamics.yaw_rate",             "yaw_rate"},
    {4,  "vehicle.dynamics.applied_throttle",     "applied_throttle"},
    {5,  "vehicle.dynamics.applied_front_brake",  "applied_front_brake"},
    {6,  "vehicle.dynamics.applied_rear_brake",   "applied_rear_brake"},
    // Actuated brake states (after dynamics model) — what the hardware
    // actually delivers vs. what was commanded:
    {7,  "vehicle.dynamics.front_brake_pressure", "front_brake_pressure"},
    {8,  "vehicle.dynamics.rear_brake_position",  "rear_brake_position"},
    {9,  "vehicle.dynamics.steering_torque",      "steering_torque"},
    {10, "vehicle.dynamics.wheel_omega_fl",       "wheel_omega_fl"},
    {11, "vehicle.dynamics.wheel_omega_fr",       "wheel_omega_fr"},
    {12, "vehicle.dynamics.wheel_omega_rl",       "wheel_omega_rl"},
    {13, "vehicle.dynamics.wheel_omega_rr",       "wheel_omega_rr"},
    {20, "vehicle.dynamics.slip_ratio_fl",        "slip_ratio_fl"},
    {21, "vehicle.dynamics.slip_ratio_fr",        "slip_ratio_fr"},
    {22, "vehicle.dynamics.slip_ratio_rl",        "slip_ratio_rl"},
    {23, "vehicle.dynamics.slip_ratio_rr",        "slip_ratio_rr"},
};
constexpr int kNumDynamics = static_cast<int>(sizeof(kDynamicsNames) /
                                               sizeof(kDynamicsNames[0]));

// ---------------------------------------------------------------------------
// Build the endpoint table once.
// ---------------------------------------------------------------------------
// kNumDriverInputs covers all driver inputs on the main harness segment
// (electricsim_ev1_bus), all output from ev1sim:
//   6900 brake_pedal_q8, 6901 steering_deg_q8, 6902 gear_selector,
//   6903 throttle_q8, 6904 brake_switch, 6944 hazard_request,
//   6948 turn_signal_left, 6949 turn_signal_right, 6964 seatbelt_buckled,
//   6971 rsa_mode_button,
//   6975 rsa_keypad_button1, 6976..6979 (buttons 2-5)  (15 total).
//   + 1 new: 6952 ipc_trip_reset.  (Cruise 6953-6957 and wiper 6958/6959 moved
//     to the chassis switch cavities below.)
//   + 4 new: 6980-6983 power window switches (driver up/down, passenger up/down).
// (Slot 6970 is reserved; not registered.)
// +1 for the charge coupler presence (ID 4060, chassis segment).
// +kNumPrndSelector for the 4 PRND selector lines (IDs 4050-4053, chassis segment).
// +kNumMotorSignals for motor RPM + torque + DC current (IDs 4070-4072, chassis segment).
// +kNumWiperSignals for wiper motor command (4080) + washer pump command (4081).
// +kNumHvacSignals for hvac_blower_level (4082) + defrost_grid_active (4083).
// +kNumAmbientSignals for ambient temp (4090) + ambient humidity (4091).
// +kNumDoorLockPwSignals for door lock cmds (4084/4085) + power window motor cmds (4086/4087).
// +kNumRsaShiftBlockedSignals for RSA shift-blocked cue (4088) — RSA → ev1sim.
// +kNumIpcTelltaleSignals for IPC seatbelt telltales (4130/4131) — IPC → ev1sim.
// +kNumIpcTripDistSignals for IPC trip distance (4132) — IPC → ev1sim.
// +kNumIpcBtcmTelltaleSignals for IPC BTCM/airbag telltales (4134–4138) — IPC → ev1sim.
// +kNumIpcExtraTelltaleSignals for IPC extra LCD telltales (4140–4145) — IPC → ev1sim.
// +kNumBpmPackVoltageSignals for BPM pack voltage (4139) — BPM → ev1sim.
// +kNumBtcmActuatorChassisSignals for the BTCM per-wheel actuator state
//   on the chassis bus (4147-4154) — BTCM → ev1sim, parallel to the
//   legacy main-harness 5010-5015 path.
// +kNumExtContractSignals for the 3D-sim integration contract publish
//   (6920-6932 physics telemetry + 6960-6963 cabin sensors) on the main
//   harness — ev1sim → electricsim, locking the wire format for the
//   eventual subscribers.
// Wiper/washer switch cavities (4054-4057): ev1sim publishes the raw contacts
// computed from the detent enum; RHJB's WSW decoder consumes them.
constexpr int kNumWiperSwCavitySignals = 4;
// Turn/hazard combination-switch cavities (4043-4046): ev1sim publishes the raw
// contacts (turn left/right, hazard, horn); LHJB consumes them.
constexpr int kNumTurnHazSwCavitySignals = 4;
// Cruise-control switch cavities (4047-4049): ev1sim publishes the raw contacts
// (set/coast, resume/accel, on/off); PIM's tap/hold decoder consumes them.
constexpr int kNumCruiseSwCavitySignals = 3;
constexpr int kNumEndpoints =
    NUM_LIGHTS + 2 + VehiclePanels::NUM_PANELS + kNumCombSw + 1 /*charge_coupler*/ +
    kNumPrndSelector + kNumWiperSwCavitySignals + kNumTurnHazSwCavitySignals +
    kNumCruiseSwCavitySignals +
    kNumMotorSignals + kNumSimTimeSignals + kNumThrottleCmdSignals +
    kNumSteeringCmdSignals +
    kNumBrakeSignals + kNumWiperSignals + kNumHvacSignals +
    kNumAmbientSignals + kNumDoorLockPwSignals + kNumRsaShiftBlockedSignals +
    kNumDoorLockMotorSignals + kNumSounderSignals + kNumSteeringPumpSignals +
    kNumHvacControlSignals + kNumDoorLockStateSignals +
    kNumIpcTelltaleSignals + kNumIpcTripDistSignals + kNumIpcBtcmTelltaleSignals +
    kNumIpcExtraTelltaleSignals + kNumBpmPackVoltageSignals +
    kNumBtcmActuatorChassisSignals + kNumExtContractSignals +
    kNumDynamics + kNumDriverInputs;

std::array<ExternalSimConnector::Endpoint, kNumEndpoints> BuildEndpoints() {
    std::array<ExternalSimConnector::Endpoint, kNumEndpoints> out{};
    int i = 0;
    for (int slot = 0; slot < NUM_LIGHTS; ++slot, ++i) {
        const int lid = static_cast<int>(kBulbOrder[slot]);
        out[i] = {kBulbCmdBase + static_cast<std::uint32_t>(slot),
                  kBulbQualified[lid], kBulbShort[lid], /*input_to_sim=*/true};
    }
    out[i++] = {kHornLowCmd,  "vehicle.body.horn.low_drive_line",  "horn_low_drive_line",  true};
    out[i++] = {kHornHighCmd, "vehicle.body.horn.high_drive_line", "horn_high_drive_line", true};
    for (int p = 0; p < VehiclePanels::NUM_PANELS; ++p, ++i) {
        out[i] = {kPanelBase + static_cast<std::uint32_t>(p),
                  kPanelNames[p].qualified, kPanelNames[p].shortname,
                  /*input_to_sim=*/false};
    }
    out[i++] = {kCombSwLowBeamOutId,
                "vehicle.body.combination_switch.low_beam_out",
                "comb_sw_low_beam_out", false};
    out[i++] = {kCombSwFlashToPassOutId,
                "vehicle.body.combination_switch.flash_to_pass_out",
                "comb_sw_flash_to_pass_out", false};
    out[i++] = {kCombSwParkHeadlampOutId,
                "vehicle.body.combination_switch.park_headlamp_out",
                "comb_sw_park_headlamp_out", false};
    // Turn/hazard switch cavities (4043-4046) are registered ONCE, in the
    // driver-input section below as vehicle.driver.{turn_right,turn_left,
    // hazard,horn}_contact.  (They are still published wire-level on the
    // chassis bus via the turn_haz publish block in Tick; the endpoint table
    // is metadata only.)  A second vehicle.body.turn_haz_switch.* registration
    // here was a leftover from the pre-cavity wiring and is intentionally gone.
    out[i++] = {kChargeCouplerPresentId,
                "vehicle.body.charge_coupler.present",
                "charge_coupler_present", false};
    out[i++] = {kPrndSelectorAId,
                "vehicle.driver.prnd_selector_a", "prnd_selector_a", false};
    out[i++] = {kPrndSelectorBId,
                "vehicle.driver.prnd_selector_b", "prnd_selector_b", false};
    out[i++] = {kPrndSelectorCId,
                "vehicle.driver.prnd_selector_c", "prnd_selector_c", false};
    out[i++] = {kPrndSelectorDId,
                "vehicle.driver.prnd_selector_d", "prnd_selector_d", false};
    // Wiper/washer switch cavities (4054-4057) are registered ONCE, in the
    // driver-input section below as vehicle.driver.{wiper_delay,wiper_request,
    // wiper_hi,washer_switch}_contact.  (Still published wire-level on the
    // chassis bus via the wiper publish block in Tick.)  A second
    // vehicle.body.wiper_washer_switch.* registration here was a leftover from
    // the pre-cavity wiring and is intentionally gone.
    // Motor state signals (chassis segment, ev1sim → electricsim).
    out[i++] = {kSigMotorRpm,
                "vehicle.dynamics.motor_rpm", "motor_rpm", false};
    out[i++] = {kSigMotorTorqueNm,
                "vehicle.dynamics.motor_torque_nm", "motor_torque_nm", false};
    out[i++] = {kSigMotorCurrentA,
                "vehicle.dynamics.motor_current_a", "motor_current_a", false};
    // Sim-time master clock (chassis segment, ev1sim → electricsim, uint64 LE ns).
    out[i++] = {kSigSimTimeNs,
                "vehicle.dynamics.sim_time_ns", "sim_time_ns", false};
    // Ambient environment sensors (chassis segment, ev1sim → electricsim).
    out[i++] = {kSigAmbientTempC,
                "vehicle.environment.ambient_temp_c", "ambient_temp_c", false};
    out[i++] = {kSigAmbientHumidityPct,
                "vehicle.environment.ambient_humidity_pct", "ambient_humidity_pct", false};
    // Throttle command (PIM → ev1sim, chassis segment).
    out[i++] = {kSigThrottleCmdQ8,
                "vehicle.dynamics.throttle_cmd_q8", "throttle_cmd_q8", true};
    // Steering command (electricsim → ev1sim, chassis segment, input_to_sim=true).
    out[i++] = {kSigSteeringCmd,
                "vehicle.dynamics.steering_cmd", "steering_cmd", true};
    // Brake master cylinder pressure (ev1sim → electricsim, chassis segment).
    out[i++] = {kSigBrakeMasterPressureKpa,
                "vehicle.brake.master_cylinder_pressure_kpa",
                "brake_master_pressure_kpa", false};
    // Wiper motor command + washer pump command (RHJB → ev1sim, chassis segment).
    out[i++] = {kSigWiperMotorCommand,
                "vehicle.body.wiper_motor.command", "wiper_motor_command", true};
    out[i++] = {kSigWasherPumpCommand,
                "vehicle.body.washer_pump.command", "washer_pump_command", true};
    // HVAC blower level + defrost grid (HTCM → ev1sim, chassis segment, input_to_sim=true).
    // Encoding: blower 0=OFF/1=LOW/2=MED/3=HIGH; defrost 0=off/1=on.
    out[i++] = {kSigHvacBlowerLevel,
                "vehicle.hvac.blower_level", "hvac_blower_level", true};
    out[i++] = {kSigDefrostGridActive,
                "vehicle.hvac.defrost_grid_active", "hvac_defrost_grid_active", true};
    // IPC LCD seatbelt telltales (IPC → ev1sim, chassis segment, input_to_sim=true).
    // Encoding: uint8 bool — 0=lamp off, 1=lamp on.
    // Lit when seat is unbuckled AND speed > ~8 km/h (IPC supervisor threshold).
    out[i++] = {kSigIpcSeatbeltTelltaleDriver,
                "vehicle.ipc.seatbelt_telltale_driver",
                "ipc_seatbelt_telltale_driver", true};
    out[i++] = {kSigIpcSeatbeltTelltalePassenger,
                "vehicle.ipc.seatbelt_telltale_passenger",
                "ipc_seatbelt_telltale_passenger", true};
    // IPC trip distance (IPC → ev1sim, chassis segment, input_to_sim=true).
    // Encoding: float32 LE, metres.  Published on change (epsilon ~0.5 m).
    out[i++] = {kSigIpcTripDistanceM,
                "vehicle.ipc.trip_distance_m",
                "ipc_trip_distance_m", true};
    // IPC BTCM / airbag telltales (IPC → ev1sim, chassis segment, input_to_sim=true).
    // Encoding: uint8 bool — 0=lamp off, 1=lamp on.
    out[i++] = {kSigIpcBrakeTelltale,
                "vehicle.ipc.brake_telltale",
                "ipc_brake_telltale", true};
    out[i++] = {kSigIpcParkBrakeTelltale,
                "vehicle.ipc.park_brake_telltale",
                "ipc_park_brake_telltale", true};
    out[i++] = {kSigIpcAntilockTelltale,
                "vehicle.ipc.antilock_telltale",
                "ipc_antilock_telltale", true};
    out[i++] = {kSigIpcLowTracTelltale,
                "vehicle.ipc.low_trac_telltale",
                "ipc_low_trac_telltale", true};
    out[i++] = {kSigIpcAirBagTelltale,
                "vehicle.ipc.air_bag_telltale",
                "ipc_air_bag_telltale", true};
    // IPC extra LCD telltales (IPC → ev1sim, chassis segment, input_to_sim=true).
    // Encoding: uint8 bool — 0=lamp off, 1=lamp on.
    out[i++] = {kSigIpcServiceNowTelltale,
                "vehicle.ipc.service_now_telltale",
                "ipc_service_now_telltale", true};
    out[i++] = {kSigIpcCheckMessagesTelltale,
                "vehicle.ipc.check_messages_telltale",
                "ipc_check_messages_telltale", true};
    out[i++] = {kSigIpcTempTelltale,
                "vehicle.ipc.temp_telltale",
                "ipc_temp_telltale", true};
    out[i++] = {kSigIpcBatteryLifeTelltale,
                "vehicle.ipc.battery_life_telltale",
                "ipc_battery_life_telltale", true};
    out[i++] = {kSigIpcReducedPerfTelltale,
                "vehicle.ipc.reduced_perf_telltale",
                "ipc_reduced_perf_telltale", true};
    out[i++] = {kSigIpcCheckTirePressTelltale,
                "vehicle.ipc.check_tire_press_telltale",
                "ipc_check_tire_press_telltale", true};
    // BPM pack voltage (BPM → ev1sim, chassis segment, input_to_sim=true).
    // Encoding: uint32 LE, millivolts.  Published on change (epsilon ~50 mV).
    out[i++] = {kSigBpmPackVoltageMv,
                "vehicle.bpm.pack_voltage_mv",
                "bpm_pack_voltage_mv", true};
    // Door lock commands (RSA → ev1sim, chassis segment, input_to_sim=true).
    out[i++] = {kSigDoorLockCmdDriver,
                "vehicle.body.door_lock_cmd.driver",
                "door_lock_cmd_driver", true};
    out[i++] = {kSigDoorLockCmdPassenger,
                "vehicle.body.door_lock_cmd.passenger",
                "door_lock_cmd_passenger", true};
    // Power window motor commands (RSA → ev1sim, chassis segment, input_to_sim=true).
    out[i++] = {kSigPowerWindowMotorDriver,
                "vehicle.body.power_window_motor.driver",
                "power_window_motor_driver", true};
    out[i++] = {kSigPowerWindowMotorPassenger,
                "vehicle.body.power_window_motor.passenger",
                "power_window_motor_passenger", true};
    // RSA shift-blocked cue (RSA → ev1sim, chassis segment, input_to_sim=true).
    // Level signal: 1 while a P→non-P shift is refused (brake not pressed); 0 otherwise.
    out[i++] = {kSigRsaShiftBlocked,
                "vehicle.body.rsa.shift_blocked",
                "rsa_shift_blocked", true};
    // Door-lock motor leg drives (RHJB → ev1sim, chassis segment, input_to_sim=true).
    // dual-H-bridge: [LH lock, LH unlock, RH lock, RH unlock].  See docs/peripherals.md.
    out[i++] = {kSigDoorLockMotorLhLockDrive,
                "vehicle.body.door_lock_motor.lh_lock_drive",
                "door_lock_motor_lh_lock_drive", true};
    out[i++] = {kSigDoorLockMotorLhUnlockDrive,
                "vehicle.body.door_lock_motor.lh_unlock_drive",
                "door_lock_motor_lh_unlock_drive", true};
    out[i++] = {kSigDoorLockMotorRhLockDrive,
                "vehicle.body.door_lock_motor.rh_lock_drive",
                "door_lock_motor_rh_lock_drive", true};
    out[i++] = {kSigDoorLockMotorRhUnlockDrive,
                "vehicle.body.door_lock_motor.rh_unlock_drive",
                "door_lock_motor_rh_unlock_drive", true};
    // Sounder / piezo drive (LHJB flasher → ev1sim, chassis segment, input_to_sim=true).
    out[i++] = {kSigSounderPiezoDrive,
                "vehicle.body.sounder.piezo_drive",
                "sounder_piezo_drive", true};
    // Power-steering pump motor (PSCM ↔ ev1sim, chassis segment).
    // speed_cmd_q8 flows PSCM → ev1sim (input); interlock_closed flows
    // ev1sim → PSCM (output, the molex.D/E HV interlock loop).
    out[i++] = {kSigPscmPumpSpeedCmdQ8,
                "vehicle.steering.pump_motor.speed_cmd_q8",
                "steering_pump_speed_cmd_q8", true};
    out[i++] = {kSigPscmPumpInterlockClosed,
                "vehicle.steering.pump_motor.interlock_closed",
                "steering_pump_interlock_closed", false};
    // HVAC driver controls (ev1sim → HTCM, chassis segment, input_to_sim=false).
    out[i++] = {kSigHvacTempSetpointC,
                "vehicle.hvac.temp_setpoint_c", "hvac_temp_setpoint_c", false};
    out[i++] = {kSigHvacFanRequest,
                "vehicle.hvac.fan_request", "hvac_fan_request", false};
    out[i++] = {kSigHvacModeRequest,
                "vehicle.hvac.mode_request", "hvac_mode_request", false};
    out[i++] = {kSigHvacAcRequest,
                "vehicle.hvac.ac_request", "hvac_ac_request", false};
    out[i++] = {kSigHvacDefrostRequest,
                "vehicle.hvac.defrost_request", "hvac_defrost_request", false};
    // Door lock STATE feedback (ev1sim → electricsim, chassis segment, input_to_sim=false).
    out[i++] = {kSigDoorLockStateDriver,
                "vehicle.body.door_lock_state.driver", "door_lock_state_driver", false};
    out[i++] = {kSigDoorLockStatePassenger,
                "vehicle.body.door_lock_state.passenger", "door_lock_state_passenger", false};
    out[i++] = {kSigDoorLockStateTrunk,
                "vehicle.body.door_lock_state.trunk", "door_lock_state_trunk", false};
    // BTCM per-wheel actuator state (BTCM → ev1sim, chassis segment, input_to_sim=true).
    // Mirror of the legacy main-harness publications 5010-5015 plus two
    // new per-wheel cylinder pressure measurements (4153/4154) with no
    // main-harness equivalent.
    out[i++] = {kSigChassisBtcmIsoCloseFL,
                "vehicle.btcm.iso_close.fl",
                "btcm_iso_close_fl", true};
    out[i++] = {kSigChassisBtcmIsoCloseFR,
                "vehicle.btcm.iso_close.fr",
                "btcm_iso_close_fr", true};
    out[i++] = {kSigChassisBtcmDumpOpenFL,
                "vehicle.btcm.dump_open.fl",
                "btcm_dump_open_fl", true};
    out[i++] = {kSigChassisBtcmDumpOpenFR,
                "vehicle.btcm.dump_open.fr",
                "btcm_dump_open_fr", true};
    out[i++] = {kSigChassisBtcmEmbMotorCmdLR,
                "vehicle.btcm.emb_motor_cmd.lr",
                "btcm_emb_motor_cmd_lr", true};
    out[i++] = {kSigChassisBtcmEmbMotorCmdRR,
                "vehicle.btcm.emb_motor_cmd.rr",
                "btcm_emb_motor_cmd_rr", true};
    out[i++] = {kSigChassisBtcmCylPressureFL_kPa,
                "vehicle.btcm.cyl_pressure.fl_kpa",
                "btcm_cyl_pressure_fl_kpa", true};
    out[i++] = {kSigChassisBtcmCylPressureFR_kPa,
                "vehicle.btcm.cyl_pressure.fr_kpa",
                "btcm_cyl_pressure_fr_kpa", true};
    for (int d = 0; d < kNumDynamics; ++d, ++i) {
        out[i] = {kDynamicsBase + kDynamicsNames[d].offset,
                  kDynamicsNames[d].qualified, kDynamicsNames[d].shortname,
                  /*input_to_sim=*/false};
    }
    // Driver inputs on the main harness segment (electricsim_ev1_bus).
    // All are outputs from ev1sim (input_to_sim=false).
    out[i++] = {kSigDriverBrakePedalQ8,
                "vehicle.driver.brake_pedal_q8", "driver_brake_pedal_q8", false};
    out[i++] = {kSigDriverSteeringDegQ8,
                "vehicle.driver.steering_deg_q8", "driver_steering_deg_q8", false};
    out[i++] = {kSigDriverGearSelector,
                "vehicle.driver.gear_selector", "driver_gear_selector", false};
    out[i++] = {kSigDriverThrottleQ8,
                "vehicle.driver.throttle_q8", "driver_throttle_q8", false};
    out[i++] = {kSigDriverBrakeSwitch,
                "vehicle.driver.brake_switch", "driver_brake_switch", false};
    out[i++] = {kSigDriverSeatbeltBuckled,
                "vehicle.driver.seatbelt_buckled", "driver_seatbelt_buckled", false};
    out[i++] = {kSigDriverSeatbeltBuckledPassenger,
                "vehicle.driver.seatbelt_buckled_passenger",
                "driver_seatbelt_buckled_passenger", false};
    out[i++] = {kSigTurnHazSw_HazardOut,
                "vehicle.driver.hazard_contact", "hazard_contact", false};
    out[i++] = {kSigTurnHazSw_LeftTurnOut,
                "vehicle.driver.turn_left_contact", "turn_left_contact", false};
    out[i++] = {kSigTurnHazSw_RightTurnOut,
                "vehicle.driver.turn_right_contact", "turn_right_contact", false};
    out[i++] = {kSigTurnHazSw_HornOut,
                "vehicle.driver.horn_contact", "horn_contact", false};
    out[i++] = {kSigDriverRsaModeButton,
                "vehicle.driver.rsa_mode_button", "driver_rsa_mode_button", false};
    out[i++] = {kSigDriverRsaKeypadButton1,
                "vehicle.driver.rsa_keypad_button1", "driver_rsa_keypad_button1", false};
    out[i++] = {kSigDriverRsaKeypadButton2,
                "vehicle.driver.rsa_keypad_button2", "driver_rsa_keypad_button2", false};
    out[i++] = {kSigDriverRsaKeypadButton3,
                "vehicle.driver.rsa_keypad_button3", "driver_rsa_keypad_button3", false};
    out[i++] = {kSigDriverRsaKeypadButton4,
                "vehicle.driver.rsa_keypad_button4", "driver_rsa_keypad_button4", false};
    out[i++] = {kSigDriverRsaKeypadButton5,
                "vehicle.driver.rsa_keypad_button5", "driver_rsa_keypad_button5", false};
    // New driver input: IPC trip-reset (6952).  Cruise stalk publishes as
    // chassis cavities kSigCruiseSw_* (4047-4049); wiper/washer as kSigWiperSw_*
    // (4054-4057), both below.
    out[i++] = {kSigDriverIpcTripResetButton,
                "vehicle.driver.ipc_trip_reset_button", "driver_ipc_trip_reset", false};
    // Cruise-control switch cavities (4047-4049) — raw contacts; PIM decodes.
    out[i++] = {kSigCruiseSw_SetCoastOut,
                "vehicle.driver.cruise_set_coast_contact", "cruise_set_coast_contact", false};
    out[i++] = {kSigCruiseSw_ResumeAccelOut,
                "vehicle.driver.cruise_resume_accel_contact", "cruise_resume_accel_contact", false};
    out[i++] = {kSigCruiseSw_OnOffOut,
                "vehicle.driver.cruise_on_off_contact", "cruise_on_off_contact", false};
    out[i++] = {kSigWiperSw_DelayOut,
                "vehicle.driver.wiper_delay_contact", "wiper_delay_contact", false};
    out[i++] = {kSigWiperSw_RequestOut,
                "vehicle.driver.wiper_request_contact", "wiper_request_contact", false};
    out[i++] = {kSigWiperSw_HiOut,
                "vehicle.driver.wiper_hi_contact", "wiper_hi_contact", false};
    out[i++] = {kSigWiperSw_WasherSwitchOut,
                "vehicle.driver.washer_switch_contact", "washer_switch_contact", false};
    // Power window switch signals (6980-6983) — momentary bool, held while pressed.
    // No keyboard binding; floating UI will drive press()/release() when it lands.
    // consumer = RSA (window-motor logic), future round.
    out[i++] = {kSigDriverPowerWindowDriverUp,
                "vehicle.driver.power_window_driver_up",
                "power_window_driver_up", false};
    out[i++] = {kSigDriverPowerWindowDriverDown,
                "vehicle.driver.power_window_driver_down",
                "power_window_driver_down", false};
    out[i++] = {kSigDriverPowerWindowPassengerUp,
                "vehicle.driver.power_window_passenger_up",
                "power_window_passenger_up", false};
    out[i++] = {kSigDriverPowerWindowPassengerDown,
                "vehicle.driver.power_window_passenger_down",
                "power_window_passenger_down", false};
    // RSA exterior pillar keypad (6985-6989) — momentary uint8 Option A encoding.
    // consumer = RSA exterior keypad logic, future round.
    out[i++] = {kSigDriverRsaExteriorKeypad1,
                "vehicle.driver.rsa_exterior_keypad1", "rsa_exterior_keypad1", false};
    out[i++] = {kSigDriverRsaExteriorKeypad2,
                "vehicle.driver.rsa_exterior_keypad2", "rsa_exterior_keypad2", false};
    out[i++] = {kSigDriverRsaExteriorKeypad3,
                "vehicle.driver.rsa_exterior_keypad3", "rsa_exterior_keypad3", false};
    out[i++] = {kSigDriverRsaExteriorKeypad4,
                "vehicle.driver.rsa_exterior_keypad4", "rsa_exterior_keypad4", false};
    out[i++] = {kSigDriverRsaExteriorKeypad5,
                "vehicle.driver.rsa_exterior_keypad5", "rsa_exterior_keypad5", false};
    // Door handle pull attempt signals (6990-6991) — momentary bool.
    // consumer = RSA (decides if door unlocks), future round.
    out[i++] = {kSigDriverDoorHandleAttemptDriver,
                "vehicle.driver.door_handle_attempt_driver",
                "door_handle_attempt_driver", false};
    out[i++] = {kSigDriverDoorHandleAttemptPassenger,
                "vehicle.driver.door_handle_attempt_passenger",
                "door_handle_attempt_passenger", false};
    // 3D-sim integration contract publishes — main harness segment,
    // ev1sim → electricsim, all output (input_to_sim=false).
    // PROPOSED in docs/3d_sim_contract.md; consumers TBD.
    out[i++] = {kSigExtBodyVelocityMps,
                "vehicle.dynamics.body_velocity_mps",
                "ext_body_velocity_mps", false};
    out[i++] = {kSigExtAccelLongitudinalMps2,
                "vehicle.dynamics.accel_longitudinal_mps2",
                "ext_accel_longitudinal_mps2", false};
    out[i++] = {kSigExtAccelLateralMps2,
                "vehicle.dynamics.accel_lateral_mps2",
                "ext_accel_lateral_mps2", false};
    out[i++] = {kSigExtVehiclePoseX,
                "vehicle.pose.x_m",
                "ext_vehicle_pose_x", false};
    out[i++] = {kSigExtVehiclePoseY,
                "vehicle.pose.y_m",
                "ext_vehicle_pose_y", false};
    out[i++] = {kSigExtVehiclePoseYawRad,
                "vehicle.pose.yaw_rad",
                "ext_vehicle_pose_yaw_rad", false};
    out[i++] = {kSigSensorDoorOpenDriver,
                "vehicle.sensor.door_open.driver",
                "sensor_door_open_driver", false};
    out[i++] = {kSigSensorDoorOpenPassenger,
                "vehicle.sensor.door_open.passenger",
                "sensor_door_open_passenger", false};
    out[i++] = {kSigSensorHoodOpen,
                "vehicle.sensor.hood_open",
                "sensor_hood_open", false};
    out[i++] = {kSigSensorTrunkOpen,
                "vehicle.sensor.trunk_open",
                "sensor_trunk_open", false};
    // Buttons / discrete driver inputs (3D-sim contract §1.3).
    // (Horn 6940/6941 removed — single chassis horn cavity 4046; see above.)
    out[i++] = {kSigDriverHeadlightSwitch,
                "vehicle.driver.headlight_switch",
                "ext_headlight_switch", false};
    out[i++] = {kSigDriverHeadlightDimRequest,
                "vehicle.driver.headlight_dim_request",
                "ext_headlight_dim_request", false};
    out[i++] = {kSigDriverTelltaleTestRequest,
                "vehicle.driver.telltale_test_request",
                "ext_telltale_test_request", false};
    out[i++] = {kSigDriverParkBrakeSetRequest,
                "vehicle.driver.park_brake_set_request",
                "ext_park_brake_set_request", false};
    out[i++] = {kSigDriverParkBrakeReleaseRequest,
                "vehicle.driver.park_brake_release_request",
                "ext_park_brake_release_request", false};
    // Discrete sensors (3D-sim contract §1.4) — key position.
    out[i++] = {kSigSensorKeyPosition,
                "vehicle.sensor.key_position",
                "sensor_key_position", false};
    return out;
}

const std::array<ExternalSimConnector::Endpoint, kNumEndpoints>& EndpointTable() {
    static const auto table = BuildEndpoints();
    return table;
}

// NowNs() is used both in the full Tick() path and in GetAbsPhaseFront(), so
// it lives outside the EV1SIM_HAVE_EXTERNAL_SIM guard.
std::uint64_t NowNs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

#if EV1SIM_HAVE_EXTERNAL_SIM
constexpr std::uint32_t kStreamEv1Sim = 0x45563153u; // "EV1S"
#endif

} // namespace

// ---------------------------------------------------------------------------
// State — holds all per-instance data.  In the stub build the transport
// fields are simply unused.
// ---------------------------------------------------------------------------
struct ExternalSimConnector::State {
    Status status = Status::Disabled;

    // Latched commands from the electric sim.
    bool bulb[NUM_LIGHTS]  = {};
    bool horn_low          = false;
    bool horn_high         = false;
    bool received_any_bulb = false;

    // Last published panel state — used so we only send deltas on change.
    bool panel[VehiclePanels::NUM_PANELS]      = {};
    bool panel_published[VehiclePanels::NUM_PANELS] = {};
    bool panel_ever_published                  = false;

    // 3D-sim contract sensor-panel mirror (main harness, IDs 6960-6963).
    // Parallel publication of the same panel-open data; -1 sentinel forces
    // first publish, then delta-only.
    std::int8_t sensor_door_open_driver_pub    = -1;
    std::int8_t sensor_door_open_passenger_pub = -1;
    std::int8_t sensor_hood_open_pub           = -1;
    std::int8_t sensor_trunk_open_pub          = -1;

    // 3D-sim contract buttons/discretes (6942-6947, 6966).
    // Values latched via the SetDriver*Request / SetSensor* setters and
    // published on change in the main-harness Tick block.
    // (Horn 6940/6941 removed — collapsed to the single chassis horn cavity
    //  4046, circuit 28; see SetTurnHazSwOutputs + the chassis publish block.)
    std::uint8_t ext_headlight_switch           = 0;   // 0=OFF, 1=PARK, 2=ON, 3=HI
    bool         ext_headlight_dim_request      = false;
    bool         ext_telltale_test_request      = false;
    bool         ext_park_brake_set_request     = false;
    bool         ext_park_brake_release_request = false;
    std::uint8_t ext_key_position               = 2;   // default RUN — see note in setter
    std::int8_t  ext_headlight_switch_pub   = -1;
    std::int8_t  ext_headlight_dim_pub      = -1;
    std::int8_t  ext_telltale_test_pub      = -1;
    std::int8_t  ext_park_brake_set_pub     = -1;
    std::int8_t  ext_park_brake_release_pub = -1;
    std::int8_t  ext_key_position_pub       = -1;

    // Combination switch pin outputs — latched by SetCombSwOutputs(), published
    // as wire-level booleans in Tick() when changed.
    bool comb_sw_low_beam      = false;
    bool comb_sw_flash_to_pass = false;
    bool comb_sw_park_headlamp = false;
    bool comb_sw_low_beam_pub      = false;
    bool comb_sw_flash_to_pass_pub = false;
    bool comb_sw_park_headlamp_pub = false;
    bool comb_sw_ever_published    = false;

    // Turn/hazard combination switch (12092237) output cavities — latched by
    // SetTurnHazSwOutputs(), published wire-level on the chassis segment on change.
    bool turn_haz_right_turn = false;
    bool turn_haz_left_turn  = false;
    bool turn_haz_hazard     = false;
    bool turn_haz_horn       = false;
    bool turn_haz_right_turn_pub = false;
    bool turn_haz_left_turn_pub  = false;
    bool turn_haz_hazard_pub     = false;
    bool turn_haz_horn_pub       = false;
    bool turn_haz_ever_published = false;

    // Wiper/washer switch (12092254) output cavities — latched by
    // SetWiperWasherSwOutputs(), published wire-level on the chassis segment on change.
    bool wiper_delay   = false;
    bool wiper_request = false;
    bool wiper_hi      = false;
    bool wiper_washer  = false;
    bool wiper_delay_pub   = false;
    bool wiper_request_pub = false;
    bool wiper_hi_pub      = false;
    bool wiper_washer_pub  = false;
    bool wiper_ever_published = false;

    // Vehicle dynamics snapshot — updated by SetVehicleState() each frame,
    // published in Tick() as float32 signals.
    VehicleState vstate{};
    bool         has_vstate = false;

    // Driver input snapshot — latched by SetDriver*() methods,
    // published to the main harness segment (electricsim_ev1_bus) in Tick().
    std::uint8_t  driver_brake_q8    = 0;
    std::int16_t  driver_steering_q8 = 0;
    std::uint8_t  driver_gear        = 3;  // default D
    std::uint8_t  driver_throttle_q8 = 0;
    std::uint8_t  driver_brake_pub   = 0xFF;  // force first publish
    std::int16_t  driver_steering_pub = 0x7FFF;
    std::uint8_t  driver_gear_pub    = 0xFF;
    std::uint8_t  driver_throttle_pub = 0xFF;
    // Fault injection: omit the throttle delta from the driver-input group
    // while set (SetSuppressThrottlePublish — VAT safety scenarios).
    bool          suppress_throttle_publish = false;
    // New discrete driver inputs (ID 6904, 6964, 6965) on the main harness segment.
    bool          driver_brake_switch                = false;
    bool          driver_seatbelt_buckled            = true;   // default: always buckled
    bool          driver_seatbelt_buckled_passenger  = true;   // default: always buckled
    // Published sentinels (use 0xFF-equivalent to force first publish).
    std::int8_t   driver_brake_switch_pub               = -1;  // -1 forces first publish
    std::int8_t   driver_seatbelt_buckled_pub           = -1;  // -1 forces first publish
    std::int8_t   driver_seatbelt_buckled_passenger_pub = -1;  // -1 forces first publish

    // Turn signal stalk and hazard switch (IDs 6948, 6949, 6944).
    bool          driver_turn_left        = false;
    bool          driver_turn_right       = false;
    bool          driver_hazard           = false;
    std::int8_t   driver_turn_left_pub    = -1;      // -1 forces first publish
    std::int8_t   driver_turn_right_pub   = -1;
    std::int8_t   driver_hazard_pub       = -1;
    std::int8_t   driver_horn_out_pub     = -1;   // kSigTurnHazSw_HornOut (4046)

    // RSA per-digit keypad buttons (IDs 6975-6979) and mode button (ID 6971).
    // driver_rsa_buttons[0..4] encode Option A: 0=idle, 1=tap, 2=long-press.
    // Corresponds to kSigDriverRsaKeypadButton[1..5].
    std::uint8_t  driver_rsa_buttons[5]   = {};
    std::uint8_t  driver_rsa_mode_button  = 0;
    // Published sentinels: use -1 to force first publish.
    std::int8_t   driver_rsa_btn_pub[5]   = {-1,-1,-1,-1,-1};
    std::int8_t   driver_rsa_mode_btn_pub = -1;

    // New driver input: IPC trip-reset (6952) — momentary bool.  The wiper
    // detent is held as a uint8 enum (driver_wiper_switch) and PUBLISHED as the
    // raw chassis cavities kSigWiperSw_* (4054-4057).
    bool          driver_ipc_trip_reset   = false;
    // Cruise-control switch raw contacts (kSigCruiseSw_* 4047-4049): held bools
    // driven by the keyboard/UI cruise stalk.  ev1sim does NOT pre-decode; PIM's
    // tap/hold decoder turns these into SET/RESUME/SPEED_*/CANCEL.
    bool          cruise_set_coast_contact    = false;
    bool          cruise_resume_accel_contact = false;
    bool          cruise_on_off_contact       = false;
    std::uint8_t  driver_wiper_switch     = 0;   // 0=OFF, 1=INT, 2=LOW, 3=HIGH
    bool          driver_wiper_wash       = false;
    // Published sentinels (-1 forces first publish for bool signals).
    std::int8_t   driver_ipc_trip_reset_pub    = -1;
    std::int8_t   cruise_set_coast_pub         = -1;
    std::int8_t   cruise_resume_accel_pub      = -1;
    std::int8_t   cruise_on_off_pub            = -1;
    std::int8_t   driver_wiper_delay_pub       = -1;   // kSigWiperSw_DelayOut   (4054)
    std::int8_t   driver_wiper_request_pub     = -1;   // kSigWiperSw_RequestOut (4055)
    std::int8_t   driver_wiper_hi_pub          = -1;   // kSigWiperSw_HiOut      (4056)
    std::int8_t   driver_wiper_wash_pub        = -1;   // kSigWiperSw_WasherSwitchOut (4057)

    // Power window switch states (IDs 6980-6983, main harness segment).
    // Momentary bools (0=released, 1=held this tick).
    // Default false — floating UI will toggle when it lands.
    bool          driver_pw_driver_up         = false;
    bool          driver_pw_driver_down       = false;
    bool          driver_pw_passenger_up      = false;
    bool          driver_pw_passenger_down    = false;
    std::int8_t   driver_pw_driver_up_pub     = -1;   // -1 forces first publish
    std::int8_t   driver_pw_driver_down_pub   = -1;
    std::int8_t   driver_pw_passenger_up_pub  = -1;
    std::int8_t   driver_pw_passenger_down_pub = -1;

    // RSA exterior pillar keypad (IDs 6985-6989, main harness segment).
    // Momentary uint8 (0=idle, 1=tap, 2=long-press).
    // consumer = RSA exterior keypad logic, future round.
    std::uint8_t  driver_ext_keypad[5]     = {};
    std::int8_t   driver_ext_keypad_pub[5] = {-1,-1,-1,-1,-1};

    // Door handle attempt signals (IDs 6990-6991, main harness segment).
    // Momentary bool (0=idle, 1=handle pulled this tick).
    bool          driver_door_handle_driver         = false;
    bool          driver_door_handle_passenger      = false;
    std::int8_t   driver_door_handle_driver_pub     = -1;
    std::int8_t   driver_door_handle_passenger_pub  = -1;

    // Motor state (IDs 4070-4072, chassis segment).
    // Publish-on-change with small epsilon thresholds.
    float         motor_rpm               = 0.0f;
    float         motor_torque_nm         = 0.0f;
    float         motor_current_a         = 0.0f;
    float         motor_rpm_pub           = -9999.0f;   // sentinel: always publish first
    float         motor_torque_pub        = -9999.0f;
    float         motor_current_pub       = -9999.0f;

    // Ambient environment sensors (IDs 4090-4091, chassis segment).
    // Publish-on-change with small epsilon thresholds.
    float         ambient_temp_c          = 18.0f;
    float         ambient_humidity_pct    = 55.0f;
    float         ambient_temp_pub        = -9999.0f;   // sentinel: always publish first
    float         ambient_humidity_pub    = -9999.0f;

    // Brake master cylinder pressure (ID 4074, chassis segment, kPa).
    // Computed by BrakePedal from pedal travel.  Publish-on-change.
    float         brake_master_pressure_kpa     = 0.0f;
    float         brake_master_pressure_pub_kpa = -9999.0f;  // sentinel

    // Throttle command (ID 4073, chassis segment) — received from PIM.
    // 0xFF = never received; valid values 0..255 q8.  last_update_ns tracks
    // freshness for the stale-fallback path in SimApp::ApplyElectronicsThrottle.
    std::uint8_t  throttle_cmd_q8         = 0xFFu;
    bool          has_throttle_cmd        = false;
    std::uint64_t throttle_cmd_ns         = 0;

    // Steering command (ID 4076, chassis segment) — received from electricsim.
    // float32 normalized -1..+1 (positive = left).  Freshness tracked like
    // throttle for the stale-fallback in SimApp::ApplyElectronicsSteering.
    float         steering_cmd            = 0.0f;
    bool          has_steering_cmd        = false;
    std::uint64_t steering_cmd_ns         = 0;

    // Wiper motor command (ID 4080, chassis segment) — received from RHJB.
    // 0xFF = never received; valid values 0=OFF, 1=INT, 2=LOW, 3=HIGH.
    std::uint8_t  wiper_motor_cmd         = 0xFFu;
    bool          has_wiper_motor_cmd     = false;
    // Washer pump command (ID 4081, chassis segment) — received from RHJB.
    // 0=idle, 1=pump active.
    bool          washer_pump_cmd         = false;
    bool          has_washer_pump_cmd     = false;

    // HVAC blower level (ID 4082, chassis segment) — received from HTCM.
    // uint8: 0=OFF, 1=LOW, 2=MED, 3=HIGH.  0xFF = never received.
    std::uint8_t  hvac_blower_level       = 0xFFu;
    bool          has_hvac_blower_level   = false;
    // HVAC defrost grid active (ID 4083, chassis segment) — received from HTCM.
    // bool: false=inactive, true=active.
    bool          hvac_defrost_grid_active     = false;
    bool          has_hvac_defrost_grid_active = false;

    // IPC LCD seatbelt telltales (IDs 4130/4131, chassis segment) — received from IPC.
    // bool: false=lamp off, true=lamp on.  Lit when seat unbuckled AND speed > ~8 km/h.
    bool          ipc_seatbelt_telltale_driver         = false;
    bool          has_ipc_seatbelt_telltale_driver     = false;
    bool          ipc_seatbelt_telltale_passenger      = false;
    bool          has_ipc_seatbelt_telltale_passenger  = false;

    // IPC trip distance (ID 4132, chassis segment) — received from IPC.
    // float32 LE, metres.  Published on change (epsilon ~0.5 m).  Resets to 0
    // on trip-reset button press.  -1.0f = never received (sentinel).
    float         ipc_trip_distance_m      = -1.0f;
    bool          has_ipc_trip_distance_m  = false;

    // IPC BTCM / airbag telltales (IDs 4134–4138, chassis segment) — received from IPC.
    // bool: false=lamp off, true=lamp on.
    bool          ipc_brake_telltale              = false;
    bool          has_ipc_brake_telltale          = false;
    bool          ipc_park_brake_telltale         = false;
    bool          has_ipc_park_brake_telltale     = false;
    bool          ipc_antilock_telltale           = false;
    bool          has_ipc_antilock_telltale       = false;
    bool          ipc_low_trac_telltale           = false;
    bool          has_ipc_low_trac_telltale       = false;
    bool          ipc_air_bag_telltale            = false;
    bool          has_ipc_air_bag_telltale        = false;

    // BPM pack voltage (ID 4139, chassis segment) — received from BPM.
    // uint32 LE, millivolts.  0 = sentinel "never received"; valid range 0..~360 000 mV.
    std::uint32_t bpm_pack_voltage_mv     = 0u;
    bool          has_bpm_pack_voltage    = false;

    // Auto Disconnect HV status (IDs 5224/5225 bool, 5230 uint32 enum; main
    // harness segment) — received from the AD controller, published on
    // change. Lets scenarios observe the HV power-up sequence (precharge →
    // main contactor) and its failure modes.
    bool          ad_main_contactor       = false;
    bool          has_ad_main_contactor   = false;
    bool          ad_precharge_relay      = false;
    bool          has_ad_precharge_relay  = false;
    std::uint32_t ad_state_enum           = 0u;
    bool          has_ad_state_enum       = false;

    // IPC extra LCD telltales (IDs 4140–4145, chassis segment) — received from IPC.
    // bool: false=lamp off, true=lamp on.
    bool          ipc_service_now_telltale            = false;
    bool          has_ipc_service_now_telltale        = false;
    bool          ipc_check_messages_telltale         = false;
    bool          has_ipc_check_messages_telltale     = false;
    bool          ipc_temp_telltale                   = false;
    bool          has_ipc_temp_telltale               = false;
    bool          ipc_battery_life_telltale           = false;
    bool          has_ipc_battery_life_telltale       = false;
    bool          ipc_reduced_perf_telltale           = false;
    bool          has_ipc_reduced_perf_telltale       = false;
    bool          ipc_check_tire_press_telltale       = false;
    bool          has_ipc_check_tire_press_telltale   = false;

    // Door lock commands (IDs 4084/4085, chassis segment) — received from RSA.
    // 0=unlocked, 1=locked.  0xFF = never received.
    std::uint8_t  door_lock_cmd[2]        = {0xFFu, 0xFFu};  // [0]=driver, [1]=passenger
    bool          has_door_lock_cmd[2]    = {false, false};

    // Power window motor commands (IDs 4086/4087, chassis segment) — received from RSA.
    // 0=stop, 1=up, 2=down.  0xFF = never received.
    std::uint8_t  pw_motor_cmd[2]         = {0xFFu, 0xFFu};  // [0]=driver, [1]=passenger
    bool          has_pw_motor_cmd[2]     = {false, false};

    // RSA shift-blocked cue (ID 4088, chassis segment) — received from RSA.
    // bool: true = shift refused this tick (P→non-P, brake not pressed); false = no block.
    // Level signal: remains true while the driver holds a non-PARK request without brake.
    bool          rsa_shift_blocked       = false;
    bool          has_rsa_shift_blocked   = false;

    // Door-lock motor leg drives (IDs 4092-4095, chassis segment) — received from RHJB.
    // dual-H-bridge: [0]=LH lock, [1]=LH unlock, [2]=RH lock, [3]=RH unlock.
    // bool: true = leg energised.  Consumed by ev1sim::DoorLockMotor ×2.
    bool          door_lock_motor_drive[4]     = {};
    bool          has_door_lock_motor_drive[4] = {};

    // Sounder / piezo drive (ID 4096, chassis segment) — received from LHJB flasher.
    // bool: true = piezo energised.  Consumed by ev1sim::Sounder.
    bool          sounder_piezo_drive     = false;
    bool          has_sounder_piezo_drive = false;

    // Power-steering pump speed command (ID 4097, chassis segment) — received from PSCM.
    // uint8 q8: 0=stopped, 255=full.  0xFF = never received.  Consumed by
    // ev1sim::PowerSteeringPumpMotor.
    std::uint8_t  steering_pump_speed_cmd_q8 = 0xFFu;
    bool          has_steering_pump_speed_cmd = false;
    // Power-steering pump HV interlock-closed (ID 4098, chassis segment) — published to PSCM.
    // bool: true = molex.D/E loop closed (motor present).  Publish-on-change.
    bool          steering_pump_interlock_closed = true;   // motor present → loop closed
    std::int8_t   steering_pump_interlock_pub    = -1;     // -1 forces first publish

    // HVAC driver controls (IDs 4124-4128, chassis segment) — published to HTCM.
    // Publish-on-change; defaults match ev1sim::HvacControls.
    float         hvac_temp_setpoint_c     = 21.0f;
    std::uint8_t  hvac_fan_request         = 0u;     // OFF
    std::uint8_t  hvac_mode_request        = 0u;     // FACE
    bool          hvac_ac_request          = false;
    bool          hvac_defrost_request     = false;
    float         hvac_temp_setpoint_pub   = -9999.0f;  // sentinel: force first publish
    std::int8_t   hvac_fan_request_pub     = -1;
    std::int8_t   hvac_mode_request_pub    = -1;
    std::int8_t   hvac_ac_request_pub      = -1;
    std::int8_t   hvac_defrost_request_pub = -1;

    // Door lock STATE feedback (IDs 4155-4157, chassis segment) — published to electricsim.
    // 0=unlocked, 1=locked.  Mirror of ev1sim::DoorLocks; publish-on-change.
    bool          door_lock_state_driver       = false;   // default UNLOCKED (DoorLocks default)
    bool          door_lock_state_passenger     = false;
    bool          door_lock_state_trunk         = false;
    std::int8_t   door_lock_state_driver_pub    = -1;      // -1 forces first publish
    std::int8_t   door_lock_state_passenger_pub = -1;
    std::int8_t   door_lock_state_trunk_pub     = -1;

    // RSA run-mode broadcast (ID 5711, main harness segment).
    // Subscribed from RSA; 0xFF = never received.
    std::uint8_t  rsa_run_mode            = 0xFFu;
    bool          has_rsa_run_mode        = false;

    // PIM cruise-control state (IDs 5360/5361, main harness segment).
    // Subscribed from PIM.  cruise_active: false = OFF/STANDBY, true = ACTIVE.
    // cruise_setpoint_mps: target speed in m/s; 0.0 only when state == OFF.
    bool          pim_cruise_active            = false;
    bool          has_pim_cruise_active        = false;
    float         pim_cruise_setpoint_mps      = 0.0f;
    bool          has_pim_cruise_setpoint_mps  = false;

    // BTCM front ABS solenoid states (IDs 5010-5013, main harness segment).
    // Subscribed from BTCM.  Cached as persistent commanded-state — the
    // BTCM publishes on-change only, so a sustained HOLD or DUMP phase
    // keeps the last-known iso/dump values authoritative even when no
    // new delta has arrived in a while.  Liveness is tracked separately
    // via btcm_uart_frame_ns (the 5 Hz canonical-frame heartbeat).  See
    // electricsim docs/btcm_deferred_todos.md §8 for the mu_jump
    // regression that motivated this split.
    bool          sol_fl_iso              = false;
    bool          sol_fl_dmp             = false;
    bool          sol_fr_iso              = false;
    bool          sol_fr_dmp             = false;
    std::uint64_t sol_fl_iso_ns          = 0;  // monotonic ns of last update
    std::uint64_t sol_fl_dmp_ns          = 0;
    std::uint64_t sol_fr_iso_ns          = 0;
    std::uint64_t sol_fr_dmp_ns          = 0;

    // BTCM canonical-frame heartbeat (kSigBtcmUartFrame = 5050).  The
    // BTCM supervisor broadcasts this at 5 Hz (200 ms cadence per the
    // BTCM_UART_FRAME_BROADCAST_PERIOD_MS contract) regardless of ABS
    // modulation state.  Used as the "BTCM alive" check, separate
    // from the iso/dump-pin state which encodes commanded valve
    // position rather than liveness.  0 = never received (BTCM never
    // came up).
    std::uint64_t btcm_uart_frame_ns      = 0;

    // Rear EMB motor commands (IDs 5014-5015, main harness segment).
    // Float in [-1, +1]: +1=apply, 0=idle, -1=release.  Freshness tracked
    // via last-update timestamps so the consumer can fall back if BTCM is
    // not connected (mirrors the front ABS pattern).
    float         rear_motor_lr           = 0.0f;
    float         rear_motor_rr           = 0.0f;
    std::uint64_t rear_motor_lr_ns        = 0;
    std::uint64_t rear_motor_rr_ns        = 0;

    // BTCM per-wheel actuator state on the chassis bus (IDs 4147-4154).
    // Parallel to the main-harness 5010-5015 publications above.  When
    // the chassis-bus value has ever been observed for a given wheel
    // (chassis_*_ns != 0), the AbsPhaseFront / RearEmbCmd consumers
    // prefer it over the main-harness equivalent.  Liveness is still
    // gated by the BTCM canonical-frame heartbeat (5050) because the
    // chassis-bus signals are also on-change publications.
    //
    // 4153/4154 (cylinder pressure FL/FR, kPa) have no main-harness
    // equivalent; they are exposed via GetFrontWheelCylinderPressuresKpa.
    bool          chassis_btcm_iso_close_fl    = false;
    bool          chassis_btcm_iso_close_fr    = false;
    bool          chassis_btcm_dump_open_fl    = false;
    bool          chassis_btcm_dump_open_fr    = false;
    float         chassis_btcm_emb_motor_lr    = 0.0f;
    float         chassis_btcm_emb_motor_rr    = 0.0f;
    float         chassis_btcm_cyl_press_fl_kpa = 0.0f;
    float         chassis_btcm_cyl_press_fr_kpa = 0.0f;
    std::uint64_t chassis_btcm_iso_close_fl_ns = 0;
    std::uint64_t chassis_btcm_iso_close_fr_ns = 0;
    std::uint64_t chassis_btcm_dump_open_fl_ns = 0;
    std::uint64_t chassis_btcm_dump_open_fr_ns = 0;
    std::uint64_t chassis_btcm_emb_motor_lr_ns = 0;
    std::uint64_t chassis_btcm_emb_motor_rr_ns = 0;
    std::uint64_t chassis_btcm_cyl_press_fl_ns = 0;
    std::uint64_t chassis_btcm_cyl_press_fr_ns = 0;

    // Charge coupler presence (ID 4060, chassis segment).
    // Stubbed false; future floating-UI panel or charge-door animation updates this.
    bool          charge_coupler_present     = false;
    std::int8_t   charge_coupler_present_pub = -1;   // -1 forces first publish

    // PRND selector lines (IDs 4050-4053, chassis segment).
    // Wire-level booleans encoding the 4-bit Gray-coded PRND pattern.
    // Default: PARK (A=0, B=1, C=1, D=0).
    bool prnd_a     = false;
    bool prnd_b     = true;
    bool prnd_c     = true;
    bool prnd_d     = false;
    std::int8_t prnd_a_pub = -1;   // -1 forces first publish
    std::int8_t prnd_b_pub = -1;
    std::int8_t prnd_c_pub = -1;
    std::int8_t prnd_d_pub = -1;

    // Timers (sim_time_s based).
    double next_presence_time  = 0.0;
    double next_reconnect_time = 0.0;
    // Heartbeat re-publish for brake-state signals.  ev1sim publishes
    // these on change only — so a controller that connects to the bus
    // *after* the brake-pedal edge (e.g. BTCM startup hits the chassis
    // bus 100 ms after the driver presses the pedal) never sees the
    // transition and runs as if the pedal stays at 0 forever.
    // Re-emitting every 200 ms guarantees a late-joining consumer
    // picks up the current state within one heartbeat interval.
    double next_brake_heartbeat = 0.0;

#if EV1SIM_HAVE_EXTERNAL_SIM
    std::unique_ptr<electricsim::io::SharedMemoryTransport> transport;
    std::unique_ptr<electricsim::io::SharedMemoryTransport> main_transport;
    std::uint64_t sequence      = 1;
    std::uint64_t main_sequence = 1;
    double next_main_reconnect_time = 0.0;
#endif
};

ExternalSimConnector::ExternalSimConnector()
    : ExternalSimConnector(Options{}) {}

ExternalSimConnector::ExternalSimConnector(const Options& options)
    : m_opts(options), m_state(std::make_unique<State>()) {
    if (!m_opts.enabled) {
        m_state->status = Status::Disabled;
    } else {
#if EV1SIM_HAVE_EXTERNAL_SIM
        m_state->status = Status::Connecting;
#else
        m_state->status = Status::Unavailable;
        std::cerr << "[ExternalSimConnector] --external-sim is on, but this "
                     "build has no electricsim support; commands will be "
                     "ignored.\n";
#endif
    }
}

ExternalSimConnector::~ExternalSimConnector() = default;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void ExternalSimConnector::Start() {
    if (!m_opts.enabled) return;
#if EV1SIM_HAVE_EXTERNAL_SIM
    if (m_state->status == Status::Connected) return;
    m_state->status = Status::Connecting;
    m_state->next_reconnect_time = 0.0;   // try on the next Tick
    m_state->next_presence_time  = 0.0;
#endif
}

void ExternalSimConnector::Stop() {
#if EV1SIM_HAVE_EXTERNAL_SIM
    m_state->transport.reset();
    m_state->main_transport.reset();
    m_state->status = m_opts.enabled ? Status::Connecting : Status::Disabled;
#else
    m_state->status = m_opts.enabled ? Status::Unavailable : Status::Disabled;
#endif
}

ExternalSimConnector::Status ExternalSimConnector::GetStatus() const {
    return m_state->status;
}

const char* ExternalSimConnector::StatusString() const {
    switch (m_state->status) {
        case Status::Disabled:    return "disabled";
        case Status::Unavailable: return "unavailable";
        case Status::Connecting:  return "connecting";
        case Status::Connected:   return "connected";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Endpoint registry
// ---------------------------------------------------------------------------
const ExternalSimConnector::Endpoint* ExternalSimConnector::Endpoints() {
    return EndpointTable().data();
}

int ExternalSimConnector::EndpointCount() {
    return static_cast<int>(EndpointTable().size());
}

const ExternalSimConnector::Endpoint*
ExternalSimConnector::FindEndpoint(std::uint32_t signal_id) {
    for (const auto& e : EndpointTable()) {
        if (e.signal_id == signal_id) return &e;
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// State accessors
// ---------------------------------------------------------------------------
bool ExternalSimConnector::GetBulbCmd(LightID id) const {
    int idx = static_cast<int>(id);
    if (idx < 0 || idx >= NUM_LIGHTS) return false;
    return m_state->bulb[idx];
}

bool ExternalSimConnector::GetHornLowCmd()  const { return m_state->horn_low;  }
bool ExternalSimConnector::GetHornHighCmd() const { return m_state->horn_high; }

bool ExternalSimConnector::HasReceivedBulbData() const {
    return m_state->received_any_bulb;
}

ExternalSimConnector::ThrottleCmd ExternalSimConnector::GetThrottleCmd(
    std::chrono::milliseconds freshness_window) const {
    ThrottleCmd r{};
    r.q8            = m_state->throttle_cmd_q8;
    r.ever_received = m_state->has_throttle_cmd;
    if (!r.ever_received) {
        r.fresh = false;
        return r;
    }
    const auto now_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    const auto window_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(freshness_window).count());
    // Strict '<' so a zero-length window is always stale (deterministic even
    // when the read lands in the same steady_clock tick as the last update).
    r.fresh = (now_ns - m_state->throttle_cmd_ns) < window_ns;
    return r;
}

ExternalSimConnector::SteeringCmd ExternalSimConnector::GetSteeringCmd(
    std::chrono::milliseconds freshness_window) const {
    SteeringCmd r{};
    r.value         = m_state->steering_cmd;
    r.ever_received = m_state->has_steering_cmd;
    if (!r.ever_received) {
        r.fresh = false;
        return r;
    }
    const auto now_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    const auto window_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(freshness_window).count());
    // Strict '<' so a zero-length window is always stale (deterministic even
    // when the read lands in the same steady_clock tick as the last update).
    r.fresh = (now_ns - m_state->steering_cmd_ns) < window_ns;
    return r;
}

std::uint8_t ExternalSimConnector::GetWiperMotorCommand() const {
    return m_state->wiper_motor_cmd;
}
bool ExternalSimConnector::HasReceivedWiperMotorCommand() const {
    return m_state->has_wiper_motor_cmd;
}
bool ExternalSimConnector::GetWasherPumpCommand() const {
    return m_state->washer_pump_cmd;
}
bool ExternalSimConnector::HasReceivedWasherPumpCommand() const {
    return m_state->has_washer_pump_cmd;
}

std::uint8_t ExternalSimConnector::GetDoorLockCmd(int side) const {
    if (side < 0 || side > 1) return 0xFFu;
    return m_state->door_lock_cmd[side];
}
bool ExternalSimConnector::HasReceivedDoorLockCmd(int side) const {
    if (side < 0 || side > 1) return false;
    return m_state->has_door_lock_cmd[side];
}

std::uint8_t ExternalSimConnector::GetPowerWindowMotor(int side) const {
    if (side < 0 || side > 1) return 0xFFu;
    return m_state->pw_motor_cmd[side];
}
bool ExternalSimConnector::HasReceivedPowerWindowMotor(int side) const {
    if (side < 0 || side > 1) return false;
    return m_state->has_pw_motor_cmd[side];
}

std::uint8_t ExternalSimConnector::GetHvacBlowerLevel() const {
    return m_state->hvac_blower_level;
}
bool ExternalSimConnector::HasReceivedHvacBlowerLevel() const {
    return m_state->has_hvac_blower_level;
}

bool ExternalSimConnector::GetDefrostGridActive() const {
    return m_state->hvac_defrost_grid_active;
}
bool ExternalSimConnector::HasReceivedDefrostGridActive() const {
    return m_state->has_hvac_defrost_grid_active;
}

bool ExternalSimConnector::GetRsaShiftBlocked() const {
    return m_state->rsa_shift_blocked;
}
bool ExternalSimConnector::HasReceivedRsaShiftBlocked() const {
    return m_state->has_rsa_shift_blocked;
}

bool ExternalSimConnector::GetDoorLockMotorDrive(int leg) const {
    if (leg < 0 || leg > 3) return false;
    return m_state->door_lock_motor_drive[leg];
}
bool ExternalSimConnector::HasReceivedDoorLockMotorDrive(int leg) const {
    if (leg < 0 || leg > 3) return false;
    return m_state->has_door_lock_motor_drive[leg];
}

bool ExternalSimConnector::GetSounderPiezoDrive() const {
    return m_state->sounder_piezo_drive;
}
bool ExternalSimConnector::HasReceivedSounderPiezoDrive() const {
    return m_state->has_sounder_piezo_drive;
}

std::uint8_t ExternalSimConnector::GetSteeringPumpSpeedCmdQ8() const {
    return m_state->steering_pump_speed_cmd_q8;
}
bool ExternalSimConnector::HasReceivedSteeringPumpSpeedCmd() const {
    return m_state->has_steering_pump_speed_cmd;
}

void ExternalSimConnector::SetSteeringPumpInterlockClosed(bool closed) {
    m_state->steering_pump_interlock_closed = closed;
}

void ExternalSimConnector::SetHvacTempSetpointC(float setpoint_c) {
    // Sanitize at the boundary: a non-finite or out-of-range setpoint would
    // both violate the wire contract and break the publish-on-change sentinel
    // (a value <= -9000 would collide with the "never published" sentinel and
    // re-publish every tick).  Clamp to the cabin range (mirrors HvacControls).
    if (!std::isfinite(setpoint_c)) setpoint_c = kHvacSetpointDefaultC;
    if (setpoint_c < kHvacSetpointMinC) setpoint_c = kHvacSetpointMinC;
    if (setpoint_c > kHvacSetpointMaxC) setpoint_c = kHvacSetpointMaxC;
    m_state->hvac_temp_setpoint_c = setpoint_c;
}
void ExternalSimConnector::SetHvacFanRequest(std::uint8_t level) {
    // Clamp to the 0..3 enum: a value > 127 would overflow the int8 published
    // cache and re-publish every tick.
    m_state->hvac_fan_request = (level > kHvacFanMax) ? kHvacFanMax : level;
}
void ExternalSimConnector::SetHvacModeRequest(std::uint8_t mode) {
    m_state->hvac_mode_request = (mode > kHvacModeMax) ? kHvacModeMax : mode;
}
void ExternalSimConnector::SetHvacAcRequest(bool on) {
    m_state->hvac_ac_request = on;
}
void ExternalSimConnector::SetHvacDefrostRequest(bool on) {
    m_state->hvac_defrost_request = on;
}

void ExternalSimConnector::SetDoorLockState(bool driver_locked,
                                            bool passenger_locked,
                                            bool trunk_locked) {
    m_state->door_lock_state_driver    = driver_locked;
    m_state->door_lock_state_passenger = passenger_locked;
    m_state->door_lock_state_trunk     = trunk_locked;
}

bool ExternalSimConnector::GetIpcSeatbeltTelltaleDriver() const {
    return m_state->ipc_seatbelt_telltale_driver;
}
bool ExternalSimConnector::HasReceivedIpcSeatbeltTelltaleDriver() const {
    return m_state->has_ipc_seatbelt_telltale_driver;
}

bool ExternalSimConnector::GetIpcSeatbeltTelltalePassenger() const {
    return m_state->ipc_seatbelt_telltale_passenger;
}
bool ExternalSimConnector::HasReceivedIpcSeatbeltTelltalePassenger() const {
    return m_state->has_ipc_seatbelt_telltale_passenger;
}

float ExternalSimConnector::GetIpcTripDistanceM() const {
    return m_state->ipc_trip_distance_m;
}
bool ExternalSimConnector::HasReceivedIpcTripDistance() const {
    return m_state->has_ipc_trip_distance_m;
}

bool ExternalSimConnector::GetIpcBrakeTelltale() const {
    return m_state->ipc_brake_telltale;
}
bool ExternalSimConnector::HasReceivedIpcBrakeTelltale() const {
    return m_state->has_ipc_brake_telltale;
}

bool ExternalSimConnector::GetIpcParkBrakeTelltale() const {
    return m_state->ipc_park_brake_telltale;
}
bool ExternalSimConnector::HasReceivedIpcParkBrakeTelltale() const {
    return m_state->has_ipc_park_brake_telltale;
}

bool ExternalSimConnector::GetIpcAntilockTelltale() const {
    return m_state->ipc_antilock_telltale;
}
bool ExternalSimConnector::HasReceivedIpcAntilockTelltale() const {
    return m_state->has_ipc_antilock_telltale;
}

bool ExternalSimConnector::GetIpcLowTracTelltale() const {
    return m_state->ipc_low_trac_telltale;
}
bool ExternalSimConnector::HasReceivedIpcLowTracTelltale() const {
    return m_state->has_ipc_low_trac_telltale;
}

bool ExternalSimConnector::GetIpcAirBagTelltale() const {
    return m_state->ipc_air_bag_telltale;
}
bool ExternalSimConnector::HasReceivedIpcAirBagTelltale() const {
    return m_state->has_ipc_air_bag_telltale;
}

bool ExternalSimConnector::GetIpcServiceNowTelltale() const {
    return m_state->ipc_service_now_telltale;
}
bool ExternalSimConnector::HasReceivedIpcServiceNowTelltale() const {
    return m_state->has_ipc_service_now_telltale;
}

bool ExternalSimConnector::GetIpcCheckMessagesTelltale() const {
    return m_state->ipc_check_messages_telltale;
}
bool ExternalSimConnector::HasReceivedIpcCheckMessagesTelltale() const {
    return m_state->has_ipc_check_messages_telltale;
}

bool ExternalSimConnector::GetIpcTempTelltale() const {
    return m_state->ipc_temp_telltale;
}
bool ExternalSimConnector::HasReceivedIpcTempTelltale() const {
    return m_state->has_ipc_temp_telltale;
}

bool ExternalSimConnector::GetIpcBatteryLifeTelltale() const {
    return m_state->ipc_battery_life_telltale;
}
bool ExternalSimConnector::HasReceivedIpcBatteryLifeTelltale() const {
    return m_state->has_ipc_battery_life_telltale;
}

bool ExternalSimConnector::GetIpcReducedPerfTelltale() const {
    return m_state->ipc_reduced_perf_telltale;
}
bool ExternalSimConnector::HasReceivedIpcReducedPerfTelltale() const {
    return m_state->has_ipc_reduced_perf_telltale;
}

bool ExternalSimConnector::GetIpcCheckTirePressTelltale() const {
    return m_state->ipc_check_tire_press_telltale;
}
bool ExternalSimConnector::HasReceivedIpcCheckTirePressTelltale() const {
    return m_state->has_ipc_check_tire_press_telltale;
}

bool ExternalSimConnector::GetPimCruiseActive() const {
    return m_state->pim_cruise_active;
}
bool ExternalSimConnector::HasReceivedPimCruiseActive() const {
    return m_state->has_pim_cruise_active;
}

float ExternalSimConnector::GetPimCruiseSetpointMps() const {
    return m_state->pim_cruise_setpoint_mps;
}
bool ExternalSimConnector::HasReceivedPimCruiseSetpointMps() const {
    return m_state->has_pim_cruise_setpoint_mps;
}

std::uint32_t ExternalSimConnector::GetBpmPackVoltageMv() const {
    return m_state->bpm_pack_voltage_mv;
}
bool ExternalSimConnector::HasReceivedBpmPackVoltage() const {
    return m_state->has_bpm_pack_voltage;
}

bool ExternalSimConnector::GetAdMainContactorClosed() const {
    return m_state->ad_main_contactor;
}
bool ExternalSimConnector::HasReceivedAdMainContactor() const {
    return m_state->has_ad_main_contactor;
}
bool ExternalSimConnector::GetAdPrechargeRelayClosed() const {
    return m_state->ad_precharge_relay;
}
bool ExternalSimConnector::HasReceivedAdPrechargeRelay() const {
    return m_state->has_ad_precharge_relay;
}
std::uint32_t ExternalSimConnector::GetAdStateEnum() const {
    return m_state->ad_state_enum;
}
bool ExternalSimConnector::HasReceivedAdStateEnum() const {
    return m_state->has_ad_state_enum;
}

float ExternalSimConnector::GetVehicleSpeedMps() const {
    return m_state->has_vstate
               ? static_cast<float>(m_state->vstate.speed_mps)
               : -1.0f;
}
bool ExternalSimConnector::HasVehicleSpeed() const {
    return m_state->has_vstate;
}

float ExternalSimConnector::GetSteeringAngleRad() const {
    return m_state->has_vstate
               ? static_cast<float>(m_state->vstate.steering_angle)
               : 0.0f;
}
bool ExternalSimConnector::HasVehicleState() const {
    return m_state->has_vstate;
}

void ExternalSimConnector::SetPanelSensor(PanelID panel, bool ajar) {
    int idx = static_cast<int>(panel);
    if (idx < 0 || idx >= VehiclePanels::NUM_PANELS) return;
    m_state->panel[idx] = ajar;
}

bool ExternalSimConnector::GetPanelSensor(PanelID panel) const {
    int idx = static_cast<int>(panel);
    if (idx < 0 || idx >= VehiclePanels::NUM_PANELS) return false;
    return m_state->panel[idx];
}

void ExternalSimConnector::SetVehicleState(const VehicleState& state) {
    m_state->vstate    = state;
    m_state->has_vstate = true;
}

void ExternalSimConnector::SetCombSwOutputs(bool low_beam, bool flash_to_pass, bool park_headlamp) {
    m_state->comb_sw_low_beam      = low_beam;
    m_state->comb_sw_flash_to_pass = flash_to_pass;
    m_state->comb_sw_park_headlamp = park_headlamp;
}

void ExternalSimConnector::SetTurnHazSwOutputs(bool right_turn, bool left_turn,
                                               bool hazard, bool horn) {
    m_state->turn_haz_right_turn = right_turn;
    m_state->turn_haz_left_turn  = left_turn;
    m_state->turn_haz_hazard     = hazard;
    m_state->turn_haz_horn       = horn;
}

void ExternalSimConnector::SetWiperWasherSwOutputs(bool delay, bool request,
                                                   bool hi, bool washer) {
    m_state->wiper_delay   = delay;
    m_state->wiper_request = request;
    m_state->wiper_hi      = hi;
    m_state->wiper_washer  = washer;
}

void ExternalSimConnector::SetDriverBrakePedalQ8(std::uint8_t q8) {
    m_state->driver_brake_q8 = q8;
}

void ExternalSimConnector::SetDriverThrottleQ8(std::uint8_t q8) {
    m_state->driver_throttle_q8 = q8;
}

void ExternalSimConnector::SetSuppressThrottlePublish(bool suppress) {
    m_state->suppress_throttle_publish = suppress;
}

void ExternalSimConnector::SetDriverSteeringDegQ8(std::int16_t q8) {
    m_state->driver_steering_q8 = q8;
}

void ExternalSimConnector::SetDriverGearSelector(std::uint8_t enum_v) {
    m_state->driver_gear = enum_v;
}

void ExternalSimConnector::SetDriverBrakeSwitch(bool pressed) {
    m_state->driver_brake_switch = pressed;
}

void ExternalSimConnector::SetDriverSeatbeltBuckled(bool buckled) {
    m_state->driver_seatbelt_buckled = buckled;
}

void ExternalSimConnector::SetDriverSeatbeltBuckledPassenger(bool buckled) {
    m_state->driver_seatbelt_buckled_passenger = buckled;
}

void ExternalSimConnector::SetDriverTurnSignalLeft(bool active) {
    m_state->driver_turn_left = active;
}

void ExternalSimConnector::SetDriverTurnSignalRight(bool active) {
    m_state->driver_turn_right = active;
}

void ExternalSimConnector::SetDriverHazardRequest(bool on) {
    m_state->driver_hazard = on;
}

void ExternalSimConnector::SetChargeCouplerPresent(bool present) {
    m_state->charge_coupler_present = present;
}

void ExternalSimConnector::SetPrndSelector(bool a, bool b, bool c, bool d) {
    m_state->prnd_a = a;
    m_state->prnd_b = b;
    m_state->prnd_c = c;
    m_state->prnd_d = d;
}

void ExternalSimConnector::SetDriverRsaKeypadButton1(std::uint8_t value) {
    m_state->driver_rsa_buttons[0] = value;
}

void ExternalSimConnector::SetDriverRsaKeypadButton2(std::uint8_t value) {
    m_state->driver_rsa_buttons[1] = value;
}

void ExternalSimConnector::SetDriverRsaKeypadButton3(std::uint8_t value) {
    m_state->driver_rsa_buttons[2] = value;
}

void ExternalSimConnector::SetDriverRsaKeypadButton4(std::uint8_t value) {
    m_state->driver_rsa_buttons[3] = value;
}

void ExternalSimConnector::SetDriverRsaKeypadButton5(std::uint8_t value) {
    m_state->driver_rsa_buttons[4] = value;
}

void ExternalSimConnector::SetDriverRsaModeButton(std::uint8_t button_enum) {
    m_state->driver_rsa_mode_button = button_enum;
}

void ExternalSimConnector::SetDriverIpcTripReset(bool pressed) {
    m_state->driver_ipc_trip_reset = pressed;
}

void ExternalSimConnector::SetCruiseSetCoastContact(bool closed) {
    m_state->cruise_set_coast_contact = closed;
}

void ExternalSimConnector::SetCruiseResumeAccelContact(bool closed) {
    m_state->cruise_resume_accel_contact = closed;
}

void ExternalSimConnector::SetCruiseOnOffContact(bool closed) {
    m_state->cruise_on_off_contact = closed;
}

void ExternalSimConnector::SetDriverWiperSwitch(std::uint8_t position) {
    m_state->driver_wiper_switch = position;
}

void ExternalSimConnector::SetDriverWiperWashRequest(bool pressed) {
    m_state->driver_wiper_wash = pressed;
}

// ---------------------------------------------------------------------------
// 3D-sim contract buttons / discrete sensors (6942-6947, 6966).
// (Horn 6940/6941 removed — see SetTurnHazSwOutputs / chassis horn cavity 4046.)
// ---------------------------------------------------------------------------
void ExternalSimConnector::SetDriverHeadlightSwitch(std::uint8_t position) {
    // Clamp to the 0..3 contract range; anything else is a programmer error.
    m_state->ext_headlight_switch = (position <= 3) ? position : 0;
}
void ExternalSimConnector::SetDriverHeadlightDimRequest(bool held) {
    m_state->ext_headlight_dim_request = held;
}
void ExternalSimConnector::SetDriverTelltaleTestRequest(bool pressed) {
    m_state->ext_telltale_test_request = pressed;
}
void ExternalSimConnector::SetDriverParkBrakeSetRequest(bool pressed) {
    m_state->ext_park_brake_set_request = pressed;
}
void ExternalSimConnector::SetDriverParkBrakeReleaseRequest(bool pressed) {
    m_state->ext_park_brake_release_request = pressed;
}
void ExternalSimConnector::SetSensorKeyPosition(std::uint8_t position) {
    m_state->ext_key_position = (position <= 3) ? position : 2;
}

void ExternalSimConnector::SetDriverPowerWindowDriverUp(bool held) {
    m_state->driver_pw_driver_up = held;
}

void ExternalSimConnector::SetDriverPowerWindowDriverDown(bool held) {
    m_state->driver_pw_driver_down = held;
}

void ExternalSimConnector::SetDriverPowerWindowPassengerUp(bool held) {
    m_state->driver_pw_passenger_up = held;
}

void ExternalSimConnector::SetDriverPowerWindowPassengerDown(bool held) {
    m_state->driver_pw_passenger_down = held;
}

void ExternalSimConnector::SetDriverRsaExteriorKeypad1(std::uint8_t value) {
    m_state->driver_ext_keypad[0] = value;
}

void ExternalSimConnector::SetDriverRsaExteriorKeypad2(std::uint8_t value) {
    m_state->driver_ext_keypad[1] = value;
}

void ExternalSimConnector::SetDriverRsaExteriorKeypad3(std::uint8_t value) {
    m_state->driver_ext_keypad[2] = value;
}

void ExternalSimConnector::SetDriverRsaExteriorKeypad4(std::uint8_t value) {
    m_state->driver_ext_keypad[3] = value;
}

void ExternalSimConnector::SetDriverRsaExteriorKeypad5(std::uint8_t value) {
    m_state->driver_ext_keypad[4] = value;
}

void ExternalSimConnector::SetDriverDoorHandleAttemptDriver(bool attempted) {
    m_state->driver_door_handle_driver = attempted;
}

void ExternalSimConnector::SetDriverDoorHandleAttemptPassenger(bool attempted) {
    m_state->driver_door_handle_passenger = attempted;
}

void ExternalSimConnector::SetMotorRpm(float rpm) {
    m_state->motor_rpm = rpm;
}

void ExternalSimConnector::SetMotorTorqueNm(float torque_nm) {
    m_state->motor_torque_nm = torque_nm;
}

void ExternalSimConnector::SetMotorCurrentA(float amps) {
    m_state->motor_current_a = amps;
}

void ExternalSimConnector::SetAmbientTempC(float temp_c) {
    m_state->ambient_temp_c = temp_c;
}

void ExternalSimConnector::SetAmbientHumidityPct(float humidity_pct) {
    m_state->ambient_humidity_pct = humidity_pct;
}

void ExternalSimConnector::SetBrakeMasterPressureKpa(float pressure_kpa) {
    m_state->brake_master_pressure_kpa = pressure_kpa;
}

std::uint8_t ExternalSimConnector::GetRsaRunMode() const {
    return m_state->rsa_run_mode;
}

bool ExternalSimConnector::HasReceivedRunMode() const {
    return m_state->has_rsa_run_mode;
}

ExternalSimConnector::AbsPhaseFront ExternalSimConnector::GetAbsPhaseFront(
    std::chrono::milliseconds freshness_window) const {
    AbsPhaseFront result{};
    const auto& st = *m_state;

    const std::uint64_t window_ns =
        static_cast<std::uint64_t>(freshness_window.count()) * 1'000'000ULL;

    // NowNs() is always available (defined outside the EV1SIM_HAVE_EXTERNAL_SIM
    // guard) so freshness works correctly in both real and stub builds.
    const std::uint64_t now_ns = NowNs();

    // Freshness model — see electricsim docs/btcm_deferred_todos.md §8
    // for the full investigation.
    //
    // Liveness comes from the BTCM's 5 Hz canonical-frame heartbeat
    // (kSigBtcmUartFrame, signal 5050).  The BTCM supervisor
    // broadcasts this frame every 200 ms regardless of ABS modulation
    // state — see ev1/btcm/controller.cpp publish_supervisor_uart_frame.
    // That makes it the authoritative "BTCM is alive" signal,
    // independent of whether the algorithm has changed phase recently.
    //
    // Iso/dump pin states are *commanded state* (on-change publish),
    // not edge events.  We cache the latest values indefinitely while
    // the BTCM is alive and use them as the authoritative phase
    // decode.  No per-pin freshness check — only the per-pin
    // "have-ever-seen" gate (ts != 0) so the chassis doesn't trust
    // phase data before any has arrived.
    //
    // Source preference: BTCM publishes the same actuator state on
    // two segments — chassis bus (4147-4150) and main harness
    // (5010-5013).  We prefer chassis-bus values per-wheel: if any
    // chassis-bus update has arrived for that wheel (either iso or
    // dump timestamp != 0), the chassis source is authoritative.
    // Otherwise, fall back to the main-harness source.  This lets
    // the public chassis-bus path become primary as the producer
    // rolls out, without breaking deployments that haven't migrated.
    //
    // The freshness_window parameter is the multi-second peer-side
    // liveness tolerance (3 s per the EV1 electrical service manual
    // IPC DTC 015, the tightest spec in the corpus).  After 3 s of
    // no UART frame, the chassis declares BTCM dead and falls back
    // to local hydraulic (the manual's hydraulic-backup behavior).
    const bool btcm_alive = [&]() -> bool {
        if (st.btcm_uart_frame_ns == 0) return false;
        if (now_ns < st.btcm_uart_frame_ns) return false;  // clock-wrap guard
        return (now_ns - st.btcm_uart_frame_ns) < window_ns;
    }();

    auto pin_ever_seen = [&](std::uint64_t ts_iso, std::uint64_t ts_dmp) -> bool {
        return ts_iso != 0 && ts_dmp != 0;
    };

    auto decode_phase = [](bool iso, bool dmp) -> AbsPhaseFront::Phase {
        if (!iso && !dmp) return AbsPhaseFront::Phase::APPLY;
        if ( iso && !dmp) return AbsPhaseFront::Phase::HOLD;
        if ( iso &&  dmp) return AbsPhaseFront::Phase::DUMP;
        // iso=0, dump=1 is invalid per the ABS solenoid spec — treat as APPLY.
        return AbsPhaseFront::Phase::APPLY;
    };

    // Per-wheel source selection.  "ever-seen on chassis" wins; otherwise
    // try main-harness; otherwise stale-APPLY.
    const bool fl_chassis_seen =
        (st.chassis_btcm_iso_close_fl_ns != 0) ||
        (st.chassis_btcm_dump_open_fl_ns != 0);
    const bool fr_chassis_seen =
        (st.chassis_btcm_iso_close_fr_ns != 0) ||
        (st.chassis_btcm_dump_open_fr_ns != 0);

    const bool fl_main_seen =
        pin_ever_seen(st.sol_fl_iso_ns, st.sol_fl_dmp_ns);
    const bool fr_main_seen =
        pin_ever_seen(st.sol_fr_iso_ns, st.sol_fr_dmp_ns);

    result.fl_fresh = btcm_alive && (fl_chassis_seen || fl_main_seen);
    result.fr_fresh = btcm_alive && (fr_chassis_seen || fr_main_seen);

    auto decode_fl = [&]() -> AbsPhaseFront::Phase {
        if (fl_chassis_seen) {
            return decode_phase(st.chassis_btcm_iso_close_fl,
                                st.chassis_btcm_dump_open_fl);
        }
        if (fl_main_seen) {
            return decode_phase(st.sol_fl_iso, st.sol_fl_dmp);
        }
        return AbsPhaseFront::Phase::APPLY;
    };
    auto decode_fr = [&]() -> AbsPhaseFront::Phase {
        if (fr_chassis_seen) {
            return decode_phase(st.chassis_btcm_iso_close_fr,
                                st.chassis_btcm_dump_open_fr);
        }
        if (fr_main_seen) {
            return decode_phase(st.sol_fr_iso, st.sol_fr_dmp);
        }
        return AbsPhaseFront::Phase::APPLY;
    };

    result.fl = result.fl_fresh ? decode_fl() : AbsPhaseFront::Phase::APPLY;
    result.fr = result.fr_fresh ? decode_fr() : AbsPhaseFront::Phase::APPLY;

    return result;
}

ExternalSimConnector::RearEmbCmd ExternalSimConnector::GetRearEmbCmd(
    std::chrono::milliseconds freshness_window) const {
    RearEmbCmd r{};
    const auto& st = *m_state;
    const std::uint64_t window_ns =
        static_cast<std::uint64_t>(freshness_window.count()) * 1'000'000ULL;
    const std::uint64_t now_ns = NowNs();

    // Heartbeat-based liveness — same pattern as GetAbsPhaseFront.
    // kSigRearMotorLR/RR publish on-change only and a steady-state
    // EMB command (sustained APPLY or RELEASE) holds the same float
    // value for hundreds of ms, so the per-signal timestamp doesn't
    // refresh.  The 5 Hz kSigBtcmUartFrame heartbeat is the
    // authoritative "BTCM alive" signal; cached motor-cmd values
    // stay valid as long as the producer is alive.  See
    // electricsim docs/btcm_deferred_todos.md §8.
    //
    // Source preference (per wheel): chassis-bus value (4151/4152) wins
    // when ever-seen; otherwise main-harness value (5014/5015).  Same
    // rationale as GetAbsPhaseFront — keep both paths live so the
    // public chassis-bus subscription can become primary without
    // breaking older producers.
    const bool btcm_alive = [&]() -> bool {
        if (st.btcm_uart_frame_ns == 0) return false;
        if (now_ns < st.btcm_uart_frame_ns) return false;  // clock-wrap guard
        return (now_ns - st.btcm_uart_frame_ns) < window_ns;
    }();

    auto ever_seen = [&](std::uint64_t ts) -> bool { return ts != 0; };

    const bool lr_chassis_seen = ever_seen(st.chassis_btcm_emb_motor_lr_ns);
    const bool rr_chassis_seen = ever_seen(st.chassis_btcm_emb_motor_rr_ns);
    const bool lr_main_seen    = ever_seen(st.rear_motor_lr_ns);
    const bool rr_main_seen    = ever_seen(st.rear_motor_rr_ns);

    r.lr = lr_chassis_seen ? st.chassis_btcm_emb_motor_lr : st.rear_motor_lr;
    r.rr = rr_chassis_seen ? st.chassis_btcm_emb_motor_rr : st.rear_motor_rr;
    r.lr_fresh = btcm_alive && (lr_chassis_seen || lr_main_seen);
    r.rr_fresh = btcm_alive && (rr_chassis_seen || rr_main_seen);
    return r;
}

ExternalSimConnector::FrontWheelCylinderPressures
ExternalSimConnector::GetFrontWheelCylinderPressuresKpa(
    std::chrono::milliseconds freshness_window) const {
    FrontWheelCylinderPressures p{};
    const auto& st = *m_state;
    const std::uint64_t window_ns =
        static_cast<std::uint64_t>(freshness_window.count()) * 1'000'000ULL;
    const std::uint64_t now_ns = NowNs();

    // Liveness via the BTCM heartbeat — cylinder pressure (4153/4154)
    // is published on-change with an epsilon gate just like the
    // iso/dump signals, so per-signal staleness is not meaningful for
    // a sustained-pressure regime.
    const bool btcm_alive = [&]() -> bool {
        if (st.btcm_uart_frame_ns == 0) return false;
        if (now_ns < st.btcm_uart_frame_ns) return false;  // clock-wrap guard
        return (now_ns - st.btcm_uart_frame_ns) < window_ns;
    }();

    p.fl_kpa   = st.chassis_btcm_cyl_press_fl_kpa;
    p.fr_kpa   = st.chassis_btcm_cyl_press_fr_kpa;
    p.fl_fresh = btcm_alive && (st.chassis_btcm_cyl_press_fl_ns != 0);
    p.fr_fresh = btcm_alive && (st.chassis_btcm_cyl_press_fr_ns != 0);
    return p;
}

// ---------------------------------------------------------------------------
// Test / internal: apply an inbound signal value (as if decoded from a frame).
// ---------------------------------------------------------------------------
void ExternalSimConnector::DebugInjectDelta(std::uint32_t signal_id, bool value) {
    int lid = LightIdForBulbSignal(signal_id);
    if (lid >= 0) {
        m_state->bulb[lid] = value;
        m_state->received_any_bulb = true;
    } else if (signal_id == kHornLowCmd) {
        m_state->horn_low = value;
    } else if (signal_id == kHornHighCmd) {
        m_state->horn_high = value;
    } else if (signal_id == kSigWasherPumpCommand) {
        m_state->washer_pump_cmd     = value;
        m_state->has_washer_pump_cmd = true;
    } else if (signal_id == kSigSolFL_ISO) {
        m_state->sol_fl_iso    = value;
        m_state->sol_fl_iso_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    } else if (signal_id == kSigSolFL_DMP) {
        m_state->sol_fl_dmp    = value;
        m_state->sol_fl_dmp_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    } else if (signal_id == kSigSolFR_ISO) {
        m_state->sol_fr_iso    = value;
        m_state->sol_fr_iso_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    } else if (signal_id == kSigSolFR_DMP) {
        m_state->sol_fr_dmp    = value;
        m_state->sol_fr_dmp_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    } else if (signal_id == kSigBtcmUartFrame) {
        // Test helper: simulate a BTCM canonical-frame heartbeat arrival.
        // The bool argument is ignored — the heartbeat is purely a
        // "timestamp received" event — but the dispatch convention is
        // bool-per-signal so we accept it.  In production this is set
        // in DrainInbound when the heartbeat frame arrives.
        (void)value;
        m_state->btcm_uart_frame_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    } else if (signal_id == kSigChassisBtcmIsoCloseFL ||
               signal_id == kSigChassisBtcmIsoCloseFR ||
               signal_id == kSigChassisBtcmDumpOpenFL ||
               signal_id == kSigChassisBtcmDumpOpenFR) {
        // Chassis-bus mirror of the BTCM iso/dump solenoid state.
        // See struct State and GetAbsPhaseFront for the per-wheel
        // source-preference rule.
        const std::uint64_t now_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        switch (signal_id) {
            case kSigChassisBtcmIsoCloseFL:
                m_state->chassis_btcm_iso_close_fl    = value;
                m_state->chassis_btcm_iso_close_fl_ns = now_ns;
                break;
            case kSigChassisBtcmIsoCloseFR:
                m_state->chassis_btcm_iso_close_fr    = value;
                m_state->chassis_btcm_iso_close_fr_ns = now_ns;
                break;
            case kSigChassisBtcmDumpOpenFL:
                m_state->chassis_btcm_dump_open_fl    = value;
                m_state->chassis_btcm_dump_open_fl_ns = now_ns;
                break;
            case kSigChassisBtcmDumpOpenFR:
                m_state->chassis_btcm_dump_open_fr    = value;
                m_state->chassis_btcm_dump_open_fr_ns = now_ns;
                break;
            default: break;
        }
    } else if (signal_id == kSigDoorLockMotorLhLockDrive) {
        m_state->door_lock_motor_drive[0]     = value;
        m_state->has_door_lock_motor_drive[0] = true;
    } else if (signal_id == kSigDoorLockMotorLhUnlockDrive) {
        m_state->door_lock_motor_drive[1]     = value;
        m_state->has_door_lock_motor_drive[1] = true;
    } else if (signal_id == kSigDoorLockMotorRhLockDrive) {
        m_state->door_lock_motor_drive[2]     = value;
        m_state->has_door_lock_motor_drive[2] = true;
    } else if (signal_id == kSigDoorLockMotorRhUnlockDrive) {
        m_state->door_lock_motor_drive[3]     = value;
        m_state->has_door_lock_motor_drive[3] = true;
    } else if (signal_id == kSigSounderPiezoDrive) {
        m_state->sounder_piezo_drive     = value;
        m_state->has_sounder_piezo_drive = true;
    } else if (signal_id == kSigAdMainContactor) {
        m_state->ad_main_contactor     = value;
        m_state->has_ad_main_contactor = true;
    } else if (signal_id == kSigAdPrechargeRelay) {
        m_state->ad_precharge_relay     = value;
        m_state->has_ad_precharge_relay = true;
    }
    // Panel-sensor signals are outputs — ignore inbound.
}

void ExternalSimConnector::DebugInjectFloat(std::uint32_t signal_id, float value) {
    const std::uint64_t now_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    if (signal_id == kSigRearMotorLR) {
        m_state->rear_motor_lr    = value;
        m_state->rear_motor_lr_ns = now_ns;
    } else if (signal_id == kSigRearMotorRR) {
        m_state->rear_motor_rr    = value;
        m_state->rear_motor_rr_ns = now_ns;
    } else if (signal_id == kSigSteeringCmd) {
        m_state->steering_cmd     = value;
        m_state->has_steering_cmd = true;
        m_state->steering_cmd_ns  = now_ns;
    } else if (signal_id == kSigPimCruiseSetpointMps) {
        m_state->pim_cruise_setpoint_mps     = value;
        m_state->has_pim_cruise_setpoint_mps = true;
    } else if (signal_id == kSigIpcTripDistanceM) {
        m_state->ipc_trip_distance_m     = value;
        m_state->has_ipc_trip_distance_m = true;
    } else if (signal_id == kSigChassisBtcmEmbMotorCmdLR) {
        m_state->chassis_btcm_emb_motor_lr    = value;
        m_state->chassis_btcm_emb_motor_lr_ns = now_ns;
    } else if (signal_id == kSigChassisBtcmEmbMotorCmdRR) {
        m_state->chassis_btcm_emb_motor_rr    = value;
        m_state->chassis_btcm_emb_motor_rr_ns = now_ns;
    } else if (signal_id == kSigChassisBtcmCylPressureFL_kPa) {
        m_state->chassis_btcm_cyl_press_fl_kpa = value;
        m_state->chassis_btcm_cyl_press_fl_ns  = now_ns;
    } else if (signal_id == kSigChassisBtcmCylPressureFR_kPa) {
        m_state->chassis_btcm_cyl_press_fr_kpa = value;
        m_state->chassis_btcm_cyl_press_fr_ns  = now_ns;
    }
    // Other float signals are not currently subscribed as inputs.
}

void ExternalSimConnector::DebugInjectU8(std::uint32_t signal_id,
                                          std::uint8_t value) {
    if (signal_id == kSigThrottleCmdQ8) {
        m_state->throttle_cmd_q8 = value;
        m_state->has_throttle_cmd = true;
        m_state->throttle_cmd_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    } else if (signal_id == kSigWiperMotorCommand) {
        m_state->wiper_motor_cmd     = value;
        m_state->has_wiper_motor_cmd = true;
    } else if (signal_id == kSigWasherPumpCommand) {
        m_state->washer_pump_cmd     = (value != 0u);
        m_state->has_washer_pump_cmd = true;
    } else if (signal_id == kSigDoorLockCmdDriver) {
        m_state->door_lock_cmd[0]  = value;
        m_state->has_door_lock_cmd[0] = true;
    } else if (signal_id == kSigDoorLockCmdPassenger) {
        m_state->door_lock_cmd[1]  = value;
        m_state->has_door_lock_cmd[1] = true;
    } else if (signal_id == kSigPowerWindowMotorDriver) {
        m_state->pw_motor_cmd[0]    = value;
        m_state->has_pw_motor_cmd[0] = true;
    } else if (signal_id == kSigPowerWindowMotorPassenger) {
        m_state->pw_motor_cmd[1]    = value;
        m_state->has_pw_motor_cmd[1] = true;
    } else if (signal_id == kSigHvacBlowerLevel) {
        m_state->hvac_blower_level     = value;
        m_state->has_hvac_blower_level = true;
    } else if (signal_id == kSigDefrostGridActive) {
        m_state->hvac_defrost_grid_active     = (value != 0u);
        m_state->has_hvac_defrost_grid_active = true;
    } else if (signal_id == kSigIpcSeatbeltTelltaleDriver) {
        m_state->ipc_seatbelt_telltale_driver     = (value != 0u);
        m_state->has_ipc_seatbelt_telltale_driver = true;
    } else if (signal_id == kSigIpcSeatbeltTelltalePassenger) {
        m_state->ipc_seatbelt_telltale_passenger     = (value != 0u);
        m_state->has_ipc_seatbelt_telltale_passenger = true;
    } else if (signal_id == kSigIpcBrakeTelltale) {
        m_state->ipc_brake_telltale     = (value != 0u);
        m_state->has_ipc_brake_telltale = true;
    } else if (signal_id == kSigIpcParkBrakeTelltale) {
        m_state->ipc_park_brake_telltale     = (value != 0u);
        m_state->has_ipc_park_brake_telltale = true;
    } else if (signal_id == kSigIpcAntilockTelltale) {
        m_state->ipc_antilock_telltale     = (value != 0u);
        m_state->has_ipc_antilock_telltale = true;
    } else if (signal_id == kSigIpcLowTracTelltale) {
        m_state->ipc_low_trac_telltale     = (value != 0u);
        m_state->has_ipc_low_trac_telltale = true;
    } else if (signal_id == kSigIpcAirBagTelltale) {
        m_state->ipc_air_bag_telltale     = (value != 0u);
        m_state->has_ipc_air_bag_telltale = true;
    } else if (signal_id == kSigIpcServiceNowTelltale) {
        m_state->ipc_service_now_telltale     = (value != 0u);
        m_state->has_ipc_service_now_telltale = true;
    } else if (signal_id == kSigIpcCheckMessagesTelltale) {
        m_state->ipc_check_messages_telltale     = (value != 0u);
        m_state->has_ipc_check_messages_telltale = true;
    } else if (signal_id == kSigIpcTempTelltale) {
        m_state->ipc_temp_telltale     = (value != 0u);
        m_state->has_ipc_temp_telltale = true;
    } else if (signal_id == kSigIpcBatteryLifeTelltale) {
        m_state->ipc_battery_life_telltale     = (value != 0u);
        m_state->has_ipc_battery_life_telltale = true;
    } else if (signal_id == kSigIpcReducedPerfTelltale) {
        m_state->ipc_reduced_perf_telltale     = (value != 0u);
        m_state->has_ipc_reduced_perf_telltale = true;
    } else if (signal_id == kSigIpcCheckTirePressTelltale) {
        m_state->ipc_check_tire_press_telltale     = (value != 0u);
        m_state->has_ipc_check_tire_press_telltale = true;
    } else if (signal_id == kSigRsaShiftBlocked) {
        m_state->rsa_shift_blocked     = (value != 0u);
        m_state->has_rsa_shift_blocked = true;
    } else if (signal_id == kSigPimCruiseActive) {
        m_state->pim_cruise_active     = (value != 0u);
        m_state->has_pim_cruise_active = true;
    } else if (signal_id == kSigPscmPumpSpeedCmdQ8) {
        m_state->steering_pump_speed_cmd_q8  = value;
        m_state->has_steering_pump_speed_cmd = true;
    }
    // All other uint8 signals are not currently subscribed as inputs.
}

void ExternalSimConnector::DebugInjectU32(std::uint32_t signal_id,
                                           std::uint32_t value) {
    if (signal_id == kSigBpmPackVoltageMv) {
        m_state->bpm_pack_voltage_mv  = value;
        m_state->has_bpm_pack_voltage = true;
    } else if (signal_id == kSigAdStateEnum) {
        m_state->ad_state_enum     = value;
        m_state->has_ad_state_enum = true;
    }
    // Other uint32 signals are not currently subscribed as inputs.
}

// ---------------------------------------------------------------------------
// Tick — transport I/O (no-op in stub build)
// ---------------------------------------------------------------------------
#if EV1SIM_HAVE_EXTERNAL_SIM
namespace {

using electricsim::io::DeltaRecord;
using electricsim::io::Frame;
using electricsim::io::FrameType;
using electricsim::io::PollResult;
using electricsim::io::PollStatus;
using electricsim::io::SharedMemoryTransport;
using electricsim::io::SharedMemoryTransportOptions;
using electricsim::io::SignalEncoding;

DeltaRecord MakeBoolDelta(std::uint32_t signal_id, bool value) {
    DeltaRecord d{};
    d.signal_id = signal_id;
    d.encoding  = SignalEncoding::Unsigned;
    d.bit_width = 1;
    d.payload.push_back(value ? 1u : 0u);
    return d;
}

DeltaRecord MakeDefineDelta(std::uint32_t signal_id, const char* name) {
    DeltaRecord d{};
    d.signal_id = signal_id;
    d.encoding  = SignalEncoding::Opaque;
    d.bit_width = 8;
    const std::string s = std::string(name) + "|source:ev1sim";
    d.payload.assign(s.begin(), s.end());
    return d;
}

// IEEE 754 float32, little-endian payload.
DeltaRecord MakeFloatDelta(std::uint32_t signal_id, float value) {
    DeltaRecord d{};
    d.signal_id = signal_id;
    d.encoding  = SignalEncoding::Float;
    d.bit_width = 32;
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    d.payload.push_back(static_cast<std::uint8_t>( bits        & 0xFF));
    d.payload.push_back(static_cast<std::uint8_t>((bits >>  8) & 0xFF));
    d.payload.push_back(static_cast<std::uint8_t>((bits >> 16) & 0xFF));
    d.payload.push_back(static_cast<std::uint8_t>((bits >> 24) & 0xFF));
    return d;
}

// uint8_t single-byte unsigned payload (Q8 pedal/gear).
DeltaRecord MakeU8Delta(std::uint32_t signal_id, std::uint8_t value) {
    DeltaRecord d{};
    d.signal_id = signal_id;
    d.encoding  = SignalEncoding::Unsigned;
    d.bit_width = 8;
    d.payload.push_back(value);
    return d;
}

// int16_t two-byte signed little-endian payload (Q8 steering degrees).
DeltaRecord MakeI16Delta(std::uint32_t signal_id, std::int16_t value) {
    DeltaRecord d{};
    d.signal_id = signal_id;
    d.encoding  = SignalEncoding::Signed;
    d.bit_width = 16;
    const auto u = static_cast<std::uint16_t>(value);
    d.payload.push_back(static_cast<std::uint8_t>(u & 0xFFu));
    d.payload.push_back(static_cast<std::uint8_t>((u >> 8) & 0xFFu));
    return d;
}

// uint64_t eight-byte unsigned little-endian payload (sim-time nanoseconds).
// Matches electricsim's chassis-bus decode: sim_ns |= payload[i] << (8*i).
DeltaRecord MakeU64Delta(std::uint32_t signal_id, std::uint64_t value) {
    DeltaRecord d{};
    d.signal_id = signal_id;
    d.encoding  = SignalEncoding::Unsigned;
    d.bit_width = 64;
    for (int b = 0; b < 8; ++b)
        d.payload.push_back(static_cast<std::uint8_t>((value >> (8 * b)) & 0xFFu));
    return d;
}

// ── Cross-repo signal-ID drift guard ────────────────────────────────────────
// ev1sim re-declares the chassis-bus signal IDs it shares with electricsim
// (every "Locked in lockstep with electricsim/src/io/ev1_chassis_signals.hpp"
// constant above).  These static_asserts pin each one to electricsim's
// canonical value, so any divergence is a compile error here rather than a
// silent runtime mis-route.  The contract doc + ev1_chassis_signals.hpp stay
// the single source of truth; this only enforces it.  Add a line whenever you
// mirror a new chassis ID.  Only compiled when building against electricsim
// (EV1SIM_HAVE_EXTERNAL_SIM), so the stub build is unaffected.
#define EV1SIM_CHASSIS_ID_MATCHES(local_id, canonical)                       \
    static_assert((local_id) == electricsim::io::canonical,                  \
                  "ev1sim chassis ID " #local_id                             \
                  " drifted from electricsim::io::" #canonical)

EV1SIM_CHASSIS_ID_MATCHES(kSigMotorRpm,                  kSigChassisMotorRpm);
EV1SIM_CHASSIS_ID_MATCHES(kSigMotorTorqueNm,             kSigChassisMotorTorqueNm);
EV1SIM_CHASSIS_ID_MATCHES(kSigSimTimeNs,                 kSigChassisSimTimeNs);
EV1SIM_CHASSIS_ID_MATCHES(kSigThrottleCmdQ8,             kSigChassisThrottleCmdQ8);
EV1SIM_CHASSIS_ID_MATCHES(kSigBrakeMasterPressureKpa,    kSigChassisBrakeMasterPressureKpa);
EV1SIM_CHASSIS_ID_MATCHES(kSigWiperMotorCommand,         kSigChassisWiperMotorCommand);
EV1SIM_CHASSIS_ID_MATCHES(kSigWasherPumpCommand,         kSigChassisWasherPumpCommand);
EV1SIM_CHASSIS_ID_MATCHES(kSigHvacBlowerLevel,           kSigChassisHvacBlowerLevel);
EV1SIM_CHASSIS_ID_MATCHES(kSigDefrostGridActive,         kSigChassisDefrostGridActive);
EV1SIM_CHASSIS_ID_MATCHES(kSigDoorLockCmdDriver,         kSigChassisDoorLockCmdDriver);
EV1SIM_CHASSIS_ID_MATCHES(kSigDoorLockCmdPassenger,      kSigChassisDoorLockCmdPassenger);
EV1SIM_CHASSIS_ID_MATCHES(kSigDoorLockStateDriver,       kSigChassisDoorLockStateDriver);
EV1SIM_CHASSIS_ID_MATCHES(kSigDoorLockStatePassenger,    kSigChassisDoorLockStatePassenger);
EV1SIM_CHASSIS_ID_MATCHES(kSigDoorLockStateTrunk,        kSigChassisDoorLockStateTrunk);
EV1SIM_CHASSIS_ID_MATCHES(kSigPowerWindowMotorDriver,    kSigChassisPowerWindowMotorDriver);
EV1SIM_CHASSIS_ID_MATCHES(kSigPowerWindowMotorPassenger, kSigChassisPowerWindowMotorPassenger);
EV1SIM_CHASSIS_ID_MATCHES(kSigRsaShiftBlocked,           kSigChassisRsaShiftBlocked);
EV1SIM_CHASSIS_ID_MATCHES(kSigAmbientTempC,              kSigChassisAmbientTempC);
EV1SIM_CHASSIS_ID_MATCHES(kSigAmbientHumidityPct,        kSigChassisAmbientHumidityPct);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcSeatbeltTelltaleDriver,    kSigChassisIpcSeatbeltTelltaleDriver);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcSeatbeltTelltalePassenger, kSigChassisIpcSeatbeltTelltalePassenger);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcTripDistanceM,          kSigChassisIpcTripDistanceM);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcBrakeTelltale,          kSigChassisIpcBrakeTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcParkBrakeTelltale,      kSigChassisIpcParkBrakeTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcAntilockTelltale,       kSigChassisIpcAntilockTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcLowTracTelltale,        kSigChassisIpcLowTracTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcAirBagTelltale,         kSigChassisIpcAirBagTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigBpmPackVoltageMv,          kSigChassisBpmPackVoltageMv);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcServiceNowTelltale,     kSigChassisIpcServiceNowTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcCheckMessagesTelltale,  kSigChassisIpcCheckMessagesTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcTempTelltale,           kSigChassisIpcTempTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcBatteryLifeTelltale,    kSigChassisIpcBatteryLifeTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcReducedPerfTelltale,    kSigChassisIpcReducedPerfTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigIpcCheckTirePressTelltale, kSigChassisIpcCheckTirePressTelltale);
EV1SIM_CHASSIS_ID_MATCHES(kSigChassisBtcmIsoCloseFL,     kSigChassisBtcmIsoCloseFL);
EV1SIM_CHASSIS_ID_MATCHES(kSigChassisBtcmIsoCloseFR,     kSigChassisBtcmIsoCloseFR);
EV1SIM_CHASSIS_ID_MATCHES(kSigChassisBtcmDumpOpenFL,     kSigChassisBtcmDumpOpenFL);
EV1SIM_CHASSIS_ID_MATCHES(kSigChassisBtcmDumpOpenFR,     kSigChassisBtcmDumpOpenFR);
EV1SIM_CHASSIS_ID_MATCHES(kSigChassisBtcmEmbMotorCmdLR,  kSigChassisBtcmEmbMotorCmdLR);
EV1SIM_CHASSIS_ID_MATCHES(kSigChassisBtcmEmbMotorCmdRR,  kSigChassisBtcmEmbMotorCmdRR);
EV1SIM_CHASSIS_ID_MATCHES(kSigChassisBtcmCylPressureFL_kPa, kSigChassisBtcmCylPressureFL_kPa);
EV1SIM_CHASSIS_ID_MATCHES(kSigChassisBtcmCylPressureFR_kPa, kSigChassisBtcmCylPressureFR_kPa);
EV1SIM_CHASSIS_ID_MATCHES(kTurnHazSwRightTurnOutId, kSigTurnHazSw_RightTurnOut);
EV1SIM_CHASSIS_ID_MATCHES(kTurnHazSwLeftTurnOutId,  kSigTurnHazSw_LeftTurnOut);
EV1SIM_CHASSIS_ID_MATCHES(kTurnHazSwHazardOutId,    kSigTurnHazSw_HazardOut);
EV1SIM_CHASSIS_ID_MATCHES(kTurnHazSwHornOutId,      kSigTurnHazSw_HornOut);
EV1SIM_CHASSIS_ID_MATCHES(kWiperSwDelayOutId,        kSigWiperSw_DelayOut);
EV1SIM_CHASSIS_ID_MATCHES(kWiperSwRequestOutId,      kSigWiperSw_RequestOut);
EV1SIM_CHASSIS_ID_MATCHES(kWiperSwHiOutId,           kSigWiperSw_HiOut);
EV1SIM_CHASSIS_ID_MATCHES(kWiperSwWasherSwitchOutId, kSigWiperSw_WasherSwitchOut);
// The 4100-range dynamics block publishes by literal offset from this base.
EV1SIM_CHASSIS_ID_MATCHES(kDynamicsBase,                 kSigChassisSpeedMps);

// ── Pending electricsim adoption (allocated ev1sim-side first) ──────────────
// These chassis IDs were allocated here so electricsim's RHJB / LHJB / PSCM
// controllers can start publishing against a fixed wire contract.  They have
// no electricsim::io::kSigChassis* counterpart yet, so they intentionally do
// NOT get an EV1SIM_CHASSIS_ID_MATCHES line — adding one now would break the
// integrated (EV1SIM_HAVE_EXTERNAL_SIM) build until electricsim catches up.
// When electricsim adds the canonical constants (suggested names below), move
// each into the guard above so drift is caught from then on:
//   kSigDoorLockMotorLhLockDrive   (4092) → kSigChassisDoorLockMotorLhLockDrive
//   kSigDoorLockMotorLhUnlockDrive (4093) → kSigChassisDoorLockMotorLhUnlockDrive
//   kSigDoorLockMotorRhLockDrive   (4094) → kSigChassisDoorLockMotorRhLockDrive
//   kSigDoorLockMotorRhUnlockDrive (4095) → kSigChassisDoorLockMotorRhUnlockDrive
//   kSigSounderPiezoDrive          (4096) → kSigChassisSounderPiezoDrive
//   kSigPscmPumpSpeedCmdQ8         (4097) → kSigChassisPscmPumpSpeedCmdQ8
//   kSigPscmPumpInterlockClosed    (4098) → kSigChassisPscmPumpInterlockClosed
//   kSigHvacTempSetpointC          (4124) → kSigChassisHvacTempSetpointC
//   kSigHvacFanRequest             (4125) → kSigChassisHvacFanRequest
//   kSigHvacModeRequest            (4126) → kSigChassisHvacModeRequest
//   kSigHvacAcRequest              (4127) → kSigChassisHvacAcRequest
//   kSigHvacDefrostRequest         (4128) → kSigChassisHvacDefrostRequest
//   kSigDoorLockStateDriver        (4155) → kSigChassisDoorLockStateDriver
//   kSigDoorLockStatePassenger     (4156) → kSigChassisDoorLockStatePassenger
//   kSigDoorLockStateTrunk         (4157) → kSigChassisDoorLockStateTrunk
//   kSigSteeringCmd                (4076) → kSigChassisSteeringCmd

#undef EV1SIM_CHASSIS_ID_MATCHES

// ── Chassis-signal contract version handshake (Phase 1) ──────────────────────
// The version of the electricsim chassis-signal contract this connector
// implements. electricsim is the PRODUCER and owns the contract semver
// (electricsim::io's EV1_CHASSIS_CONTRACT_VERSION_* in ev1_chassis_signals.hpp);
// ev1sim is the CONSUMER and declares here the version it was written against.
// scripts/audit_chassis_contract.py reads both and enforces compatibility in CI
// (same MAJOR, producer MINOR >= consumer MINOR). Bump these to match the
// producer whenever this connector is updated to a newer contract.
// See electricsim docs/chassis_contract_versioning.md.
#define EV1_CHASSIS_CONTRACT_VERSION_IMPLEMENTED_MAJOR 1
#define EV1_CHASSIS_CONTRACT_VERSION_IMPLEMENTED_MINOR 0
#define EV1_CHASSIS_CONTRACT_VERSION_IMPLEMENTED_PATCH 0

// Baseline guard: if this is built against an electricsim that predates the
// contract version handshake, the producer macros are absent. Fail with a clear
// message about the required baseline rather than a murky undefined-identifier
// error in the static_asserts below.
#if !defined(EV1_CHASSIS_CONTRACT_VERSION_MAJOR) \
    || !defined(EV1_CHASSIS_CONTRACT_VERSION_MINOR)
#  error "electricsim's ev1_chassis_signals.hpp does not define \
EV1_CHASSIS_CONTRACT_VERSION_{MAJOR,MINOR} — it predates the chassis-contract \
version handshake (electricsim Phase 0). Update the electricsim checkout."
#endif

// Compile-time compatibility guard: the contract is backward-compatible across
// MINOR bumps, so a consumer must share the producer's MAJOR and may not run
// ahead of the producer's MINOR. This catches an incompatible-version build
// locally; the audit catches it cross-repo in CI. Same MAJOR required:
static_assert(EV1_CHASSIS_CONTRACT_VERSION_IMPLEMENTED_MAJOR
                  == EV1_CHASSIS_CONTRACT_VERSION_MAJOR,
              "ev1sim chassis-contract MAJOR differs from electricsim's — the "
              "shared signal contract changed incompatibly; update this "
              "connector to the new contract.");
// Consumer must not be ahead of the producer on MINOR (it would implement
// signals the producer hasn't defined):
static_assert(EV1_CHASSIS_CONTRACT_VERSION_IMPLEMENTED_MINOR
                  <= EV1_CHASSIS_CONTRACT_VERSION_MINOR,
              "ev1sim chassis-contract MINOR is ahead of electricsim's — this "
              "connector implements signals newer than the producer defines.");

} // namespace

void ExternalSimConnector::Tick(double sim_time_s) {
    if (!m_opts.enabled) return;
    auto& st = *m_state;

    // Brake-state heartbeat: every 200 ms, force a re-publish of the
    // brake-pedal Q8 (main bus) and master-cylinder pressure (chassis
    // bus) signals even if their values haven't changed.  ev1sim
    // normally publishes both on-change-only — so a controller that
    // connects to the bus *after* a brake transition fires never sees
    // it, and behaves as if the pedal were still at zero.  Re-emitting
    // periodically guarantees a late-joining consumer picks up the
    // current state within one heartbeat interval.  Declared here at
    // outer Tick scope so both publish paths (in two different inner
    // scopes below) can consult the same flag and the timer advances
    // exactly once per heartbeat.
    constexpr double kBrakeHeartbeatPeriodS = 0.200;
    const bool brake_heartbeat_due = sim_time_s >= st.next_brake_heartbeat;

    // 1. Open transport if we don't have one (reconnect logic).
    //    SharedMemoryTransport's ctor silently no-ops on failure, so we
    //    verify by round-tripping a heartbeat frame and fall back into
    //    the reconnect timer if it can't be written.
    if (!st.transport) {
        if (sim_time_s < st.next_reconnect_time) return;
        SharedMemoryTransportOptions opts{};
        opts.name   = m_opts.bus_name;
        opts.create = true;    // tolerate being first on the bus

        auto candidate = std::make_unique<SharedMemoryTransport>(opts);
        Frame hb{};
        hb.header.type              = FrameType::Heartbeat;
        hb.header.stream_id         = kStreamEv1Sim;
        hb.header.sequence          = st.sequence;
        hb.header.monotonic_time_ns = NowNs();
        if (!candidate->publish_frame(hb)) {
            st.status = Status::Connecting;
            st.next_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
            std::cerr << "[ExternalSim] connect to '" << m_opts.bus_name
                      << "' failed — retry in "
                      << m_opts.reconnect_period_s << "s\n";
            return;
        }
        st.sequence++;
        st.transport = std::move(candidate);
        st.status    = Status::Connected;
        st.next_presence_time    = 0.0;
        st.panel_ever_published  = false;  // re-publish all panels after reconnect
        st.turn_haz_ever_published = false; // re-publish turn/hazard cavities after reconnect
        st.wiper_ever_published    = false; // re-publish wiper cavities after reconnect
        // Force re-publish of all chassis outputs after reconnect.
        st.charge_coupler_present_pub = -1;
        st.prnd_a_pub = -1;
        st.prnd_b_pub = -1;
        st.prnd_c_pub = -1;
        st.prnd_d_pub = -1;
        st.steering_pump_interlock_pub = -1;
        st.hvac_temp_setpoint_pub   = -9999.0f;
        st.hvac_fan_request_pub      = -1;
        st.hvac_mode_request_pub     = -1;
        st.hvac_ac_request_pub       = -1;
        st.hvac_defrost_request_pub  = -1;
        st.door_lock_state_driver_pub    = -1;
        st.door_lock_state_passenger_pub = -1;
        st.door_lock_state_trunk_pub     = -1;
        std::cout << "[ExternalSim] connected to bus '"
                  << m_opts.bus_name << "'\n";
    }

    // 2. Drain incoming frames.
    for (;;) {
        PollResult polled = st.transport->poll_frame(std::chrono::milliseconds(0));
        if (polled.status == PollStatus::Timeout) break;
        if (polled.status == PollStatus::Closed) {
            std::cerr << "[ExternalSim] transport closed — reconnecting\n";
            st.transport.reset();
            st.status = Status::Connecting;
            st.next_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
            return;
        }
        if (polled.status == PollStatus::Corrupt) continue;
        if (polled.frame.header.stream_id == kStreamEv1Sim) continue;  // our echo
        if (polled.frame.header.type != FrameType::DeltaBatch) continue;

        for (const auto& d : polled.frame.deltas) {
            const Endpoint* ep = FindEndpoint(d.signal_id);
            if (!ep || !ep->input_to_sim) continue;
            if (d.payload.empty()) continue;
            // float32 LE chassis-bus signals — decode as 4-byte little-endian float.
            if (d.signal_id == kSigIpcTripDistanceM            ||
                d.signal_id == kSigSteeringCmd                 ||
                d.signal_id == kSigChassisBtcmEmbMotorCmdLR    ||
                d.signal_id == kSigChassisBtcmEmbMotorCmdRR    ||
                d.signal_id == kSigChassisBtcmCylPressureFL_kPa ||
                d.signal_id == kSigChassisBtcmCylPressureFR_kPa) {
                if (d.payload.size() >= 4) {
                    DebugInjectFloat(d.signal_id,
                        [&]() -> float {
                            std::uint32_t bits = 0;
                            for (int b = 0; b < 4; ++b)
                                bits |= static_cast<std::uint32_t>(d.payload[static_cast<std::size_t>(b)]) << (b * 8);
                            float v; std::memcpy(&v, &bits, 4); return v;
                        }());
                }
            // uint32 LE chassis-bus signals — decode as 4-byte little-endian unsigned.
            } else if (d.signal_id == kSigBpmPackVoltageMv) {
                if (d.payload.size() >= 4) {
                    std::uint32_t mv = 0;
                    for (int b = 0; b < 4; ++b)
                        mv |= static_cast<std::uint32_t>(d.payload[static_cast<std::size_t>(b)]) << (b * 8);
                    DebugInjectU32(d.signal_id, mv);
                }
            // uint8 chassis-bus signals — decode as raw byte.
            } else if (d.signal_id == kSigThrottleCmdQ8 ||
                d.signal_id == kSigWiperMotorCommand ||
                d.signal_id == kSigWasherPumpCommand ||
                d.signal_id == kSigDoorLockCmdDriver   ||
                d.signal_id == kSigDoorLockCmdPassenger ||
                d.signal_id == kSigPowerWindowMotorDriver ||
                d.signal_id == kSigPowerWindowMotorPassenger ||
                d.signal_id == kSigRsaShiftBlocked     ||
                d.signal_id == kSigHvacBlowerLevel     ||
                d.signal_id == kSigDefrostGridActive   ||
                d.signal_id == kSigIpcSeatbeltTelltaleDriver ||
                d.signal_id == kSigIpcSeatbeltTelltalePassenger ||
                d.signal_id == kSigIpcBrakeTelltale    ||
                d.signal_id == kSigIpcParkBrakeTelltale ||
                d.signal_id == kSigIpcAntilockTelltale ||
                d.signal_id == kSigIpcLowTracTelltale  ||
                d.signal_id == kSigIpcAirBagTelltale   ||
                d.signal_id == kSigIpcServiceNowTelltale    ||
                d.signal_id == kSigIpcCheckMessagesTelltale ||
                d.signal_id == kSigIpcTempTelltale          ||
                d.signal_id == kSigIpcBatteryLifeTelltale   ||
                d.signal_id == kSigIpcReducedPerfTelltale   ||
                d.signal_id == kSigIpcCheckTirePressTelltale ||
                d.signal_id == kSigPscmPumpSpeedCmdQ8) {
                DebugInjectU8(d.signal_id, d.payload[0]);
            } else {
                // All other inbound signals are boolean (bool) — decode LSB.
                // (door-lock motor legs 4092-4095 and sounder piezo 4096 land here.)
                const bool v = (d.payload[0] & 1u) != 0u;
                DebugInjectDelta(d.signal_id, v);
            }
        }
    }

    // 3. Publish any panel-sensor changes and combination switch pin changes
    //    since last tick.
    std::vector<DeltaRecord> outbound;
    for (int p = 0; p < VehiclePanels::NUM_PANELS; ++p) {
        if (!st.panel_ever_published || st.panel[p] != st.panel_published[p]) {
            outbound.push_back(MakeBoolDelta(kPanelBase + static_cast<std::uint32_t>(p),
                                             st.panel[p]));
            st.panel_published[p] = st.panel[p];
        }
    }
    st.panel_ever_published = true;

    if (!st.comb_sw_ever_published ||
        st.comb_sw_low_beam      != st.comb_sw_low_beam_pub      ||
        st.comb_sw_flash_to_pass != st.comb_sw_flash_to_pass_pub ||
        st.comb_sw_park_headlamp != st.comb_sw_park_headlamp_pub) {
        outbound.push_back(MakeBoolDelta(kCombSwLowBeamOutId,      st.comb_sw_low_beam));
        outbound.push_back(MakeBoolDelta(kCombSwFlashToPassOutId,  st.comb_sw_flash_to_pass));
        outbound.push_back(MakeBoolDelta(kCombSwParkHeadlampOutId, st.comb_sw_park_headlamp));
        st.comb_sw_low_beam_pub      = st.comb_sw_low_beam;
        st.comb_sw_flash_to_pass_pub = st.comb_sw_flash_to_pass;
        st.comb_sw_park_headlamp_pub = st.comb_sw_park_headlamp;
        st.comb_sw_ever_published    = true;
    }

    // Turn/hazard combination switch cavities (IDs 4043-4046) — on change.
    if (!st.turn_haz_ever_published ||
        st.turn_haz_right_turn != st.turn_haz_right_turn_pub ||
        st.turn_haz_left_turn  != st.turn_haz_left_turn_pub  ||
        st.turn_haz_hazard     != st.turn_haz_hazard_pub     ||
        st.turn_haz_horn       != st.turn_haz_horn_pub) {
        outbound.push_back(MakeBoolDelta(kTurnHazSwRightTurnOutId, st.turn_haz_right_turn));
        outbound.push_back(MakeBoolDelta(kTurnHazSwLeftTurnOutId,  st.turn_haz_left_turn));
        outbound.push_back(MakeBoolDelta(kTurnHazSwHazardOutId,    st.turn_haz_hazard));
        outbound.push_back(MakeBoolDelta(kTurnHazSwHornOutId,      st.turn_haz_horn));
        st.turn_haz_right_turn_pub = st.turn_haz_right_turn;
        st.turn_haz_left_turn_pub  = st.turn_haz_left_turn;
        st.turn_haz_hazard_pub     = st.turn_haz_hazard;
        st.turn_haz_horn_pub       = st.turn_haz_horn;
        st.turn_haz_ever_published = true;
    }

    // Wiper/washer switch cavities (IDs 4054-4057) — on change.
    if (!st.wiper_ever_published ||
        st.wiper_delay   != st.wiper_delay_pub   ||
        st.wiper_request != st.wiper_request_pub ||
        st.wiper_hi      != st.wiper_hi_pub      ||
        st.wiper_washer  != st.wiper_washer_pub) {
        outbound.push_back(MakeBoolDelta(kWiperSwDelayOutId,        st.wiper_delay));
        outbound.push_back(MakeBoolDelta(kWiperSwRequestOutId,      st.wiper_request));
        outbound.push_back(MakeBoolDelta(kWiperSwHiOutId,           st.wiper_hi));
        outbound.push_back(MakeBoolDelta(kWiperSwWasherSwitchOutId, st.wiper_washer));
        st.wiper_delay_pub   = st.wiper_delay;
        st.wiper_request_pub = st.wiper_request;
        st.wiper_hi_pub      = st.wiper_hi;
        st.wiper_washer_pub  = st.wiper_washer;
        st.wiper_ever_published = true;
    }

    // Charge coupler presence (ID 4060) — publish delta on change.
    if (st.charge_coupler_present_pub < 0 ||
        static_cast<bool>(st.charge_coupler_present_pub) != st.charge_coupler_present) {
        outbound.push_back(MakeBoolDelta(kChargeCouplerPresentId, st.charge_coupler_present));
        st.charge_coupler_present_pub = static_cast<std::int8_t>(st.charge_coupler_present ? 1 : 0);
    }

    // PRND selector lines (IDs 4050-4053) — publish deltas on change.
    {
        auto pub_prnd = [&](std::uint32_t id, bool val, std::int8_t& pub) {
            if (pub < 0 || static_cast<bool>(pub) != val) {
                outbound.push_back(MakeBoolDelta(id, val));
                pub = static_cast<std::int8_t>(val ? 1 : 0);
            }
        };
        pub_prnd(kPrndSelectorAId, st.prnd_a, st.prnd_a_pub);
        pub_prnd(kPrndSelectorBId, st.prnd_b, st.prnd_b_pub);
        pub_prnd(kPrndSelectorCId, st.prnd_c, st.prnd_c_pub);
        pub_prnd(kPrndSelectorDId, st.prnd_d, st.prnd_d_pub);
    }

    // Power-steering pump HV interlock-closed (ID 4098) — publish delta on change.
    // The pump motor body closes the molex.D/E loop while present; PSCM senses it.
    if (st.steering_pump_interlock_pub < 0 ||
        static_cast<bool>(st.steering_pump_interlock_pub) != st.steering_pump_interlock_closed) {
        outbound.push_back(MakeBoolDelta(kSigPscmPumpInterlockClosed,
                                         st.steering_pump_interlock_closed));
        st.steering_pump_interlock_pub =
            static_cast<std::int8_t>(st.steering_pump_interlock_closed ? 1 : 0);
    }

    // HVAC driver controls (IDs 4124-4128) — publish deltas on change.
    {
        if (st.hvac_temp_setpoint_pub < -9000.0f ||
            std::fabs(st.hvac_temp_setpoint_pub - st.hvac_temp_setpoint_c) > 0.01f) {
            outbound.push_back(MakeFloatDelta(kSigHvacTempSetpointC, st.hvac_temp_setpoint_c));
            st.hvac_temp_setpoint_pub = st.hvac_temp_setpoint_c;
        }
        auto pub_hvac_u8 = [&](std::uint32_t id, std::uint8_t val, std::int8_t& pub) {
            if (pub < 0 || static_cast<std::uint8_t>(pub) != val) {
                outbound.push_back(MakeU8Delta(id, val));
                pub = static_cast<std::int8_t>(val);
            }
        };
        pub_hvac_u8(kSigHvacFanRequest,     st.hvac_fan_request,  st.hvac_fan_request_pub);
        pub_hvac_u8(kSigHvacModeRequest,    st.hvac_mode_request, st.hvac_mode_request_pub);
        pub_hvac_u8(kSigHvacAcRequest,      st.hvac_ac_request      ? 1u : 0u, st.hvac_ac_request_pub);
        pub_hvac_u8(kSigHvacDefrostRequest, st.hvac_defrost_request ? 1u : 0u, st.hvac_defrost_request_pub);
    }

    // Door lock STATE feedback (IDs 4155-4157) — publish deltas on change.
    {
        auto pub_lock = [&](std::uint32_t id, bool val, std::int8_t& pub) {
            const std::int8_t v = val ? 1 : 0;
            if (pub != v) {
                outbound.push_back(MakeU8Delta(id, static_cast<std::uint8_t>(v)));
                pub = v;
            }
        };
        pub_lock(kSigDoorLockStateDriver,    st.door_lock_state_driver,    st.door_lock_state_driver_pub);
        pub_lock(kSigDoorLockStatePassenger, st.door_lock_state_passenger, st.door_lock_state_passenger_pub);
        pub_lock(kSigDoorLockStateTrunk,     st.door_lock_state_trunk,     st.door_lock_state_trunk_pub);
    }

    if (!outbound.empty()) {
        Frame f{};
        f.header.type              = FrameType::DeltaBatch;
        f.header.stream_id         = kStreamEv1Sim;
        f.header.sequence          = st.sequence++;
        f.header.monotonic_time_ns = NowNs();
        f.deltas                   = std::move(outbound);
        if (!st.transport->publish_frame(f)) {
            std::cerr << "[ExternalSim] publish_frame failed — reconnecting\n";
            st.transport.reset();
            st.status = Status::Connecting;
            st.next_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
            return;
        }
    }

    // 4. Publish vehicle dynamics snapshot (float32 signals, every frame).
    if (st.has_vstate) {
        const auto& vs = st.vstate;
        std::vector<DeltaRecord> dyn;
        dyn.reserve(static_cast<std::size_t>(kNumDynamics + kNumSimTimeSignals));
        // Sim-time master clock (chassis 4075, uint64 LE ns) — every tick,
        // monotonic.  Lets electricsim's SimClock drive ECUs off sim-time
        // instead of wall-clock; see kSigSimTimeNs.
        dyn.push_back(MakeU64Delta(
            kSigSimTimeNs,
            static_cast<std::uint64_t>(vs.sim_time * 1.0e9)));
        dyn.push_back(MakeFloatDelta(4100, static_cast<float>(vs.speed_mps)));
        dyn.push_back(MakeFloatDelta(4101, static_cast<float>(vs.accel_long)));
        dyn.push_back(MakeFloatDelta(4102, static_cast<float>(vs.accel_lat)));
        dyn.push_back(MakeFloatDelta(4103, static_cast<float>(vs.yaw_rate)));
        dyn.push_back(MakeFloatDelta(4104, static_cast<float>(vs.applied_throttle)));
        dyn.push_back(MakeFloatDelta(4105, static_cast<float>(vs.applied_front_brake)));
        dyn.push_back(MakeFloatDelta(4106, static_cast<float>(vs.applied_rear_brake)));
        dyn.push_back(MakeFloatDelta(4107, static_cast<float>(vs.front_brake_pressure)));
        dyn.push_back(MakeFloatDelta(4108, static_cast<float>(vs.rear_brake_position)));
        dyn.push_back(MakeFloatDelta(4109, static_cast<float>(vs.steering_torque)));
        for (int w = 0; w < 4; ++w)
            dyn.push_back(MakeFloatDelta(4110 + static_cast<std::uint32_t>(w),
                                         static_cast<float>(vs.wheel_omega[w])));
        for (int w = 0; w < 4; ++w)
            dyn.push_back(MakeFloatDelta(4120 + static_cast<std::uint32_t>(w),
                                         static_cast<float>(vs.slip_ratio[w])));

        Frame df{};
        df.header.type              = FrameType::DeltaBatch;
        df.header.stream_id         = kStreamEv1Sim;
        df.header.sequence          = st.sequence++;
        df.header.monotonic_time_ns = NowNs();
        df.deltas                   = std::move(dyn);
        if (!st.transport->publish_frame(df)) {
            std::cerr << "[ExternalSim] publish_frame (dynamics) failed — reconnecting\n";
            st.transport.reset();
            st.status = Status::Connecting;
            st.next_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
            return;
        }
    }

    // 5. Open / maintain the main harness segment (electricsim_ev1_bus),
    //    drain incoming frames (RSA run-mode broadcast), and publish driver
    //    input signals.
    if (!st.main_transport && sim_time_s >= st.next_main_reconnect_time) {
        SharedMemoryTransportOptions main_opts{};
        main_opts.name   = m_opts.main_harness_bus_name;
        main_opts.create = true;
        auto candidate = std::make_unique<SharedMemoryTransport>(main_opts);
        Frame hb{};
        hb.header.type              = FrameType::Heartbeat;
        hb.header.stream_id         = kStreamEv1Sim;
        hb.header.sequence          = st.main_sequence;
        hb.header.monotonic_time_ns = NowNs();
        if (!candidate->publish_frame(hb)) {
            st.next_main_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
            std::cerr << "[ExternalSim] connect to main bus '"
                      << m_opts.main_harness_bus_name
                      << "' failed — retry in " << m_opts.reconnect_period_s << "s\n";
        } else {
            st.main_sequence++;
            st.main_transport = std::move(candidate);
            // Reset published sentinels so we force-publish on first tick.
            st.driver_brake_pub            = 0xFF;
            st.driver_steering_pub         = 0x7FFF;
            st.driver_gear_pub             = 0xFF;
            st.driver_throttle_pub         = 0xFF;
            st.driver_brake_switch_pub     = -1;
            st.driver_seatbelt_buckled_pub = -1;
            st.driver_turn_left_pub        = -1;
            st.driver_turn_right_pub       = -1;
            st.driver_hazard_pub           = -1;
            for (int bi = 0; bi < 5; ++bi) st.driver_rsa_btn_pub[bi] = -1;
            st.driver_rsa_mode_btn_pub      = -1;
            st.driver_pw_driver_up_pub      = -1;
            st.driver_pw_driver_down_pub    = -1;
            st.driver_pw_passenger_up_pub   = -1;
            st.driver_pw_passenger_down_pub = -1;
            for (int ei = 0; ei < 5; ++ei) st.driver_ext_keypad_pub[ei] = -1;
            st.driver_door_handle_driver_pub    = -1;
            st.driver_door_handle_passenger_pub = -1;
            // 3D-sim contract sentinels — force first-publish on reconnect.
            st.sensor_door_open_driver_pub      = -1;
            st.sensor_door_open_passenger_pub   = -1;
            st.sensor_hood_open_pub             = -1;
            st.sensor_trunk_open_pub            = -1;
            st.ext_headlight_switch_pub         = -1;
            st.ext_headlight_dim_pub            = -1;
            st.ext_telltale_test_pub            = -1;
            st.ext_park_brake_set_pub           = -1;
            st.ext_park_brake_release_pub       = -1;
            st.ext_key_position_pub             = -1;
            std::cout << "[ExternalSim] connected to main harness bus '"
                      << m_opts.main_harness_bus_name << "'\n";
        }
    }
    if (st.main_transport) {
        // Drain main harness incoming frames — subscribe to RSA run-mode broadcast.
        for (;;) {
            PollResult polled = st.main_transport->poll_frame(std::chrono::milliseconds(0));
            if (polled.status == PollStatus::Timeout) break;
            if (polled.status == PollStatus::Closed) {
                std::cerr << "[ExternalSim] main harness transport closed — reconnecting\n";
                st.main_transport.reset();
                st.next_main_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
                goto main_transport_done;
            }
            if (polled.status == PollStatus::Corrupt) continue;
            if (polled.frame.header.stream_id == kStreamEv1Sim) continue;  // our echo
            if (polled.frame.header.type != FrameType::DeltaBatch) continue;
            for (const auto& d : polled.frame.deltas) {
                if (d.signal_id == kSigRunModeBroadcast && !d.payload.empty()) {
                    const std::uint8_t new_mode = d.payload[0];
                    if (!st.has_rsa_run_mode || st.rsa_run_mode != new_mode) {
                        st.rsa_run_mode     = new_mode;
                        st.has_rsa_run_mode = true;
                    }
                } else if (d.signal_id == kSigPimCruiseActive && !d.payload.empty()) {
                    // PIM cruise active flag: uint8 bool (0=off/standby, 1=engaged).
                    st.pim_cruise_active     = (d.payload[0] != 0u);
                    st.has_pim_cruise_active = true;
                } else if (d.signal_id == kSigPimCruiseSetpointMps &&
                           d.payload.size() >= 4) {
                    // PIM cruise setpoint: float32 LE, m/s.
                    std::uint32_t bits = 0;
                    for (int b = 0; b < 4; ++b)
                        bits |= static_cast<std::uint32_t>(d.payload[b]) << (b * 8);
                    std::memcpy(&st.pim_cruise_setpoint_mps, &bits, 4);
                    st.has_pim_cruise_setpoint_mps = true;
                } else if (d.signal_id == kSigAdMainContactor &&
                           !d.payload.empty()) {
                    st.ad_main_contactor     = (d.payload[0] & 1u) != 0;
                    st.has_ad_main_contactor = true;
                } else if (d.signal_id == kSigAdPrechargeRelay &&
                           !d.payload.empty()) {
                    st.ad_precharge_relay     = (d.payload[0] & 1u) != 0;
                    st.has_ad_precharge_relay = true;
                } else if (d.signal_id == kSigAdStateEnum &&
                           d.payload.size() >= 4) {
                    std::uint32_t v = 0;
                    for (int b = 0; b < 4; ++b)
                        v |= static_cast<std::uint32_t>(d.payload[b]) << (b * 8);
                    st.ad_state_enum     = v;
                    st.has_ad_state_enum = true;
                } else if (d.signal_id == kSigBtcmUartFrame && !d.payload.empty()) {
                    // BTCM 5 Hz canonical-frame heartbeat.  Record the
                    // arrival timestamp; payload itself (the 16-byte
                    // status frame) is consumed elsewhere (e.g. IPC
                    // frame decoder).  See SimApp.h::kAbsFreshnessWindow
                    // for the consumer-side liveness check.
                    st.btcm_uart_frame_ns = polled.frame.header.monotonic_time_ns
                                            ? polled.frame.header.monotonic_time_ns
                                            : NowNs();
                } else if (d.signal_id == kSigSolFL_ISO && !d.payload.empty()) {
                    st.sol_fl_iso     = (d.payload[0] & 1u) != 0;
                    st.sol_fl_iso_ns  = polled.frame.header.monotonic_time_ns
                                        ? polled.frame.header.monotonic_time_ns
                                        : NowNs();
                } else if (d.signal_id == kSigSolFL_DMP && !d.payload.empty()) {
                    st.sol_fl_dmp     = (d.payload[0] & 1u) != 0;
                    st.sol_fl_dmp_ns  = polled.frame.header.monotonic_time_ns
                                        ? polled.frame.header.monotonic_time_ns
                                        : NowNs();
                } else if (d.signal_id == kSigSolFR_ISO && !d.payload.empty()) {
                    st.sol_fr_iso     = (d.payload[0] & 1u) != 0;
                    st.sol_fr_iso_ns  = polled.frame.header.monotonic_time_ns
                                        ? polled.frame.header.monotonic_time_ns
                                        : NowNs();
                } else if (d.signal_id == kSigSolFR_DMP && !d.payload.empty()) {
                    st.sol_fr_dmp     = (d.payload[0] & 1u) != 0;
                    st.sol_fr_dmp_ns  = polled.frame.header.monotonic_time_ns
                                        ? polled.frame.header.monotonic_time_ns
                                        : NowNs();
                } else if ((d.signal_id == kSigRearMotorLR ||
                            d.signal_id == kSigRearMotorRR) &&
                           d.payload.size() >= 4) {
                    // Float32 LE in [-1, +1].
                    std::uint32_t bits = 0;
                    for (int b = 0; b < 4; ++b)
                        bits |= static_cast<std::uint32_t>(d.payload[b]) << (b * 8);
                    float v;
                    std::memcpy(&v, &bits, 4);
                    const std::uint64_t now_ns =
                        polled.frame.header.monotonic_time_ns
                            ? polled.frame.header.monotonic_time_ns
                            : NowNs();
                    if (d.signal_id == kSigRearMotorLR) {
                        st.rear_motor_lr    = v;
                        st.rear_motor_lr_ns = now_ns;
                    } else {
                        st.rear_motor_rr    = v;
                        st.rear_motor_rr_ns = now_ns;
                    }
                }
            }
        }

        std::vector<DeltaRecord> drv;
        // Heartbeat-due flag: see Tick() outer-scope decl for rationale.
        // When true, force-publish driver-input + chassis brake pressure
        // even though the change-detection thresholds wouldn't fire.
        if (brake_heartbeat_due ||
            st.driver_brake_q8    != st.driver_brake_pub ||
            st.driver_steering_q8 != st.driver_steering_pub ||
            st.driver_gear        != st.driver_gear_pub ||
            // While the throttle delta is suppressed its _pub cache is
            // frozen, so a changing pedal would hold this mismatch true
            // and re-publish the OTHER group members every tick. Gate the
            // term on suppression: during a fault window the group reverts
            // to heartbeat + real brake/steer/gear changes only, and the
            // restore re-publish happens via this same term on the first
            // un-suppressed Tick (or the heartbeat, whichever is sooner).
            (!st.suppress_throttle_publish &&
             st.driver_throttle_q8 != st.driver_throttle_pub)) {
            drv.push_back(MakeU8Delta(kSigDriverBrakePedalQ8,  st.driver_brake_q8));
            drv.push_back(MakeI16Delta(kSigDriverSteeringDegQ8, st.driver_steering_q8));
            drv.push_back(MakeU8Delta(kSigDriverGearSelector,   st.driver_gear));
            if (!st.suppress_throttle_publish) {
                // Fault injection (scenario "fail_throttle_input"): while
                // suppressed the throttle delta is withheld — heartbeats
                // included — so a consumer-side freshness window genuinely
                // expires. The _pub cache is left untouched so restoring
                // re-publishes on the next heartbeat at the latest.
                drv.push_back(MakeU8Delta(kSigDriverThrottleQ8,
                                          st.driver_throttle_q8));
                st.driver_throttle_pub = st.driver_throttle_q8;
            }
            st.driver_brake_pub    = st.driver_brake_q8;
            st.driver_steering_pub = st.driver_steering_q8;
            st.driver_gear_pub     = st.driver_gear;
        }
        // Brake switch and seatbelt — publish on change (separate booleans so
        // they don't force-publish the Q8 group and vice-versa).
        const std::int8_t brake_sw_val = st.driver_brake_switch ? 1 : 0;
        if (brake_sw_val != st.driver_brake_switch_pub) {
            drv.push_back(MakeBoolDelta(kSigDriverBrakeSwitch, st.driver_brake_switch));
            st.driver_brake_switch_pub = brake_sw_val;
        }
        const std::int8_t seatbelt_val = st.driver_seatbelt_buckled ? 1 : 0;
        if (seatbelt_val != st.driver_seatbelt_buckled_pub) {
            drv.push_back(MakeBoolDelta(kSigDriverSeatbeltBuckled, st.driver_seatbelt_buckled));
            st.driver_seatbelt_buckled_pub = seatbelt_val;
        }
        // Passenger seatbelt (6965) — publish on change.
        const std::int8_t seatbelt_p_val = st.driver_seatbelt_buckled_passenger ? 1 : 0;
        if (seatbelt_p_val != st.driver_seatbelt_buckled_passenger_pub) {
            drv.push_back(MakeBoolDelta(kSigDriverSeatbeltBuckledPassenger,
                                        st.driver_seatbelt_buckled_passenger));
            st.driver_seatbelt_buckled_passenger_pub = seatbelt_p_val;
        }
        // Turn/hazard combination switch — published on the CHASSIS cavities
        // kSigTurnHazSw_* (4043-4045) that LHJB consumes; publish on change.
        const std::int8_t turn_left_val  = st.driver_turn_left  ? 1 : 0;
        const std::int8_t turn_right_val = st.driver_turn_right ? 1 : 0;
        const std::int8_t hazard_val     = st.driver_hazard     ? 1 : 0;
        if (turn_left_val != st.driver_turn_left_pub) {
            drv.push_back(MakeBoolDelta(kSigTurnHazSw_LeftTurnOut, st.driver_turn_left));
            st.driver_turn_left_pub = turn_left_val;
        }
        if (turn_right_val != st.driver_turn_right_pub) {
            drv.push_back(MakeBoolDelta(kSigTurnHazSw_RightTurnOut, st.driver_turn_right));
            st.driver_turn_right_pub = turn_right_val;
        }
        if (hazard_val != st.driver_hazard_pub) {
            drv.push_back(MakeBoolDelta(kSigTurnHazSw_HazardOut, st.driver_hazard));
            st.driver_hazard_pub = hazard_val;
        }
        // Horn — single combo-switch contact (circuit 28), published on the
        // main-harness horn cavity in lockstep with the chassis cavity 4046.
        // (The old high/low horn requests 6940/6941 were collapsed to this one
        // contact; mirror the same turn_haz_horn state SetTurnHazSwOutputs latches.)
        const bool horn_out = st.turn_haz_horn;
        const std::int8_t horn_out_val = horn_out ? 1 : 0;
        if (horn_out_val != st.driver_horn_out_pub) {
            drv.push_back(MakeBoolDelta(kSigTurnHazSw_HornOut, horn_out));
            st.driver_horn_out_pub = horn_out_val;
        }
        // RSA per-digit keypad buttons (IDs 6975-6979) — one-shot: publish on
        // change.  Payload: 0=idle, 1=tap (lower digit), 2=long-press (higher digit).
        {
            constexpr std::uint32_t kBtnIds[5] = {
                kSigDriverRsaKeypadButton1, kSigDriverRsaKeypadButton2,
                kSigDriverRsaKeypadButton3, kSigDriverRsaKeypadButton4,
                kSigDriverRsaKeypadButton5,
            };
            for (int bi = 0; bi < 5; ++bi) {
                const std::int8_t bval = static_cast<std::int8_t>(st.driver_rsa_buttons[bi]);
                if (st.driver_rsa_btn_pub[bi] < 0 || bval != st.driver_rsa_btn_pub[bi]) {
                    drv.push_back(MakeU8Delta(kBtnIds[bi], st.driver_rsa_buttons[bi]));
                    st.driver_rsa_btn_pub[bi] = bval;
                }
            }
        }
        // RSA mode button (ID 6971) — one-shot: publish current value.
        {
            const std::int8_t mode_val = static_cast<std::int8_t>(st.driver_rsa_mode_button);
            if (st.driver_rsa_mode_btn_pub < 0 || mode_val != st.driver_rsa_mode_btn_pub) {
                drv.push_back(MakeU8Delta(kSigDriverRsaModeButton, st.driver_rsa_mode_button));
                st.driver_rsa_mode_btn_pub = mode_val;
            }
        }
        // IPC trip-reset (6952), cruise switch cavities (4047-4049), wiper.
        // Momentary/level bools: publish on change only (sentinel -1 forces first publish).
        auto publish_bool_change = [&](std::uint32_t sig_id, bool cur_val, std::int8_t& pub) {
            const std::int8_t v = cur_val ? 1 : 0;
            if (pub < 0 || v != pub) {
                drv.push_back(MakeBoolDelta(sig_id, cur_val));
                pub = v;
            }
        };
        publish_bool_change(kSigDriverIpcTripResetButton, st.driver_ipc_trip_reset,
                            st.driver_ipc_trip_reset_pub);
        // Cruise-control raw contacts — held state from the stalk model; PIM's
        // tap/hold decoder interprets them (held duration = tap vs. hold).
        publish_bool_change(kSigCruiseSw_SetCoastOut,    st.cruise_set_coast_contact,
                            st.cruise_set_coast_pub);
        publish_bool_change(kSigCruiseSw_ResumeAccelOut, st.cruise_resume_accel_contact,
                            st.cruise_resume_accel_pub);
        publish_bool_change(kSigCruiseSw_OnOffOut,       st.cruise_on_off_contact,
                            st.cruise_on_off_pub);
        // Wiper switch — published as the three raw contact cavities computed
        // from the detent enum, matching RHJB's WSW decoder (ESM p.511):
        // OFF=000, INT=110, LOW=010, HIGH=011.  Washer on its own cavity (4057).
        {
            const std::uint8_t e = st.driver_wiper_switch;  // 0=OFF,1=INT,2=LOW,3=HIGH
            publish_bool_change(kSigWiperSw_DelayOut,   (e == 1U),
                                st.driver_wiper_delay_pub);
            publish_bool_change(kSigWiperSw_RequestOut, (e >= 1U && e <= 3U),
                                st.driver_wiper_request_pub);
            publish_bool_change(kSigWiperSw_HiOut,      (e == 3U),
                                st.driver_wiper_hi_pub);
        }
        publish_bool_change(kSigWiperSw_WasherSwitchOut, st.driver_wiper_wash,
                            st.driver_wiper_wash_pub);
        // Power window switches (6980-6983) — momentary bool, held while pressed.
        // Default false (no UI source yet); floating UI will set press()/release().
        publish_bool_change(kSigDriverPowerWindowDriverUp,
                            st.driver_pw_driver_up,     st.driver_pw_driver_up_pub);
        publish_bool_change(kSigDriverPowerWindowDriverDown,
                            st.driver_pw_driver_down,   st.driver_pw_driver_down_pub);
        publish_bool_change(kSigDriverPowerWindowPassengerUp,
                            st.driver_pw_passenger_up,  st.driver_pw_passenger_up_pub);
        publish_bool_change(kSigDriverPowerWindowPassengerDown,
                            st.driver_pw_passenger_down, st.driver_pw_passenger_down_pub);
        // RSA exterior keypad (6985-6989) — uint8 Option A encoding.
        {
            constexpr std::uint32_t kExtIds[5] = {
                kSigDriverRsaExteriorKeypad1, kSigDriverRsaExteriorKeypad2,
                kSigDriverRsaExteriorKeypad3, kSigDriverRsaExteriorKeypad4,
                kSigDriverRsaExteriorKeypad5,
            };
            for (int bi = 0; bi < 5; ++bi) {
                const std::int8_t bval = static_cast<std::int8_t>(st.driver_ext_keypad[bi]);
                if (st.driver_ext_keypad_pub[bi] < 0 || bval != st.driver_ext_keypad_pub[bi]) {
                    drv.push_back(MakeU8Delta(kExtIds[bi], st.driver_ext_keypad[bi]));
                    st.driver_ext_keypad_pub[bi] = bval;
                }
            }
        }
        // Door handle attempt signals (6990-6991) — momentary bool.
        publish_bool_change(kSigDriverDoorHandleAttemptDriver,
                            st.driver_door_handle_driver,    st.driver_door_handle_driver_pub);
        publish_bool_change(kSigDriverDoorHandleAttemptPassenger,
                            st.driver_door_handle_passenger, st.driver_door_handle_passenger_pub);
        // 3D-sim contract panel-sensor mirrors (6960-6963) — parallel
        // publication of the chassis-bus panel ajar sensors (4030-4033)
        // on the main harness segment so consumers there don't need to
        // bridge segments.  PanelID maps: DOOR_LEFT=driver in a US LHD
        // EV1; DOOR_RIGHT=passenger.
        publish_bool_change(kSigSensorDoorOpenDriver,
                            st.panel[static_cast<int>(PanelID::DOOR_LEFT)],
                            st.sensor_door_open_driver_pub);
        publish_bool_change(kSigSensorDoorOpenPassenger,
                            st.panel[static_cast<int>(PanelID::DOOR_RIGHT)],
                            st.sensor_door_open_passenger_pub);
        publish_bool_change(kSigSensorHoodOpen,
                            st.panel[static_cast<int>(PanelID::HOOD)],
                            st.sensor_hood_open_pub);
        publish_bool_change(kSigSensorTrunkOpen,
                            st.panel[static_cast<int>(PanelID::TRUNK)],
                            st.sensor_trunk_open_pub);
        // 3D-sim contract buttons + key position (6942-6947, 6966).
        // (Horn 6940/6941 removed — single chassis horn cavity 4046.)
        publish_bool_change(kSigDriverHeadlightDimRequest,
                            st.ext_headlight_dim_request,
                            st.ext_headlight_dim_pub);
        publish_bool_change(kSigDriverTelltaleTestRequest,
                            st.ext_telltale_test_request,
                            st.ext_telltale_test_pub);
        publish_bool_change(kSigDriverParkBrakeSetRequest,
                            st.ext_park_brake_set_request,
                            st.ext_park_brake_set_pub);
        publish_bool_change(kSigDriverParkBrakeReleaseRequest,
                            st.ext_park_brake_release_request,
                            st.ext_park_brake_release_pub);
        // Headlight switch + key position — uint8 enums.
        {
            const std::int8_t hsw_val =
                static_cast<std::int8_t>(st.ext_headlight_switch);
            if (st.ext_headlight_switch_pub < 0 ||
                hsw_val != st.ext_headlight_switch_pub) {
                drv.push_back(MakeU8Delta(kSigDriverHeadlightSwitch,
                                          st.ext_headlight_switch));
                st.ext_headlight_switch_pub = hsw_val;
            }
            const std::int8_t key_val =
                static_cast<std::int8_t>(st.ext_key_position);
            if (st.ext_key_position_pub < 0 ||
                key_val != st.ext_key_position_pub) {
                drv.push_back(MakeU8Delta(kSigSensorKeyPosition,
                                          st.ext_key_position));
                st.ext_key_position_pub = key_val;
            }
        }
        // 3D-sim contract physics telemetry (6920-6932) — every-tick floats,
        // not delta-gated.  Only sent when we have a VehicleState snapshot
        // (cold start emits nothing until the host calls SetVehicleState).
        if (st.has_vstate) {
            const auto& vs = st.vstate;
            // Wrap yaw_deg → rad in (-π, π].  yaw_deg is unbounded; wrap
            // to a 2π window centred on 0, with +π exclusive to ensure
            // canonical positive-π representation.
            constexpr double kPi  = 3.141592653589793;
            constexpr double kTau = 6.283185307179586;
            double yaw_rad = vs.yaw_deg * (kPi / 180.0);
            yaw_rad = std::fmod(yaw_rad + kPi, kTau);
            if (yaw_rad < 0.0) yaw_rad += kTau;
            yaw_rad -= kPi;
            drv.push_back(MakeFloatDelta(kSigExtBodyVelocityMps,
                                         static_cast<float>(vs.speed_mps)));
            drv.push_back(MakeFloatDelta(kSigExtAccelLongitudinalMps2,
                                         static_cast<float>(vs.accel_long)));
            drv.push_back(MakeFloatDelta(kSigExtAccelLateralMps2,
                                         static_cast<float>(vs.accel_lat)));
            drv.push_back(MakeFloatDelta(kSigExtVehiclePoseX,
                                         static_cast<float>(vs.pos_x)));
            drv.push_back(MakeFloatDelta(kSigExtVehiclePoseY,
                                         static_cast<float>(vs.pos_y)));
            drv.push_back(MakeFloatDelta(kSigExtVehiclePoseYawRad,
                                         static_cast<float>(yaw_rad)));
        }
        if (!drv.empty()) {
            Frame mf{};
            mf.header.type              = FrameType::DeltaBatch;
            mf.header.stream_id         = kStreamEv1Sim;
            mf.header.sequence          = st.main_sequence++;
            mf.header.monotonic_time_ns = NowNs();
            mf.deltas                   = std::move(drv);
            if (!st.main_transport->publish_frame(mf)) {
                std::cerr << "[ExternalSim] publish_frame (driver inputs) failed\n";
                st.main_transport.reset();
                st.next_main_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
            }
        }
    }
    main_transport_done:;

    // 5b. Publish motor state (RPM + torque + DC current) on the chassis segment.
    // Publish-on-change with small epsilon thresholds to avoid flooding.
    {
        constexpr float kRpmEps     = 0.01f;
        constexpr float kTorqueEps  = 0.01f;
        constexpr float kCurrentEps = 0.1f;   // amps; current spans ±~330 A
        const bool rpm_changed     = std::abs(st.motor_rpm - st.motor_rpm_pub) > kRpmEps;
        const bool torque_changed  = std::abs(st.motor_torque_nm - st.motor_torque_pub) > kTorqueEps;
        const bool current_changed = std::abs(st.motor_current_a - st.motor_current_pub) > kCurrentEps;
        if (rpm_changed || torque_changed || current_changed) {
            std::vector<DeltaRecord> mdyn;
            if (rpm_changed) {
                mdyn.push_back(MakeFloatDelta(kSigMotorRpm, st.motor_rpm));
                st.motor_rpm_pub = st.motor_rpm;
            }
            if (torque_changed) {
                mdyn.push_back(MakeFloatDelta(kSigMotorTorqueNm, st.motor_torque_nm));
                st.motor_torque_pub = st.motor_torque_nm;
            }
            if (current_changed) {
                mdyn.push_back(MakeFloatDelta(kSigMotorCurrentA, st.motor_current_a));
                st.motor_current_pub = st.motor_current_a;
            }
            Frame mf{};
            mf.header.type              = FrameType::DeltaBatch;
            mf.header.stream_id         = kStreamEv1Sim;
            mf.header.sequence          = st.sequence++;
            mf.header.monotonic_time_ns = NowNs();
            mf.deltas                   = std::move(mdyn);
            if (!st.transport->publish_frame(mf)) {
                std::cerr << "[ExternalSim] publish_frame (motor state) failed — reconnecting\n";
                st.transport.reset();
                st.status = Status::Connecting;
                st.next_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
                return;
            }
        }
    }

    // 5c. Publish ambient environment sensors (temp + humidity) on the chassis segment.
    // Publish-on-change with small epsilon thresholds to avoid flooding.
    {
        constexpr float kTempEps     = 0.01f;
        constexpr float kHumidityEps = 0.05f;
        const bool temp_changed     = std::abs(st.ambient_temp_c - st.ambient_temp_pub) > kTempEps;
        const bool humidity_changed = std::abs(st.ambient_humidity_pct - st.ambient_humidity_pub) > kHumidityEps;
        if (temp_changed || humidity_changed) {
            std::vector<DeltaRecord> env;
            if (temp_changed) {
                env.push_back(MakeFloatDelta(kSigAmbientTempC, st.ambient_temp_c));
                st.ambient_temp_pub = st.ambient_temp_c;
            }
            if (humidity_changed) {
                env.push_back(MakeFloatDelta(kSigAmbientHumidityPct, st.ambient_humidity_pct));
                st.ambient_humidity_pub = st.ambient_humidity_pct;
            }
            Frame ef{};
            ef.header.type              = FrameType::DeltaBatch;
            ef.header.stream_id         = kStreamEv1Sim;
            ef.header.sequence          = st.sequence++;
            ef.header.monotonic_time_ns = NowNs();
            ef.deltas                   = std::move(env);
            if (!st.transport->publish_frame(ef)) {
                std::cerr << "[ExternalSim] publish_frame (ambient env) failed — reconnecting\n";
                st.transport.reset();
                st.status = Status::Connecting;
                st.next_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
                return;
            }
        }
    }

    // 5d. Publish brake master cylinder pressure (chassis segment).
    // Same heartbeat rationale as kSigDriverBrakePedalQ8 — BTCM uses
    // this signal as its primary brake-effort input.  Without periodic
    // re-publish a late-joining consumer never learns the pedal is
    // pressed.
    {
        constexpr float kPressureEps = 1.0f;  // 1 kPa quantum is plenty
        if (brake_heartbeat_due ||
            std::abs(st.brake_master_pressure_kpa -
                     st.brake_master_pressure_pub_kpa) > kPressureEps) {
            std::vector<DeltaRecord> bdyn;
            bdyn.push_back(MakeFloatDelta(kSigBrakeMasterPressureKpa,
                                          st.brake_master_pressure_kpa));
            st.brake_master_pressure_pub_kpa = st.brake_master_pressure_kpa;

            Frame bf{};
            bf.header.type              = FrameType::DeltaBatch;
            bf.header.stream_id         = kStreamEv1Sim;
            bf.header.sequence          = st.sequence++;
            bf.header.monotonic_time_ns = NowNs();
            bf.deltas                   = std::move(bdyn);
            if (!st.transport->publish_frame(bf)) {
                std::cerr << "[ExternalSim] publish_frame (brake pressure) "
                             "failed — reconnecting\n";
                st.transport.reset();
                st.status = Status::Connecting;
                st.next_reconnect_time = sim_time_s + m_opts.reconnect_period_s;
                return;
            }
        }
    }

    // Now that both brake-state heartbeat consumers have evaluated, roll
    // the timer forward.  Doing this here (rather than inside either
    // branch) guarantees both paths fire on the same tick when the
    // heartbeat is due, instead of one consuming the slot and the other
    // missing it.
    if (brake_heartbeat_due) {
        st.next_brake_heartbeat = sim_time_s + kBrakeHeartbeatPeriodS;
    }

    // 6. Announce our endpoints periodically so other bus peers can discover us.
    if (sim_time_s >= st.next_presence_time) {
        Frame def{};
        def.header.type              = FrameType::SignalDefine;
        def.header.stream_id         = kStreamEv1Sim;
        def.header.sequence          = st.sequence++;
        def.header.monotonic_time_ns = NowNs();
        for (const auto& ep : EndpointTable()) {
            def.deltas.push_back(MakeDefineDelta(ep.signal_id, ep.qualified_name));
        }
        st.transport->publish_frame(def);
        st.next_presence_time = sim_time_s + m_opts.presence_period_s;
    }
}
#else   // EV1SIM_HAVE_EXTERNAL_SIM
void ExternalSimConnector::Tick(double /*sim_time_s*/) {
    // Built without electricsim — nothing to do.
}
#endif  // EV1SIM_HAVE_EXTERNAL_SIM
