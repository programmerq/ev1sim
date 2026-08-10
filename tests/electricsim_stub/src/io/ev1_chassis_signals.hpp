#pragma once

// Chassis-signal IDs for the physics ↔ electrical boundary between ev1sim
// and the electricsim ECU modules.
//
// These IDs name wires; the wires themselves live as cells on the wire-truth
// shared-memory substrate (src/io/wire_table.{cpp,hpp}). The legacy
// segmented message-queue layer that previously carried them is retired.
//
// **Source of truth** for signal IDs and qualified names is ev1sim's
// `src/ExternalSimConnector.cpp` (the `kDynamicsNames`, `kPanelNames`,
// `kBulbOrder`, and horn endpoint tables).  This header mirrors them in full.
// Numeric IDs must stay locked with ev1sim's tables; qualified names must
// match so the snooper and scan-tool identify signals identically on both
// sides.
//
// Signal directions:
//   electricsim → ev1sim : bulb feed lines (4000–4016)
//                          horn drive lines (4020–4021)
//   ev1sim → electricsim : panel ajar switches (4030–4033)
//                          vehicle dynamics (4100–4113, 4120–4123)
//                          sim-time clock         (4075)
//
// See docs/3d_sim_contract.md for the full peer contract including
// cadence, freshness, and button/discrete-sensor responsibilities.

#include <cstdint>

namespace electricsim::io {

// ---------------------------------------------------------------------------
// Chassis-signal contract version (semver). Note the split of authority: the
// *signal IDs and qualified names* are sourced from ev1sim's ExternalSimConnector
// tables (see the "Source of truth" note above) and mirrored here. This
// *version number*, by contrast, is owned by electricsim as the PRODUCER of the
// contract — it is the single semver both sides reconcile against. Bump it in
// the SAME commit that changes any contract-observable property of the chassis
// signals (an ID/name, scale, range, unit, or enum):
//   MAJOR = breaking (remove/rename a signal, change a scale/unit/enum, narrow
//           a range)
//   MINOR = additive, backward-compatible (new signal, widened range, appended
//           optional enum value)
//   PATCH = comment/provenance only; no wire-observable change
// Consumers (ev1sim's ExternalSimConnector) declare the version they implement
// and are compatible iff
//   producer.MAJOR == consumer.MAJOR && producer.MINOR >= consumer.MINOR
// (a consumer may lag on MINOR; a MAJOR mismatch or a consumer ahead of the
// producer is an error). scripts/audit_chassis_contract.py runs in *electricsim's*
// CI (the cross-repo-contract job, --skip-if-missing) and catches a MAJOR mismatch
// or a consumer ahead — but a consumer that merely lags on MINOR (even by many
// revs, as today: producer 1.9.0 vs the ev1sim consumer's 1.0.0) still reads as
// compatible, and ev1sim's OWN CI does not run this audit (it compiles against a
// committed stub). So this is a partial guard, not end-to-end enforcement;
// hardening it (generate the stub from this header, run the audit in ev1sim CI,
// add a MINOR-lag drift budget) is tracked as BL-0030..BL-0035 in
// docs/proposals/engineering_health_review_2026-06-15.md.
// @design 2026-06-03 — see docs/chassis_contract_versioning.md
#define EV1_CHASSIS_CONTRACT_VERSION_MAJOR 1
#define EV1_CHASSIS_CONTRACT_VERSION_MINOR 9
#define EV1_CHASSIS_CONTRACT_VERSION_PATCH 0
#define EV1_CHASSIS_CONTRACT_VERSION "1.9.0"
// @design 2026-06-14 — MINOR 8 -> 9: additive pair kSigChassisRhjbPmmModuleBPlus
//   (4196) — the RHJB Power-Moding Module's 840-family switched module-B+ feed,
//   the faithful run-mode-module boot supply (distinct from the RUN-1/RUN-2 load
//   buses), gated by each run-mode controller via SwitchedFeedGate — and
//   kSigChassisApmTwelveVoltOutputMa (4197) — the APM DC/DC 12 V output current
//   (mA), the aux-battery host's real charge-current source (replacing the fixed
//   bool-gated assumption). Backward-compatible — old consumers ignore both.
// @design 2026-06-13 — MINOR 7 -> 8: additive signal kSigChassisAuxBatteryMaintWakeReq
//   (4195) — the aux-battery host's periodic low-12 V maintenance wake request.
//   ORed into the RHJB PMM wake_req and the APM vehicle-wake-up so a low aux
//   battery wakes the car to charge it. Backward-compatible — old consumers
//   ignore it.
// @design 2026-06-13 — MINOR 6 -> 7: additive 12 V auxiliary-battery rail signals
//   kSigChassisAuxBatteryTerminalMv (4192), kSigChassisAuxBatteryPresent (4193),
//   kSigChassisAuxBatterySocPct (4194) — the aux battery's positive-post voltage
//   (the always-on low-voltage rail), the battery-connected flag, and its
//   state-of-charge. Published by the aux-battery host
//   (src/models/battery/aux_battery_host); the substrate for power-gated module
//   boot ("no 12 V → no boot"). Backward-compatible — old consumers ignore them.
// @design 2026-06-07 — MINOR 0 -> 1: additive signal kSigChassisLhjbChargeWakePassthru
//   (4187) for the LHJB -> RHJB charge-receptacle wake-up passthrough (circuit
//   2027G/2027A). Backward-compatible — old consumers ignore it.
// @design 2026-06-07 — MINOR 1 -> 2: additive signal kSigChassisBtcmRegenDisableOut
//   (4188) for the BTCM -> PCM discrete REGEN DISABLE OUT line (circuit 2021),
//   the DISCRETE half of the regen-disable correlation (DTC 109/110).
//   Backward-compatible — old consumers ignore it.
// @design 2026-06-12 — MINOR 5 -> 6: additive signal
//   kSigChassisBtcmRetardRequestDutyQ8 (4191) — BTCM retard-request PWM duty
//   (uint16 Q8 %; circuit J1-18), the DISCRETE half of the DTC 042 correlation.
//   Closes the deferred PWM leg in
//   notes/manual_supplements.yaml#2026-06-07-btcm-pim-regen-discrete-producers.
//   Backward-compatible — old consumers ignore the new ID.
// @design 2026-06-12 — MINOR 4 -> 5: additive signals
//   kSigChassisRsaDimLevelStep (4189) — RSA interior-dimmer step (circuit 230,
//   IPC C12) — and kSigChassisLhjbParkLampsOn (4190) — LHJB park-lamp net
//   present (circuit 301, IPC C4). Close the two producer gaps recorded in
//   notes/dtc_pass/concerns.yaml#2026-06-07-ipc-dim-and-parklamp-producer-gap.
//   Backward-compatible — old consumers ignore the new IDs.
// @design 2026-06-11 — MINOR 2 -> 3: additive signals
//   kSigChassisDoorLockStateDriver/Passenger/Trunk (4165-4167), adopting into
//   the contract the door-lock STATE feedback trio ev1sim had been publishing
//   on 4155-4157 — IDs this header allocates to the HV bus rail. That
//   un-contracted squat was a live cross-repo collision (every co-sim run
//   published lock-state bytes onto the rail consumers' IDs); see
//   notes/manual_supplements.yaml#2026-06-11-chassis-4155-doorlock-collision.
//   Backward-compatible — old consumers ignore the new IDs; ev1sim moves its
//   publisher to 4165-4167 with compile-time drift guards in the same round.
// @design 2026-06-11 — MINOR 3 -> 4: additive signals kSigChassisRoadGradePct
//   (4168) + kSigChassisPitchDeg (4169) — terrain slope under the wheelbase
//   and chassis body attitude from the physics sim (persona round 2, pair C
//   survivor; unblocks cruise-on-grade / hill-hold acceptance scenarios).
//   Backward-compatible — no electricsim consumer wired yet.

// ---------------------------------------------------------------------------
// Bulb feed lines (electricsim → ev1sim)
// Signal IDs 4000–4016.  Each ID is kBulbFeedLineBase + the slot index from
// ev1sim's ExternalSimConnector kBulbOrder[].  Slots 4017–4018 (LRTL/RRTL
// dual-filament tail elements) are ev1sim-only; electricsim leaves them unused.
// Encoding: 1-bit unsigned (bool); 1 = feed-line energized, 0 = de-energized.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kBulbFeedLineBase        = 4000U;

inline constexpr std::uint32_t kSigBulbFeedLine_LBL     = 4000U;  // slot 0  — backup lamp left
inline constexpr std::uint32_t kSigBulbFeedLine_RBL     = 4001U;  // slot 1  — backup lamp right
inline constexpr std::uint32_t kSigBulbFeedLine_LHBH    = 4002U;  // slot 2  — hi-beam left
inline constexpr std::uint32_t kSigBulbFeedLine_RHBH    = 4003U;  // slot 3  — hi-beam right
inline constexpr std::uint32_t kSigBulbFeedLine_LLBH    = 4004U;  // slot 4  — lo-beam left
inline constexpr std::uint32_t kSigBulbFeedLine_RLBH    = 4005U;  // slot 5  — lo-beam right
inline constexpr std::uint32_t kSigBulbFeedLine_LRSM    = 4006U;  // slot 6  — rear side-marker left
inline constexpr std::uint32_t kSigBulbFeedLine_RRSM    = 4007U;  // slot 7  — rear side-marker right
inline constexpr std::uint32_t kSigBulbFeedLine_LFML    = 4008U;  // slot 8  — front marker left
inline constexpr std::uint32_t kSigBulbFeedLine_RFML    = 4009U;  // slot 9  — front marker right
inline constexpr std::uint32_t kSigBulbFeedLine_LFTS    = 4010U;  // slot 10 — front turn signal left
inline constexpr std::uint32_t kSigBulbFeedLine_RFTS    = 4011U;  // slot 11 — front turn signal right
inline constexpr std::uint32_t kSigBulbFeedLine_LRTS    = 4012U;  // slot 12 — rear turn signal left
inline constexpr std::uint32_t kSigBulbFeedLine_RRTS    = 4013U;  // slot 13 — rear turn signal right
inline constexpr std::uint32_t kSigBulbFeedLine_LRSL    = 4014U;  // slot 14 — stop lamp left
inline constexpr std::uint32_t kSigBulbFeedLine_CHMSL   = 4015U;  // slot 15 — CHMSL
inline constexpr std::uint32_t kSigBulbFeedLine_RRSL    = 4016U;  // slot 16 — stop lamp right
// 4017 = LRTL, 4018 = RRTL — ev1sim dual-filament tail elements; not modeled here.

// Convenience range for iteration (inclusive).
inline constexpr std::uint32_t kSigBulbFeedLine_First   = kSigBulbFeedLine_LBL;
inline constexpr std::uint32_t kSigBulbFeedLine_Last    = kSigBulbFeedLine_RRSL;
inline constexpr int           kBulbFeedLineCount       = 17;  // slots 0–16

// ---------------------------------------------------------------------------
// Horn drive lines (electricsim → ev1sim)
// Encoding: 1-bit unsigned (bool); 1 = relay energized.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigHornDriveLine_Low    = 4020U;  // low-tone horn relay
inline constexpr std::uint32_t kSigHornDriveLine_High   = 4021U;  // high-tone horn relay

// ---------------------------------------------------------------------------
// Combination switch outputs (ev1sim → electricsim)
// The driver-facing turn/headlamp/cruise stalk has a 6-way connector
// (12084699 blue) on the EV1.  Three of the six pins carry meaningful
// information out to LHJB; the other three are B+ supply lines.  Each output
// pin is published as a 1-bit signal on the chassis segment when its state
// changes.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigCombSw_LowBeamOut       = 4040U;  // pin C, YEL 525B
inline constexpr std::uint32_t kSigCombSw_FlashToPassOut   = 4041U;  // pin B, PPL 524B
inline constexpr std::uint32_t kSigCombSw_ParkHeadlampOut  = 4042U;  // pin F, LTBLU 74

// ---------------------------------------------------------------------------
// Turn/hazard combination switch outputs (ev1sim → electricsim)
// Steering-column "combination switch" — connector 12092237 (DARK GRAY), the
// turn/hazard/horn stalk (ev1-connections/ev1_peripherals.yaml turn_hazard_switch).
// Only the cavities carrying meaningful output to LHJB are published; the 41G
// RUN1 supply (cavity E) is power *into* the switch, gated downstream by LHJB,
// and is not a bus signal.  The hazard signal carries the derived hazard-active
// state, not the raw 640H B+ supply (cavity D) — mirroring how the headlamp
// switch above publishes derived pin truth, never a supply pin.  Horn is a
// SINGLE command (circuit 28); there is no driver hi/lo horn choice — LHJB
// energizes both the 400 Hz and 500 Hz sounders (4020/4021) together.
// Each is a 1-bit signal published on the chassis segment when its state changes.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigTurnHazSw_RightTurnOut  = 4043U;  // pin B, DK BLU/WHT 1415
inline constexpr std::uint32_t kSigTurnHazSw_LeftTurnOut   = 4044U;  // pin C, LT BLU/WHT 1414
inline constexpr std::uint32_t kSigTurnHazSw_HazardOut     = 4045U;  // hazard active (cavity D 640H)
inline constexpr std::uint32_t kSigTurnHazSw_HornOut       = 4046U;  // pin H, BLK 28 (single horn cmd)

// ---------------------------------------------------------------------------
// Cruise-control switch outputs (ev1sim → electricsim)
// Steering-column cruise stalk — three raw contacts wired to PIM J1 (redux
// ev1_peripherals.yaml cruise_control_switch; ESM p.548).  PIM's
// pim_cruise_input decoder turns these into cruise actions (tap → SET/RESUME,
// hold → SPEED_DOWN/UP, ON/OFF falling edge → CANCEL).  Each is a 1-bit signal
// published on the chassis segment on change.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigCruiseSw_SetCoastOut    = 4047U;  // circuit 84,  PIM J1 pin 9
inline constexpr std::uint32_t kSigCruiseSw_ResumeAccelOut = 4048U;  // circuit 87,  PIM J1 pin 25
inline constexpr std::uint32_t kSigCruiseSw_OnOffOut       = 4049U;  // circuit 397, PIM J1 pin 10

// ---------------------------------------------------------------------------
// PRND selector lines (ev1sim → electricsim)
// Models the four PIM prnd_a/b/c/d cavities — physical wires from the floor
// selector lever to the PIM (propulsion manual p. 343).
//
// Encoding (Gray-coded with parity; not one-hot):
//   PARK    A=0 B=1 C=1 D=0   (0110)  — source: pim_prnd.c / pinout.yaml §343
//   REVERSE A=0 B=0 C=1 D=1   (0011)
//   NEUTRAL A=1 B=0 C=1 D=0   (1010)
//   DRIVE   A=1 B=0 C=0 D=1   (1001)
//   D is an even-parity bit over A,B,C,D (enables single-switch fault detection).
//   Any other 4-bit pattern is invalid (DTC 061 after 60 s hold; PIM assumes N).
//
// Encoding: 1-bit unsigned (bool); 1-byte uint8 payload per chassis-bus
// convention.  LOW (false) = line grounded; HIGH (true) = line open/high.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigPrndSelector_A = 4050U;  // prnd_a cavity
inline constexpr std::uint32_t kSigPrndSelector_B = 4051U;  // prnd_b cavity
inline constexpr std::uint32_t kSigPrndSelector_C = 4052U;  // prnd_c cavity
inline constexpr std::uint32_t kSigPrndSelector_D = 4053U;  // prnd_d cavity (parity)

// ---------------------------------------------------------------------------
// Wiper/washer switch outputs (ev1sim → electricsim)
// Steering-column wiper/washer switch — connector 12092254 (BLACK)
// (ev1-connections/ev1_peripherals.yaml wiper_washer_switch).  HI layers on top
// of the LOW "request" wire, so RHJB's decode
// `hi?HIGH : request?LOW : delay?INT : OFF` is order-robust.  The 42A B+ supply
// (cavity F) is not published.  Each is a 1-bit signal published on the chassis
// segment on change.
// VERIFY: position->cavity truth table against EV1 ESM p.643 (RHJB must mirror).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigWiperSw_DelayOut        = 4054U;  // pin B, GRA 112  (INT)
inline constexpr std::uint32_t kSigWiperSw_RequestOut      = 4055U;  // pin C, DK GRN 113 (LOW/HIGH)
inline constexpr std::uint32_t kSigWiperSw_HiOut           = 4056U;  // pin D, PPL 92   (HIGH)
inline constexpr std::uint32_t kSigWiperSw_WasherSwitchOut = 4057U;  // pin E, PNK 228C (wash)

// ---------------------------------------------------------------------------
// Charge coupler presence (chassis bus, both directions)
// Chassis ID 4060.
//
// True when the J1772/Avcon paddle is mated to the vehicle.  Two
// publishers may write to this signal:
//   - ev1sim, when the user toggles the floating-UI charger panel
//   - ev1/charger/ peer, in standalone-demo mode where it owns the
//     coupler-mated state (no external operator UI)
//
// Consumers:
//   - BPM     (gates `bpm_supervisor_set_coupler_present`, drives
//              charge-state SM transitions)
//   - PIM     (disable drive while coupler present — propulsion
//              control module's cavity-7 signal in pinout.yaml)
//   - charger peer (its own SM uses this to enter / leave
//                   WAIT_FOR_COUPLER → IDLE)
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChargerCouplerPresent = 4060U;

// ---------------------------------------------------------------------------
// Panel ajar switches (ev1sim → electricsim)
// Encoding: 1-bit unsigned (bool); 1 = ajar / open.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigPanelAjar_Hood       = 4030U;
inline constexpr std::uint32_t kSigPanelAjar_Trunk      = 4031U;
inline constexpr std::uint32_t kSigPanelAjar_DoorLeft   = 4032U;
inline constexpr std::uint32_t kSigPanelAjar_DoorRight  = 4033U;

// ---------------------------------------------------------------------------
// Vehicle dynamics (ev1sim → electricsim)
// All payloads: IEEE 754 float32, little-endian, 4 bytes.
// Qualified names match ev1sim's ExternalSimConnector kDynamicsNames[] table.
// ---------------------------------------------------------------------------

// Forward speed (m/s).
inline constexpr std::uint32_t kSigChassisSpeedMps           = 4100U;

// Chassis-frame accelerations (m/s²).
inline constexpr std::uint32_t kSigChassisAccelLong          = 4101U;
inline constexpr std::uint32_t kSigChassisAccelLat           = 4102U;

// Yaw rate (rad/s).
inline constexpr std::uint32_t kSigChassisYawRate            = 4103U;

// Commanded (applied) driver inputs echoed through the physics model.
inline constexpr std::uint32_t kSigChassisAppliedThrottle    = 4104U;  // 0..1
inline constexpr std::uint32_t kSigChassisAppliedFrontBrake  = 4105U;  // 0..1
inline constexpr std::uint32_t kSigChassisAppliedRearBrake   = 4106U;  // 0..1

// Actuated brake states after hydraulic lag and rate-limiting.
inline constexpr std::uint32_t kSigChassisFrontBrakePressure = 4107U;  // 0..1
inline constexpr std::uint32_t kSigChassisRearBrakePosition  = 4108U;  // 0..1

// Front-axle steering reaction torque (Nm) — sum of front-tire Mz moments.
// FFB peers use this for on-centre return, grip-loss feel, and speed-dependent
// build-up; consumers apply their own scaling and direction inversion.
inline constexpr std::uint32_t kSigChassisSteeringTorque     = 4109U;

// Wheel rotational speeds (rad/s) — FL, FR, RL, RR.
inline constexpr std::uint32_t kSigChassisWheelOmegaFL       = 4110U;
inline constexpr std::uint32_t kSigChassisWheelOmegaFR       = 4111U;
inline constexpr std::uint32_t kSigChassisWheelOmegaRL       = 4112U;
inline constexpr std::uint32_t kSigChassisWheelOmegaRR       = 4113U;

// Wheel slip ratios (0 = free-rolling, +1 = locked, −1 = spinning).
inline constexpr std::uint32_t kSigChassisSlipRatioFL        = 4120U;
inline constexpr std::uint32_t kSigChassisSlipRatioFR        = 4121U;
inline constexpr std::uint32_t kSigChassisSlipRatioRL        = 4122U;
inline constexpr std::uint32_t kSigChassisSlipRatioRR        = 4123U;

// Motor state (ev1sim → electricsim) — IEEE 754 float32 LE, 4 bytes each.
// Published by ev1sim's Chrono motor model each physics step.
// PIM subscribes to these to cache latest motor state for scan-tool visibility
// and future efficiency-curve / regen calculations.
inline constexpr std::uint32_t kSigChassisMotorRpm           = 4070U;  // signed rpm (negative = reverse)
inline constexpr std::uint32_t kSigChassisMotorTorqueNm      = 4071U;  // signed Nm (positive = drive, negative = regen)

// Derived motor current (electricsim/PIM → chassis bus) — IEEE 754 float32 LE, 4 bytes.
// Computed by PIM each tick: I = (torque_nm × omega_rad_s) / (V_pack × eta).
// Negative = regen (current flowing into pack). Published at 1/kTickMs Hz cadence.
// TODO: replace V_pack=312V nominal + eta=0.92 constant with real pack voltage
// subscription (BPM publishes pack_v in its 16-byte status frame) and a 2-D
// efficiency map once the manufacturer efficiency curve is digitized.
inline constexpr std::uint32_t kSigChassisMotorCurrentA      = 4072U;  // signed A, float32 LE

// ---------------------------------------------------------------------------
// Throttle command (electricsim/PIM → ev1sim chassis bus)
// Signal ID 4073.  Encoding: uint8 q8 — 0=zero throttle, 255=full throttle.
// PIM consumes the driver throttle pedal (kSigDriverThrottleQ8 = 6903), applies
// cruise-control authority when cruise is ACTIVE (P-controller toward setpoint
// speed), and publishes the resulting commanded throttle here.  ev1sim's
// Chrono powertrain consumes this signal when running in
// "vehicle.dynamics.driver = electronics" mode; otherwise it falls back to
// the local pedal input.  Stale-after threshold: 200 ms (matches ABS pattern).
// Published every PIM tick on change.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisThrottleCmdQ8      = 4073U;

// ---------------------------------------------------------------------------
// Brake master cylinder pressure (ev1sim → electricsim chassis bus)
// Signal ID 4074.  Encoding: float32 LE, kPa.  0 = released; ~12000-15000 at
// full pedal travel (typical passenger-car spec).
//
// ev1sim's BrakePedal PhysicalWorld component computes this from normalized
// pedal travel via a two-stage curve:
//   travel < dead_band            → 0 kPa
//   travel < transition           → k1 × (t - dead_band)
//   else                          → soft + k2 × (t - transition)
//
// The two-stage shape matches the feel of a hardware brake pedal with a
// two-stage spring on a sim-racing rig (a real spring resists motion, the
// simulator computes the resulting hydraulic pressure).
//
// BTCM consumes this as the primary brake-effort input.  The existing
// kSigDriverBrakeSwitch (6904) keeps its role as the ABS pump-prime
// threshold — a binary "any meaningful pedal application" signal.
//
// Default calibration in ev1sim/src/PhysicalWorld.h `BrakePedal::Calibration`:
//   dead_band  = 0.07
//   transition = 0.35
//   k1         = 8000  kPa per unit travel
//   k2         = 18000 kPa per unit travel
//   max        = 15000 kPa
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisBrakeMasterPressureKpa = 4074U;

// ---------------------------------------------------------------------------
// Sim-time clock (ev1sim → electricsim chassis bus)
// Signal ID 4075.  Encoding: uint64_t LE, 8 bytes, nanoseconds since the
// sim start (i.e. monotonic, zero-based).
//
// Published by ev1sim every Chrono physics step with the current simulated
// time in nanoseconds.  Subscribed by each electricsim controller's
// SimClock (src/io/sim_clock.{hpp,cpp}); when present, the controller
// derives tick cadence, sleeps, and simavr cycle-advance budgets from
// sim-time deltas instead of host wall-clock.  When absent (standalone
// test-plant mode, no ev1sim) the SimClock stays on std::chrono::
// steady_clock — the existing wall-clock pattern is the fallback.
//
// Why nanoseconds (uint64): a Chrono step is typically 1 ms (1e6 ns); a
// uint64 nanosecond counter wraps after ~584 years of sim time which is
// well past any plausible run length.  The same payload encoding the
// 16-byte SHM frame already supports — see protocol.cpp f64/u64 codecs.
//
// Faster-than-realtime: ev1sim's `config.simulation.realtime = false`
// causes Chrono to advance multiple sim-time units per wall-clock unit;
// kSigChassisSimTimeNs reflects sim time, so subscribers naturally
// "see" the faster cadence without any host-side acceleration.
//
// Mid-run mode switches between realtime and non-realtime are out of
// scope and require a controller restart; the SimClock has a
// monotonicity guard but won't smoothly handle a downward-jumping
// time source.
//
// @design 2026-05-15 claude-opus-4.7 — slot 4075 chosen as the first
// unused chassis ID after the motor/throttle/brake block (4070–4074).
// docs/TODO.md "Sim-time sync (faster-than-realtime support)".
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisSimTimeNs = 4075U;

// ---------------------------------------------------------------------------
// Wiper motor command (electricsim/RHJB → ev1sim chassis bus)
// Signal ID 4080.  Encoding: uint8 enum — 0=OFF, 1=INT, 2=LOW, 3=HIGH.
// RHJB decodes `kSigDriverWiperSwitch` (6958) and gates on run1; publishes
// on change.  ev1sim renders the front wiper sweep cadence accordingly.
// Intermittent cadence timing is deferred — RHJB publishes the mode only.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisWiperMotorCommand  = 4080U;

// ---------------------------------------------------------------------------
// Washer pump command (electricsim/RHJB → ev1sim chassis bus)
// Signal ID 4081.  Encoding: uint8 bool — 0=idle, 1=pump active.
// Passthrough of `kSigDriverWiperWashRequest` (6959) gated on run1.  The
// pump runs as long as the wash button is held; RHJB publishes on change.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisWasherPumpCommand  = 4081U;

// ---------------------------------------------------------------------------
// HVAC blower level (electricsim/HTCM → ev1sim chassis bus)
// Signal ID 4082.  Encoding: uint8 enum — 0=OFF, 1=LOW, 2=MED, 3=HIGH.
// HTCM is the publisher.  ev1sim eventually renders blower noise at the
// appropriate level; future real control logic will derive this from
// temperature setpoint, ambient temp, etc.  Placeholder at startup: 0=OFF.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisHvacBlowerLevel    = 4082U;

// ---------------------------------------------------------------------------
// Defrost grid active (electricsim/HTCM → ev1sim chassis bus)
// Signal ID 4083.  Encoding: bool (uint8: 0=inactive, 1=active).
// HTCM is the publisher.  ev1sim eventually renders the rear defrost grid
// visual when active.  Placeholder at startup: false=inactive.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisDefrostGridActive  = 4083U;

// ---------------------------------------------------------------------------
// Door lock commands (electricsim/RSA → ev1sim chassis bus)
// Signal IDs 4084-4085.  RSA publishes these when it decides to lock or
// unlock a door (e.g. after exterior keypad auth + door handle attempt).
// Encoding: uint8 — 0=unlocked, 1=locked.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisDoorLockCmdDriver    = 4084U;  // driver door
inline constexpr std::uint32_t kSigChassisDoorLockCmdPassenger = 4085U;  // passenger door

// ---------------------------------------------------------------------------
// Power window motor commands (electricsim/RSA → ev1sim chassis bus)
// Signal IDs 4086-4087.  RSA publishes the motor direction while the user
// holds the corresponding switch.  Gated on run_mode != OFF.
// Encoding: uint8 — 0=stop, 1=up, 2=down.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisPowerWindowMotorDriver    = 4086U;  // driver window
inline constexpr std::uint32_t kSigChassisPowerWindowMotorPassenger = 4087U;  // passenger window

// ---------------------------------------------------------------------------
// RSA shift-blocked cue (electricsim/RSA → ev1sim chassis bus)
// Signal ID 4088.  Encoding: uint8 bool — 0=no block, 1=shift blocked this tick.
// RSA publishes 1 for one tick when a P→non-P shift is refused because the
// brake pedal switch is not pressed (shift-out-of-park interlock solenoid model).
// The pulse drops back to 0 on the following tick.  ev1sim uses this for the
// floating-UI "Shift Blocked: BRAKE TO SHIFT" cue.
// Momentary semantics: rising-edge on the blocked tick, 0 otherwise.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisRsaShiftBlocked = 4088U;

// ---------------------------------------------------------------------------
// Ambient environment sensors (ev1sim → electricsim chassis bus)
// Publisher: ev1sim's AmbientTempSensor component (PhysicalWorld).
// Default model: naive diurnal sinusoid (almanac-style); no live weather API.
//   mean_temp=18°C, diurnal_amp=8°C, phase_peak=14h (2 pm).
//   humidity inversely correlated with temperature.
// Intended consumers: HTCM (heat-pump model), BPM (battery thermal model).
// No consumer wiring yet — future round will subscribe both modules.
// Encoding: IEEE 754 float32, little-endian, 4 bytes.
// ---------------------------------------------------------------------------

// Ambient air temperature (°C).  float32 LE.
inline constexpr std::uint32_t kSigChassisAmbientTempC         = 4090U;

// Ambient relative humidity (%).  float32 LE.  Range: 0..100.
inline constexpr std::uint32_t kSigChassisAmbientHumidityPct   = 4091U;

// ---------------------------------------------------------------------------
// IPC seatbelt telltale outputs (electricsim/IPC → ev1sim chassis bus)
// Signal IDs 4130–4131.  IPC publishes these on change each tick so
// the 3D sim and floating UI can reflect the lamp state independently
// of the full IPC LCD summary (5600).
// Encoding: uint8 bool — 0=lamp off, 1=lamp on.
// Lit when the corresponding seat is unbuckled AND vehicle speed exceeds
// ~8 km/h (IPC_SUPERVISOR_SEATBELT_SPEED_THRESHOLD_MPS).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisIpcSeatbeltTelltaleDriver    = 4130U;
inline constexpr std::uint32_t kSigChassisIpcSeatbeltTelltalePassenger = 4131U;

// ---------------------------------------------------------------------------
// IPC trip distance (electricsim/IPC → ev1sim chassis bus)
// Signal ID 4132.  IPC publishes the accumulated trip distance on change
// (epsilon ~0.5 m) after each supervisor tick so the 3D sim and floating
// UI can display trip mileage independent of the full IPC LCD summary.
// Encoding: IEEE 754 float32, little-endian, 4 bytes.  Units: metres.
// Value resets to 0.0 on a trip-reset pulse (kSigDriverIpcTripResetButton).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisIpcTripDistanceM = 4132U;

// ---------------------------------------------------------------------------
// APM B+ rail active (electricsim/APM → chassis bus)
// Signal ID 4133.  APM publishes this on change each tick.
// Encoding: uint8 bool — 0=B+ not available, 1=B+ active.
// True when the APM supervisor is keyed on (key_on == true) and has not
// latched a shutdown (shutdown_latched == false).  LHJB subscribes to
// gate all lamp outputs: when false, even the hazard B+ bypass is dark.
// Default at LHJB startup is false (safe — outputs off until APM speaks).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisApmBPlusActive = 4133U;

// ---------------------------------------------------------------------------
// IPC BTCM / airbag telltale outputs (electricsim/IPC → ev1sim chassis bus)
// Signal IDs 4134–4138.  IPC publishes these on change each tick so the
// floating UI can reflect BTCM-reported lamp state and the airbag indicator
// independently of the full IPC LCD summary (5600).
// Encoding: uint8 bool — 0=lamp off, 1=lamp on.
//
// Sources in the supervisor:
//   4134 IPC_LCD_SYM_BRAKE      — DTC 42, set via ipc_supervisor_ingest_btcm_payload
//   4135 IPC_LCD_SYM_PARK_BRAKE — DTC 44, set via ipc_supervisor_ingest_btcm_payload
//   4136 IPC_LCD_SYM_ANTILOCK   — DTC 41, set via ipc_supervisor_ingest_btcm_payload
//   4137 IPC_LCD_SYM_LOW_TRAC   — DTC 43, set via ipc_supervisor_ingest_btcm_payload
//   4138 IPC_LCD_SYM_AIR_BAG    — DTC 40, set via ipc_supervisor_set_airbag_input
//
// Skipped (declared in ipc_lcd.h but never driven in current supervisor code):
//   IPC_LCD_SYM_HIGH_BEAM, IPC_LCD_SYM_LEFT_TURN, IPC_LCD_SYM_RIGHT_TURN,
//   IPC_LCD_SYM_WAIT — future work.
// Deferred (driven but lower value / complex semantics; cap of 5 applied):
//   IPC_LCD_SYM_SERVICE_NOW, IPC_LCD_SYM_TEMP, IPC_LCD_SYM_BATTERY_LIFE,
//   IPC_LCD_SYM_REDUCED_PERF, IPC_LCD_SYM_CHECK_TIRE_PRESS,
//   IPC_LCD_SYM_CHECK_MESSAGES — TODO: surface in a follow-up round.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisIpcBrakeTelltale     = 4134U;
inline constexpr std::uint32_t kSigChassisIpcParkBrakeTelltale = 4135U;
inline constexpr std::uint32_t kSigChassisIpcAntilockTelltale  = 4136U;
inline constexpr std::uint32_t kSigChassisIpcLowTracTelltale   = 4137U;
inline constexpr std::uint32_t kSigChassisIpcAirBagTelltale    = 4138U;

// ---------------------------------------------------------------------------
// 4139 — historical: kSigChassisBpmPackVoltageMv (BPM pack terminal voltage,
// uint32 mV). Retired 2026-06-15 (Phase C-b retire-not-migrate): PIM, the
// only consumer, moved to the modelled HV-bus rail (kSigChassisHvBusVoltageMv
// 4155) because the rail is the physically-correct node for the motor-current
// formula (see ev1/pim/controller.cpp ~L555). The BPM publish then had no live
// consumer and no ev1sim contract use (0 mentions in docs/3d_sim_contract.md),
// so it was removed instead of migrated to a wire. The numeric ID is held
// reserved here (not reused) and the constant is left as a tombstone for
// historical context. @design 2026-06-15 claude;
// notes/phase_c_chassis_migration_scope.md §9.1.

// ---------------------------------------------------------------------------
// IPC extra LCD telltale outputs (electricsim/IPC → ev1sim chassis bus)
// Signal IDs 4140–4145.  IPC publishes these on change each tick so the
// floating UI can reflect lamp state independently of the full IPC LCD
// summary (5600).  All are driven by existing supervisor logic.
// Encoding: uint8 bool — 0=lamp off, 1=lamp on.
//
// Sources in the supervisor (update_lcd_from_dtcs_):
//   4140 IPC_LCD_SYM_SERVICE_NOW    — DTCs 31/33/35/45 via telltale requests
//                                      (HTCM/PCM/BPM aggregator)
//   4141 IPC_LCD_SYM_CHECK_MESSAGES — aggregate: any comm loss OR any request
//                                      OR any BTCM indicator
//   4142 IPC_LCD_SYM_TEMP           — DTCs 32/34/36 via telltale requests
//                                      (HTCM/PCM/BPM aggregator)
//   4143 IPC_LCD_SYM_BATTERY_LIFE   — DTC 38 via IPC_REQ_BATTERY_LIFE_FROM_BPM
//   4144 IPC_LCD_SYM_REDUCED_PERF   — DTC 37 via IPC_REQ_REDUCED_PERF_FROM_PCM
//   4145 IPC_LCD_SYM_CHECK_TIRE_PRESS — DTC 39 via
//                                        IPC_REQ_CHECK_TIRE_PRESS_FROM_RSA
//
// Not yet exposed (declared in ipc_lcd.h but supervisor never drives them):
//   IPC_LCD_SYM_WAIT, IPC_LCD_SYM_HIGH_BEAM, IPC_LCD_SYM_LEFT_TURN,
//   IPC_LCD_SYM_RIGHT_TURN — future work; drive from supervisor first.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisIpcServiceNowTelltale    = 4140U;
inline constexpr std::uint32_t kSigChassisIpcCheckMessagesTelltale = 4141U;
inline constexpr std::uint32_t kSigChassisIpcTempTelltale          = 4142U;
inline constexpr std::uint32_t kSigChassisIpcBatteryLifeTelltale   = 4143U;
inline constexpr std::uint32_t kSigChassisIpcReducedPerfTelltale   = 4144U;
inline constexpr std::uint32_t kSigChassisIpcCheckTirePressTelltale = 4145U;

// ---------------------------------------------------------------------------
// BTCM Enhanced Drive Control regen-reduction factor (BTCM → chassis bus)
// Signal ID 4146. Encoding: uint8 Q8 (0..255). 255 = no derate (full
// requested regen authority passes through), 0 = full cut (PCM regen
// request multiplied to zero).
//
// BTCM controller publishes this on change from the firmware's
// `g_abs_diag.edc_derate_factor_q8` snapshot. Default at startup is 255
// (no derate) so consumers default to "regen unchanged" until the BTCM
// has actually run a tick and reported.
//
// Source: ev1/btcm/abs_algo.c::abs_edc_update. "ABS for regen" — when
// drive-wheel slip during coastdown indicates the regen torque is
// dragging the fronts below ground speed, the BTCM ramps this factor
// down so the PCM (PIM) backs off enough regen authority to let the
// wheels spin back up.
//
// Consumers:
//   - PIM caches and exposes via supervisor.scan for observability.
//     A future revision can multiply outgoing regen-torque commands by
//     (q8 / 255.0f) directly. Today the PIM does not produce an
//     outgoing regen torque command (PIM IS the PCM in this sim), so
//     the cache + scan exposure is the minimum end-to-end path.
//
// Per brakes manual p. 313 + docs/btcm_deferred_todos.md §7b.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisBtcmRegenReductionQ8 = 4146U;

// ---------------------------------------------------------------------------
// BTCM brake actuator state (electricsim/BTCM → ev1sim chassis bus)
// Signal IDs 4147–4154.  BTCM publishes per-wheel actuator state on the
// chassis bus on change after each main-loop iteration so ev1sim's Project
// Chrono physics can drive per-corner brake torque from the BTCM-modulated
// actuator state instead of the local brake pedal.  This makes ABS modulation
// visible in vehicle dynamics.
//
// Spec: docs/TODO.md "Bus-mediated physics" → Brake actuator → physics path.
// @design 2026-05-15 brake actuator → physics chassis path.
//
// Direction conventions:
//   - Iso-close booleans mirror the BTCM `kSigSolFL_ISO` / `kSigSolFR_ISO`
//     pin state (1 = solenoid energized = iso valve closed, isolating the
//     caliper from the master cylinder so the front-modulator motor controls
//     per-wheel pressure).
//   - Dump-open booleans mirror the BTCM `kSigSolFL_DMP` / `kSigSolFR_DMP`
//     bypass-to-accumulator pin state (1 = solenoid energized = bypass open,
//     bleeding caliper pressure to the accumulator for pressure reduction).
//   - EMB motor command floats are signed in [-1.0, +1.0] (matching the
//     `kSigRearMotorLR` / `kSigRearMotorRR` main-bus encoding): positive =
//     apply direction, negative = release direction, zero = hold/off.
//   - Cylinder pressure floats are the BTCM's view (i.e. echoed sensor value)
//     of LF/RF wheel cylinder pressure in kPa.  ev1sim uses these to drive
//     per-corner caliper torque under ABS modulation.  Source: the
//     `kSigPressLF` / `kSigPressRF` main-bus signals (MPa) the BTCM receives
//     from `vehicle_sim` / ev1sim, converted to kPa for chassis-side
//     consumption.  Stale-after threshold (ev1sim side): 200 ms (matches
//     the rest of the brake chain).
//
// Publish cadence: on change.  Epsilon thresholds:
//   - iso/dump bools: exact change (no epsilon).
//   - EMB motor command: ±0.01 (matches the ±1.0 range and avoids flooding
//     on numerical noise from the firmware H-bridge duty math).
//   - Cylinder pressure: ±1.0 kPa (≈ 0.001 MPa, well below the resolution
//     of the firmware's Q-format scratch, but enough to keep an ABS dump
//     cycle visible as discrete steps).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisBtcmIsoCloseFL          = 4147U;
inline constexpr std::uint32_t kSigChassisBtcmIsoCloseFR          = 4148U;
inline constexpr std::uint32_t kSigChassisBtcmDumpOpenFL          = 4149U;
inline constexpr std::uint32_t kSigChassisBtcmDumpOpenFR          = 4150U;
inline constexpr std::uint32_t kSigChassisBtcmEmbMotorCmdLR       = 4151U;
inline constexpr std::uint32_t kSigChassisBtcmEmbMotorCmdRR       = 4152U;
inline constexpr std::uint32_t kSigChassisBtcmCylPressureFL_kPa   = 4153U;
inline constexpr std::uint32_t kSigChassisBtcmCylPressureFR_kPa   = 4154U;

// ---------------------------------------------------------------------------
// HV distribution-bus rail (electricsim/hv_bus plant → chassis bus)
// Signal IDs 4155–4157.  The HV-bus plant (src/models/hv_bus/hv_bus_plant)
// publishes the sensed DC-link / PIM HV± bus-bar state so live consumers
// (AD precharge tracking, PIM pack_voltage, APM hv_input, PSCM hv_present)
// can read the MODELLED rail instead of their current externally-fed priors
// (round-7 Item 1; notes/round7_work_items.md).
//
// Publish cadence: on change.  Epsilon thresholds:
//   - bus voltage: ±100 mV (well below the ~42.5 V HV-present threshold and
//     the 180–440 V PIM band edges; keeps a precharge ramp visible as steps).
//   - bus current: ±50 mA.
//   - hv_present bool: exact change.
//
// Encodings:
//   4155 bus_voltage_mv   — uint32, millivolts (0 = de-energized).
//   4156 hv_present       — uint8 bool — bus_voltage_mv ≥ EV1_HV_BUS_HV_PRESENT_MV.
//   4157 bus_current_ma   — uint32, milliamps of total DRAW (Σ positive studs;
//                           the unsigned sink total the source side supplies).
// @design 2026-06-04 — see docs/chassis_contract_versioning.md for the chassis
// contract; the rail values come from ev1_hv_bus_plant outputs.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisHvBusVoltageMv          = 4155U;
inline constexpr std::uint32_t kSigChassisHvBusPresent            = 4156U;
inline constexpr std::uint32_t kSigChassisHvBusCurrentMa          = 4157U;

// ---------------------------------------------------------------------------
// IPC discrete-display telltale outputs (electricsim/IPC → ev1sim chassis bus)
// Signal IDs 4158–4163.  IPC publishes these on change each tick so the
// floating UI / VFD renderer can reflect the cluster indicators driven by the
// IPC's *discrete display inputs* — turn arrows, high beam, park lamp, door
// ajar — plus the VFD dim level.  These complete the "future work" left by the
// 4134–4145 telltale blocks (those comments list HIGH_BEAM / LEFT_TURN /
// RIGHT_TURN as "drive from supervisor first").
//
// IMPORTANT: these are DISPLAY indicators, not fault telltales.  The EV1
// Electrical manual prints NO IPC DTC for any of the turn / park-lamp /
// door-ajar / dim discrete inputs (the IPC DTC set is {011-016, 031-045};
// DTC 031 = HTCM SERVICE NOW TELLTALE, a peer telltale request, NOT a
// discrete-input fault).  See
// notes/manual_supplements.yaml#2026-06-07-ipc-discrete-display-inputs.
//
// Sources in the supervisor (update_display_inputs_), fed from the named
// chassis-bus producers by the IPC controller:
//   4158 IPC_LCD_SYM_LEFT_TURN  — C6 1334C  <- kSigBulbFeedLine_LRTS (4012)
//   4159 IPC_LCD_SYM_RIGHT_TURN — C8 1335C  <- kSigBulbFeedLine_RRTS (4013)
//   4160 IPC_LCD_SYM_HIGH_BEAM  — D14 1192  <- kSigBulbFeedLine_LHBH (4002)
//   4161 IPC_LCD_SYM_PARK_LAMP  — C4 301C   <- kSigChassisLhjbParkLampsOn (4190) faithful net
//   4162 IPC_LCD_SYM_DOOR_AJAR  — C16 1495F <- kSigPanelAjar_DoorLeft|Right (4032/4033)
//   4163 dim_pwm_duty_pct (0..100) — C12 230E <- kSigChassisRsaDimLevelStep (4189) RSA host-bridge
// Park-lamp + dim producer gaps resolved by @design 2026-06-12;
// see notes/dtc_pass/concerns.yaml#2026-06-07-ipc-dim-and-parklamp-producer-gap.
//
// Encoding: 4158–4162 uint8 bool (0=off, 1=on); 4163 uint8 duty percent (0..100).
// @design 2026-06-07 — first free block above the HV bus signals (4155–4157).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisIpcLeftTurnTelltale   = 4158U;
inline constexpr std::uint32_t kSigChassisIpcRightTurnTelltale  = 4159U;
inline constexpr std::uint32_t kSigChassisIpcHighBeamTelltale   = 4160U;
inline constexpr std::uint32_t kSigChassisIpcParkLampTelltale   = 4161U;
inline constexpr std::uint32_t kSigChassisIpcDoorAjarTelltale   = 4162U;
inline constexpr std::uint32_t kSigChassisIpcDimDutyPct         = 4163U;

// ---------------------------------------------------------------------------
// Door-lock STATE feedback (ev1sim → electricsim chassis bus)
// Signal IDs 4165–4167.  The physical lock state of each door after the DLM
// motor pulse acts (ev1sim's body model is the authority on where the
// linkage actually ended up): uint8 bool, 0 = unlocked, 1 = locked,
// published on change (plus an initial publish so consumers latch the
// startup state).  No electricsim consumer is wired yet; the natural one is
// RSA correlating commanded lock state (kSigChassisDoorLockCmd*, 4084/4085)
// against achieved state.
//
// HISTORY (@design 2026-06-11): ev1sim had been publishing this trio on
// 4155–4157 — IDs this contract allocates to the HV bus rail — because the
// trio was never adopted into the contract header and the drift guard can
// only check IDs that exist on both sides. Live collision in every co-sim
// run; details + the guard-gap lesson in
// notes/manual_supplements.yaml#2026-06-11-chassis-4155-doorlock-collision.
// Allocated here from the free block above the IPC discrete-display
// telltales (4158–4163; 4164 left spare).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisDoorLockStateDriver    = 4165U;
inline constexpr std::uint32_t kSigChassisDoorLockStatePassenger = 4166U;
inline constexpr std::uint32_t kSigChassisDoorLockStateTrunk     = 4167U;

// ---------------------------------------------------------------------------
// Road grade + chassis pitch (ev1sim → electricsim chassis bus)
// Signal IDs 4168–4169.  float32 LE, published on change by the physics sim:
//   4168 road_grade_pct — slope of the ROAD under the wheelbase, percent
//        (100 × rise/run between the rear- and front-axle contact heights;
//        positive = uphill in the direction of travel). Derived from wheel
//        spindle positions, so body squat/dive does NOT leak into it.
//   4169 pitch_deg      — chassis BODY attitude about the lateral axis,
//        degrees, positive = nose up. Includes suspension squat/dive —
//        deliberately distinct from road grade (their difference is the
//        suspension's doing).
// @design 2026-06-11 — semantics are an engineering allocation for grade-
// aware scenarios (cruise-on-grade, hill-hold); the EV1 manuals document no
// pitch/grade display or sensor, so NO claim of production-instrument
// fidelity attaches to these (the IPC shows no pitch readout). No consumer
// wired yet; PIM cruise / future hill-hold logic are the natural ones.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisRoadGradePct = 4168U;
inline constexpr std::uint32_t kSigChassisPitchDeg     = 4169U;

// ---------------------------------------------------------------------------
// Door-lock switch inputs (ev1sim → electricsim chassis bus)
// Signal IDs 4170–4173.  Physical door-lock-switch LOCK/UNLOCK contacts,
// one pair per door (LH = left-hand / driver, RH = right-hand / passenger).
// Each is a 1-bit signal (bool): true = switch closed / contact active.
// The RHJB DLM subscribes to all four to drive its momentary lock/unlock
// motor pulse via ev1_rhjb_door_lock_step().
//
// Physical wiring per redux round-5 / elec-337 (PSMELC67861AB):
//   LH LOCK   — circuit 780A, J9.C13  (↔ LHJB.J9.B6 via body harness)
//   LH UNLOCK — circuit 781A, J9.D16  (↔ LHJB.J8.D14)
//   RH LOCK   — circuit 780C, J3.A2   (direct from RH door switch)
//   RH UNLOCK — circuit 781C, J3.A1
// @design 2026-06-06 — chassis signal IDs 4170-4173 chosen as first free
// block above the HV bus signals (4155-4157).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigDoorLockSw_LhLockOut   = 4170U;  // 780A LH LOCK   (J9.C13)
inline constexpr std::uint32_t kSigDoorLockSw_LhUnlockOut = 4171U;  // 781A LH UNLOCK (J9.D16)
inline constexpr std::uint32_t kSigDoorLockSw_RhLockOut   = 4172U;  // 780C RH LOCK   (J3.A2)
inline constexpr std::uint32_t kSigDoorLockSw_RhUnlockOut = 4173U;  // 781C RH UNLOCK (J3.A1)

// ---------------------------------------------------------------------------
// RHJB sub-module outputs (electricsim/RHJB → ev1sim chassis bus)
// Signal IDs 4180–4186.
//
// PMM (Power Moding Module) run-bus state — two bools gated by the
//   SLEEP/AWAKE/RUN1/RUN2 state machine AND the 5A RUN-1 distribution PTC.
//   4180 run1_bus: true when 41-family bus is energised (state ≥ RUN1
//       and 5A PTC passing).
//       @source:manual electrical p281 — 5A RUN-1 PTC (PSMELC67947AA).
//   4181 run2_bus: true when 241-family bus is energised (state == RUN2).
//       @source:manual electrical p243 — RSA RUN-2 command (circuit 248).
//
// DLM (Door Lock Module) motor-leg drives — four bools (one DLM,
//   two SPDT relays driving both LH and RH motors in lockstep).
//   4182 lh_motor_lock_drive   — circuit 294A, J9.C5  → LH motor LOCK leg
//   4183 lh_motor_unlock_drive — circuit 295A, J9.C6  → LH motor UNLOCK leg
//   4184 rh_motor_lock_drive   — circuit 294C, J3.A6  → RH motor LOCK leg
//   4185 rh_motor_unlock_drive — circuit 295C, J3.A7  → RH motor UNLOCK leg
//   @source:manual electrical p337 — DLM sub-block PSMELC67861AB.
//
// DILM (Delayed Interior Lamp Module) dimming level — uint8 0-255.
//   (0=off, 255=full on; intermediate values represent the linear fade.)
//   4186: drives the 1732-family outputs (map lamp, cargo lamp, four-bar
//       lamp) through the retriggerable off-delay + theatre-dimming ramp.
//       @source:manual electrical p298/p299 — PSMELC67873AB / PSMELC67874AA.
//
// @design 2026-06-06 — IDs 4180-4186 allocated as first free block above
// the door-lock switch inputs (4170-4173), leaving a small gap for future
// door-lock input additions.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisRhjbPmmRun1Bus   = 4180U;  // PMM: 41-family RUN-1 bus active
inline constexpr std::uint32_t kSigChassisRhjbPmmRun2Bus   = 4181U;  // PMM: 241-family RUN-2 bus active
inline constexpr std::uint32_t kSigChassisRhjbDlmLhLock    = 4182U;  // DLM: LH motor LOCK drive (294A)
inline constexpr std::uint32_t kSigChassisRhjbDlmLhUnlock  = 4183U;  // DLM: LH motor UNLOCK drive (295A)
inline constexpr std::uint32_t kSigChassisRhjbDlmRhLock    = 4184U;  // DLM: RH motor LOCK drive (294C)
inline constexpr std::uint32_t kSigChassisRhjbDlmRhUnlock  = 4185U;  // DLM: RH motor UNLOCK drive (295C)
inline constexpr std::uint32_t kSigChassisRhjbDilmLevel    = 4186U;  // DILM: interior lamp level (0-255)

// ---------------------------------------------------------------------------
// LHJB -> RHJB charge-receptacle wake-up passthrough (electricsim-internal,
// chassis bus).  Signal ID 4187.  Encoding: uint8 bool — 1 = wake asserted.
//
// Models circuit 2027G/2027A, the charge-receptacle vehicle wake-up wire:
//   charger_receptacle_lv pin G (WHT 2027G)
//     -> LHJB J1.A2  (WHT 2027G "CHARGE RECEPTACLE WAKE-UP")
//     -> [LHJB-internal cross, segment-letter change 2027G -> 2027A]
//     -> LHJB J8.C16 (WHT 2027A "BATT REC SW TO CHARGE RECEPT")
//     -> RHJB J9.C16 (WHT 2027A "CHARGER RECEPTACLE")
//     -> PMM (Power Moding Module) WAKE input.
// @source:redux ev1_lhjb_module.yaml J1.A2 (2027G) + J8.C16 (2027A);
//   ev1_rhjb_module.yaml J9.C16 (2027A
//   "CHARGER RECEPTACLE"); ESM elec-321 + batt-786 (charge-receptacle wake-up),
//   elec-320/321 (PMM 2027-family wake-up master).
//
// Producer: the LHJB controller, which derives the 2027G assertion from the
//   charge-receptacle coupler-present event it already reads off the chassis
//   bus (kSigChargerCouplerPresent, 4060 — produced by ev1sim / the charger
//   peer): paddle mated -> the receptacle pulses 2027G to wake the vehicle for
//   charging.  Republished here on change as the 2027A segment leaving LHJB
//   J8.C16.  Modeling the LHJB cross as a distinct chassis signal (rather than
//   re-routing 4060 straight into the RHJB) keeps the harness topology honest:
//   the wake-up wire (WHT 2027G) and the coupler-present switch wire
//   (DK GRN 2023 -> PIM) are physically separate circuits.
// Consumer: the RHJB controller, which ORs this into the PMM wake_req so the
//   PMM transitions SLEEP -> AWAKE when a charge paddle is inserted while the
//   vehicle sleeps.
// @design 2026-06-07 — first free chassis ID above the RHJB sub-module block
//   (4180-4186); MINOR-bumped the chassis contract (additive).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisLhjbChargeWakePassthru = 4187U;

// ---------------------------------------------------------------------------
// BTCM discrete REGEN DISABLE OUT line (BTCM -> PCM/PIM, chassis bus).
// Signal ID 4188.  Encoding: uint8 bool — 1 = BTCM commanding regen disable
// (line driven HIGH), 0 = regen permitted (line LOW).
//
// Models circuit 2021, the discrete "REGEN DISABLE" wire BTCM -> PCM that
// runs ALONGSIDE the serial-data copy in the BTCM 0x5A status frame
// (btcm_uart_frame.h regen-flags byte, BTCM_UART_FRAME_REGEN_DISABLE_SERIAL_BIT).
// The PCM (PIM, on J1-7) correlates the discrete line voltage against the
// serial copy and sets DTC 109 (discrete stuck HIGH while serial says LOW)
// or DTC 110 (discrete stuck LOW while serial says HIGH).
// @source:manual prop-228/230 (DTC 109/110): "The BTCM requests regen disable
//   via a discrete input and also by serial data."  Circuit number 2021 is the
//   discrete REGEN DISABLE leg.
//
// Producer: the BTCM controller, which publishes this on change from the SAME
//   supervisor regen-disable state that feeds the serial copy
//   (btcm_supervisor.regen_disable_serial, set via
//   btcm_supervisor_set_regen_disable).  Driving both the wire and the serial
//   shadow from ONE truth is the faithful healthy-harness behaviour: the two
//   AGREE, so DTC 109/110 stay quiet during normal operation.  The DTCs are
//   wire-fault detectors — they fire when the measured discrete level diverges
//   from the BTCM's intent (a stuck/short/open wire), which the PIM models via
//   a discrete-line fault-injection override independent of this signal.
// Consumer: the PIM controller, which maps 1 -> ~5000 mV (HIGH, "disable") and
//   0 -> ~500 mV (LOW, "permit") into pim_regen_tick's regen_disable_discrete_mv
//   argument, replacing the former static idle default.  This closes the
//   DTC 109/110 runtime loop against two BTCM-sourced-but-independently-wired
//   signals end-to-end (notes/manual_supplements.yaml#2026-06-07-btcm-pim-regen-serial-fields
//   suggested_action; follow-up #2026-06-07-btcm-pim-regen-discrete-producers).
// @design 2026-06-07 — first free chassis ID above the LHJB passthrough (4187);
//   additive (MINOR) chassis-contract bump.  The retard-request PWM discrete
//   leg (DTC 042, PIM J1-18) is NOT published here — the BTCM host supervisor
//   does not yet model a 128 Hz PWM waveform producer; that leg stays the
//   documented deferral.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisBtcmRegenDisableOut = 4188U;

// ---------------------------------------------------------------------------
// RSA interior-dimmer level (RSA → chassis bus, → IPC C12 circuit 230).
// Signal ID 4189.  Encoding: uint8 step index 0..10 — maps directly to
// ipc_supervisor_set_dim_level_step() (step 0 = full bright / 100 % duty,
// step 10 = minimum brightness; the IPC supervisor owns the step→duty table).
//
// On the real EV1 the RSA varies the analog voltage on circuit 230 via its
// dim-up/dim-down switches when park lamps are on; the IPC reads it and steps
// the VFD backlight PWM (EV1 Electrical Service Manual IPC C12 / circuit 230E).
// The RSA host supervisor does NOT model a dimmer-rheostat input (there is no
// dimmer/headlamp switch in the host supervisor today).  This signal is
// therefore sourced from a host-bridge setter on the RSA controller side
// (rsa_supervisor_set_dim_level_step in rsa_supervisor.h, or the chassis-bus
// injector below) with a default of 0 (full bright = no dimming).  The gap is
// documented; when a dimmer-switch input is modelled for RSA, it will publish
// here instead. @inferred 2026-06-12 — circuit 230 / IPC C12 is @source:manual;
// the step encoding and default are @design 2026-06-12 (no manual printout of
// the step count or voltage-to-step map; mirrors ipc_supervisor's kDimDutyByStep).
// @design 2026-06-12 — closes
//   notes/dtc_pass/concerns.yaml#2026-06-07-ipc-dim-and-parklamp-producer-gap
//   (dim-level producer side).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisRsaDimLevelStep = 4189U;

// ---------------------------------------------------------------------------
// LHJB park-lamp net present (LHJB → chassis bus, → IPC C4 circuit 301).
// Signal ID 4190.  Encoding: uint8 bool — 1 = park-lamp net energised
// (circuit 301 hot), 0 = dark.
//
// On the real EV1, circuit 301 is the park-lamp power net: when the driver
// turns the headlamp switch to park or head, the LHJB feeds B+ onto 301,
// energising the IPC's C4 PARKLAMPS ON input and lighting the park-lamp
// telltale (IPC_LCD_SYM_PARK_LAMP).  The LHJB's ev1_lhjb_outputs_t::feed_park_lamps
// is the internal model of this net; before this signal it was internal-only
// (never published) so the IPC controller proxied circuit 74 (the switch
// output, kSigCombSw_ParkHeadlampOut 4042) instead.
// @source:manual EV1 Electrical Service Manual, IPC C4 / circuit 301C
//   (PARKLAMPS ON IN); LHJB J9.A1 / circuit 9A → tail/marker feed = same net.
// @design 2026-06-12 — closes
//   notes/dtc_pass/concerns.yaml#2026-06-07-ipc-dim-and-parklamp-producer-gap
//   (park-lamp-on producer side). IPC retargets C4 ingestion from the
//   switch-proxy (4042) to this faithful net signal.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisLhjbParkLampsOn = 4190U;

// ---------------------------------------------------------------------------
// BTCM retard-request PWM duty (BTCM → chassis bus, → PIM J1-18 RETARD REQUEST IN).
// Signal ID 4191.  Encoding: uint16 little-endian, Q8 percent (0..25600 ≈ 0..100 %).
//
// On the real EV1 the BTCM drives a 128 Hz PWM waveform on circuit J1-18;
// the duty encodes the amount of regen torque it is requesting from the PCM.
// The PIM measures this waveform and uses it as one leg of the DTC 042 correlation
// (discrete duty > 20.7 % AND serial duty < 19.1 % for 2 s → DTC 042 set).
//
// Derivation from sup.regen_rtd_req_dc (uint8 0-255):
//   duty_q8 = (uint16_t)((uint32_t)regen_rtd_req_dc * 25600u / 255u)
// This gives 0 at idle (no regen request) and 25600 (100 % Q8) at full request.
// The serial shadow follows the same source via retard_request_count in the 0x5A
// frame; both move together in healthy operation so DTC 042 stays clear.
//
// Before this signal, PIM held a static healthy idle default (18 % / 128 Hz / 6 V)
// for J1-18. Publishing the live value from the BTCM supervisor closes the deferred
// PWM leg in notes/manual_supplements.yaml#2026-06-07-btcm-pim-regen-discrete-producers.
// @source:manual EV1 Service Manual prop-220/221 (DTC 042 correlation)
// @design 2026-06-12 — closes
//   notes/manual_supplements.yaml#2026-06-07-btcm-pim-regen-discrete-producers
//   (retard-request PWM discrete leg, DTC 042 correlation).
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisBtcmRetardRequestDutyQ8 = 4191U;

// ---------------------------------------------------------------------------
// 12 V auxiliary-battery rail (electricsim/aux_battery host → chassis bus)
// Signal IDs 4192–4194.  The single 12 V lead-acid auxiliary battery is the
// EV1's always-on low-voltage source: its positive post carries voltage at
// all times (even with the car asleep and HV down), feeding the always-on
// wiring, the RHJB Power-Moding Module wake logic, RSA keypad-detect, and —
// once the PMM brings a bus up — every module's switched-B+ feed.
//
// Published by the aux-battery host (src/models/battery/aux_battery_host),
// which ticks src/models/battery/aux_battery and bridges its outputs here on
// change.  These are the substrate for power-gated module boot: a controller
// reads 4192 and only runs its supervisor while the rail is above its
// brown-out threshold ("no 12 V → no boot").  4193 models pulling the
// battery (post → 0 V).  4194 feeds SOC display + the periodic 12 V-
// maintenance wake decision.
//
//   4192 terminal_mv  — uint32 LE, millivolts on the positive post.  0 when
//                       the battery is absent (open terminal).  A healthy
//                       full aux battery rests ~12 850 mV; under load it sags.
//                       Publish epsilon: ±50 mV.
//   4193 present       — uint8 bool, 1 = battery connected.  Exact change.
//   4194 soc_pct       — uint8, state-of-charge percent (0..100).  Publish
//                       epsilon: 1 %.
// @design 2026-06-13 — first free chassis IDs above the BTCM retard-request
//   PWM (4191); additive (MINOR 6 → 7) chassis-contract bump.  Sourced from
//   src/models/battery/aux_battery; see
//   notes/manual_supplements.yaml#2026-06-13-sla-battery-shared-chemistry.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisAuxBatteryTerminalMv = 4192U;
inline constexpr std::uint32_t kSigChassisAuxBatteryPresent    = 4193U;
inline constexpr std::uint32_t kSigChassisAuxBatterySocPct     = 4194U;

// ---------------------------------------------------------------------------
// Aux-battery periodic 12 V-maintenance wake request (aux_battery host → chassis)
// Signal ID 4195.  Encoding: uint8 bool — 1 = the aux battery is asking the car
// to wake and charge it.
//
// The real EV1 periodically wakes itself (every several hours) to check the
// 12 V auxiliary battery and, if it has drained, brings up the APM to top it
// up — so a parked car doesn't go flat. The aux-battery host models that: when
// SOC falls below a low threshold it asserts this request (with hysteresis so
// it clears once charged). It is ORed into:
//   - the RHJB Power-Moding Module wake_req (alongside the 4187 charge-paddle
//     passthrough) → PMM SLEEP → AWAKE, bringing up module B+, and
//   - the APM vehicle-wake-up (apm_supervisor_set_vehicle_wake_up) → the APM
//     keys on and, with HV available, charges the 12 V battery (APM B+ active
//     4133 feeds back to the host as the charge source).
// @design 2026-06-13 — first free chassis ID above the aux-battery rail block
//   (4192-4194); additive (MINOR 7 → 8). See
//   notes/manual_supplements.yaml#2026-06-13-sla-battery-shared-chemistry.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisAuxBatteryMaintWakeReq = 4195U;

// ---------------------------------------------------------------------------
// RHJB PMM switched module-B+ rail (electricsim/RHJB → chassis bus)
// Signal ID 4196.  Encoding: uint8 bool — 1 = 840-family switched module B+
// energised, 0 = down.
//
// The 840-family "module B+" is the switched low-voltage feed the RHJB
// Power-Moding Module brings up the moment the car leaves SLEEP: it is the rail
// that powers each run-mode ECU's microcontroller (the supply that boots the
// module), distinct from the functional RUN-1 (41-family, 4180) and RUN-2
// (241-family, 4181) LOAD buses.  The PMM model already computes it as
// ev1_rhjb_power_moding_out_t::module_bplus (rhjb_power_moding.c — true whenever
// the PMM is AWAKE/RUN1/RUN2); before this signal it was internal-only and never
// crossed onto the bus.
//
// Publisher: the RHJB controller, which bridges the PMM's module_bplus output
//   here on change after each power-moding tick — the same pattern it already
//   uses for run1_bus (4180) / run2_bus (4181).
// Consumers: the run-mode controllers (BTCM, PIM, …) gate their supervisor boot
//   on this — "no module B+ → MCU stays dark".  They USED to do it through
//   SwitchedFeedGate (src/models/battery/switched_feed_gate.hpp), which was
//   default-powered until the feed was first observed, so a harness that never
//   published 4196 behaved as if it had.  That header is deleted and the gate
//   is now io::energised() over the module's feed conductor
//   (src/io/topology/feed_query.hpp): a harness that never publishes 4196 gets
//   a dark MCU, which is the correct answer and NOT the old one.
// @source:manual electrical p280/p281 — PMM switched-B+ distribution downstream
//   of the SLEEP/AWAKE/RUN state machine (the run-mode module supply).
// @design 2026-06-14 — first free chassis ID above the aux-battery maintenance
//   wake (4195); additive (MINOR 8 → 9).  Pairs with SwitchedFeedGate to give the
//   run-mode modules a faithful 840-family boot feed rather than a RUN-1/2 proxy.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisRhjbPmmModuleBPlus = 4196U;

// ---------------------------------------------------------------------------
// APM 12 V output (charge) current (electricsim/APM → chassis bus)
// Signal ID 4197.  Encoding: uint32 LE, milliamps the APM DC/DC converter
// delivers to the 12 V auxiliary rail (≥ 0 — the APM is a source-only HV→LV
// down-converter, it never sinks).  0 when the APM is off or HV is unavailable.
//
// The APM's accessory-power DC/DC converter is the 12 V battery's charge source:
// keyed on with HV present, it holds the LV rail at ~13–14 V and pushes current
// into the aux battery + accessory loads.  The APM supervisor already carries
// this as output_current_q8 (apm_supervisor.h — int16 Q8 amps, the same value its
// 70–100 A current-limit fault logic watches); the APM controller republishes it
// here converted to mA (clamped ≥ 0).
//
// Before this signal the aux-battery host modelled charging from the APM-B+-active
// bool (4133) alone, at a fixed assumed --charge-ma (10 000 mA); publishing the
// APM's modelled output current lets the host integrate REAL charge current
// (net = APM_out − load draw) into SOC, closing the SIMPLIFICATION follow-up noted
// in src/models/battery/aux_battery_host.
//
// Publisher: the APM controller, from supervisor output_current_q8 (→ mA).
// Consumer: the aux-battery host (src/models/battery/aux_battery_host), which feeds
//   it into the aux battery's charge integration; falls back to the prior
//   bool-gated assumed current when 4197 is stale/absent.
// @design 2026-06-14 — second of the MINOR 8 → 9 additive pair (with 4196); real
//   APM-derived charge current for the 12 V rail.  See
//   notes/manual_supplements.yaml#2026-06-13-sla-battery-shared-chemistry.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisApmTwelveVoltOutputMa = 4197U;

// ---------------------------------------------------------------------------
// TJB rear-lamp branch outputs (electricsim/TJB → chassis bus)
// Signal IDs 4198–4204.  Encoding: uint8 bool per branch — 1 = branch hot.
//
// The Trunk Junction Block distributes the LHJB's rear lighting feeds (park /
// turn-left / turn-right / brake, all under RUN1) to the physical rear-lamp
// branches: LR/RR tail, license, LR/RR turn, LR/RR stop.
// @source:manual EV1 Electrical Service Manual, trunk junction block chapter
//   (rear lamp + license branch distribution from LHJB feeds) — the branch
//   set mirrors ev1_tjb_outputs_t (ev1/junction_boxes/junction_boxes.h).
//
// Before these cells the TJB had NO live process at all — ev1_tjb_tick and
// its 13 route_* families were unit-tested dormant code, now wired into a
// live process for the first time.
// ex_tjb_controller consumes RUN1 + the LHJB feed cells and publishes these
// seven branches. Observational today (consumers: []); candidates for the
// ev1sim rear-lamp contract — a possible future migration is TJB taking over
// rear-lamp production from the LHJB-written BULB_FEED_LINE_* cells, which is
// a cross-repo contract change (docs/3d_sim_contract.md) deliberately NOT
// done here.
//
// Publisher: ex_tjb_controller (ev1/tjb/tjb_host.cpp).
// Consumers: none in-repo (observational; ev1sim candidate).
// @design 2026-07-05 — ID allocation 4198–4204, next free chassis block.
// ---------------------------------------------------------------------------
inline constexpr std::uint32_t kSigChassisTjbLrTailLamp   = 4198U;
inline constexpr std::uint32_t kSigChassisTjbRrTailLamp   = 4199U;
inline constexpr std::uint32_t kSigChassisTjbLicenseLamps = 4200U;
inline constexpr std::uint32_t kSigChassisTjbLrTurnLamp   = 4201U;
inline constexpr std::uint32_t kSigChassisTjbRrTurnLamp   = 4202U;
inline constexpr std::uint32_t kSigChassisTjbLrStopLamp   = 4203U;
inline constexpr std::uint32_t kSigChassisTjbRrStopLamp   = 4204U;

}  // namespace electricsim::io
