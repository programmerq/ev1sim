#include "ExternalSimConnector.h"

#include <array>
#include <cstring>
#include <iostream>
#include <string>

#if EV1SIM_HAVE_EXTERNAL_SIM
#  include "protocol.hpp"
#  include "shm_transport.hpp"
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
// Number of driver-input endpoints on the main harness segment.
constexpr int           kNumDriverInputs          = 6;

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
// kNumDriverInputs covers the 4 existing driver inputs (6900-6903) plus the
// 2 new ones (brake_switch 6904, seatbelt_buckled 6964) — all on the main
// harness segment (electricsim_ev1_bus), output from ev1sim.
// +1 for the charge coupler presence (ID 4060, chassis segment).
// +kNumPrndSelector for the 4 PRND selector lines (IDs 4050-4053, chassis segment).
constexpr int kNumEndpoints =
    NUM_LIGHTS + 2 + VehiclePanels::NUM_PANELS + kNumCombSw + 1 /*charge_coupler*/ +
    kNumPrndSelector + kNumDynamics + kNumDriverInputs;

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
    return out;
}

const std::array<ExternalSimConnector::Endpoint, kNumEndpoints>& EndpointTable() {
    static const auto table = BuildEndpoints();
    return table;
}

#if EV1SIM_HAVE_EXTERNAL_SIM
constexpr std::uint32_t kStreamEv1Sim = 0x45563153u; // "EV1S"

std::uint64_t NowNs() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}
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

    // Combination switch pin outputs — latched by SetCombSwOutputs(), published
    // as wire-level booleans in Tick() when changed.
    bool comb_sw_low_beam      = false;
    bool comb_sw_flash_to_pass = false;
    bool comb_sw_park_headlamp = false;
    bool comb_sw_low_beam_pub      = false;
    bool comb_sw_flash_to_pass_pub = false;
    bool comb_sw_park_headlamp_pub = false;
    bool comb_sw_ever_published    = false;

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
    // New discrete driver inputs (ID 6904, 6964) on the main harness segment.
    bool          driver_brake_switch      = false;
    bool          driver_seatbelt_buckled  = true;   // default: always buckled
    // Published sentinels (use 0xFF-equivalent to force first publish).
    std::int8_t   driver_brake_switch_pub     = -1;  // -1 forces first publish
    std::int8_t   driver_seatbelt_buckled_pub = -1;  // -1 forces first publish

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

void ExternalSimConnector::SetDriverBrakePedalQ8(std::uint8_t q8) {
    m_state->driver_brake_q8 = q8;
}

void ExternalSimConnector::SetDriverThrottleQ8(std::uint8_t q8) {
    m_state->driver_throttle_q8 = q8;
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

void ExternalSimConnector::SetChargeCouplerPresent(bool present) {
    m_state->charge_coupler_present = present;
}

void ExternalSimConnector::SetPrndSelector(bool a, bool b, bool c, bool d) {
    m_state->prnd_a = a;
    m_state->prnd_b = b;
    m_state->prnd_c = c;
    m_state->prnd_d = d;
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
    }
    // Panel-sensor signals are outputs — ignore inbound.
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

} // namespace

void ExternalSimConnector::Tick(double sim_time_s) {
    if (!m_opts.enabled) return;
    auto& st = *m_state;

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
        // Force re-publish of all chassis outputs after reconnect.
        st.charge_coupler_present_pub = -1;
        st.prnd_a_pub = -1;
        st.prnd_b_pub = -1;
        st.prnd_c_pub = -1;
        st.prnd_d_pub = -1;
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
            const bool v = !d.payload.empty() && (d.payload[0] & 1u);
            DebugInjectDelta(d.signal_id, v);
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
        dyn.reserve(static_cast<std::size_t>(kNumDynamics));
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

    // 5. Open / maintain the main harness segment (electricsim_ev1_bus) and
    //    publish driver input signals.
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
            std::cout << "[ExternalSim] connected to main harness bus '"
                      << m_opts.main_harness_bus_name << "'\n";
        }
    }
    if (st.main_transport) {
        std::vector<DeltaRecord> drv;
        if (st.driver_brake_q8    != st.driver_brake_pub ||
            st.driver_steering_q8 != st.driver_steering_pub ||
            st.driver_gear        != st.driver_gear_pub ||
            st.driver_throttle_q8 != st.driver_throttle_pub) {
            drv.push_back(MakeU8Delta(kSigDriverBrakePedalQ8,  st.driver_brake_q8));
            drv.push_back(MakeI16Delta(kSigDriverSteeringDegQ8, st.driver_steering_q8));
            drv.push_back(MakeU8Delta(kSigDriverGearSelector,   st.driver_gear));
            drv.push_back(MakeU8Delta(kSigDriverThrottleQ8,     st.driver_throttle_q8));
            st.driver_brake_pub    = st.driver_brake_q8;
            st.driver_steering_pub = st.driver_steering_q8;
            st.driver_gear_pub     = st.driver_gear;
            st.driver_throttle_pub = st.driver_throttle_q8;
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
