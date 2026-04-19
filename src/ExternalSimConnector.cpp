#include "ExternalSimConnector.h"

#include <array>
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
// Bulb command IDs (4000..4018) mirror the external electrical sim's LightIdx
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
//   4020  horn_low_cmd                               (input to ev1sim)
//   4021  horn_high_cmd                              (input to ev1sim)
//   4030..4033  panel ajar switches (HOOD/TRUNK/DL/DR)    (output from ev1sim)
// ---------------------------------------------------------------------------
constexpr std::uint32_t kBulbCmdBase   = 4000;
constexpr std::uint32_t kHornLowCmd    = 4020;
constexpr std::uint32_t kHornHighCmd   = 4021;
constexpr std::uint32_t kPanelBase     = 4030;

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
    "lhbh_bulb_cmd",  "llbh_bulb_cmd",
    "rhbh_bulb_cmd",  "rlbh_bulb_cmd",
    "lfts_bulb_cmd",  "rfts_bulb_cmd",
    "lfml_bulb_cmd",  "rfml_bulb_cmd",
    "lrsl_bulb_cmd",  "rrsl_bulb_cmd",
    "lrtl_bulb_cmd",  "rrtl_bulb_cmd",
    "lrts_bulb_cmd",  "rrts_bulb_cmd",
    "lrsm_bulb_cmd",  "rrsm_bulb_cmd",
    "chmsl_bulb_cmd",
    "lbl_bulb_cmd",   "rbl_bulb_cmd",
};

constexpr const char* kBulbQualified[NUM_LIGHTS] = {
    "vehicle.body.lhbh.bulb_cmd",  "vehicle.body.llbh.bulb_cmd",
    "vehicle.body.rhbh.bulb_cmd",  "vehicle.body.rlbh.bulb_cmd",
    "vehicle.body.lfts.bulb_cmd",  "vehicle.body.rfts.bulb_cmd",
    "vehicle.body.lfml.bulb_cmd",  "vehicle.body.rfml.bulb_cmd",
    "vehicle.body.lrsl.bulb_cmd",  "vehicle.body.rrsl.bulb_cmd",
    "vehicle.body.lrtl.bulb_cmd",  "vehicle.body.rrtl.bulb_cmd",
    "vehicle.body.lrts.bulb_cmd",  "vehicle.body.rrts.bulb_cmd",
    "vehicle.body.lrsm.bulb_cmd",  "vehicle.body.rrsm.bulb_cmd",
    "vehicle.body.chmsl.bulb_cmd",
    "vehicle.body.lbl.bulb_cmd",   "vehicle.body.rbl.bulb_cmd",
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
// Build the endpoint table once.
// ---------------------------------------------------------------------------
constexpr int kNumEndpoints = NUM_LIGHTS + 2 + VehiclePanels::NUM_PANELS;

std::array<ExternalSimConnector::Endpoint, kNumEndpoints> BuildEndpoints() {
    std::array<ExternalSimConnector::Endpoint, kNumEndpoints> out{};
    int i = 0;
    for (int slot = 0; slot < NUM_LIGHTS; ++slot, ++i) {
        const int lid = static_cast<int>(kBulbOrder[slot]);
        out[i] = {kBulbCmdBase + static_cast<std::uint32_t>(slot),
                  kBulbQualified[lid], kBulbShort[lid], /*input_to_sim=*/true};
    }
    out[i++] = {kHornLowCmd,  "vehicle.body.horn.low_cmd",  "horn_low_cmd",  true};
    out[i++] = {kHornHighCmd, "vehicle.body.horn.high_cmd", "horn_high_cmd", true};
    for (int p = 0; p < VehiclePanels::NUM_PANELS; ++p, ++i) {
        out[i] = {kPanelBase + static_cast<std::uint32_t>(p),
                  kPanelNames[p].qualified, kPanelNames[p].shortname,
                  /*input_to_sim=*/false};
    }
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

    // Timers (sim_time_s based).
    double next_presence_time  = 0.0;
    double next_reconnect_time = 0.0;

#if EV1SIM_HAVE_EXTERNAL_SIM
    std::unique_ptr<electricsim::io::SharedMemoryTransport> transport;
    std::uint64_t sequence = 1;
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

    // 3. Publish any panel-sensor changes since last tick.
    std::vector<DeltaRecord> outbound;
    for (int p = 0; p < VehiclePanels::NUM_PANELS; ++p) {
        if (!st.panel_ever_published || st.panel[p] != st.panel_published[p]) {
            outbound.push_back(MakeBoolDelta(kPanelBase + static_cast<std::uint32_t>(p),
                                             st.panel[p]));
            st.panel_published[p] = st.panel[p];
        }
    }
    st.panel_ever_published = true;

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

    // 4. Announce our endpoints periodically so other bus peers can discover us.
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
