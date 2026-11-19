#include "BusSnoop.h"

#include <array>

#if defined(EV1SIM_HAVE_WIRE_TRUTH) && EV1SIM_HAVE_WIRE_TRUTH
#include "gm8192/gm8192_rx_framer.hpp"    // electricsim::io::Gm8192RxFramer
#include "topology/topology_generated.h"  // kWireGM8192_*_TX
#include "wire_table.hpp"
#endif

namespace ev1sim {

namespace {

// Payload lengths (N) of the decoded frames, from the same contracts as the IDs.
constexpr std::uint8_t kNPcmResponse  = 7u;   // PIM_UART_FRAME_DATA_LEN
constexpr std::uint8_t kNPcmPoll      = 6u;   // BPM_PCM_POLL_DATA_LEN
constexpr std::uint8_t kNDscmPoll     = 5u;   // BPM_DSCM_POLL_DATA_LEN
constexpr std::uint8_t kNBtcmPoll     = 2u;   // BPM_BTCM_POLL_DATA_LEN
constexpr std::uint8_t kNBtcmResponse = 11u;  // BTCM_UART_FRAME_DATA_LEN
constexpr std::uint8_t kNDscmResponse = 3u;   // IPC_UART_FRAME_DATA_LEN

constexpr bool Bit(std::uint32_t v, unsigned b) { return ((v >> b) & 1u) != 0u; }

std::uint16_t BigEndian16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

// Payload index = wire byte index − 2 (ID and Length precede the payload).
DashPcmStatus DecodePcmResponse(const std::uint8_t* p) {
    DashPcmStatus s;
    s.service_soon_tt       = Bit(p[0], 0);
    s.reduced_perf_tt       = Bit(p[0], 1);
    s.pcm_temp_tt           = Bit(p[0], 2);
    s.coast_down_led        = Bit(p[0], 3);
    s.charge_mode_display   = Bit(p[0], 4);
    s.pcm_service_now_tt    = Bit(p[1], 1);
    s.pcm_connection_detect = Bit(p[1], 2);
    s.prnd_code = static_cast<std::uint8_t>((p[1] >> 3) & 0x1Fu);
    s.gear = DecodePrndCode(s.prnd_code);
    s.speed_kph = p[4];
    return s;
}

DashPackStatus DecodePcmPoll(const std::uint8_t* p) {
    DashPackStatus s;
    s.pack_voltage_v = static_cast<std::uint16_t>(p[0] * 2u);
    const std::uint16_t status = BigEndian16(&p[1]);
    s.charger_connected    = Bit(status, 15);
    s.pack_malfunction     = Bit(status, 14);
    s.pack_voltage_fault   = Bit(status, 13);
    s.pack_current_fault   = Bit(status, 12);
    s.auto_disconnect_open = Bit(status, 11);
    s.pack_current_a = 0.5f * static_cast<float>(status & 0x07FFu) - 400.0f;
    s.pack_soc_pct = p[4];
    return s;
}

DashBpmDisplay DecodeDscmPoll(const std::uint8_t* p) {
    DashBpmDisplay s;
    s.user_soc_pct = p[0];
    s.ttf_hours    = p[1];
    s.ttf_minutes  = p[2];
    const std::uint16_t word = BigEndian16(&p[3]);
    s.battery_life_tt    = Bit(word, 15);
    s.bpm_temp_tt        = Bit(word, 14);
    s.bpm_service_now_tt = Bit(word, 13);
    s.power_capability = static_cast<std::uint8_t>((word >> 10) & 0x07u);
    s.range_km = static_cast<std::uint16_t>(word & 0x03FFu);
    return s;
}

// Bars #1 (bit 0, bottom) .. #11 (bit 10, top) light cumulatively; the count is
// the highest lit bar, as the external sim's bpm_btcm_poll_decode reads it.
std::uint8_t DecodeBtcmPollBars(const std::uint8_t* p) {
    const std::uint16_t word = BigEndian16(&p[0]);
    std::uint8_t bars = 0;
    for (unsigned b = 0; b < 11u; ++b) {
        if (Bit(word, b)) bars = static_cast<std::uint8_t>(b + 1u);
    }
    return bars;
}

DashBrakeStatus DecodeBtcmResponse(const std::uint8_t* p) {
    DashBrakeStatus s;
    s.park_brake              = Bit(p[0], 0);
    s.service_brake           = Bit(p[0], 1);
    s.brake_tt                = Bit(p[0], 2);
    s.antilock_tt             = Bit(p[0], 3);
    s.redundant_regen_disable = Bit(p[0], 4);
    s.abs_tcs_active          = Bit(p[0], 5);
    s.retard_achieved_fault   = Bit(p[0], 6);
    s.wheel_speed_valid       = Bit(p[0], 7);
    DecodeRetardRequest(p[1], s);
    for (unsigned w = 0; w < 4u; ++w) s.wheel_tau[w] = BigEndian16(&p[2u + 2u * w]);
    s.service_soon_req   = Bit(p[10], 0);
    s.front_power_assist = Bit(p[10], 1);
    s.tim_wheel_accel    = Bit(p[10], 2);
    s.tim_vehicle_accel  = Bit(p[10], 3);
    s.tim_accel_limit    = Bit(p[10], 4);
    return s;
}

DashOdometer DecodeDscmResponse(const std::uint8_t* p) {
    DashOdometer s;
    s.odometer_fault     = Bit(p[0], 7);
    s.bar_gage_power_use = Bit(p[0], 6);
    s.season_odometer_dkm = (static_cast<std::uint32_t>(p[0] & 0x3Fu) << 16) |
                            (static_cast<std::uint32_t>(p[1]) << 8) |
                            static_cast<std::uint32_t>(p[2]);
    return s;
}

}  // namespace

void DecodeRetardRequest(std::uint8_t count, DashBrakeStatus& s) {
    s.retard_request_count = count;
    s.coastdown_reduction_pct = 0.0f;
    s.braking_retard_nm = 0.0f;
    if (count >= kRetardCoastdownMin && count <= kRetardCoastdownMax) {
        s.retard_band = DashRetardBand::kCoastdown;
        s.coastdown_reduction_pct = 5.0f * static_cast<float>(kRetardCoastdownMax - count);
    } else if (count >= kRetardBrakingMin && count <= kRetardBrakingMax) {
        s.retard_band = DashRetardBand::kBraking;
        s.braking_retard_nm = 10.875f * static_cast<float>(count - kRetardBrakingMin);
    } else {
        s.retard_band = DashRetardBand::kOutOfRange;
    }
}

std::optional<double> BtcmResponseField(const DashBrakeStatus& b, const std::string& name) {
    auto flag = [](bool v) { return std::optional<double>(v ? 1.0 : 0.0); };
    if (name == "park_brake")              return flag(b.park_brake);
    if (name == "service_brake")           return flag(b.service_brake);
    if (name == "brake_tt")                return flag(b.brake_tt);
    if (name == "antilock_tt")             return flag(b.antilock_tt);
    if (name == "redundant_regen_disable") return flag(b.redundant_regen_disable);
    if (name == "abs_tcs_active")          return flag(b.abs_tcs_active);
    if (name == "retard_achieved_fault")   return flag(b.retard_achieved_fault);
    if (name == "wheel_speed_valid")       return flag(b.wheel_speed_valid);
    if (name == "retard_request_count")    return static_cast<double>(b.retard_request_count);
    if (name == "retard_band")             return static_cast<double>(b.retard_band);
    if (name == "coastdown_reduction_pct") return static_cast<double>(b.coastdown_reduction_pct);
    if (name == "braking_retard_nm")       return static_cast<double>(b.braking_retard_nm);
    if (name == "wheel_tau_lf")            return static_cast<double>(b.wheel_tau[0]);
    if (name == "wheel_tau_rf")            return static_cast<double>(b.wheel_tau[1]);
    if (name == "wheel_tau_lr")            return static_cast<double>(b.wheel_tau[2]);
    if (name == "wheel_tau_rr")            return static_cast<double>(b.wheel_tau[3]);
    if (name == "service_soon_req")        return flag(b.service_soon_req);
    if (name == "front_power_assist")      return flag(b.front_power_assist);
    if (name == "tim_wheel_accel")         return flag(b.tim_wheel_accel);
    if (name == "tim_vehicle_accel")       return flag(b.tim_vehicle_accel);
    if (name == "tim_accel_limit")         return flag(b.tim_accel_limit);
    return std::nullopt;
}

DashGear DecodePrndCode(std::uint8_t code5) {
    switch (code5 & 0x1Fu) {
        case 0x01u: return DashGear::kError;    // PIM_UART_PRND_CODE_ERROR
        case 0x10u: return DashGear::kPark;     // PIM_UART_PRND_CODE_PARK
        case 0x08u: return DashGear::kReverse;  // PIM_UART_PRND_CODE_REVERSE
        case 0x04u: return DashGear::kNeutral;  // PIM_UART_PRND_CODE_NEUTRAL
        case 0x02u: return DashGear::kDrive;    // PIM_UART_PRND_CODE_DRIVE
        default:    return DashGear::kUnknown;  // "other combinations not used"
    }
}

#if defined(EV1SIM_HAVE_WIRE_TRUTH) && EV1SIM_HAVE_WIRE_TRUTH

struct BusSnoop::Wire {
    // GM-8192 physical layer: 8192 baud → 122.07 µs/bit. @source:manual.
    static constexpr std::uint64_t kBitPeriodNs = 122070;

    explicit Wire(electricsim::io::WireTable* table)
        : framers{{
              {table, electricsim::topology::kWireGM8192_PIM_TX, kBitPeriodNs},
              {table, electricsim::topology::kWireGM8192_BPM_TX, kBitPeriodNs},
              {table, electricsim::topology::kWireGM8192_BTCM_TX, kBitPeriodNs},
              {table, electricsim::topology::kWireGM8192_IPC_TX, kBitPeriodNs},
          }} {}

    std::array<electricsim::io::Gm8192RxFramer, 4> framers;
};

bool BusSnoop::Attach(electricsim::io::WireTable* table) {
    if (table == nullptr) {
        wire_.reset();
        return false;
    }
    wire_ = std::make_unique<Wire>(table);
    return true;
}

void BusSnoop::Step(double now_s) {
    if (!wire_) return;
    const auto now_ns = static_cast<std::uint64_t>(now_s * 1.0e9);
    for (auto& framer : wire_->framers) {
        // The frame view's payload lives in the framer's buffer until the next
        // step(); DecodeFrame copies what it keeps before that.
        while (auto frame = framer.step(now_ns)) {
            DecodeFrame(frame->id, frame->payload, frame->n, now_s);
        }
    }
}

#else  // !EV1SIM_HAVE_WIRE_TRUTH

struct BusSnoop::Wire {};

bool BusSnoop::Attach(electricsim::io::WireTable*) { return false; }
void BusSnoop::Step(double) {}

#endif

BusSnoop::BusSnoop() = default;
BusSnoop::~BusSnoop() = default;

bool BusSnoop::attached() const { return wire_ != nullptr; }
void BusSnoop::Detach() { wire_.reset(); }

bool BusSnoop::DecodeFrame(std::uint8_t id, const std::uint8_t* payload,
                           std::uint8_t n, double now_s) {
    std::uint8_t want = 0;
    switch (id) {
        case kDashMsgPcmResponse:  want = kNPcmResponse;  break;
        case kDashMsgPcmPoll:      want = kNPcmPoll;      break;
        case kDashMsgDscmPoll:     want = kNDscmPoll;     break;
        case kDashMsgBtcmPoll:     want = kNBtcmPoll;     break;
        case kDashMsgBtcmResponse: want = kNBtcmResponse; break;
        case kDashMsgDscmResponse: want = kNDscmResponse; break;
        default: return false;  // not a frame the cluster reads
    }
    if (n != want || payload == nullptr) {
        ++frames_rejected_;
        return false;
    }
    switch (id) {
        case kDashMsgPcmResponse:  pcm_        = {DecodePcmResponse(payload), now_s};  break;
        case kDashMsgPcmPoll:      pack_       = {DecodePcmPoll(payload), now_s};      break;
        case kDashMsgDscmPoll:     display_    = {DecodeDscmPoll(payload), now_s};     break;
        case kDashMsgBtcmPoll:     power_bars_ = {DecodeBtcmPollBars(payload), now_s}; break;
        case kDashMsgBtcmResponse:
            brake_ = {DecodeBtcmResponse(payload), now_s};
            ++btcm_responses_;
            break;
        case kDashMsgDscmResponse: odometer_   = {DecodeDscmResponse(payload), now_s}; break;
        default: break;
    }
    ++frames_decoded_;
    return true;
}

template <typename T>
std::optional<T> BusSnoop::Fresh(const Stamped<T>& s, double now_s) const {
    if (!s.value) return std::nullopt;
    if (now_s - s.at_s > stale_after_s_) return std::nullopt;
    return s.value;
}

DashSnapshot BusSnoop::Snapshot(double now_s) const {
    DashSnapshot snap;
    snap.pcm        = Fresh(pcm_, now_s);
    snap.pack       = Fresh(pack_, now_s);
    snap.display    = Fresh(display_, now_s);
    snap.power_bars = Fresh(power_bars_, now_s);
    snap.brake      = Fresh(brake_, now_s);
    snap.odometer   = Fresh(odometer_, now_s);
    return snap;
}

}  // namespace ev1sim
