// WireTruthChassis implementation. See WireTruthChassis.h for the contract.
//
// @design 2026-06-15 — wire-truth migration kickoff (proof-of-life batch).

#include "WireTruthChassis.h"

#if defined(EV1SIM_HAVE_WIRE_TRUTH)

// Real implementation: attach electricsim's shared WireTable. These headers
// come from the electricsim tree (compiled into the electricsim_wire_substrate
// static lib); the include root is ${ELECTRICSIM_DIR}/src/io.
#include "topology/env_open.hpp"          // electricsim::topology::try_open_from_env
#include "topology/topology_generated.h"  // kTopologyHash, kWireHORN_DRIVE_LINE_*
#include "wire_table.hpp"                 // electricsim::io::WireTable

#include <cstring>
#include <unordered_map>
#include <utility>

// Use the topology namespace for kWire* constants.
using namespace electricsim::topology;
using electricsim::io::WireId;
using electricsim::io::WireType;

namespace ev1sim {

// ---------------------------------------------------------------------------
// Producer registry — maps every ev1sim-produced ring signal_id to the
// corresponding electricsim WireId + WireType.  Generated from
// /tmp/producer_cells.tsv (88 entries).  Using the real kWire* constants from
// topology_generated.h is the drift guard: a renamed/removed constant fails
// to compile, forcing this table to be updated in lockstep with the topology.
// @design 2026-06-15 — table-driven dual-write (Phase C-b producer batch).
// ---------------------------------------------------------------------------
struct ProducerCell {
    WireId   wire_id;
    WireType wire_type;
};

static const std::unordered_map<std::uint32_t, ProducerCell>& ProducerRegistry() {
    static const std::unordered_map<std::uint32_t, ProducerCell> kRegistry = {
        // Panel ajar switches (bit)
        {4030U, {kWirePANEL_AJAR_HOOD,              WireType::kBit}},
        {4031U, {kWirePANEL_AJAR_TRUNK,             WireType::kBit}},
        {4032U, {kWirePANEL_AJAR_DOOR_LEFT,         WireType::kBit}},
        {4033U, {kWirePANEL_AJAR_DOOR_RIGHT,        WireType::kBit}},
        // Combination switch outputs (bit)
        {4040U, {kWireCOMB_SW_LOW_BEAM_OUT,          WireType::kBit}},
        {4041U, {kWireCOMB_SW_FLASH_TO_PASS_OUT,     WireType::kBit}},
        {4042U, {kWireCOMB_SW_PARK_HEADLAMP_OUT,     WireType::kBit}},
        // Turn/hazard switch outputs (bit)
        {4043U, {kWireTURN_HAZ_SW_RIGHT_TURN_OUT,   WireType::kBit}},
        {4044U, {kWireTURN_HAZ_SW_LEFT_TURN_OUT,    WireType::kBit}},
        {4045U, {kWireTURN_HAZ_SW_HAZARD_OUT,       WireType::kBit}},
        {4046U, {kWireTURN_HAZ_SW_HORN_OUT,         WireType::kBit}},
        // Cruise switch outputs (bit)
        {4047U, {kWireCRUISE_SW_SET_COAST_OUT,      WireType::kBit}},
        {4048U, {kWireCRUISE_SW_RESUME_ACCEL_OUT,   WireType::kBit}},
        {4049U, {kWireCRUISE_SW_ON_OFF_OUT,         WireType::kBit}},
        // PRND selector lines (bit)
        {4050U, {kWirePRND_SELECTOR_A,              WireType::kBit}},
        {4051U, {kWirePRND_SELECTOR_B,              WireType::kBit}},
        {4052U, {kWirePRND_SELECTOR_C,              WireType::kBit}},
        {4053U, {kWirePRND_SELECTOR_D,              WireType::kBit}},
        // Wiper switch outputs (bit)
        {4054U, {kWireWIPER_SW_DELAY_OUT,           WireType::kBit}},
        {4055U, {kWireWIPER_SW_REQUEST_OUT,         WireType::kBit}},
        {4056U, {kWireWIPER_SW_HI_OUT,              WireType::kBit}},
        {4057U, {kWireWIPER_SW_WASHER_SWITCH_OUT,   WireType::kBit}},
        // Motor state (float32)
        {4070U, {kWireCHASSIS_MOTOR_RPM,            WireType::kFloat32}},
        {4071U, {kWireCHASSIS_MOTOR_TORQUE_NM,      WireType::kFloat32}},
        // Brake master pressure (float32)
        {4074U, {kWireCHASSIS_BRAKE_MASTER_PRESSURE_KPA, WireType::kFloat32}},
        // Sim-time master clock (uint64)
        {4075U, {kWireCHASSIS_SIM_TIME_NS,          WireType::kUint64}},
        // Ambient environment (float32)
        {4090U, {kWireCHASSIS_AMBIENT_TEMP_C,       WireType::kFloat32}},
        {4091U, {kWireCHASSIS_AMBIENT_HUMIDITY_PCT, WireType::kFloat32}},
        // Vehicle dynamics (float32)
        {4100U, {kWireCHASSIS_SPEED_MPS,            WireType::kFloat32}},
        {4101U, {kWireCHASSIS_ACCEL_LONG,           WireType::kFloat32}},
        {4102U, {kWireCHASSIS_ACCEL_LAT,            WireType::kFloat32}},
        {4103U, {kWireCHASSIS_YAW_RATE,             WireType::kFloat32}},
        {4104U, {kWireCHASSIS_APPLIED_THROTTLE,     WireType::kFloat32}},
        {4105U, {kWireCHASSIS_APPLIED_FRONT_BRAKE,  WireType::kFloat32}},
        {4106U, {kWireCHASSIS_APPLIED_REAR_BRAKE,   WireType::kFloat32}},
        {4107U, {kWireCHASSIS_FRONT_BRAKE_PRESSURE, WireType::kFloat32}},
        {4108U, {kWireCHASSIS_REAR_BRAKE_POSITION,  WireType::kFloat32}},
        {4109U, {kWireCHASSIS_STEERING_TORQUE,      WireType::kFloat32}},
        {4110U, {kWireCHASSIS_WHEEL_OMEGA_FL,       WireType::kFloat32}},
        {4111U, {kWireCHASSIS_WHEEL_OMEGA_FR,       WireType::kFloat32}},
        {4112U, {kWireCHASSIS_WHEEL_OMEGA_RL,       WireType::kFloat32}},
        {4113U, {kWireCHASSIS_WHEEL_OMEGA_RR,       WireType::kFloat32}},
        {4120U, {kWireCHASSIS_SLIP_RATIO_FL,        WireType::kFloat32}},
        {4121U, {kWireCHASSIS_SLIP_RATIO_FR,        WireType::kFloat32}},
        {4122U, {kWireCHASSIS_SLIP_RATIO_RL,        WireType::kFloat32}},
        {4123U, {kWireCHASSIS_SLIP_RATIO_RR,        WireType::kFloat32}},
        // NOTE: HV bus 4155-4157 and charge-wake 4187 are intentionally ABSENT.
        // They are electricsim-PRODUCED cells — topology.yaml gives them a
        // `driver:` (hv_bus_host / lhjb_ecu), not the omitted-producer form
        // ev1sim's own outputs use — and were migrated in an earlier electricsim
        // step. ev1sim neither produces nor publishes them, so mirroring them
        // here would be a wrong-owner write onto a cell electricsim drives.
        // (docs/wire_truth_migration_scope.md §1: ev1sim produces 84 cells.)
        // Door lock state + road grade / pitch (bit / float32)
        {4165U, {kWireCHASSIS_DOOR_LOCK_STATE_DRIVER,    WireType::kBit}},
        {4166U, {kWireCHASSIS_DOOR_LOCK_STATE_PASSENGER, WireType::kBit}},
        {4167U, {kWireCHASSIS_DOOR_LOCK_STATE_TRUNK,     WireType::kBit}},
        {4168U, {kWireCHASSIS_ROAD_GRADE_PCT,            WireType::kFloat32}},
        {4169U, {kWireCHASSIS_PITCH_DEG,                 WireType::kFloat32}},
        // Door lock switch outputs (bit)
        {4170U, {kWireDOOR_LOCK_SW_LH_LOCK_OUT,    WireType::kBit}},
        {4171U, {kWireDOOR_LOCK_SW_LH_UNLOCK_OUT,  WireType::kBit}},
        {4172U, {kWireDOOR_LOCK_SW_RH_LOCK_OUT,    WireType::kBit}},
        {4173U, {kWireDOOR_LOCK_SW_RH_UNLOCK_OUT,  WireType::kBit}},
        // (4187 LHJB_CHARGE_WAKE_PASSTHRU also excluded — see the HV-bus note
        //  above: it is lhjb_ecu-driven, not an ev1sim output.)
        // Driver inputs — main segment (byte / uint16 / bit / float32)
        {6900U, {kWireDRIVER_BRAKE_PEDAL_Q8,            WireType::kByte}},
        {6901U, {kWireDRIVER_STEERING_DEG_Q8,           WireType::kUint16}},
        {6902U, {kWireDRIVER_GEAR_SELECTOR,             WireType::kByte}},
        {6903U, {kWireDRIVER_THROTTLE_Q8,               WireType::kByte}},
        {6904U, {kWireDRIVER_BRAKE_SWITCH,              WireType::kBit}},
        // External wheel speed (float32)
        {6910U, {kWireEXT_WHEEL_SPEED_FL,               WireType::kFloat32}},
        {6911U, {kWireEXT_WHEEL_SPEED_FR,               WireType::kFloat32}},
        {6912U, {kWireEXT_WHEEL_SPEED_RL,               WireType::kFloat32}},
        {6913U, {kWireEXT_WHEEL_SPEED_RR,               WireType::kFloat32}},
        // Driver buttons (bit / byte)
        {6952U, {kWireDRIVER_IPC_TRIP_RESET_BUTTON,      WireType::kBit}},
        {6964U, {kWireDRIVER_SEATBELT_BUCKLED,           WireType::kBit}},
        {6965U, {kWireDRIVER_SEATBELT_BUCKLED_PASSENGER, WireType::kBit}},
        {6971U, {kWireDRIVER_RSA_MODE_BUTTON,            WireType::kByte}},
        {6975U, {kWireDRIVER_RSA_KEYPAD_BUTTON1,         WireType::kBit}},
        {6976U, {kWireDRIVER_RSA_KEYPAD_BUTTON2,         WireType::kBit}},
        {6977U, {kWireDRIVER_RSA_KEYPAD_BUTTON3,         WireType::kBit}},
        {6978U, {kWireDRIVER_RSA_KEYPAD_BUTTON4,         WireType::kBit}},
        {6979U, {kWireDRIVER_RSA_KEYPAD_BUTTON5,         WireType::kBit}},
        // Power window switches (bit)
        {6980U, {kWireDRIVER_POWER_WINDOW_DRIVER_UP,      WireType::kBit}},
        {6981U, {kWireDRIVER_POWER_WINDOW_DRIVER_DOWN,    WireType::kBit}},
        {6982U, {kWireDRIVER_POWER_WINDOW_PASSENGER_UP,   WireType::kBit}},
        {6983U, {kWireDRIVER_POWER_WINDOW_PASSENGER_DOWN, WireType::kBit}},
        // RSA exterior keypad (byte)
        {6985U, {kWireDRIVER_RSA_EXTERIOR_KEYPAD1, WireType::kByte}},
        {6986U, {kWireDRIVER_RSA_EXTERIOR_KEYPAD2, WireType::kByte}},
        {6987U, {kWireDRIVER_RSA_EXTERIOR_KEYPAD3, WireType::kByte}},
        {6988U, {kWireDRIVER_RSA_EXTERIOR_KEYPAD4, WireType::kByte}},
        {6989U, {kWireDRIVER_RSA_EXTERIOR_KEYPAD5, WireType::kByte}},
        // Door handle attempt (bit)
        {6990U, {kWireDRIVER_DOOR_HANDLE_ATTEMPT_DRIVER,    WireType::kBit}},
        {6991U, {kWireDRIVER_DOOR_HANDLE_ATTEMPT_PASSENGER, WireType::kBit}},
    };
    return kRegistry;
}

struct WireTruthChassis::Impl {
    std::unique_ptr<electricsim::io::WireTable> table;
};

WireTruthChassis::WireTruthChassis() : impl_(std::make_unique<Impl>()) {}
WireTruthChassis::~WireTruthChassis() = default;

std::unique_ptr<WireTruthChassis> WireTruthChassis::OpenFromEnv(
    const char* default_role) {
    auto table = electricsim::topology::try_open_from_env(default_role);
    if (!table) return nullptr;  // disabled / segment down / hash mismatch
    std::unique_ptr<WireTruthChassis> self(new WireTruthChassis());
    self->impl_->table = std::move(table);
    return self;
}

std::unique_ptr<WireTruthChassis> WireTruthChassis::Attach(
    const std::string& segment_name) {
    electricsim::io::WireTableOptions opts;
    opts.name = segment_name;
    opts.topology_hash = electricsim::topology::kTopologyHash;
    auto table = electricsim::io::WireTable::attach(opts);
    if (!table) return nullptr;
    std::unique_ptr<WireTruthChassis> self(new WireTruthChassis());
    self->impl_->table = std::move(table);
    return self;
}

bool WireTruthChassis::attached() const {
    return impl_ && impl_->table != nullptr;
}

std::optional<bool> WireTruthChassis::read_bit(std::uint32_t wire_id) const {
    if (!attached()) return std::nullopt;
    electricsim::io::WireTable::Sample<bool> sample;
    if (!impl_->table->read_bit_sample(wire_id, &sample)) return std::nullopt;
    if (!sample.written()) return std::nullopt;  // producer hasn't moved yet
    return sample.value;
}

bool WireTruthChassis::write_bit(std::uint32_t wire_id, bool value) {
    if (!attached()) return false;
    return impl_->table->write_bit(wire_id, value);
}

bool WireTruthChassis::write_byte(std::uint32_t wire_id, std::uint8_t value) {
    if (!attached()) return false;
    return impl_->table->write_byte(wire_id, value);
}

bool WireTruthChassis::write_uint16(std::uint32_t wire_id, std::uint16_t value) {
    if (!attached()) return false;
    return impl_->table->write_uint16(wire_id, value);
}

bool WireTruthChassis::write_uint32(std::uint32_t wire_id, std::uint32_t value) {
    if (!attached()) return false;
    return impl_->table->write_uint32(wire_id, value);
}

bool WireTruthChassis::write_uint64(std::uint32_t wire_id, std::uint64_t value) {
    if (!attached()) return false;
    return impl_->table->write_uint64(wire_id, value);
}

bool WireTruthChassis::write_float32(std::uint32_t wire_id, float value) {
    if (!attached()) return false;
    return impl_->table->write_float32(wire_id, value);
}

bool WireTruthChassis::mirror_signal(std::uint32_t signal_id,
                                     const std::uint8_t* payload, std::size_t n) {
    if (!attached()) return false;
    const auto& reg = ProducerRegistry();
    auto it = reg.find(signal_id);
    if (it == reg.end()) return false;
    const ProducerCell& cell = it->second;
    switch (cell.wire_type) {
        case WireType::kBit:
            if (n < 1u) return false;
            return impl_->table->write_bit(cell.wire_id, (payload[0] & 1u) != 0u);
        case WireType::kByte:
            if (n < 1u) return false;
            return impl_->table->write_byte(cell.wire_id, payload[0]);
        case WireType::kUint16: {
            if (n < 2u) return false;
            std::uint16_t v = static_cast<std::uint16_t>(payload[0]) |
                              (static_cast<std::uint16_t>(payload[1]) << 8u);
            return impl_->table->write_uint16(cell.wire_id, v);
        }
        case WireType::kUint32: {
            if (n < 4u) return false;
            std::uint32_t v = static_cast<std::uint32_t>(payload[0])        |
                              (static_cast<std::uint32_t>(payload[1]) << 8u) |
                              (static_cast<std::uint32_t>(payload[2]) << 16u)|
                              (static_cast<std::uint32_t>(payload[3]) << 24u);
            return impl_->table->write_uint32(cell.wire_id, v);
        }
        case WireType::kUint64: {
            if (n < 8u) return false;
            std::uint64_t v = 0;
            for (std::size_t i = 0u; i < 8u; ++i)
                v |= static_cast<std::uint64_t>(payload[i]) << (8u * i);
            return impl_->table->write_uint64(cell.wire_id, v);
        }
        case WireType::kFloat32: {
            if (n < 4u) return false;
            std::uint32_t bits = static_cast<std::uint32_t>(payload[0])        |
                                 (static_cast<std::uint32_t>(payload[1]) << 8u) |
                                 (static_cast<std::uint32_t>(payload[2]) << 16u)|
                                 (static_cast<std::uint32_t>(payload[3]) << 24u);
            float f;
            std::memcpy(&f, &bits, sizeof(f));
            return impl_->table->write_float32(cell.wire_id, f);
        }
        default:
            return false;
    }
}

std::optional<bool> WireTruthChassis::horn_low_drive() const {
    return read_bit(electricsim::topology::kWireHORN_DRIVE_LINE_LOW);
}

std::optional<bool> WireTruthChassis::horn_high_drive() const {
    return read_bit(electricsim::topology::kWireHORN_DRIVE_LINE_HIGH);
}

}  // namespace ev1sim

#else  // !EV1SIM_HAVE_WIRE_TRUTH

// Disabled stub: the electricsim wire-truth substrate is not available at build
// time (e.g. the CI integrated-build against tests/electricsim_stub, which has
// no wire_table.cpp / topology_generated.h). Factories return nullptr so every
// caller's `if (wire)` guard takes the legacy-ring path; the accessors exist
// only to satisfy the linker and never run.

namespace ev1sim {

struct WireTruthChassis::Impl {};

WireTruthChassis::WireTruthChassis() = default;
WireTruthChassis::~WireTruthChassis() = default;

std::unique_ptr<WireTruthChassis> WireTruthChassis::OpenFromEnv(const char*) {
    return nullptr;
}
std::unique_ptr<WireTruthChassis> WireTruthChassis::Attach(const std::string&) {
    return nullptr;
}
bool WireTruthChassis::attached() const { return false; }
std::optional<bool> WireTruthChassis::read_bit(std::uint32_t) const {
    return std::nullopt;
}
bool WireTruthChassis::write_bit(std::uint32_t, bool)          { return false; }
bool WireTruthChassis::write_byte(std::uint32_t, std::uint8_t) { return false; }
bool WireTruthChassis::write_uint16(std::uint32_t, std::uint16_t) { return false; }
bool WireTruthChassis::write_uint32(std::uint32_t, std::uint32_t) { return false; }
bool WireTruthChassis::write_uint64(std::uint32_t, std::uint64_t) { return false; }
bool WireTruthChassis::write_float32(std::uint32_t, float)     { return false; }
bool WireTruthChassis::mirror_signal(std::uint32_t, const std::uint8_t*, std::size_t) {
    return false;
}
std::optional<bool> WireTruthChassis::horn_low_drive() const {
    return std::nullopt;
}
std::optional<bool> WireTruthChassis::horn_high_drive() const {
    return std::nullopt;
}

}  // namespace ev1sim

#endif  // EV1SIM_HAVE_WIRE_TRUTH
