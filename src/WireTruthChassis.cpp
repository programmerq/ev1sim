// WireTruthChassis implementation. See WireTruthChassis.h for the contract.
//
// @design 2026-06-15 — wire-truth migration kickoff (proof-of-life batch).

#include "WireTruthChassis.h"

#if defined(EV1SIM_HAVE_WIRE_TRUTH)

// Real implementation: attach the external sim's shared WireTable. These headers
// come from the external sim's source tree (compiled into the wire-substrate library
// static lib); the include root is ${ELECTRICSIM_DIR}/src/io.
#include "topology/env_open.hpp"          // electricsim::topology::try_open_from_env
#include "topology/topology_generated.h"  // kTopologyHash, kWireHORN_DRIVE_LINE_*
#include "wire_table.hpp"                 // electricsim::io::WireTable
#include "gm8192/gm8192_rx_framer.hpp"    // electricsim::io::Gm8192RxFramer (frame snoop)

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unordered_map>
#include <utility>

// Use the topology namespace for kWire* constants.
using namespace electricsim::topology;
using electricsim::io::WireId;
using electricsim::io::WireType;

// ---------------------------------------------------------------------------
// ReadOnlyWireId — the ONE way a ConductorId becomes a numeric id in ev1sim.
//
// The external sim's topology now emits conductor cells as a distinct type with no
// write overload, so that a module cannot state that its own feed is hot: a
// conductor's energisation is an OUTPUT of the solver, not a claim any peer makes
// about itself. ev1sim is downstream of that solver and CONSUMES conductor cells
// (lamp feeds, telltales, horn drive) — reading them is legitimate and is the
// whole reason ev1sim attaches at all.
//
// But ev1sim's read accessors take a plain numeric id, because at runtime the ids
// arrive from the consumer registry rather than as constants. So a conversion has
// to exist somewhere. This is it, and it is deliberately shaped so it cannot
// become a write:
//
//   * It is named for what it authorises. A future reader reaching for it to build
//     a write has to type "ReadOnly" to do it.
//   * It is the ONLY place in src/ that names the conductor type — enforced by
//     tests/check_conductor_discipline.sh, a ratchet that fails on != so a second
//     site cannot be introduced by netting one against a deleted legitimate use.
//   * The write path refuses conductor cells at runtime regardless (see
//     RefuseConductorWrite below), so even reaching a numeric conductor id does
//     not buy a write.
//
// The type-split's guarantee is preserved where it matters: there is no expression
// in ev1sim that writes a conductor cell.
static constexpr WireId ReadOnlyWireId(electricsim::io::ConductorId id) {
    return static_cast<WireId>(id);
}

namespace ev1sim {

// ---------------------------------------------------------------------------
// Producer registry — maps every ev1sim-produced ring signal_id to the
// corresponding external WireId + WireType.  Generated from
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
        // Charger coupler (bit) — MULTI-PRODUCER: both ev1sim (operator UI) and
        // the in-repo charger demo peer write it. The wire mirror is plain
        // last-writer-wins, identical to the ring's existing behavior for this
        // cell (the external sim's topology declares it multi-producer); ev1sim already
        // publishes 4060 on the ring on-change, this extends the same write to
        // the wire so the coupler consumers (bpm/pim/lhjb/ad) see the operator
        // UI's toggle once they are wire-authoritative. Required before ev1sim
        // can drop its ring writes (else the coupler would have no wire writer).
        // @design 2026-06-16 — Phase 5 prep; external sim, flag if you want
        // arbitration beyond last-writer-wins.
        {4060U, {kWireCHARGER_COUPLER_PRESENT,      WireType::kBit}},
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
        // They are externally-produced cells — topology.yaml gives them a
        // `driver:` (hv_bus_host / lhjb_ecu), not the omitted-producer form
        // ev1sim's own outputs use — and were migrated in an earlier external sim
        // step. ev1sim neither produces nor publishes them, so mirroring them
        // here would be a wrong-owner write onto a cell external sim drives.
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
        // Interior keypad buttons are BYTE, not bit: the cell carries the
        // tap-vs-long-press digit encoding (0=idle, 1=tap/lower, 2=long/higher),
        // matching the external sim's topology (the external sim's topology declaration
        // DRIVER_RSA_KEYPAD_BUTTON{1..5}: type byte, since the 2026-06-19
        // rsa-keypad-window-wire-type change) and the byte read in RSA's
        // rsa_apply_driver_input_wires(). Declaring them kBit here silently sent
        // every digit to a bit cell RSA never reads as a byte, so RSA never
        // authenticated and the co-sim vehicle never left PARK. The mode button
        // (6971) and exterior keypad (6985-6989) were already byte, which is why
        // ACC was received but no interior code ever was.
        // @design 2026-06-25 claude — cross-repo wire-type sync fix.
        {6975U, {kWireDRIVER_RSA_KEYPAD_BUTTON1,         WireType::kByte}},
        {6976U, {kWireDRIVER_RSA_KEYPAD_BUTTON2,         WireType::kByte}},
        {6977U, {kWireDRIVER_RSA_KEYPAD_BUTTON3,         WireType::kByte}},
        {6978U, {kWireDRIVER_RSA_KEYPAD_BUTTON4,         WireType::kByte}},
        {6979U, {kWireDRIVER_RSA_KEYPAD_BUTTON5,         WireType::kByte}},
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
    // GM-8192 frame snoop (Phase 1): lazily constructed on the first snoop_step()
    // after `table` is attached. Drains GM8192_PIM_TX → $41 frames → vehicle speed.
    std::unique_ptr<electricsim::io::Gm8192RxFramer> pim_framer;
    std::optional<std::uint8_t>                      last_pim_speed_kph;
};

WireTruthChassis::WireTruthChassis() : impl_(std::make_unique<Impl>()) {}
WireTruthChassis::~WireTruthChassis() = default;

// Outcome of the most recent OpenFromEnv(). Process-wide because the attach is a
// once-per-process fleet event; the connector attaches exactly once and latches.
static WireTruthChassis::AttachOutcome g_last_attach_outcome =
    WireTruthChassis::AttachOutcome::kNotAttempted;

// Was wire truth REQUESTED? This is the distinction OpenFromEnv()'s nullptr
// cannot carry. It mirrors try_open_from_env's own gate exactly: an unset or
// empty ELECTRICSIM_WIRES_NAME is the documented "disabled" state, and anything
// else is a request. Read here rather than inferred from the failure so that
// "nobody asked" can never be reported for a run that did ask.
static bool WireTruthRequested() {
    const char* name = std::getenv("ELECTRICSIM_WIRES_NAME");
    return name != nullptr && name[0] != '\0';
}

std::unique_ptr<WireTruthChassis> WireTruthChassis::OpenFromEnv(
    const char* default_role) {
    const bool requested = WireTruthRequested();
    auto table = electricsim::topology::try_open_from_env(default_role);
    if (!table) {
        // The two nullptr cases are opposite facts — see AttachOutcome in the
        // header. Recording which one happened is the whole point: the caller
        // decides, and it can only decide if it is told.
        g_last_attach_outcome = requested ? AttachOutcome::kRequestedButUnavailable
                                          : AttachOutcome::kDisabled;
        return nullptr;
    }
    std::unique_ptr<WireTruthChassis> self(new WireTruthChassis());
    self->impl_->table = std::move(table);
    g_last_attach_outcome = AttachOutcome::kAttached;
    return self;
}

WireTruthChassis::AttachOutcome WireTruthChassis::LastAttachOutcome() {
    return g_last_attach_outcome;
}

const char* WireTruthChassis::AttachOutcomeName(AttachOutcome outcome) {
    switch (outcome) {
        case AttachOutcome::kNotAttempted:            return "NOT_ATTEMPTED";
        case AttachOutcome::kDisabled:                return "DISABLED";
        case AttachOutcome::kAttached:                return "ATTACHED";
        case AttachOutcome::kRequestedButUnavailable: return "REQUESTED_BUT_UNAVAILABLE";
    }
    return "UNKNOWN";
}

std::uint32_t WireTruthChassis::CompiledTopologyHash() {
    return electricsim::topology::kTopologyHash;
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

// RefuseConductorWrite — the runtime half of the conductor discipline.
//
// The compile-time half (the ConductorId type) protects code that names a cell as
// a CONSTANT. ev1sim's write accessors take a runtime id, so the type is not in
// play at these call sites and a numeric conductor id could reach them — from the
// producer registry, from a ring signal_id mapping, or from a future caller that
// computes an id.
//
// ev1sim produces 85 cells and NONE of them is a conductor (measured against the
// migrated topology: every ProducerRegistry cell is semantic or element-state).
// So this refusal is not a policy choice about what ev1sim ought to write — it is
// a statement of what ev1sim already does, made enforceable. Correct operation
// never trips it, and it fires the moment that stops being true.
//
// Returns true if the write must be refused, having reported it.
static bool RefuseConductorWrite(std::uint32_t wire_id, const char* accessor) {
    if (cell_class_for(static_cast<WireId>(wire_id)) !=
        electricsim::io::CellClass::kConductor) {
        return false;
    }
    // Loud, once per (cell, accessor) pair — a wrong-owner write that repeats every
    // tick must not be able to bury the rest of the log, but it must also never be
    // silent.
    static std::unordered_map<std::uint64_t, bool> reported;
    const std::uint64_t key =
        (static_cast<std::uint64_t>(wire_id) << 8) ^
        static_cast<std::uint64_t>(accessor[0]);
    if (!reported[key]) {
        reported[key] = true;
        std::cerr << "[ExternalSim] REFUSED " << accessor << " to conductor cell "
                  << wire_id << " (" << wire_name_for(static_cast<WireId>(wire_id))
                  << ")\n"
                     "    A conductor's energisation is solved from the circuit, not\n"
                     "    asserted by a peer. ev1sim consumes conductor cells and must\n"
                     "    never produce one. If ev1sim genuinely needs to drive this,\n"
                     "    it is an element-state or semantic cell in the topology, not\n"
                     "    a conductor — fix the classification, not this check.\n"
                  << std::flush;
    }
    return true;
}

bool WireTruthChassis::write_bit(std::uint32_t wire_id, bool value) {
    if (!attached()) return false;
    if (RefuseConductorWrite(wire_id, "write_bit")) return false;
    return impl_->table->write_bit(wire_id, value);
}

bool WireTruthChassis::write_byte(std::uint32_t wire_id, std::uint8_t value) {
    if (!attached()) return false;
    if (RefuseConductorWrite(wire_id, "write_byte")) return false;
    return impl_->table->write_byte(wire_id, value);
}

bool WireTruthChassis::write_uint16(std::uint32_t wire_id, std::uint16_t value) {
    if (!attached()) return false;
    if (RefuseConductorWrite(wire_id, "write_uint16")) return false;
    return impl_->table->write_uint16(wire_id, value);
}

bool WireTruthChassis::write_uint32(std::uint32_t wire_id, std::uint32_t value) {
    if (!attached()) return false;
    if (RefuseConductorWrite(wire_id, "write_uint32")) return false;
    return impl_->table->write_uint32(wire_id, value);
}

bool WireTruthChassis::write_uint64(std::uint32_t wire_id, std::uint64_t value) {
    if (!attached()) return false;
    if (RefuseConductorWrite(wire_id, "write_uint64")) return false;
    return impl_->table->write_uint64(wire_id, value);
}

bool WireTruthChassis::write_float32(std::uint32_t wire_id, float value) {
    if (!attached()) return false;
    if (RefuseConductorWrite(wire_id, "write_float32")) return false;
    return impl_->table->write_float32(wire_id, value);
}

// ── Co-sim tick barrier (primitive-4 B): leader-side forwarders ──────────────
// ev1sim is the barrier LEADER. These thin-forward to the external WireTable
// barrier primitive. No-ops when not attached so a wire-disabled run is inert.
void WireTruthChassis::BarrierArm() {
    if (attached()) impl_->table->barrier_arm();
}
void WireTruthChassis::BarrierPublishTick() {
    if (attached()) impl_->table->barrier_publish_tick();
}
bool WireTruthChassis::BarrierAwaitAcks(std::uint32_t consumer_count,
                                        int timeout_ms) {
    if (!attached()) return true;  // no barrier → nothing to wait for
    return impl_->table->barrier_await_acks(consumer_count, timeout_ms);
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

std::optional<std::uint8_t> WireTruthChassis::read_byte(std::uint32_t wire_id) const {
    if (!attached()) return std::nullopt;
    electricsim::io::WireTable::Sample<std::uint8_t> sample;
    if (!impl_->table->read_byte_sample(wire_id, &sample)) return std::nullopt;
    if (!sample.written()) return std::nullopt;
    return sample.value;
}

std::optional<std::uint16_t> WireTruthChassis::read_uint16(std::uint32_t wire_id) const {
    if (!attached()) return std::nullopt;
    electricsim::io::WireTable::Sample<std::uint16_t> sample;
    if (!impl_->table->read_uint16_sample(wire_id, &sample)) return std::nullopt;
    if (!sample.written()) return std::nullopt;
    return sample.value;
}

std::optional<float> WireTruthChassis::read_float32(std::uint32_t wire_id) const {
    if (!attached()) return std::nullopt;
    electricsim::io::WireTable::Sample<float> sample;
    if (!impl_->table->read_float32_sample(wire_id, &sample)) return std::nullopt;
    if (!sample.written()) return std::nullopt;
    return sample.value;
}

std::optional<std::uint32_t> WireTruthChassis::read_uint32(std::uint32_t wire_id) const {
    if (!attached()) return std::nullopt;
    electricsim::io::WireTable::Sample<std::uint32_t> sample;
    if (!impl_->table->read_uint32_sample(wire_id, &sample)) return std::nullopt;
    if (!sample.written()) return std::nullopt;
    return sample.value;
}

// ---------------------------------------------------------------------------
// Consumer registry — maps every ev1sim-CONSUMED ring signal_id to the
// corresponding external WireId + WireType.  Generated from
// /tmp/consumer_cells_v2.tsv (66 entries).  Using the real kWire* constants
// from topology_generated.h is the drift guard: a renamed/removed constant
// fails to compile, forcing this table to be updated in lockstep with the
// topology.  NOTE: types are bit/byte/float32/uint32 ONLY (no uint16/uint64
// in the consumer set).
// @design 2026-06-15 — consumer overlay batch.
// ---------------------------------------------------------------------------
struct ConsumerCell {
    WireId   wire_id;
    WireType wire_type;
};

static const std::unordered_map<std::uint32_t, ConsumerCell>& ConsumerRegistry() {
    static const std::unordered_map<std::uint32_t, ConsumerCell> kRegistry = {
        // Bulb feed lines (bit)
        {4000U, {ReadOnlyWireId(kWireBULB_FEED_LINE_LBL),              WireType::kBit}},
        {4001U, {ReadOnlyWireId(kWireBULB_FEED_LINE_RBL),              WireType::kBit}},
        {4002U, {ReadOnlyWireId(kWireBULB_FEED_LINE_LHBH),             WireType::kBit}},
        {4003U, {ReadOnlyWireId(kWireBULB_FEED_LINE_RHBH),             WireType::kBit}},
        {4004U, {ReadOnlyWireId(kWireBULB_FEED_LINE_LLBH),             WireType::kBit}},
        {4005U, {ReadOnlyWireId(kWireBULB_FEED_LINE_RLBH),             WireType::kBit}},
        {4006U, {ReadOnlyWireId(kWireBULB_FEED_LINE_LRSM),             WireType::kBit}},
        {4007U, {ReadOnlyWireId(kWireBULB_FEED_LINE_RRSM),             WireType::kBit}},
        {4008U, {ReadOnlyWireId(kWireBULB_FEED_LINE_LFML),             WireType::kBit}},
        {4009U, {ReadOnlyWireId(kWireBULB_FEED_LINE_RFML),             WireType::kBit}},
        {4010U, {ReadOnlyWireId(kWireBULB_FEED_LINE_LFTS),             WireType::kBit}},
        {4011U, {ReadOnlyWireId(kWireBULB_FEED_LINE_RFTS),             WireType::kBit}},
        {4012U, {ReadOnlyWireId(kWireBULB_FEED_LINE_LRTS),             WireType::kBit}},
        {4013U, {ReadOnlyWireId(kWireBULB_FEED_LINE_RRTS),             WireType::kBit}},
        {4014U, {ReadOnlyWireId(kWireBULB_FEED_LINE_LRSL),             WireType::kBit}},
        {4015U, {ReadOnlyWireId(kWireBULB_FEED_LINE_CHMSL),            WireType::kBit}},
        {4016U, {ReadOnlyWireId(kWireBULB_FEED_LINE_RRSL),             WireType::kBit}},
        // Horn drive lines (bit)
        {4020U, {ReadOnlyWireId(kWireHORN_DRIVE_LINE_LOW),              WireType::kBit}},
        {4021U, {ReadOnlyWireId(kWireHORN_DRIVE_LINE_HIGH),             WireType::kBit}},
        // Motor current (float32)
        {4072U, {kWireCHASSIS_MOTOR_CURRENT_A,          WireType::kFloat32}},
        // PIM commanded throttle (byte, PIM -> ev1sim). MISSING from the
        // original wire-truth consumer set: the ring used to deliver 4073 as a
        // 1-byte delta into DebugInjectU8, and the wire-truth migration ported the
        // sibling PIM output (motor current 4072, above) but not this one — so
        // after the wire-truth migration ev1sim never saw PIM's command at all. GetThrottleCmd()
        // stayed at its never-received default (q8=0xFF, fresh=false): the
        // electronics driver-mode plant silently fell back to the local pedal,
        // and the VAT stats columns (throttle_cmd_q8 / throttle_cmd_fresh)
        // recorded the 255/0 placeholder — exactly what the nightly's
        // safety_pps_triplet_fail / safety_brake_throttle_override /
        // cruise_hold failures were asserting against. Found 2026-07-04
        // triaging those. @design 2026-07-04 — restore the consumer mapping.
        {4073U, {kWireCHASSIS_THROTTLE_CMD_Q8,          WireType::kByte}},
        // Wiper motor command + washer pump command (byte / bit)
        {4080U, {kWireCHASSIS_WIPER_MOTOR_COMMAND,      WireType::kByte}},
        {4081U, {kWireCHASSIS_WASHER_PUMP_COMMAND,      WireType::kBit}},
        // HVAC blower level + defrost grid (byte / bit)
        {4082U, {kWireCHASSIS_HVAC_BLOWER_LEVEL,        WireType::kByte}},
        {4083U, {kWireCHASSIS_DEFROST_GRID_ACTIVE,      WireType::kBit}},
        // Door lock commands (byte)
        {4084U, {kWireCHASSIS_DOOR_LOCK_CMD_DRIVER,     WireType::kByte}},
        {4085U, {kWireCHASSIS_DOOR_LOCK_CMD_PASSENGER,  WireType::kByte}},
        // Power window motor commands (byte)
        {4086U, {kWireCHASSIS_POWER_WINDOW_MOTOR_DRIVER,    WireType::kByte}},
        {4087U, {kWireCHASSIS_POWER_WINDOW_MOTOR_PASSENGER, WireType::kByte}},
        // RSA shift-blocked cue (bit)
        {4088U, {kWireCHASSIS_RSA_SHIFT_BLOCKED,        WireType::kBit}},
        // IPC seatbelt telltales (bit)
        {4130U, {ReadOnlyWireId(kWireCHASSIS_IPC_SEATBELT_TELLTALE_DRIVER),    WireType::kBit}},
        {4131U, {ReadOnlyWireId(kWireCHASSIS_IPC_SEATBELT_TELLTALE_PASSENGER), WireType::kBit}},
        // IPC trip distance (float32)
        {4132U, {kWireCHASSIS_IPC_TRIP_DISTANCE_M,      WireType::kFloat32}},
        // IPC BTCM / airbag telltales (bit)
        {4134U, {ReadOnlyWireId(kWireCHASSIS_IPC_BRAKE_TELLTALE),       WireType::kBit}},
        {4135U, {ReadOnlyWireId(kWireCHASSIS_IPC_PARK_BRAKE_TELLTALE),  WireType::kBit}},
        {4136U, {ReadOnlyWireId(kWireCHASSIS_IPC_ANTILOCK_TELLTALE),    WireType::kBit}},
        {4138U, {ReadOnlyWireId(kWireCHASSIS_IPC_AIR_BAG_TELLTALE),     WireType::kBit}},
        // IPC extra LCD telltales (bit)
        {4140U, {ReadOnlyWireId(kWireCHASSIS_IPC_SERVICE_NOW_TELLTALE),      WireType::kBit}},
        {4141U, {ReadOnlyWireId(kWireCHASSIS_IPC_CHECK_MESSAGES_TELLTALE),   WireType::kBit}},
        {4142U, {ReadOnlyWireId(kWireCHASSIS_IPC_TEMP_TELLTALE),             WireType::kBit}},
        {4143U, {ReadOnlyWireId(kWireCHASSIS_IPC_BATTERY_LIFE_TELLTALE),     WireType::kBit}},
        {4144U, {ReadOnlyWireId(kWireCHASSIS_IPC_REDUCED_PERF_TELLTALE),     WireType::kBit}},
        {4145U, {ReadOnlyWireId(kWireCHASSIS_IPC_CHECK_TIRE_PRESS_TELLTALE), WireType::kBit}},
        // BTCM actuator state — chassis bus (bit / float32)
        {4147U, {ReadOnlyWireId(kWireCHASSIS_BTCM_ISO_CLOSE_FL),        WireType::kBit}},
        {4148U, {ReadOnlyWireId(kWireCHASSIS_BTCM_ISO_CLOSE_FR),        WireType::kBit}},
        {4149U, {ReadOnlyWireId(kWireCHASSIS_BTCM_DUMP_OPEN_FL),        WireType::kBit}},
        {4150U, {ReadOnlyWireId(kWireCHASSIS_BTCM_DUMP_OPEN_FR),        WireType::kBit}},
        {4151U, {kWireCHASSIS_BTCM_EMB_MOTOR_CMD_LR,    WireType::kFloat32}},
        {4152U, {kWireCHASSIS_BTCM_EMB_MOTOR_CMD_RR,    WireType::kFloat32}},
        {4153U, {kWireCHASSIS_BTCM_CYL_PRESSURE_FL_K_PA, WireType::kFloat32}},
        {4154U, {kWireCHASSIS_BTCM_CYL_PRESSURE_FR_K_PA, WireType::kFloat32}},
        // IPC turn / high-beam / park-lamp / door-ajar telltales + dim duty (bit / byte)
        {4158U, {ReadOnlyWireId(kWireCHASSIS_IPC_LEFT_TURN_TELLTALE),   WireType::kBit}},
        {4159U, {ReadOnlyWireId(kWireCHASSIS_IPC_RIGHT_TURN_TELLTALE),  WireType::kBit}},
        {4160U, {ReadOnlyWireId(kWireCHASSIS_IPC_HIGH_BEAM_TELLTALE),   WireType::kBit}},
        {4161U, {ReadOnlyWireId(kWireCHASSIS_IPC_PARK_LAMP_TELLTALE),   WireType::kBit}},
        {4162U, {ReadOnlyWireId(kWireCHASSIS_IPC_DOOR_AJAR_TELLTALE),   WireType::kBit}},
        {4163U, {kWireCHASSIS_IPC_DIM_DUTY_PCT,         WireType::kByte}},
        // RHJB PMM run buses + DLM legs + DILM level (bit / byte)
        {4180U, {ReadOnlyWireId(kWireCHASSIS_RHJB_PMM_RUN1_BUS),        WireType::kBit}},
        {4181U, {ReadOnlyWireId(kWireCHASSIS_RHJB_PMM_RUN2_BUS),        WireType::kBit}},
        {4182U, {ReadOnlyWireId(kWireCHASSIS_RHJB_DLM_LH_LOCK),         WireType::kBit}},
        {4183U, {ReadOnlyWireId(kWireCHASSIS_RHJB_DLM_LH_UNLOCK),       WireType::kBit}},
        {4184U, {ReadOnlyWireId(kWireCHASSIS_RHJB_DLM_RH_LOCK),         WireType::kBit}},
        {4185U, {ReadOnlyWireId(kWireCHASSIS_RHJB_DLM_RH_UNLOCK),       WireType::kBit}},
        {4186U, {kWireCHASSIS_RHJB_DILM_LEVEL,          WireType::kByte}},
        // BTCM retard-request PWM duty (uint16, Q8 percent 0..25600). This is
        // the wire the BTCM asks the propulsion side for regen torque on
        // (@source:manual brakes-313 "the drive unit motor is electrically
        // converted into a generator"; PIM J1-18 RETARD REQUEST IN). Consumed
        // here so an acceptance scenario can witness "regen was COMMANDED"
        // rather than inferring it from the deceleration trace, which friction
        // braking produces identically.
        {4191U, {kWireCHASSIS_BTCM_RETARD_REQUEST_DUTY_Q8, WireType::kUint16}},
        // TJB rear-lamp branches (bit). The LHJB's BULB_FEED_LINE_LRSL/_RRSL
        // (4014/4016, already consumed above) are the feeds INTO the trunk
        // junction block; these two are what the TJB actually hands the rear
        // stop lamps, downstream of its RUN1 gate. Asserting on the LHJB feed
        // alone cannot tell a live TJB from an absent one.
        {4203U, {ReadOnlyWireId(kWireCHASSIS_TJB_LR_STOP_LAMP),         WireType::kBit}},
        {4204U, {ReadOnlyWireId(kWireCHASSIS_TJB_RR_STOP_LAMP),         WireType::kBit}},
        {4198U, {ReadOnlyWireId(kWireCHASSIS_TJB_LR_TAIL_LAMP),         WireType::kBit}},
        {4199U, {ReadOnlyWireId(kWireCHASSIS_TJB_RR_TAIL_LAMP),         WireType::kBit}},
        // Aux battery (uint32 / bit / byte)
        {4192U, {kWireCHASSIS_AUX_BATTERY_TERMINAL_MV,  WireType::kUint32}},
        {4193U, {kWireCHASSIS_AUX_BATTERY_PRESENT,      WireType::kBit}},
        {4194U, {kWireCHASSIS_AUX_BATTERY_SOC_PCT,      WireType::kByte}},
        // NOTE: HV bus 4155-4157 and charge-wake 4187 are intentionally ABSENT.
        // They are externally-produced cells (topology `driver:`), not ev1sim
        // consumer cells. ev1sim does not read them from this class.
        // (See the same NOTE in ProducerRegistry above.)
    };
    return kRegistry;
}

int WireTruthChassis::apply_consumer_overlay(const ConsumerSinks& sinks) const {
    if (!attached()) return 0;
    int count = 0;
    for (const auto& [signal_id, cell] : ConsumerRegistry()) {
        switch (cell.wire_type) {
            case WireType::kBit: {
                if (!sinks.on_bit) break;
                if (auto v = read_bit(cell.wire_id)) {
                    sinks.on_bit(signal_id, *v);
                    ++count;
                }
                break;
            }
            case WireType::kByte: {
                if (!sinks.on_byte) break;
                if (auto v = read_byte(cell.wire_id)) {
                    sinks.on_byte(signal_id, *v);
                    ++count;
                }
                break;
            }
            case WireType::kFloat32: {
                if (!sinks.on_float) break;
                if (auto v = read_float32(cell.wire_id)) {
                    sinks.on_float(signal_id, *v);
                    ++count;
                }
                break;
            }
            case WireType::kUint32: {
                if (!sinks.on_uint32) break;
                if (auto v = read_uint32(cell.wire_id)) {
                    sinks.on_uint32(signal_id, *v);
                    ++count;
                }
                break;
            }
            case WireType::kUint16: {
                if (!sinks.on_uint16) break;
                if (auto v = read_uint16(cell.wire_id)) {
                    sinks.on_uint16(signal_id, *v);
                    ++count;
                }
                break;
            }
            default:
                break;
        }
    }
    return count;
}

std::optional<bool> WireTruthChassis::horn_low_drive() const {
    return read_bit(ReadOnlyWireId(electricsim::topology::kWireHORN_DRIVE_LINE_LOW));
}

std::optional<bool> WireTruthChassis::horn_high_drive() const {
    return read_bit(ReadOnlyWireId(electricsim::topology::kWireHORN_DRIVE_LINE_HIGH));
}

// ── ECU-bus semantic helpers (wire-truth Phase 4) ───────────────────────────
std::optional<bool> WireTruthChassis::rsa_run_active() const {
    // RSA_RUN1_OUT and RSA_RUN2_OUT are written identically by the RSA
    // controller (both = run_active); read RUN1 as the representative.
    return read_bit(electricsim::topology::kWireRSA_RUN1_OUT);
}

std::optional<bool> WireTruthChassis::ad_main_contactor_closed() const {
    // Read the AD's OWN commanded main-contactor output (AD_MAIN_CONTACTOR),
    // not the APM's downstream echo (APM_HV_CONTACTOR_CLOSED) it was
    // mis-mapped to during the wire-truth port. The ring signal this getter
    // replaces (kSigAdMainContactor 5224) was published BY THE AD, and the
    // echo cell is only written when an APM is in the fleet — the VAT
    // safety_ad_precharge_timeout fleet is [hv_bus, ad], so the echo stayed
    // unwritten and the healthy-run "main contactor closed" discriminator
    // read a permanent 0 while the AD's own log showed main=1. Found
    // 2026-07-04 triaging the nightly. @design 2026-07-04 — authoritative
    // source, matching the sibling getters (ad_precharge_relay, ad_state_enum),
    // which already read the AD's own cells.
    return read_bit(electricsim::topology::kWireAD_MAIN_CONTACTOR);
}

std::optional<std::uint32_t> WireTruthChassis::ad_state_enum() const {
    // Reconstruct the AD state enum from the discrete state lines + power
    // supply, replicating the external sim's AD-state decode (ad_state_decode).
    // All four contributing cells must be written; otherwise a partial set
    // would decode to a wrong state, so we return nullopt (caller keeps its
    // "no data" path) until the AD has driven the full line set.
    const auto a     = read_bit(ReadOnlyWireId(electricsim::topology::kWireAD_STATE_A));
    const auto b     = read_bit(ReadOnlyWireId(electricsim::topology::kWireAD_STATE_B));
    const auto c     = read_bit(ReadOnlyWireId(electricsim::topology::kWireAD_STATE_C));
    const auto power = read_bit(ReadOnlyWireId(electricsim::topology::kWireAD_POWER_SUPPLY));
    if (!a || !b || !c || !power) return std::nullopt;

    // AD_STATE_POWER_LOST = 8 (ad_state.h). Power-lost dominates the lines.
    if (!*power) return std::uint32_t{8u};

    // idx = (A<<2)|(B<<1)|C → ad_state_t (see ad_state.c decode table). The
    // enum integer values below match the external sim's AD-state header exactly.
    const unsigned idx = (*a ? 4u : 0u) | (*b ? 2u : 0u) | (*c ? 1u : 0u);
    switch (idx) {
        case 0x0: return std::uint32_t{0u};  // AD_STATE_OK
        case 0x1: return std::uint32_t{7u};  // AD_STATE_CAPACITOR_PRECHARGE_FAIL
        case 0x2: return std::uint32_t{5u};  // AD_STATE_BPM_COMMANDED_OPEN
        case 0x3: return std::uint32_t{3u};  // AD_STATE_INTERLOCK
        case 0x4: return std::uint32_t{6u};  // AD_STATE_CAPACITOR_PRECHARGE
        case 0x5: return std::uint32_t{4u};  // AD_STATE_LOW_PACK_VOLTAGE
        case 0x6: return std::uint32_t{2u};  // AD_STATE_GROUND_LOSS
        case 0x7: return std::uint32_t{1u};  // AD_STATE_LOSS_OF_ISOLATION
        default:  return std::uint32_t{0u};  // unreachable (idx is 3-bit)
    }
}

std::optional<std::uint64_t> WireTruthChassis::btcm_tx_total_bits() const {
    if (!attached()) return std::nullopt;
    std::uint64_t total = 0;
    if (!impl_->table->bit_stream_total(
            electricsim::topology::kWireGM8192_BTCM_TX, &total)) {
        return std::nullopt;  // undeclared / type mismatch
    }
    if (total == 0) return std::nullopt;  // nothing ever appended (BTCM silent)
    return total;
}

void WireTruthChassis::snoop_step(double now_s) {
    if (!attached()) return;
    // GM-8192 physical layer: 8192 baud → 122.07 µs/bit. Mirrors the external sim's LHJB controller (kGm8192BitPeriodNs). @source:manual.
    constexpr std::uint64_t kGm8192BitPeriodNs = 122070;
    if (!impl_->pim_framer) {
        impl_->pim_framer = std::make_unique<electricsim::io::Gm8192RxFramer>(
            impl_->table.get(), electricsim::topology::kWireGM8192_PIM_TX,
            kGm8192BitPeriodNs);
    }
    // now_ns is advisory under the bit-stream transport (the appended bits
    // self-pace), so a coarse render tick still replays every buffered bit.
    // Drain every frame produced since the last tick. $41 = PIM PCM Data
    // Response; payload[4] (wire byte 6) is vehicle speed, 1 km/h/count (0..162).
    // See the external sim's PIM UART-frame definition (PIM_UART_FRAME_ID 0x41, N=7).
    const auto now_ns = static_cast<std::uint64_t>(now_s * 1.0e9);
    while (auto frame = impl_->pim_framer->step(now_ns)) {
        if (frame->id == 0x41u && frame->n == 7u && frame->payload != nullptr) {
            impl_->last_pim_speed_kph = frame->payload[4];
        }
    }
}

std::optional<std::uint8_t> WireTruthChassis::pim_vehicle_speed_kph() const {
    if (!impl_) return std::nullopt;
    return impl_->last_pim_speed_kph;
}

std::optional<bool> WireTruthChassis::pim_cruise_active() const {
    return read_bit(electricsim::topology::kWirePIM_CRUISE_ACTIVE);
}

std::optional<float> WireTruthChassis::pim_cruise_setpoint_mps() const {
    return read_float32(electricsim::topology::kWirePIM_CRUISE_SETPOINT_MPS);
}

std::optional<bool> WireTruthChassis::ad_precharge_relay() const {
    return read_bit(electricsim::topology::kWireAD_PRECHARGE_RELAY);
}

}  // namespace ev1sim

#else  // !EV1SIM_HAVE_WIRE_TRUTH

// Disabled stub: the external sim wire-truth substrate is not available at build
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

// Compiled out: the substrate is absent from this binary, so wire truth cannot be
// "requested but unavailable" — there is nothing to request it of. kDisabled is
// the honest answer and it keeps the connector's fatal path from firing on a
// build that was never meant to have a substrate. The distinction is not lost:
// EV1SIM_HAVE_WIRE_TRUTH is a build-time fact a reader can check directly,
// whereas the runtime hash mismatch this enum exists for is not.
WireTruthChassis::AttachOutcome WireTruthChassis::LastAttachOutcome() {
    return AttachOutcome::kDisabled;
}
const char* WireTruthChassis::AttachOutcomeName(AttachOutcome outcome) {
    switch (outcome) {
        case AttachOutcome::kNotAttempted:            return "NOT_ATTEMPTED";
        case AttachOutcome::kDisabled:                return "DISABLED";
        case AttachOutcome::kAttached:                return "ATTACHED";
        case AttachOutcome::kRequestedButUnavailable: return "REQUESTED_BUT_UNAVAILABLE";
    }
    return "UNKNOWN";
}
std::uint32_t WireTruthChassis::CompiledTopologyHash() { return 0U; }

bool WireTruthChassis::attached() const { return false; }
std::optional<bool> WireTruthChassis::read_bit(std::uint32_t) const {
    return std::nullopt;
}
std::optional<std::uint8_t> WireTruthChassis::read_byte(std::uint32_t) const {
    return std::nullopt;
}
std::optional<std::uint16_t> WireTruthChassis::read_uint16(std::uint32_t) const {
    return std::nullopt;
}
std::optional<float> WireTruthChassis::read_float32(std::uint32_t) const {
    return std::nullopt;
}
std::optional<std::uint32_t> WireTruthChassis::read_uint32(std::uint32_t) const {
    return std::nullopt;
}
int WireTruthChassis::apply_consumer_overlay(const ConsumerSinks&) const {
    return 0;
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
std::optional<bool> WireTruthChassis::rsa_run_active() const {
    return std::nullopt;
}
std::optional<bool> WireTruthChassis::ad_main_contactor_closed() const {
    return std::nullopt;
}
std::optional<std::uint32_t> WireTruthChassis::ad_state_enum() const {
    return std::nullopt;
}
std::optional<std::uint64_t> WireTruthChassis::btcm_tx_total_bits() const {
    return std::nullopt;
}
void WireTruthChassis::snoop_step(double) {}
std::optional<std::uint8_t> WireTruthChassis::pim_vehicle_speed_kph() const {
    return std::nullopt;
}
std::optional<bool> WireTruthChassis::pim_cruise_active() const {
    return std::nullopt;
}
std::optional<float> WireTruthChassis::pim_cruise_setpoint_mps() const {
    return std::nullopt;
}
std::optional<bool> WireTruthChassis::ad_precharge_relay() const {
    return std::nullopt;
}

}  // namespace ev1sim

#endif  // EV1SIM_HAVE_WIRE_TRUTH
