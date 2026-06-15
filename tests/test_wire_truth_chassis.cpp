// Proof-of-life for the wire-truth migration: ev1sim attaches electricsim's
// shared WireTable and round-trips the horn drive-line cells (chassis
// 4020/4021 -> HORN_DRIVE_LINE_LOW/HIGH) through WireTruthChassis.
//
// The test plays BOTH cross-repo roles in-process: it CREATES the shared table
// the way an electricsim driver does (env_open's creator path: declare_all +
// the canonical topology hash), then ATTACHES through ev1sim's WireTruthChassis
// and asserts the values match. This proves, against the REAL generated
// topology (not a stand-in): the topology-hash attach gate, the WireId
// resolution (kWireHORN_DRIVE_LINE_*), the bit type, the written()/freshness
// fallback contract, and both read (consumer) and write (producer) directions.
//
// Only compiled when EV1SIM_HAVE_WIRE_TRUTH (the electricsim substrate is
// checked out); see CMakeLists.txt.
//
// @design 2026-06-15 — wire-truth migration kickoff.

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <unistd.h>  // getpid

#include "WireTruthChassis.h"
#include "ExternalSimConnector.h"  // end-to-end: wire -> overlay -> getter

// electricsim substrate (creator side) — same headers env_open.cpp uses.
#include "topology/topology_generated.h"
#include "wire_table.hpp"

namespace {

using electricsim::io::WireTable;
using electricsim::io::WireTableOptions;

// A unique-per-run segment name so a killed test never collides with a leaked
// segment (mirrors electricsim's $ELECTRICSIM_BUS_NAME convention). The creator
// WireTable unlinks the segment on destruction.
std::string unique_segment(const char* tag) {
    return std::string("ev1sim_wt_") + tag + "_" +
           std::to_string(static_cast<long>(::getpid()));
}

// Create the shared table as an electricsim driver would: declare the full
// topology under the canonical hash before publishing init_complete.
std::unique_ptr<WireTable> create_fleet_table(const std::string& name,
                                              std::uint32_t hash) {
    WireTableOptions opts;
    opts.name = name;
    opts.topology_hash = hash;
    return WireTable::create(opts, [](WireTable& t) {
        return electricsim::topology::declare_all(t);
    });
}

}  // namespace

TEST_CASE("WireTruthChassis: horn drive lines round-trip electricsim -> ev1sim",
          "[wire_truth]") {
    const std::string seg = unique_segment("horn");
    auto producer = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(producer != nullptr);  // creator path must succeed

    // electricsim (LHJB) writes the low-tone horn drive line on the wire.
    REQUIRE(producer->write_bit(
        electricsim::topology::kWireHORN_DRIVE_LINE_LOW, true));

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);
    REQUIRE(wire->attached());

    // Consumer side: low was written -> live value; high never written ->
    // nullopt, so the connector keeps its legacy-ring fallback (kHold-safe).
    REQUIRE(wire->horn_low_drive() == std::optional<bool>(true));
    REQUIRE(wire->horn_high_drive() == std::nullopt);

    // Producer flips low off and writes high on; the consumer tracks it.
    REQUIRE(producer->write_bit(
        electricsim::topology::kWireHORN_DRIVE_LINE_LOW, false));
    REQUIRE(producer->write_bit(
        electricsim::topology::kWireHORN_DRIVE_LINE_HIGH, true));
    REQUIRE(wire->horn_low_drive() == std::optional<bool>(false));
    REQUIRE(wire->horn_high_drive() == std::optional<bool>(true));
}

TEST_CASE("WireTruthChassis: never-written cell reads as nullopt (fallback)",
          "[wire_truth]") {
    const std::string seg = unique_segment("fresh");
    auto producer = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(producer != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);
    REQUIRE(wire->attached());

    // Nothing written yet: every read is nullopt so the consumer stays on its
    // legacy value. This is the seam that lets ev1sim migrate a consumer cell
    // BEFORE electricsim's producer has moved onto the wire.
    REQUIRE(wire->horn_low_drive() == std::nullopt);
    REQUIRE(wire->horn_high_drive() == std::nullopt);
    REQUIRE(wire->read_bit(electricsim::topology::kWireHORN_DRIVE_LINE_LOW) ==
            std::nullopt);
}

TEST_CASE("WireTruthChassis: producer write is visible to a peer reader",
          "[wire_truth]") {
    const std::string seg = unique_segment("prod");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // ev1sim writes a bit (the producer direction for ev1sim-sourced cells);
    // the creator peer observes it through the shared table.
    REQUIRE(wire->write_bit(electricsim::topology::kWireHORN_DRIVE_LINE_HIGH,
                            true));
    bool seen = false;
    REQUIRE(creator->read_bit(
        electricsim::topology::kWireHORN_DRIVE_LINE_HIGH, &seen));
    REQUIRE(seen == true);
}

TEST_CASE("WireTruthChassis: topology-hash mismatch refuses to attach",
          "[wire_truth]") {
    const std::string seg = unique_segment("hash");
    // A segment created under a DIFFERENT topology hash: WireTruthChassis
    // attaches with the canonical kTopologyHash, so the gate must reject it.
    auto stale = create_fleet_table(seg, electricsim::topology::kTopologyHash ^ 0x1U);
    REQUIRE(stale != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire == nullptr);  // hash gate fired
}

TEST_CASE("WireTruthChassis: attach to a missing segment returns nullptr",
          "[wire_truth]") {
    auto wire = ev1sim::WireTruthChassis::Attach(unique_segment("absent"));
    REQUIRE(wire == nullptr);
}

// ---------------------------------------------------------------------------
// mirror_signal round-trip tests — one cell per WireType.
// @design 2026-06-15 — producer dual-write batch.
// ---------------------------------------------------------------------------

TEST_CASE("WireTruthChassis::mirror_signal: kBit round-trip (4030 PANEL_AJAR_HOOD)",
          "[wire_truth][mirror_signal]") {
    const std::string seg = unique_segment("ms_bit");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);
    REQUIRE(wire->attached());

    // Build a 1-byte LE payload encoding "true"
    const std::uint8_t payload_true[]  = {0x01u};
    const std::uint8_t payload_false[] = {0x00u};

    // Write true via mirror_signal, read back via creator table.
    REQUIRE(wire->mirror_signal(4030U, payload_true, 1u));
    bool out = false;
    REQUIRE(creator->read_bit(electricsim::topology::kWirePANEL_AJAR_HOOD, &out));
    REQUIRE(out == true);

    // Flip to false.
    REQUIRE(wire->mirror_signal(4030U, payload_false, 1u));
    REQUIRE(creator->read_bit(electricsim::topology::kWirePANEL_AJAR_HOOD, &out));
    REQUIRE(out == false);
}

TEST_CASE("WireTruthChassis::mirror_signal: kByte round-trip (6900 DRIVER_BRAKE_PEDAL_Q8)",
          "[wire_truth][mirror_signal]") {
    const std::string seg = unique_segment("ms_byte");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    const std::uint8_t payload[] = {0xABu};
    REQUIRE(wire->mirror_signal(6900U, payload, 1u));

    std::uint8_t out = 0;
    REQUIRE(creator->read_byte(electricsim::topology::kWireDRIVER_BRAKE_PEDAL_Q8, &out));
    REQUIRE(out == 0xABu);
}

TEST_CASE("WireTruthChassis::mirror_signal: kUint16 round-trip (6901 DRIVER_STEERING_DEG_Q8)",
          "[wire_truth][mirror_signal]") {
    const std::string seg = unique_segment("ms_u16");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // 0x1234 LE: low byte 0x34, high byte 0x12
    const std::uint8_t payload[] = {0x34u, 0x12u};
    REQUIRE(wire->mirror_signal(6901U, payload, 2u));

    std::uint16_t out = 0;
    REQUIRE(creator->read_uint16(electricsim::topology::kWireDRIVER_STEERING_DEG_Q8, &out));
    REQUIRE(out == 0x1234u);
}

TEST_CASE("WireTruthChassis: uint32 accessor round-trips; electricsim-owned cells are not mirrored",
          "[wire_truth][mirror_signal]") {
    const std::string seg = unique_segment("ms_u32");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // There is no ev1sim-PRODUCED uint32 chassis cell: the only uint32 cells
    // (HV bus 4155/4157) are electricsim-produced (topology `driver:`), so they
    // are deliberately EXCLUDED from the producer registry. mirror_signal must
    // therefore REFUSE 4155 — ev1sim never writes a cell electricsim drives.
    const std::uint8_t payload[] = {0xEFu, 0xBEu, 0xADu, 0xDEu};  // 0xDEADBEEF LE
    REQUIRE_FALSE(wire->mirror_signal(4155U, payload, 4u));

    // The write_uint32 accessor itself still round-trips (decode-path coverage).
    REQUIRE(wire->write_uint32(electricsim::topology::kWireHV_BUS_VOLTAGE_MV,
                               0xDEADBEEFu));
    std::uint32_t out = 0;
    REQUIRE(creator->read_uint32(electricsim::topology::kWireHV_BUS_VOLTAGE_MV, &out));
    REQUIRE(out == 0xDEADBEEFu);
}

TEST_CASE("WireTruthChassis::mirror_signal: kUint64 round-trip (4075 CHASSIS_SIM_TIME_NS)",
          "[wire_truth][mirror_signal]") {
    const std::string seg = unique_segment("ms_u64");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // 0x0102030405060708 LE
    const std::uint8_t payload[] = {0x08u, 0x07u, 0x06u, 0x05u,
                                    0x04u, 0x03u, 0x02u, 0x01u};
    REQUIRE(wire->mirror_signal(4075U, payload, 8u));

    std::uint64_t out = 0;
    REQUIRE(creator->read_uint64(electricsim::topology::kWireCHASSIS_SIM_TIME_NS, &out));
    REQUIRE(out == UINT64_C(0x0102030405060708));
}

TEST_CASE("WireTruthChassis::mirror_signal: kFloat32 round-trip (4100 CHASSIS_SPEED_MPS)",
          "[wire_truth][mirror_signal]") {
    const std::string seg = unique_segment("ms_f32");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // Encode 27.5f as LE IEEE 754 bit-pattern.
    const float expected = 27.5f;
    std::uint32_t bits = 0;
    std::memcpy(&bits, &expected, sizeof(bits));
    const std::uint8_t payload[] = {
        static_cast<std::uint8_t>(bits & 0xFFu),
        static_cast<std::uint8_t>((bits >> 8u)  & 0xFFu),
        static_cast<std::uint8_t>((bits >> 16u) & 0xFFu),
        static_cast<std::uint8_t>((bits >> 24u) & 0xFFu),
    };
    REQUIRE(wire->mirror_signal(4100U, payload, 4u));

    float out = 0.0f;
    REQUIRE(creator->read_float32(electricsim::topology::kWireCHASSIS_SPEED_MPS, &out));
    REQUIRE(out == expected);
}

TEST_CASE("WireTruthChassis::mirror_signal: unregistered signal_id returns false",
          "[wire_truth][mirror_signal]") {
    const std::string seg = unique_segment("ms_unreg");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // 9999 is not in the producer registry.
    const std::uint8_t payload[] = {0x01u};
    REQUIRE_FALSE(wire->mirror_signal(9999U, payload, 1u));
}

// ---------------------------------------------------------------------------
// apply_consumer_overlay tests.
// @design 2026-06-15 — consumer overlay batch.
//
// Four representative consumed cells, one per wire type that the consumer set
// uses (bit/byte/float32/uint32).  The creator plays electricsim's role —
// writes the cells onto the wire — then ev1sim's apply_consumer_overlay reads
// them and invokes the matching sinks.
//
//   bit     4000  kWireBULB_FEED_LINE_LBL
//   byte    4082  kWireCHASSIS_HVAC_BLOWER_LEVEL
//   float32 4132  kWireCHASSIS_IPC_TRIP_DISTANCE_M
//   uint32  4192  kWireCHASSIS_AUX_BATTERY_TERMINAL_MV
// ---------------------------------------------------------------------------

TEST_CASE("WireTruthChassis::apply_consumer_overlay: written cells invoke matching sinks",
          "[wire_truth][consumer_overlay]") {
    const std::string seg = unique_segment("co_written");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    // electricsim (LHJB / IPC / BTCM / BPM) writes one cell of each consumed type.
    REQUIRE(creator->write_bit(
        electricsim::topology::kWireBULB_FEED_LINE_LBL, true));
    REQUIRE(creator->write_byte(
        electricsim::topology::kWireCHASSIS_HVAC_BLOWER_LEVEL,
        static_cast<std::uint8_t>(2u)));
    REQUIRE(creator->write_float32(
        electricsim::topology::kWireCHASSIS_IPC_TRIP_DISTANCE_M, 123.5f));
    REQUIRE(creator->write_uint32(
        electricsim::topology::kWireCHASSIS_AUX_BATTERY_TERMINAL_MV, 12650u));

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);
    REQUIRE(wire->attached());

    // Capturing sinks record what was delivered.
    std::unordered_map<std::uint32_t, bool>          bit_map;
    std::unordered_map<std::uint32_t, std::uint8_t>  byte_map;
    std::unordered_map<std::uint32_t, float>         float_map;
    std::unordered_map<std::uint32_t, std::uint32_t> u32_map;

    ev1sim::WireTruthChassis::ConsumerSinks sinks;
    sinks.on_bit    = [&](std::uint32_t id, bool v)          { bit_map[id]   = v; };
    sinks.on_byte   = [&](std::uint32_t id, std::uint8_t v)  { byte_map[id]  = v; };
    sinks.on_float  = [&](std::uint32_t id, float v)         { float_map[id] = v; };
    sinks.on_uint32 = [&](std::uint32_t id, std::uint32_t v) { u32_map[id]   = v; };

    const int n = wire->apply_consumer_overlay(sinks);

    // All four written cells must have fired their respective sinks.
    REQUIRE(bit_map.count(4000U) == 1);
    REQUIRE(bit_map.at(4000U) == true);

    REQUIRE(byte_map.count(4082U) == 1);
    REQUIRE(byte_map.at(4082U) == static_cast<std::uint8_t>(2u));

    REQUIRE(float_map.count(4132U) == 1);
    REQUIRE(float_map.at(4132U) == 123.5f);

    REQUIRE(u32_map.count(4192U) == 1);
    REQUIRE(u32_map.at(4192U) == 12650u);

    // Return count must be >= 4 (the four we wrote; other cells may also be
    // registered consumer cells but were not written so do not count).
    REQUIRE(n >= 4);
    // The count must not exceed the total number of consumer cells (66).
    REQUIRE(n <= 66);
}

TEST_CASE("WireTruthChassis::apply_consumer_overlay: no-write returns 0, no sink fires",
          "[wire_truth][consumer_overlay]") {
    const std::string seg = unique_segment("co_dormant");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);
    // Nothing written — every cell remains at gen==0.

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);
    REQUIRE(wire->attached());

    bool any_fired = false;
    ev1sim::WireTruthChassis::ConsumerSinks sinks;
    sinks.on_bit    = [&](std::uint32_t, bool)          { any_fired = true; };
    sinks.on_byte   = [&](std::uint32_t, std::uint8_t)  { any_fired = true; };
    sinks.on_float  = [&](std::uint32_t, float)         { any_fired = true; };
    sinks.on_uint32 = [&](std::uint32_t, std::uint32_t) { any_fired = true; };

    const int n = wire->apply_consumer_overlay(sinks);

    REQUIRE(n == 0);
    REQUIRE_FALSE(any_fired);
}

// ---------------------------------------------------------------------------
// End-to-end horn round-trip: a value written on the wire the way electricsim's
// LHJB writes it (Batch B, write_bit on kWireHORN_DRIVE_LINE_*) flows through
// ev1sim's REAL connector consumer path — the same ConsumerSinks the connector
// installs in Tick() (on_bit -> DebugInjectDelta) — out to GetHornLowCmd/High.
// This composes the full ev1sim side of the cross-repo round-trip against the
// real generated topology + hash, without the ring/Chrono plumbing.
// @design 2026-06-15 — Batch B horn end-to-end verification.
// ---------------------------------------------------------------------------
TEST_CASE("Horn round-trip end-to-end: wire -> connector overlay -> getter",
          "[wire_truth][e2e]") {
    // electricsim side (LHJB): create the segment + write both horn lines.
    const std::string seg = unique_segment("e2e_horn");
    auto lhjb = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(lhjb != nullptr);
    REQUIRE(lhjb->write_bit(electricsim::topology::kWireHORN_DRIVE_LINE_LOW, true));
    REQUIRE(lhjb->write_bit(electricsim::topology::kWireHORN_DRIVE_LINE_HIGH, true));

    // ev1sim side: attach the wire + run the SAME sinks the connector installs
    // in Tick() (on_bit -> DebugInjectDelta, etc.), then read the getters.
    ExternalSimConnector c;
    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    ev1sim::WireTruthChassis::ConsumerSinks sinks;
    sinks.on_bit    = [&c](std::uint32_t id, bool v)          { c.DebugInjectDelta(id, v); };
    sinks.on_byte   = [&c](std::uint32_t id, std::uint8_t v)  { c.DebugInjectU8(id, v); };
    sinks.on_float  = [&c](std::uint32_t id, float v)         { c.DebugInjectFloat(id, v); };
    sinks.on_uint32 = [&c](std::uint32_t id, std::uint32_t v) { c.DebugInjectU32(id, v); };

    REQUIRE(wire->apply_consumer_overlay(sinks) >= 2);  // both horn lines applied
    CHECK(c.GetHornLowCmd());
    CHECK(c.GetHornHighCmd());

    // Producer drops the low tone; ev1sim tracks it on the next overlay pass.
    REQUIRE(lhjb->write_bit(electricsim::topology::kWireHORN_DRIVE_LINE_LOW, false));
    wire->apply_consumer_overlay(sinks);
    CHECK_FALSE(c.GetHornLowCmd());
    CHECK(c.GetHornHighCmd());
}
