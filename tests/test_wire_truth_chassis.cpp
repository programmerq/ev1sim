// Proof-of-life for the wire-truth migration: ev1sim attaches the external sim's
// shared WireTable and round-trips the horn drive-line cells (chassis
// 4020/4021 -> HORN_DRIVE_LINE_LOW/HIGH) through WireTruthChassis.
//
// The test plays BOTH cross-repo roles in-process: it CREATES the shared table
// the way an external sim driver does (env_open's creator path: declare_all +
// the canonical topology hash), then ATTACHES through ev1sim's WireTruthChassis
// and asserts the values match. This proves, against the REAL generated
// topology (not a stand-in): the topology-hash attach gate, the WireId
// resolution (kWireHORN_DRIVE_LINE_*), the bit type, the written()/freshness
// fallback contract, and both read (consumer) and write (producer) directions.
//
// Only compiled when EV1SIM_HAVE_WIRE_TRUTH (the external sim substrate is
// checked out); see CMakeLists.txt.
//
// @design 2026-06-15 — wire-truth migration kickoff.

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include <unistd.h>  // getpid, close, dup, dup2

#include "WireTruthChassis.h"
#include "ExternalSimConnector.h"  // end-to-end: wire -> overlay -> getter

// external sim substrate (creator side) — same headers env_open.cpp uses.
#include "topology/topology_generated.h"
#include "wire_table.hpp"
#include "gm8192/gm8192_frame.h"  // gm8192_encode (build a $41 frame for the snoop test)
#include "uart/uart_tx.hpp"       // electricsim::io::UartTx (serialise onto a TX cell)
#include "net_host/conductor_publisher.hpp"  // the publish edge for conductor cells

namespace {

using electricsim::io::ConductorId;
using electricsim::io::WireId;
using electricsim::io::WireTable;
using electricsim::io::WireTableOptions;

// A unique-per-run segment name so a killed test never collides with a leaked
// segment (mirrors the external sim's $ELECTRICSIM_BUS_NAME convention). The creator
// WireTable unlinks the segment on destruction.
std::string unique_segment(const char* tag) {
    return std::string("ev1sim_wt_") + tag + "_" +
           std::to_string(static_cast<long>(::getpid()));
}

// Create the shared table as an external sim driver would: declare the full
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

// Publish a conductor cell the way the external sim's solver does.
//
// A conductor's energisation is an OUTPUT of the external sim's provenance
// computation, so the substrate deliberately gives conductor cells no write_bit()
// overload: no peer can assert that its own feed is hot. These tests stand in for
// that solver — they seed a lamp feed or horn line and then check ev1sim reads it
// — so they publish through the same edge the solver uses, net_host::
// ConductorPublisher, rather than casting the type away.
//
// Casting would have been one line shorter and would have quietly made the test a
// place where conductor writes are legal. Then the day the publish edge changes,
// these tests keep passing against an API nobody uses.
bool publish_conductor(WireTable& table, ConductorId id, bool energised) {
    electricsim::net_host::ConductorPublisher publisher(table);
    return publisher.publish(id, energised);
}

// The millivolt form of the same edge, for conductor cells carrying a measured
// potential rather than a bare energised/not bit — the aux battery terminal is the
// first of those. Same reasoning as above: the test stands in for the solver, so it
// seeds the cell through the publisher rather than casting the conductor type away.
bool publish_conductor_mv(WireTable& table, ConductorId id, std::uint32_t millivolts) {
    electricsim::net_host::ConductorPublisher publisher(table);
    return publisher.publish_mv(id, millivolts);
}

// Numeric id of a conductor cell, for the read accessors (which take a runtime
// id). Reads are the legitimate direction for ev1sim — see ReadOnlyWireId in
// src/WireTruthChassis.cpp for the same conversion on the production side.
constexpr WireId conductor_wire_id(ConductorId id) {
    return static_cast<WireId>(id);
}

}  // namespace

TEST_CASE("WireTruthChassis: horn drive lines round-trip external sim -> ev1sim",
          "[wire_truth]") {
    const std::string seg = unique_segment("horn");
    auto producer = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(producer != nullptr);  // creator path must succeed

    // external sim (LHJB) writes the low-tone horn drive line on the wire.
    REQUIRE(publish_conductor(*producer, electricsim::topology::kWireHORN_DRIVE_LINE_LOW, true));

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);
    REQUIRE(wire->attached());

    // Consumer side: low was written -> live value; high never written ->
    // nullopt, so the connector keeps its legacy-ring fallback (kHold-safe).
    REQUIRE(wire->horn_low_drive() == std::optional<bool>(true));
    REQUIRE(wire->horn_high_drive() == std::nullopt);

    // Producer flips low off and writes high on; the consumer tracks it.
    REQUIRE(publish_conductor(*producer, electricsim::topology::kWireHORN_DRIVE_LINE_LOW, false));
    REQUIRE(publish_conductor(*producer, electricsim::topology::kWireHORN_DRIVE_LINE_HIGH, true));
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
    // BEFORE the external sim's producer has moved onto the wire.
    REQUIRE(wire->horn_low_drive() == std::nullopt);
    REQUIRE(wire->horn_high_drive() == std::nullopt);
    REQUIRE(wire->read_bit(conductor_wire_id(electricsim::topology::kWireHORN_DRIVE_LINE_LOW)) ==
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
    //
    // This case used to demonstrate the producer direction on HORN_DRIVE_LINE_HIGH,
    // which the migration reclassified as a CONDUCTOR — a cell whose energisation
    // the external sim's solver derives and no peer asserts. ev1sim cannot write it
    // any more, and should never have: the horn line is something ev1sim READS to
    // decide whether to sound the horn. The cell here is PANEL_AJAR_HOOD, one of the
    // 85 cells ev1sim genuinely produces (a hood switch is a physical input ev1sim
    // owns), so the test now demonstrates the producer direction on a cell where
    // ev1sim actually has one.
    REQUIRE(wire->write_bit(electricsim::topology::kWirePANEL_AJAR_HOOD, true));
    bool seen = false;
    REQUIRE(creator->read_bit(
        electricsim::topology::kWirePANEL_AJAR_HOOD, &seen));
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

TEST_CASE("WireTruthChassis: uint32 accessor round-trips; externally-owned cells are not mirrored",
          "[wire_truth][mirror_signal]") {
    const std::string seg = unique_segment("ms_u32");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // There is no ev1sim-PRODUCED uint32 chassis cell: the only uint32 cells
    // (HV bus 4155/4157) are externally-produced (topology `driver:`), so they
    // are deliberately EXCLUDED from the producer registry. mirror_signal must
    // therefore REFUSE 4155 — ev1sim never writes a cell external sim drives.
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
// uses (bit/byte/float32/uint32).  The creator plays the external sim's role —
// writes the cells onto the wire — then ev1sim's apply_consumer_overlay reads
// them and invokes the matching sinks.
//
//   bit     4000  kWireBULB_FEED_LINE_LBL               (conductor — published)
//   byte    4082  kWireCHASSIS_HVAC_BLOWER_LEVEL
//   float32 4132  kWireCHASSIS_IPC_TRIP_DISTANCE_M
//   uint32  4192  kWireCHASSIS_AUX_BATTERY_TERMINAL_MV  (conductor — published_mv)
//
// The two conductors are seeded through ConductorPublisher, not write_*: their
// values are solver outputs, so the substrate gives them no write overload.
// ---------------------------------------------------------------------------

TEST_CASE("WireTruthChassis::apply_consumer_overlay: written cells invoke matching sinks",
          "[wire_truth][consumer_overlay]") {
    const std::string seg = unique_segment("co_written");
    auto creator = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(creator != nullptr);

    // external sim (LHJB / IPC / BTCM / BPM) writes one cell of each consumed type.
    REQUIRE(publish_conductor(*creator, electricsim::topology::kWireBULB_FEED_LINE_LBL, true));
    REQUIRE(creator->write_byte(
        electricsim::topology::kWireCHASSIS_HVAC_BLOWER_LEVEL,
        static_cast<std::uint8_t>(2u)));
    REQUIRE(creator->write_float32(
        electricsim::topology::kWireCHASSIS_IPC_TRIP_DISTANCE_M, 123.5f));
    REQUIRE(publish_conductor_mv(
        *creator, electricsim::topology::kWireCHASSIS_AUX_BATTERY_TERMINAL_MV, 12650u));
    // PIM commanded throttle (4073) — the cell the wire-truth migration dropped from
    // the consumer set (restored 2026-07-04); pin it here so it can't fall out
    // again silently (the VAT safety failsafe assertions read this via stats).
    REQUIRE(creator->write_byte(
        electricsim::topology::kWireCHASSIS_THROTTLE_CMD_Q8,
        static_cast<std::uint8_t>(128u)));

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

    REQUIRE(byte_map.count(4073U) == 1);
    REQUIRE(byte_map.at(4073U) == static_cast<std::uint8_t>(128u));

    // Return count must be >= 5 (the five we wrote; other cells may also be
    // registered consumer cells but were not written so do not count).
    REQUIRE(n >= 5);
    // The count must not exceed the total number of consumer cells (67).
    REQUIRE(n <= 67);
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
// End-to-end horn round-trip: a value written on the wire the way the external sim's
// LHJB writes it (Batch B, write_bit on kWireHORN_DRIVE_LINE_*) flows through
// ev1sim's REAL connector consumer path — the same ConsumerSinks the connector
// installs in Tick() (on_bit -> DebugInjectDelta) — out to GetHornLowCmd/High.
// This composes the full ev1sim side of the cross-repo round-trip against the
// real generated topology + hash, without the ring/Chrono plumbing.
// @design 2026-06-15 — Batch B horn end-to-end verification.
// ---------------------------------------------------------------------------
TEST_CASE("Horn round-trip end-to-end: wire -> connector overlay -> getter",
          "[wire_truth][e2e]") {
    // external sim side (LHJB): create the segment + write both horn lines.
    const std::string seg = unique_segment("e2e_horn");
    auto lhjb = create_fleet_table(seg, electricsim::topology::kTopologyHash);
    REQUIRE(lhjb != nullptr);
    REQUIRE(publish_conductor(*lhjb, electricsim::topology::kWireHORN_DRIVE_LINE_LOW, true));
    REQUIRE(publish_conductor(*lhjb, electricsim::topology::kWireHORN_DRIVE_LINE_HIGH, true));

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
    REQUIRE(publish_conductor(*lhjb, electricsim::topology::kWireHORN_DRIVE_LINE_LOW, false));
    wire->apply_consumer_overlay(sinks);
    CHECK_FALSE(c.GetHornLowCmd());
    CHECK(c.GetHornHighCmd());
}

// ---------------------------------------------------------------------------
// Batch A/J/K/L live-render verification: with external sim producing these
// cells on the wire (Batch A bulbs, J wiper/washer, K HVAC/defrost, L RSA),
// ev1sim must render them through the connector's REAL overlay sinks out to the
// getters. Exercises both render types and, critically, the byte-coerced-bool
// cells (defrost 4083, shift 4088) that the connector serves from DebugInjectU8
// even though their wire is `bit` — these drop unless on_bit double-dispatches.
// @design 2026-06-16 — per-batch verification (A/J/K/L).
// ---------------------------------------------------------------------------
TEST_CASE("Batches A/J/K/L render through the connector overlay (wire -> getter)",
          "[wire_truth][e2e]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("e2e_batches");
    auto prod = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(prod != nullptr);

    // external sim producers write one representative cell per landed batch.
    REQUIRE(publish_conductor(*prod, topo::kWireBULB_FEED_LINE_LBL, true)); // A bit
    REQUIRE(prod->write_byte(topo::kWireCHASSIS_WIPER_MOTOR_COMMAND,  3u));   // J byte
    REQUIRE(prod->write_bit (topo::kWireCHASSIS_WASHER_PUMP_COMMAND,  true)); // J bit
    REQUIRE(prod->write_byte(topo::kWireCHASSIS_HVAC_BLOWER_LEVEL,    4u));   // K byte
    REQUIRE(prod->write_bit (topo::kWireCHASSIS_DEFROST_GRID_ACTIVE,  true)); // K bit (U8-served)
    REQUIRE(prod->write_byte(topo::kWireCHASSIS_DOOR_LOCK_CMD_DRIVER, 1u));   // L byte
    REQUIRE(prod->write_bit (topo::kWireCHASSIS_RSA_SHIFT_BLOCKED,    true)); // L bit (U8-served)

    ExternalSimConnector c;
    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // The SAME ConsumerSinks the connector installs in Tick() (on_bit routes to
    // both DebugInjectDelta and DebugInjectU8).
    ev1sim::WireTruthChassis::ConsumerSinks sinks;
    sinks.on_bit    = [&c](std::uint32_t id, bool v)          { c.DebugInjectDelta(id, v); c.DebugInjectU8(id, v ? 1u : 0u); };
    sinks.on_byte   = [&c](std::uint32_t id, std::uint8_t v)  { c.DebugInjectU8(id, v); };
    sinks.on_float  = [&c](std::uint32_t id, float v)         { c.DebugInjectFloat(id, v); };
    sinks.on_uint32 = [&c](std::uint32_t id, std::uint32_t v) { c.DebugInjectU32(id, v); };
    REQUIRE(wire->apply_consumer_overlay(sinks) >= 7);

    CHECK(c.GetBulbCmd(LightID::LBL));        // Batch A
    CHECK(c.GetWiperMotorCommand() == 3u);    // Batch J
    CHECK(c.GetWasherPumpCommand());          // Batch J
    CHECK(c.GetHvacBlowerLevel() == 4u);      // Batch K
    CHECK(c.GetDefrostGridActive());          // Batch K  (regresses without the on_bit fix)
    CHECK(c.GetDoorLockCmd(0) == 1u);         // Batch L
    CHECK(c.GetRsaShiftBlocked());            // Batch L  (regresses without the on_bit fix)
}

// ---------------------------------------------------------------------------
// Batch M live-render verification: IPC telltales (bit, byte-coerced-bool in the
// connector — served by DebugInjectU8) + trip distance (float32). These are the
// exact cells the on_bit double-dispatch fix targets; the bit telltales below
// would NOT render without it. Confirms Batch M renders end-to-end off the wire.
// @design 2026-06-16 — per-batch verification (Batch M, IPC telltales).
// ---------------------------------------------------------------------------
TEST_CASE("Batch M: IPC telltales + trip render through the connector overlay",
          "[wire_truth][e2e]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("e2e_ipc");
    auto ipc = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(ipc != nullptr);

    REQUIRE(publish_conductor(*ipc, topo::kWireCHASSIS_IPC_SEATBELT_TELLTALE_DRIVER, true));
    REQUIRE(publish_conductor(*ipc, topo::kWireCHASSIS_IPC_BRAKE_TELLTALE, true));
    REQUIRE(publish_conductor(*ipc, topo::kWireCHASSIS_IPC_ANTILOCK_TELLTALE, true));
    REQUIRE(publish_conductor(*ipc, topo::kWireCHASSIS_IPC_SERVICE_NOW_TELLTALE, true));
    REQUIRE(ipc->write_float32(topo::kWireCHASSIS_IPC_TRIP_DISTANCE_M,          12.5f));

    ExternalSimConnector c;
    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    ev1sim::WireTruthChassis::ConsumerSinks sinks;
    sinks.on_bit    = [&c](std::uint32_t id, bool v)          { c.DebugInjectDelta(id, v); c.DebugInjectU8(id, v ? 1u : 0u); };
    sinks.on_byte   = [&c](std::uint32_t id, std::uint8_t v)  { c.DebugInjectU8(id, v); };
    sinks.on_float  = [&c](std::uint32_t id, float v)         { c.DebugInjectFloat(id, v); };
    sinks.on_uint32 = [&c](std::uint32_t id, std::uint32_t v) { c.DebugInjectU32(id, v); };
    REQUIRE(wire->apply_consumer_overlay(sinks) >= 5);

    CHECK(c.GetIpcSeatbeltTelltaleDriver());  // bit telltale (U8-served; needs on_bit fix)
    CHECK(c.GetIpcBrakeTelltale());
    CHECK(c.GetIpcAntilockTelltale());
    CHECK(c.GetIpcServiceNowTelltale());
    CHECK(c.GetIpcTripDistanceM() == 12.5f);  // float32 (exact)
}

// ---------------------------------------------------------------------------
// Coupler (4060) wire mirror — the one ev1sim-produced cell that was ring-only
// until Phase 5 prep. Multi-producer (ev1sim UI + charger peer), last-writer-
// wins. Confirms ev1sim now mirrors its coupler-present onto the wire so the
// consumers can read it wire-authoritatively (and ev1sim can later drop the ring).
// @design 2026-06-16 — Phase 5 prep.
// ---------------------------------------------------------------------------
TEST_CASE("Coupler 4060 mirrors to the wire (Phase 5 prep)", "[wire_truth][mirror_signal]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("ms_coupler");
    auto creator = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(creator != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    const std::uint8_t on[]  = {0x01u};
    const std::uint8_t off[] = {0x00u};
    REQUIRE(wire->mirror_signal(4060U, on, 1u));   // coupler mated
    bool out = false;
    REQUIRE(creator->read_bit(topo::kWireCHARGER_COUPLER_PRESENT, &out));
    REQUIRE(out == true);
    REQUIRE(wire->mirror_signal(4060U, off, 1u));  // unmated
    REQUIRE(creator->read_bit(topo::kWireCHARGER_COUPLER_PRESENT, &out));
    REQUIRE(out == false);
}

// ---------------------------------------------------------------------------
// ECU-bus semantic helpers (wire-truth Phase 4). These cells used to arrive as
// ring DeltaBatches on the main harness segment SharedMemoryTransport; the
// connector now reads them off the shared WireTable through these helpers.
// The creator plays the producing ECU's role (RSA / APM / AD / BTCM).
// @design 2026-06-17 — Phase 4 ECU-bus migration.
// ---------------------------------------------------------------------------

TEST_CASE("WireTruthChassis::rsa_run_active follows RSA_RUN1_OUT", "[wire_truth][ecu]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("ecu_rsa");
    auto rsa = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(rsa != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // Unwritten -> nullopt (consumer keeps its "no run-mode" default).
    REQUIRE(wire->rsa_run_active() == std::nullopt);

    REQUIRE(rsa->write_bit(topo::kWireRSA_RUN1_OUT, true));
    REQUIRE(wire->rsa_run_active() == std::optional<bool>(true));
    REQUIRE(rsa->write_bit(topo::kWireRSA_RUN1_OUT, false));
    REQUIRE(wire->rsa_run_active() == std::optional<bool>(false));
}

TEST_CASE("WireTruthChassis::ad_main_contactor_closed follows the AD's own AD_MAIN_CONTACTOR",
          "[wire_truth][ecu]") {
    // Re-pointed 2026-07-04: was wrongly wired to APM_HV_CONTACTOR_CLOSED (the
    // APM's downstream echo, only written when an APM is in the fleet), which
    // read a permanent nullopt/0 in an APM-less fleet (VAT
    // safety_ad_precharge_timeout is [hv_bus, ad] — no APM) even though the AD
    // itself had genuinely closed the contactor. The getter must follow the
    // AD's OWN commanded output, matching its sibling getters
    // (ad_precharge_relay, ad_state_enum) below.
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("ecu_ad_main");
    auto ad = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(ad != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    REQUIRE(wire->ad_main_contactor_closed() == std::nullopt);  // unwritten
    REQUIRE(ad->write_bit(topo::kWireAD_MAIN_CONTACTOR, true));
    REQUIRE(wire->ad_main_contactor_closed() == std::optional<bool>(true));

    // The APM echo cell must NOT be what the getter follows — even with the
    // echo written to a DIFFERENT value, the getter must keep reporting the
    // AD's own cell.
    REQUIRE(ad->write_bit(topo::kWireAPM_HV_CONTACTOR_CLOSED, false));
    REQUIRE(wire->ad_main_contactor_closed() == std::optional<bool>(true));
}

TEST_CASE("WireTruthChassis::ad_state_enum reconstructs the AD line code",
          "[wire_truth][ecu]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("ecu_ad");
    auto ad = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(ad != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // All-or-nothing: with no lines written, decode is nullopt.
    REQUIRE(wire->ad_state_enum() == std::nullopt);

    // Helper to write the four contributing cells.
    auto set_lines = [&](bool a, bool b, bool c, bool power) {
        REQUIRE(publish_conductor(*ad, topo::kWireAD_STATE_A, a));
        REQUIRE(publish_conductor(*ad, topo::kWireAD_STATE_B, b));
        REQUIRE(publish_conductor(*ad, topo::kWireAD_STATE_C, c));
        REQUIRE(publish_conductor(*ad, topo::kWireAD_POWER_SUPPLY, power));
    };

    // OK = (0,0,0) power -> 0 (AD_STATE_OK).
    set_lines(false, false, false, true);
    REQUIRE(wire->ad_state_enum() == std::optional<std::uint32_t>(0u));

    // CAPACITOR_PRECHARGE = (1,0,0) -> 6 (matches the value the old
    // kSigAdStateEnum carried, per the external sim's AD-state header).
    set_lines(true, false, false, true);
    REQUIRE(wire->ad_state_enum() == std::optional<std::uint32_t>(6u));

    // CAPACITOR_PRECHARGE_FAIL = (0,0,1) -> 7.
    set_lines(false, false, true, true);
    REQUIRE(wire->ad_state_enum() == std::optional<std::uint32_t>(7u));

    // Power lost dominates the lines -> 8 (AD_STATE_POWER_LOST).
    set_lines(true, true, true, false);
    REQUIRE(wire->ad_state_enum() == std::optional<std::uint32_t>(8u));
}

TEST_CASE("WireTruthChassis::btcm_tx_total_bits tracks the GM8192 TX bit-stream",
          "[wire_truth][ecu]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("ecu_btcm");
    auto btcm = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(btcm != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // Nothing appended yet -> nullopt (BTCM silent; btcm_alive stays false).
    REQUIRE(wire->btcm_tx_total_bits() == std::nullopt);

    // BTCM transmits a byte's worth of frame bits; the total advances.
    REQUIRE(btcm->append_bit(topo::kWireGM8192_BTCM_TX, true));
    auto t1 = wire->btcm_tx_total_bits();
    REQUIRE(t1.has_value());
    REQUIRE(*t1 >= 1u);

    const bool more[] = {false, true, true, false, true};
    REQUIRE(btcm->append_bits(topo::kWireGM8192_BTCM_TX, more, 5u));
    auto t2 = wire->btcm_tx_total_bits();
    REQUIRE(t2.has_value());
    REQUIRE(*t2 > *t1);  // advanced => "BTCM is transmitting" liveness proxy
}

TEST_CASE("WireTruthChassis::snoop decodes vehicle speed from the PIM $41 frame",
          "[wire_truth][snoop]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("snoop_pim");
    auto pim = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(pim != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // Nothing transmitted yet -> no decoded speed.
    wire->snoop_step(0.0);
    REQUIRE(wire->pim_vehicle_speed_kph() == std::nullopt);

    // Serialise a real $41 PCM Data Response with vehicle speed = 50 km/h onto
    // the PIM TX bit-stream cell, exactly as the PIM controller's UartTx would.
    // payload[4] (wire byte 6) = vehicle speed (1 km/h/count); see the external sim's PIM UART-frame definition.
    std::uint8_t payload[7] = {0u, 0u, 0u, 0u, 50u, 0u, 0u};
    std::uint8_t frame[GM8192_MAX_FRAME_LEN] = {0u};
    std::size_t  frame_len = 0;
    REQUIRE(gm8192_encode(0x41u, payload, 7u, frame, sizeof(frame), &frame_len)
            == GM8192_OK);

    constexpr std::uint64_t kBitPeriodNs = 122070;
    electricsim::io::UartTx tx(pim.get(), topo::kWireGM8192_PIM_TX, kBitPeriodNs);
    tx.enqueue(frame, frame_len);

    // Pump TX + snoop in lockstep, one bit period per step, until the frame
    // (frame_len bytes × 10 UART bits) has serialised + decoded, plus slack.
    std::uint64_t now = 1000u;
    const int steps = static_cast<int>(frame_len) * 10 + 20;
    for (int i = 0; i < steps; ++i) {
        now += kBitPeriodNs;
        tx.tick(now);
        wire->snoop_step(static_cast<double>(now) / 1.0e9);
    }
    REQUIRE(wire->pim_vehicle_speed_kph() == std::optional<std::uint8_t>(50u));
}

TEST_CASE("WireTruthChassis::pim_cruise_active/setpoint follow the PIM cruise cells",
          "[wire_truth][ecu]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("ecu_pim");
    auto pim = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(pim != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // Unwritten -> nullopt (consumer keeps its "no cruise data" default).
    REQUIRE(wire->pim_cruise_active() == std::nullopt);
    REQUIRE(wire->pim_cruise_setpoint_mps() == std::nullopt);

    REQUIRE(pim->write_bit(topo::kWirePIM_CRUISE_ACTIVE, true));
    REQUIRE(wire->pim_cruise_active() == std::optional<bool>(true));

    // 25.0 m/s is exactly representable, so the round-trip is bit-exact.
    REQUIRE(pim->write_float32(topo::kWirePIM_CRUISE_SETPOINT_MPS, 25.0f));
    REQUIRE(wire->pim_cruise_setpoint_mps() == std::optional<float>(25.0f));
}

TEST_CASE("WireTruthChassis::ad_precharge_relay follows AD_PRECHARGE_RELAY",
          "[wire_truth][ecu]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("ecu_ad_pre");
    auto ad = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(ad != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    REQUIRE(wire->ad_precharge_relay() == std::nullopt);  // unwritten
    REQUIRE(ad->write_bit(topo::kWireAD_PRECHARGE_RELAY, true));
    REQUIRE(wire->ad_precharge_relay() == std::optional<bool>(true));
    REQUIRE(ad->write_bit(topo::kWireAD_PRECHARGE_RELAY, false));
    REQUIRE(wire->ad_precharge_relay() == std::optional<bool>(false));
}

// ---------------------------------------------------------------------------
// Door-lock loop end-to-end, both directions, across the seam that broke.
//
// The consumer overlay resolves CHASSIS_RHJB_DLM_* to the chassis IDs the
// contract adopted, and the connector's inbound dispatch answers on the IDs it
// was written against.  Those two lists disagreed, so every leg the RHJB wrote
// was delivered under an ID nothing handled and vanished — while the connector's
// own DebugInjectDelta tests kept passing, because they inject on the dispatch
// side and never touch the overlay.
//
// The INBOUND half here refuses to be satisfiable that way: it writes the wire
// cell BY NAME, routes it through the overlay, and asserts on the public getter,
// naming no numeric chassis ID anywhere along the path — so it can only pass
// when the overlay and the dispatch agree.  It drives the four legs to four
// DIFFERENT values, so a swap of any pair is visible; driving both LOCK legs
// together would leave an LH<->RH cross-wire invisible.
//
// The OUTBOUND half is weaker by construction: mirror_signal takes a signal id,
// so those lines do name 4170-4173.  It pins signal-id -> cell routing only.
// That the connector's own Tick actually emits those four is pinned separately,
// in test_wire_truth_attach_outcome.cpp.
// ---------------------------------------------------------------------------
TEST_CASE("Door-lock loop end-to-end: RHJB legs in, switch contacts out",
          "[wire_truth][e2e]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("e2e_door_lock");
    auto rhjb = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(rhjb != nullptr);

    ExternalSimConnector c;
    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // The same sink set the connector installs in Tick(), including the on_bit
    // double-dispatch: some byte-coerced-bool cells are served from
    // DebugInjectU8 even though their wire is a bit, and a single-dispatch
    // stand-in silently drops those.
    ev1sim::WireTruthChassis::ConsumerSinks sinks;
    sinks.on_bit    = [&c](std::uint32_t id, bool v) {
        c.DebugInjectDelta(id, v);
        c.DebugInjectU8(id, v ? 1u : 0u);
    };
    sinks.on_byte   = [&c](std::uint32_t id, std::uint8_t v)  { c.DebugInjectU8(id, v); };
    sinks.on_float  = [&c](std::uint32_t id, float v)         { c.DebugInjectFloat(id, v); };
    sinks.on_uint32 = [&c](std::uint32_t id, std::uint32_t v) { c.DebugInjectU32(id, v); };
    sinks.on_uint16 = [&c](std::uint32_t id, std::uint16_t v) { c.DebugInjectU16(id, v); };

    // Nothing on the wire yet: the connector must still report never-received,
    // so the assertions below cannot be satisfied by a default that happens to
    // match.
    for (int leg = 0; leg < 4; ++leg)
        REQUIRE_FALSE(c.HasReceivedDoorLockMotorDrive(leg));

    // --- Inbound: the RHJB's lock module drives both LOCK legs. ---
    // The legs are conductor cells (their energisation is the solver's output),
    // so this stands in for the solver via the same publish edge it uses.
    // Four DIFFERENT values, so every leg is distinguishable from every other.
    // A physical DLM never drives this combination; that is the point — the
    // question here is whether each wire lands on the leg it names.
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_LH_LOCK,   true));
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_LH_UNLOCK, false));
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_RH_LOCK,   false));
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_RH_UNLOCK, true));
    REQUIRE(wire->apply_consumer_overlay(sinks) >= 4);

    CHECK(c.GetDoorLockMotorDrive(0));         // LH lock
    CHECK_FALSE(c.GetDoorLockMotorDrive(1));   // LH unlock
    CHECK_FALSE(c.GetDoorLockMotorDrive(2));   // RH lock
    CHECK(c.GetDoorLockMotorDrive(3));         // RH unlock
    for (int leg = 0; leg < 4; ++leg)
        CHECK(c.HasReceivedDoorLockMotorDrive(leg));

    // Invert all four: a mapping that happened to satisfy the pattern above by
    // luck cannot satisfy its complement too.
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_LH_LOCK,   false));
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_LH_UNLOCK, true));
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_RH_LOCK,   true));
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_RH_UNLOCK, false));
    wire->apply_consumer_overlay(sinks);
    CHECK_FALSE(c.GetDoorLockMotorDrive(0));
    CHECK(c.GetDoorLockMotorDrive(1));
    CHECK(c.GetDoorLockMotorDrive(2));
    CHECK_FALSE(c.GetDoorLockMotorDrive(3));

    // A real lock pulse, then its release, which is what the plant consumes.
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_LH_UNLOCK, false));
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_LH_LOCK,   true));
    wire->apply_consumer_overlay(sinks);
    CHECK(c.GetDoorLockMotorDrive(0));
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_LH_LOCK, false));
    REQUIRE(publish_conductor(*rhjb, topo::kWireCHASSIS_RHJB_DLM_RH_LOCK, false));
    wire->apply_consumer_overlay(sinks);
    CHECK_FALSE(c.GetDoorLockMotorDrive(0));
    CHECK_FALSE(c.GetDoorLockMotorDrive(2));

    // --- Outbound: the driver's rocker reaches the RHJB's own input cells. ---
    // (The switch contacts are ordinary wire cells — ev1sim is their producer.)
    bool sw = true;

    const std::uint8_t closed[] = {0x01u};
    const std::uint8_t open[]   = {0x00u};
    REQUIRE(wire->mirror_signal(4170U, closed, 1u));   // LH LOCK contact closed
    REQUIRE(rhjb->read_bit(topo::kWireDOOR_LOCK_SW_LH_LOCK_OUT, &sw));
    CHECK(sw);
    REQUIRE(wire->mirror_signal(4170U, open, 1u));     // released — the pulse edge
    REQUIRE(rhjb->read_bit(topo::kWireDOOR_LOCK_SW_LH_LOCK_OUT, &sw));
    CHECK_FALSE(sw);

    // The other three contacts land on their own cells, not on the LH LOCK one.
    REQUIRE(wire->mirror_signal(4171U, closed, 1u));
    REQUIRE(wire->mirror_signal(4172U, closed, 1u));
    REQUIRE(wire->mirror_signal(4173U, closed, 1u));
    REQUIRE(rhjb->read_bit(topo::kWireDOOR_LOCK_SW_LH_UNLOCK_OUT, &sw));
    CHECK(sw);
    REQUIRE(rhjb->read_bit(topo::kWireDOOR_LOCK_SW_RH_LOCK_OUT, &sw));
    CHECK(sw);
    REQUIRE(rhjb->read_bit(topo::kWireDOOR_LOCK_SW_RH_UNLOCK_OUT, &sw));
    CHECK(sw);
    REQUIRE(rhjb->read_bit(topo::kWireDOOR_LOCK_SW_LH_LOCK_OUT, &sw));
    CHECK_FALSE(sw);   // still open — no cross-talk from the other three
}

// ---------------------------------------------------------------------------
// RefuseConductorWrite — the runtime half of the conductor discipline
// (src/WireTruthChassis.cpp). Redirects stderr to a temp file so the test can
// count the diagnostics the refusal path prints, not just its return value.
// @design 2026-08-07 — adversarial-review finding: the refusal's dedup key
// used only accessor[0], and every write_* accessor name starts with 'w', so
// a second violation via a DIFFERENT accessor on the SAME cell was silently
// swallowed instead of getting its own report. This pins the fix.
// ---------------------------------------------------------------------------
namespace {

// RAII: redirect stderr to a temp file for the scope's lifetime, restoring
// the original stream on destruction (including on a REQUIRE failure, which
// Catch2 raises as an exception — the destructor still runs during unwind).
class StderrCapture {
 public:
    StderrCapture() {
        std::snprintf(path_, sizeof(path_), "/tmp/ev1sim_stderr_capture_%d_XXXXXX",
                      static_cast<int>(::getpid()));
        int fd = ::mkstemp(path_);
        REQUIRE(fd >= 0);
        ::close(fd);
        std::fflush(stderr);
        saved_ = ::dup(fileno(stderr));
        REQUIRE(saved_ >= 0);
        REQUIRE(std::freopen(path_, "w", stderr) != nullptr);
    }
    ~StderrCapture() {
        std::fflush(stderr);
        ::dup2(saved_, fileno(stderr));
        ::close(saved_);
        std::remove(path_);
    }
    StderrCapture(const StderrCapture&) = delete;
    StderrCapture& operator=(const StderrCapture&) = delete;

    // Read back everything written so far (flushes first).
    std::string contents() const {
        std::fflush(stderr);
        std::string out;
        std::FILE* f = std::fopen(path_, "r");
        if (f == nullptr) return out;
        char buf[4096];
        std::size_t n;
        while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
            out.append(buf, n);
        }
        std::fclose(f);
        return out;
    }

 private:
    char path_[64]{};
    int saved_{-1};
};

// Count non-overlapping occurrences of `needle` in `haystack`.
std::size_t count_occurrences(const std::string& haystack, const std::string& needle) {
    std::size_t count = 0;
    std::size_t pos = 0;
    while ((pos = haystack.find(needle, pos)) != std::string::npos) {
        ++count;
        pos += needle.size();
    }
    return count;
}

}  // namespace

TEST_CASE("RefuseConductorWrite: a write via a DIFFERENT accessor on the same "
          "conductor cell gets its own report, not a swallowed duplicate",
          "[wire_truth][conductor_discipline]") {
    namespace topo = electricsim::topology;
    const std::string seg = unique_segment("refuse_multi_accessor");
    auto producer = create_fleet_table(seg, topo::kTopologyHash);
    REQUIRE(producer != nullptr);

    auto wire = ev1sim::WireTruthChassis::Attach(seg);
    REQUIRE(wire != nullptr);

    // HORN_DRIVE_LINE_LOW is a `class: conductor` cell (see the horn round-trip
    // test above) — ev1sim has no legitimate write path to it.
    const WireId conductor_id = conductor_wire_id(topo::kWireHORN_DRIVE_LINE_LOW);

    StderrCapture capture;
    REQUIRE_FALSE(wire->write_bit(conductor_id, true));    // accessor 1: "write_bit"
    REQUIRE_FALSE(wire->write_byte(conductor_id, 1u));      // accessor 2: "write_byte"
    const std::string log = capture.contents();

    // Each accessor must produce its OWN "REFUSED ... <accessor>" line naming
    // itself. Before the fix, both calls hashed to the same dedup key (every
    // write_* name starts with 'w'), so the second line never printed.
    REQUIRE(count_occurrences(log, "REFUSED write_bit") == 1);
    REQUIRE(count_occurrences(log, "REFUSED write_byte") == 1);

    // Same accessor, same cell, again: THIS repeat must stay deduplicated —
    // the guarantee is "once per (cell, accessor) pair", not "once per call".
    REQUIRE_FALSE(wire->write_bit(conductor_id, false));
    REQUIRE(count_occurrences(capture.contents(), "REFUSED write_bit") == 1);
}
