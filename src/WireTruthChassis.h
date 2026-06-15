// WireTruthChassis — ev1sim's attachment to electricsim's wire-truth substrate.
//
// electricsim is migrating the chassis bus off the legacy ring
// (SharedMemoryTransport "electricsim_chassis_bus", DeltaRecord publish/poll)
// onto a shared-memory WireTable: a directory of named, typed cells indexed by
// WireId, with a topology_hash gate so peers built against different topologies
// refuse to attach. See electricsim docs/3d_sim_contract.md and
// notes/phase_c_chassis_migration_scope.md, and ev1sim
// docs/wire_truth_migration_scope.md for the migration plan.
//
// This class is ev1sim's side of that substrate. It attaches the shared
// WireTable as a fleet peer — the SAME create/attach idiom every electricsim
// controller uses via electricsim::topology::try_open_from_env() (segment
// "electricsim_wires", topology hash electricsim::topology::kTopologyHash). It
// exposes typed accessors that honour the substrate's written()/freshness
// contract: a read returns std::nullopt when the cell has never been written
// (the producer hasn't moved onto the wire yet), so the caller falls back to
// its legacy ring value — the kHold-safe migration seam.
//
// The header is deliberately electricsim-free (PIMPL). When the electricsim
// tree is not checked out next to ev1sim (EV1SIM_HAVE_WIRE_TRUTH undefined —
// e.g. the CI stub build), the factories return nullptr and the class is a
// compiled-out no-op; the connector's `if (wire)` guards keep working.
//
// @design 2026-06-15 — wire-truth migration kickoff (proof-of-life batch).

#ifndef EV1SIM_WIRE_TRUTH_CHASSIS_H
#define EV1SIM_WIRE_TRUTH_CHASSIS_H

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace ev1sim {

class WireTruthChassis {
public:
    ~WireTruthChassis();
    WireTruthChassis(const WireTruthChassis&) = delete;
    WireTruthChassis& operator=(const WireTruthChassis&) = delete;

    // Attach as a fleet peer using electricsim::topology::try_open_from_env():
    // honours ELECTRICSIM_WIRES_NAME / ELECTRICSIM_WIRES_ROLE exactly as every
    // electricsim controller does (run_ev1_vehicle.sh sets these). Returns
    // nullptr when wire-truth is disabled (env unset), the segment never comes
    // up, the topology hash mismatches, or the substrate is compiled out. A
    // nullptr return is the normal "stay on the legacy ring" path.
    static std::unique_ptr<WireTruthChassis> OpenFromEnv(
        const char* default_role = "attacher");

    // Attach to an explicitly named segment with the canonical topology hash.
    // For tests and ad-hoc tooling; OpenFromEnv() is the fleet path. Returns
    // nullptr on any attach failure (missing segment / hash mismatch / stub).
    static std::unique_ptr<WireTruthChassis> Attach(
        const std::string& segment_name);

    // True once the shared WireTable is attached (hash matched).
    bool attached() const;

    // Generic typed read of a bit cell, honouring the written() contract:
    //   - std::nullopt  -> not attached, undeclared/type-mismatched id, OR the
    //                      producer has never written it (gen == 0). Caller
    //                      keeps its legacy/fallback value.
    //   - true/false    -> the live wire value.
    std::optional<bool> read_bit(std::uint32_t wire_id) const;

    // Generic typed write of a bit cell (producer side). Returns false when not
    // attached, the substrate is compiled out, or the id is undeclared / wrong
    // type. Any attached peer may write; the wire-truth model expects one
    // declared producer per cell.
    bool write_bit(std::uint32_t wire_id, bool value);

    // Typed write accessors for ev1sim-produced cells (producer side).
    // Mirrors the electricsim WireTable::write_* surface; returns false when
    // not attached, the substrate is compiled out, or the id is undeclared /
    // type-mismatched. @design 2026-06-15 — producer dual-write batch.
    bool write_byte(std::uint32_t wire_id, std::uint8_t value);
    bool write_uint16(std::uint32_t wire_id, std::uint16_t value);
    bool write_uint32(std::uint32_t wire_id, std::uint32_t value);
    bool write_uint64(std::uint32_t wire_id, std::uint64_t value);
    bool write_float32(std::uint32_t wire_id, float value);

    // Mirror one ring DeltaRecord payload onto its wire cell, if signal_id is
    // a registered ev1sim-produced cell and we're attached. The payload is the
    // raw little-endian bytes from the DeltaRecord (same encoding as the ring).
    // Returns true if a write was performed. @design 2026-06-15 — table-driven
    // dual-write so every chassis producer cell reaches the WireTable without
    // per-signal connector code.
    bool mirror_signal(std::uint32_t signal_id, const std::uint8_t* payload,
                       std::size_t n);

    // Semantic helpers for the proof-of-life horn batch (chassis 4020/4021,
    // LHJB -> ev1sim). Resolve to the real generated WireIds in the .cpp, so
    // ev1sim never hardcodes the order-assigned numeric id.
    std::optional<bool> horn_low_drive() const;   // HORN_DRIVE_LINE_LOW  (4020)
    std::optional<bool> horn_high_drive() const;  // HORN_DRIVE_LINE_HIGH (4021)

private:
    WireTruthChassis();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ev1sim

#endif  // EV1SIM_WIRE_TRUTH_CHASSIS_H
