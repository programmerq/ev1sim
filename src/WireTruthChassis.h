// WireTruthChassis — ev1sim's attachment to the external sim's wire-truth substrate.
//
// external sim moved the chassis bus off the legacy ring
// (SharedMemoryTransport "the chassis-bus segment", DeltaRecord publish/poll)
// onto a shared-memory WireTable: a directory of named, typed cells indexed by
// WireId, with a topology_hash gate so peers built against different topologies
// refuse to attach. See the external sim's 3D-sim signal contract and
// the external sim's Phase-C chassis-migration scope note, and ev1sim
// docs/wire_truth_migration_scope.md for the migration plan.
//
// This class is ev1sim's side of that substrate. It attaches the shared
// WireTable as a fleet peer — the SAME create/attach idiom every external sim
// controller uses via electricsim::topology::try_open_from_env() (segment name
// from the ELECTRICSIM_WIRES_NAME env var, topology hash
// electricsim::topology::kTopologyHash). It
// exposes typed accessors that honour the substrate's written()/freshness
// contract: a read returns std::nullopt when the cell has never been written
// (the producer hasn't written it this run), so the caller keeps its own
// last/default value — the kHold-safe seam.
//
// The header is deliberately external-sim-free (PIMPL). When the external sim
// tree is not checked out next to ev1sim (EV1SIM_HAVE_WIRE_TRUTH undefined —
// e.g. the CI stub build), the factories return nullptr and the class is a
// compiled-out no-op; the connector's `if (wire)` guards keep working.
//
// @design 2026-06-15 — wire-truth migration kickoff (proof-of-life batch).

#ifndef EV1SIM_WIRE_TRUTH_CHASSIS_H
#define EV1SIM_WIRE_TRUTH_CHASSIS_H

#include <cstdint>
#include <functional>
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
    // external-sim controller does (run_ev1_vehicle.sh sets these). Returns
    // nullptr when wire-truth is disabled (env unset), the segment never comes
    // up, the topology hash mismatches, or the substrate is compiled out. A
    // nullptr return is the normal "external sim not present" path.
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
    //                      keeps its own last/default value.
    //   - true/false    -> the live wire value.
    std::optional<bool>          read_bit(std::uint32_t wire_id) const;

    // Generic typed reads for byte, float32, and uint32 cells. Same contract
    // as read_bit: nullopt when not attached, undeclared, type-mismatched, or
    // not yet written by the producer. @design 2026-06-15 — consumer overlay.
    std::optional<std::uint8_t>  read_byte(std::uint32_t wire_id) const;
    std::optional<float>         read_float32(std::uint32_t wire_id) const;
    std::optional<std::uint32_t> read_uint32(std::uint32_t wire_id) const;

    // Sinks for apply_consumer_overlay. Leave any sink empty (default-
    // constructed std::function) to skip that type; only non-empty sinks are
    // invoked. Each sink receives (signal_id, value) — the same signal_id the
    // legacy ring uses so callers can reuse existing per-id dispatch.
    struct ConsumerSinks {
        std::function<void(std::uint32_t, bool)>          on_bit;
        std::function<void(std::uint32_t, std::uint8_t)>  on_byte;
        std::function<void(std::uint32_t, float)>         on_float;
        std::function<void(std::uint32_t, std::uint32_t)> on_uint32;
    };

    // For each ev1sim-consumed chassis cell that is attached + written(),
    // reads it by type and invokes the matching non-empty sink with
    // (signal_id, value). Returns the number of cells for which a sink was
    // invoked. Returns 0 (no-op) when not attached or substrate compiled out.
    // The written() gate makes this kHold-safe: a cell whose producer has not
    // written it yet is skipped so the consumer's last value persists.
    // @design 2026-06-15 — consumer overlay batch.
    int apply_consumer_overlay(const ConsumerSinks& sinks) const;

    // Generic typed write of a bit cell (producer side). Returns false when not
    // attached, the substrate is compiled out, or the id is undeclared / wrong
    // type. Any attached peer may write; the wire-truth model expects one
    // declared producer per cell.
    bool write_bit(std::uint32_t wire_id, bool value);

    // Typed write accessors for ev1sim-produced cells (producer side).
    // Mirrors the external WireTable::write_* surface; returns false when
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

    // ── Co-sim tick barrier (primitive-4 B): leader-side ────────────────────
    // ev1sim is the barrier LEADER. After publishing all cells for a tick, it
    // opens the tick (BarrierPublishTick) and waits for all consumers to ack
    // (BarrierAwaitAcks) before stepping the next physics tick — a deterministic
    // per-tick lockstep. Inert (no-ops / returns true) when not attached, so a
    // wire-disabled or unarmed run is byte-identical. See
    // the external sim's co-sim determinism design note.
    void BarrierArm();
    void BarrierPublishTick();
    bool BarrierAwaitAcks(std::uint32_t consumer_count, int timeout_ms);

    // Semantic helpers for the proof-of-life horn batch (chassis 4020/4021,
    // LHJB -> ev1sim). Resolve to the real generated WireIds in the .cpp, so
    // ev1sim never hardcodes the order-assigned numeric id.
    std::optional<bool> horn_low_drive() const;   // HORN_DRIVE_LINE_LOW  (4020)
    std::optional<bool> horn_high_drive() const;  // HORN_DRIVE_LINE_HIGH (4021)

    // ── ECU-bus semantic helpers (wire-truth Phase 4) ───────────────────────
    // These resolve the ECU-side kWire* names inside WireTruthChassis.cpp, so
    // the connector reads ECU telemetry off the WireTable without hardcoding
    // numeric WireIds (mirrors the horn_low_drive() pattern). Each honours the
    // written() contract: nullopt until the producing ECU has written the cell.
    // @design 2026-06-17 — Phase 4 ECU-bus migration off SharedMemoryTransport.

    // RSA power-mode "run active" (RSA_RUN1_OUT). The RSA controller asserts
    // RUN1_OUT == RUN2_OUT == (run_mode is RUN or START); see the external sim's RSA controller. true => RUN/START, false => OFF/ACC. The old
    // kSigRunModeBroadcast (5711) carried the full 4-value enum, but only the
    // run-active bit survives on the wire (the ACC output has no cell yet —
    // external-sim gap), so OFF-vs-ACC and RUN-vs-START are not
    // distinguishable here. Reads RSA_RUN1_OUT (RUN2 is identical).
    std::optional<bool> rsa_run_active() const;    // RSA_RUN1_OUT (rsa_ecu)

    // Auto-Disconnect main HV contactor closed (AD_MAIN_CONTACTOR — the AD's
    // own commanded output, the authoritative successor of kSigAdMainContactor
    // 5224). Re-pointed 2026-07-04 off the APM_HV_CONTACTOR_CLOSED echo cell it
    // was mis-mapped to in the wire-truth port: the echo is only written when an APM
    // is in the fleet, so AD-only fleets read a permanent 0 (see .cpp comment).
    std::optional<bool> ad_main_contactor_closed() const;  // AD_MAIN_CONTACTOR

    // Auto-Disconnect state enum, reconstructed from the three discrete AD state
    // lines (AD_STATE_A/B/C) + the AD power-supply discrete (AD_POWER_SUPPLY),
    // using the external sim's ad_state_decode contract (its AD-state header):
    //   power lost            -> 8  (AD_STATE_POWER_LOST)
    //   idx=(a<<2)|(b<<1)|c   -> {0=OK,1=PRECHARGE_FAIL,2=BPM_OPEN,3=INTERLOCK,
    //                             4=PRECHARGE,5=LOW_PACK,6=GROUND_LOSS,
    //                             7=LOSS_OF_ISOLATION}
    // These integer codes match the enum the old kSigAdStateEnum (5230) carried,
    // so ev1sim's downstream interpretation (0=OK, 6=precharge, 7=fail) is
    // unchanged. Returns nullopt unless ALL FOUR contributing cells are written
    // (an all-or-nothing reconstruct — a partial line set would decode to a
    // wrong state). The decode table is replicated here with a pointer to the
    // canonical source; revalidated at integration.
    std::optional<std::uint32_t> ad_state_enum() const;

    // BTCM GM-8192 TX liveness proxy: the total bit count ever appended to the
    // BTCM transmit bit-stream cell (GM8192_BTCM_TX). The BTCM broadcasts its
    // canonical status frame at 5 Hz, so a growing total => "BTCM transmitting"
    // (alive). The connector stamps btcm_uart_frame_ns whenever this advances —
    // a cheap stand-in for the old kSigBtcmUartFrame (5050) heartbeat that keeps
    // the ABS/EMB freshness gate (btcm_alive) live WITHOUT reconstructing frame
    // payloads off the bit FIFO (deferred — see WireTruthChassis.cpp /
    // ExternalSimConnector.cpp TODOs). nullopt when not attached / undeclared /
    // never appended.  @inferred 2026-06-17 — total-bits-advancing ≈ alive.
    std::optional<std::uint64_t> btcm_tx_total_bits() const;

    // GM-8192 frame snoop (IPC-cluster Phase 1) — decode module TX frames off
    // the wire to feed the dashboard, rather than a dedicated chassis cell per
    // display value (docs/ipc_rsa_display_plan.md). snoop_step() drains the
    // per-module bit-stream TX cells through gm8192_rx_framer once per render
    // tick; the bit-stream self-paces, so a coarse tick replays every missed bit.
    void snoop_step(double now_s);
    // Latest vehicle speed (km/h) decoded from the PIM $41 PCM Data Response
    // (GM8192_PIM_TX, payload[4] = wire byte 6, 1 km/h/count; see the external sim's PIM UART-frame definition). nullopt until a $41 frame has been decoded.
    std::optional<std::uint8_t> pim_vehicle_speed_kph() const;

    // PIM cruise-control state — the external sim declared these cells
    // (was kSigPimCruiseActive/SetpointMps 5360/5361). Read off PIM_CRUISE_ACTIVE
    // / PIM_CRUISE_SETPOINT_MPS; nullopt until PIM writes them.
    std::optional<bool>  pim_cruise_active() const;        // PIM_CRUISE_ACTIVE
    std::optional<float> pim_cruise_setpoint_mps() const;  // PIM_CRUISE_SETPOINT_MPS

    // Auto-Disconnect precharge relay closed — external sim declared this cell on
    // (was kSigAdPrechargeRelay 5225). nullopt until AD writes it.
    std::optional<bool>  ad_precharge_relay() const;       // AD_PRECHARGE_RELAY

private:
    WireTruthChassis();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ev1sim

#endif  // EV1SIM_WIRE_TRUTH_CHASSIS_H
