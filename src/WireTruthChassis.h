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

#include "DashSnapshot.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace electricsim::io {
class WireTable;                          // opaque here; see table()
enum class ElementStateId : std::uint32_t;
}

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
    // up, the topology hash mismatches, or the substrate is compiled out.
    //
    // A nullptr return is AMBIGUOUS and callers must not treat it as "external sim
    // not present" — call LastAttachOutcome() to find out which of those cases
    // actually happened. See AttachOutcome below for why.
    static std::unique_ptr<WireTruthChassis> OpenFromEnv(
        const char* default_role = "attacher");

    // Attach to an explicitly named segment with the canonical topology hash.
    // For tests and ad-hoc tooling; OpenFromEnv() is the fleet path. Returns
    // nullptr on any attach failure (missing segment / hash mismatch / stub).
    static std::unique_ptr<WireTruthChassis> Attach(
        const std::string& segment_name);

    // Why the last OpenFromEnv() attempt did or did not yield a live substrate.
    //
    // THIS ENUM EXISTS BECAUSE nullptr WAS AMBIGUOUS. OpenFromEnv() returns
    // nullptr both when nobody asked for the substrate and when the substrate was
    // asked for and did not come up. Those are opposite facts, and collapsing them
    // into one pointer is what let ev1sim run a whole session with every wire dark:
    // the connector's `if (wire)` guard is false either way, no diagnostic is
    // printed, and the vehicle looks alive while consuming nothing. A topology
    // change on the producer side lands in exactly that window — this binary keeps
    // running against a segment whose hash it no longer matches.
    //
    // Three-valued on purpose so the caller can treat the middle case as the error
    // it is (see the attach site in ExternalSimConnector.cpp, which refuses to run
    // dark rather than degrade silently).
    enum class AttachOutcome {
        // OpenFromEnv() has not been called in this process yet.
        kNotAttempted,
        // Wire truth was never requested: ELECTRICSIM_WIRES_NAME unset/empty, or
        // the substrate is compiled out of this binary. NOT an error — this is the
        // documented "run ev1sim standalone" path.
        kDisabled,
        // Live substrate; the segment's topology hash matched this binary's.
        kAttached,
        // The environment ASKED for wire truth and it did not come up: segment
        // absent or down, or the segment's topology hash differs from the one this
        // binary was compiled against. This is an error, and never a reason to
        // continue — a co-sim that has lost its electrical fleet produces
        // confident, wrong output.
        kRequestedButUnavailable,
    };

    // The outcome of the most recent OpenFromEnv() call in this process.
    static AttachOutcome LastAttachOutcome();

    // Stable spelling of an AttachOutcome, for logs and test assertions.
    static const char* AttachOutcomeName(AttachOutcome outcome);

    // The topology hash THIS BINARY was compiled against. When an attach fails
    // with kRequestedButUnavailable this is half of what a reader needs; the other
    // half lives in whichever peer created the segment. Zero when the substrate is
    // compiled out.
    static std::uint32_t CompiledTopologyHash();

    // True once the shared WireTable is attached (hash matched).
    bool attached() const;

    // Generic typed read of a bit cell, honouring the written() contract:
    //   - std::nullopt  -> not attached, undeclared/type-mismatched id, OR the
    //                      producer has never written it (gen == 0). Caller
    //                      keeps its own last/default value.
    //   - true/false    -> the live wire value.
    std::optional<bool>          read_bit(std::uint32_t wire_id) const;
    // The same read for a switch or contactor contact (an element_state cell).
    std::optional<bool>          read_contact(electricsim::io::ElementStateId id) const;

    // Generic typed reads for byte, float32, and uint32 cells. Same contract
    // as read_bit: nullopt when not attached, undeclared, type-mismatched, or
    // not yet written by the producer. @design 2026-06-15 — consumer overlay.
    std::optional<std::uint8_t>  read_byte(std::uint32_t wire_id) const;
    std::optional<float>         read_float32(std::uint32_t wire_id) const;
    std::optional<std::uint32_t> read_uint32(std::uint32_t wire_id) const;

    // uint16 read. Same contract as read_bit. Added for the BTCM
    // retard-request PWM duty (4191), the first uint16 cell ev1sim CONSUMES
    // — write_uint16 already existed for the producer side (steering 6901).
    std::optional<std::uint16_t> read_uint16(std::uint32_t wire_id) const;
    std::optional<std::uint64_t> read_uint64(std::uint32_t wire_id) const;

    // Sinks for apply_consumer_overlay. Leave any sink empty (default-
    // constructed std::function) to skip that type; only non-empty sinks are
    // invoked. Each sink receives (signal_id, value) — the same signal_id the
    // legacy ring uses so callers can reuse existing per-id dispatch.
    struct ConsumerSinks {
        std::function<void(std::uint32_t, bool)>          on_bit;
        std::function<void(std::uint32_t, std::uint8_t)>  on_byte;
        std::function<void(std::uint32_t, float)>         on_float;
        std::function<void(std::uint32_t, std::uint32_t)> on_uint32;
        std::function<void(std::uint32_t, std::uint16_t)> on_uint16;
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
    // distinguishable here. Reads RSA_RUN1_OUT (RUN2 is identical) relative
    // to its declared rest level (rsa_run_commanded_level), never as a raw
    // bit: the line is active-low on the car.
    std::optional<bool> rsa_run_active() const;    // RSA_RUN1_OUT (rsa_ecu)
    // Whether a written RSA_RUN1_OUT level means RUN is commanded: any level
    // other than the cell's declared default (the line's rest level).
    static bool rsa_run_commanded_level(bool level);

    // Auto-Disconnect main HV contactor closed: the AD's own contactor
    // command, the contact the HV bus plant reads. nullopt until the AD writes it.
    std::optional<bool> ad_main_contactor_closed() const;  // HV_ES_MAIN_CONTACTOR

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

    // HV isolation-loss chain (BL-2026-07-18-hv-isolation-loss-vat). Plain
    // reads of the electricsim cells, nullopt until written:
    //   AD_ISOLATION_CHASSIS_REF_PERMILLE — the AD detector's measurand
    //   CHASSIS_AD_ACTIVE_DTC_BITMAP      — AD DTCs, bit code-1 (003 = isolation)
    //   CHASSIS_BPM_AD_DTC_BITMAP         — BPM 272..280, bit code-272
    //   CHASSIS_IPC_SERVICE_SOON_TELLTALE — the SERVICE SOON lamp (conductor)
    std::optional<std::uint32_t> ad_isolation_chassis_ref_permille() const;
    std::optional<std::uint32_t> ad_active_dtc_bitmap() const;
    std::optional<std::uint16_t> bpm_ad_dtc_bitmap() const;

    // HV rail sag witnesses (electricsim BL-2026-07-10 VAT low-voltage HV bus
    // sag scenario). Plain reads, nullopt until written:
    //   HV_BUS_VOLTAGE_MV        — the DC-link rail at the PIM, mV
    //   HV_BUS_PACK_CURRENT_MA   — current leaving the pack through the
    //                              contactors, signed mA (discharge positive),
    //                              an int32 riding a uint32 cell
    //   BPM_PACK_VOLTAGE         — pack terminal voltage, Q8 V
    //   BPM_PACK_CURRENT         — pack shunt current, signed Q8 A (int32 bits)
    //   CHASSIS_PIM_ACTIVE_DTC_BITMAP_LO/HI — PIM active DTCs, bit (code-1)
    //                              for 001..064 and bit (code-65) for 065..128
    std::optional<std::uint32_t> hv_bus_voltage_mv() const;
    std::optional<std::int32_t>  hv_bus_pack_current_ma() const;
    std::optional<float>         bpm_pack_voltage_v() const;
    std::optional<float>         bpm_pack_current_a() const;
    std::optional<std::uint64_t> pim_active_dtc_bitmap_lo() const;
    std::optional<std::uint64_t> pim_active_dtc_bitmap_hi() const;
    std::optional<bool>          ipc_service_soon_telltale() const;
    std::optional<bool>          ipc_wait_telltale() const;  // CHASSIS_IPC_WAIT_TELLTALE

    // The RSA console's output cells in one read (RSA_RUN1_OUT, CHASSIS_RSA_ACC_OUT,
    // _VALID_CODE, _SECURITY_INDICATOR, _SHIFT_BLOCKED, _BTSI_ENERGISED, the four
    // _SELECTOR_LED_* and RSA_ES_DIM_LEVEL_DRV). Unwritten cells stay nullopt.
    ev1sim::DashRsaConsole rsa_console() const;
    // The IPC's COMMAND to ground circuit 2029 (IPC_ES_WAIT_TT_DRV, an
    // element_state cause the IPC writes). Read-only here, as every cell ev1sim
    // consumes. Separate from the lamp because the lamp also needs its bank's
    // B+ feed.
    std::optional<bool>          ipc_wait_drive() const;

    // The vehicle's injected insulation fault: HV_ISOLATION_FAULT_LEAD (0 none,
    // 1 HV+, 2 HV-) and HV_ISOLATION_FAULT_KOHM. ev1sim is these cells'
    // producer (the vehicle plant). Returns false if not attached.
    bool publish_hv_isolation_fault(std::uint8_t lead, std::uint32_t kohm);

    // The steering pump motor's case temperature, uint16 Q8 °C
    // (CHASSIS_PS_MOTOR_CASE_TEMP_Q8, read by the PSCM's 150 °C switch). ev1sim
    // is this cell's producer (the pump peripheral). Returns false if not attached.
    bool publish_ps_motor_case_temp_q8(std::uint16_t q8);

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

    // The attached WireTable, for passive readers that drain cells on their own
    // schedule (BusSnoop's GM-8192 frame snoop for the instrument cluster,
    // docs/ipc_rsa_display_plan.md). nullptr when not attached or compiled out.
    // The table lives as long as this WireTruthChassis.
    electricsim::io::WireTable* table() const;

    // PIM cruise-control state — the external sim declared these cells
    // (was kSigPimCruiseActive/SetpointMps 5360/5361). Read off PIM_CRUISE_ACTIVE
    // / PIM_CRUISE_SETPOINT_MPS; nullopt until PIM writes them.
    std::optional<bool>  pim_cruise_active() const;        // PIM_CRUISE_ACTIVE
    std::optional<float> pim_cruise_setpoint_mps() const;  // PIM_CRUISE_SETPOINT_MPS

    // Auto-Disconnect precharge contactor closed, the contact the HV bus plant
    // reads. nullopt until the AD writes it.
    std::optional<bool>  ad_precharge_relay() const;       // HV_ES_PRECHARGE_CONTACTOR

    // The PIM's coast-down drag, Nm at the motor shaft (>= 0, opposing the
    // shaft's rotation). nullopt until the PIM writes CHASSIS_PIM_COAST_DRAG_NM.
    std::optional<float> pim_coast_drag_nm() const;        // CHASSIS_PIM_COAST_DRAG_NM

    // The BTCM's four brake motors, each driven through two leads: a front
    // modulator's restore and release leads, a rear apply motor's apply and
    // release leads (BTCM_FRONT_MOD_{LF,RF}_{RESTORE,RELEASE}, BTCM_REAR_EMB_
    // {LR,RR}_{APPLY,RELEASE}, conductors the solver publishes). Each level is
    // the fraction of full drive while that lead drives current through its
    // motor, 0 otherwise: a front lead is a bit (0 or 1), a rear lead carries
    // the BTCM's duty in permille. nullopt until the solver has written both
    // leads of that motor.
    enum class BtcmBrakeMotor { FrontLeft, FrontRight, RearLeft, RearRight };
    struct BtcmMotorLeads {
        float drive   = 0.0f;  // restore (front) or apply (rear), 0..1
        float release = 0.0f;
    };
    std::optional<BtcmMotorLeads> btcm_motor_leads(BtcmBrakeMotor motor) const;

    // The retard-request line at the PIM (PIM_RETARD_REQUEST_IN): the BTCM's
    // request duty in permille while the line is live, 0 while dark. nullopt
    // until the solver writes it.
    std::optional<std::uint16_t> pim_retard_request_permille() const;

    // The wiper blades' position, percent of one full sweep cycle (0 = park,
    // 50 = end of sweep), from the RHJB's wiper-motor plant. nullopt until the
    // RHJB writes CHASSIS_WIPER_SWEEP_POSITION.
    std::optional<std::uint8_t> wiper_sweep_position_pct() const;  // CHASSIS_WIPER_SWEEP_POSITION

    // The shifter's coast-down button position (1 = coast down), read by the
    // RSA. ev1sim is this cell's producer. Returns false if not attached.
    bool publish_coast_down_switch(bool on);               // DRIVER_COAST_DOWN_SWITCH

    // Accelerator pedal travel, permille of full travel
    // (DRIVER_ES_PEDAL_TRAVEL); the solver turns it into the PIM's pedal
    // position tracks. ev1sim is this cell's producer. Returns false if not
    // attached.
    bool publish_pedal_travel(std::uint16_t permille);

    // PRND switch #2's REVERSE SELECT contact (DRIVER_ES_REVERSE_SELECT_SWITCH),
    // closed while the lever is in R; it lights the back-up lamps. ev1sim is
    // this cell's producer. Returns false if not attached.
    bool publish_reverse_select_switch(bool closed);

    // One click (press then release) of the RSA's DIM UP or DIM DOWN button;
    // the RSA moves its dimmer step one level per click. Returns false if not
    // attached.
    bool publish_rsa_dim_click(bool up);                   // DRIVER_RSA_DIM_UP / _DOWN

private:
    WireTruthChassis();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ev1sim

#endif  // EV1SIM_WIRE_TRUTH_CHASSIS_H
