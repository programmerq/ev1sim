#pragma once

#include "DashSnapshot.h"

#include <cstdint>
#include <memory>

// ---------------------------------------------------------------------------
// BusSnoop — the dashboard's passive listener on the GM-8192 bus.
//
// Phase 1 of docs/ipc_rsa_display_plan.md. Like the period Palm dashboard app
// reading the serial line, BusSnoop reads the frames modules already transmit
// and decodes the ones the instrument cluster cares about into a DashSnapshot:
//
//   GM8192_PIM_TX   $41 PCM Data Response   speed, PRND, PCM telltale requests
//   GM8192_BPM_TX   $40 PCM Data Poll       pack voltage / current / SOC
//                   $20 DSCM Data Poll      user SOC, range, time-to-full, BPM telltales
//                   $90 BTCM Data Poll      power bargraph
//   GM8192_BTCM_TX  $91 BTCM Data Response  brake / anti-lock telltales
//   GM8192_IPC_TX   $21 DSCM Data Response  season odometer
//
// It decodes already-framed application bytes only: the wire's bits become
// frames through the external sim's own gm8192_rx_framer, one per TX cell,
// on the connector's WireTable. BusSnoop adds no line, UART or timing
// model of its own and never writes the bus.
//
// Two seams:
//   * DecodeFrame() — pure: one frame's ID + payload → snapshot state. Always
//     compiled, so the decode is unit-testable without the substrate.
//   * Attach() + Step() — drain the four *_TX cells of a WireTable through one
//     framer each. Compiled only with EV1SIM_HAVE_WIRE_TRUTH; without it Attach()
//     returns false and Step() is a no-op, matching WireTruthChassis's stub.
//
// Freshness: each frame's group is stamped with the sim time it was decoded
// at. Snapshot(now_s) blanks a group older than stale_after_s(), so a module
// that stops transmitting reads as "---" rather than a frozen value. Time is
// the caller's sim time; BusSnoop never reads a host clock.
// ---------------------------------------------------------------------------

namespace electricsim::io { class WireTable; }

namespace ev1sim {

// Message IDs the snoop decodes. Hardcoded locally per the external sim's
// peer-ID convention; each comment names the authoritative declaration there.
inline constexpr std::uint8_t kDashMsgPcmResponse  = 0x41u;  // PIM_UART_FRAME_ID (ev1/pim/pim_uart_frame.h)
inline constexpr std::uint8_t kDashMsgPcmPoll      = 0x40u;  // BPM_PCM_POLL_ID (ev1/bpm/bpm_uart_frame.h)
inline constexpr std::uint8_t kDashMsgDscmPoll     = 0x20u;  // BPM_DSCM_POLL_ID (ev1/bpm/bpm_uart_frame.h)
inline constexpr std::uint8_t kDashMsgBtcmPoll     = 0x90u;  // BPM_BTCM_POLL_ID (ev1/bpm/bpm_uart_frame.h)
inline constexpr std::uint8_t kDashMsgBtcmResponse = 0x91u;  // BTCM_UART_FRAME_ID (ev1/btcm/btcm_uart_frame.h)
inline constexpr std::uint8_t kDashMsgDscmResponse = 0x21u;  // IPC_UART_FRAME_ID (ev1/ipc/ipc_uart_frame.h)

// PRND 5-bit wire code → gear. Codes are PIM_UART_PRND_CODE_* in the external
// sim's ev1/pim/pim_uart_frame.h (Park/Reverse/Neutral from the frame spec;
// Error/Drive are that header's own inferences, flagged there for re-check).
DashGear DecodePrndCode(std::uint8_t code5);

class BusSnoop {
public:
    // Display staleness horizon. @design 2026-10-13 claude — a display choice,
    // not an EV1 fact: just over twice the slowest decoded cadence ($20/$21,
    // 960 ms), so one late slow frame never blanks a field but a silent module
    // does within a couple of seconds.
    static constexpr double kDefaultStaleAfterS = 2.0;

    BusSnoop();
    ~BusSnoop();
    BusSnoop(const BusSnoop&) = delete;
    BusSnoop& operator=(const BusSnoop&) = delete;

    // Pure decode of one frame (ID + payload, SumCheck already verified by the
    // framer). Returns true when the frame is one the dashboard reads and its
    // payload length matches the contract; a known ID with the wrong N is
    // counted in frames_rejected() and changes nothing.
    bool DecodeFrame(std::uint8_t id, const std::uint8_t* payload, std::uint8_t n,
                     double now_s);

    // Bind to a WireTable and create one framer per snooped *_TX cell. The
    // table must outlive this BusSnoop (or the next Attach). Returns false when
    // the substrate is compiled out or `table` is null.
    bool Attach(electricsim::io::WireTable* table);
    bool attached() const;
    // Drop the framers (call before the attached table is destroyed). Decoded
    // groups stay and age out as usual.
    void Detach();

    // Drain every frame the snooped cells produced since the last call. The
    // bit-stream cells self-pace, so a coarse render tick replays every bit.
    void Step(double now_s);

    // The fresh view at `now_s`: groups decoded within stale_after_s() of it.
    DashSnapshot Snapshot(double now_s) const;

    void   set_stale_after_s(double s) { stale_after_s_ = s; }
    double stale_after_s() const { return stale_after_s_; }

    std::uint64_t frames_decoded() const { return frames_decoded_; }
    std::uint64_t frames_rejected() const { return frames_rejected_; }

private:
    template <typename T> struct Stamped {
        std::optional<T> value;
        double at_s = 0.0;
    };

    template <typename T>
    std::optional<T> Fresh(const Stamped<T>& s, double now_s) const;

    struct Wire;  // framers; empty when the substrate is compiled out
    std::unique_ptr<Wire> wire_;

    Stamped<DashPcmStatus>   pcm_;
    Stamped<DashPackStatus>  pack_;
    Stamped<DashBpmDisplay>  display_;
    Stamped<std::uint8_t>    power_bars_;
    Stamped<DashBrakeStatus> brake_;
    Stamped<DashOdometer>    odometer_;

    double stale_after_s_ = kDefaultStaleAfterS;
    std::uint64_t frames_decoded_ = 0;
    std::uint64_t frames_rejected_ = 0;
};

}  // namespace ev1sim
