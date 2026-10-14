// BusSnoop — the dashboard's GM-8192 frame decode (docs/ipc_rsa_display_plan.md
// Phase 1). Payloads below are written byte by byte from the external sim's
// frame contracts (ev1/{pim,bpm,btcm,ipc}/*_uart_frame.h), not from BusSnoop's
// own decode, so a decode that drifts from the contract fails here.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "BusSnoop.h"

#include <cstdint>

#if defined(EV1SIM_HAVE_WIRE_TRUTH) && EV1SIM_HAVE_WIRE_TRUTH
#include "gm8192/gm8192_frame.h"
#include "topology/topology_generated.h"
#include "uart/uart_tx.hpp"
#include "wire_table.hpp"

#include <string>
#include <unistd.h>
#include <vector>
#endif

using ev1sim::BusSnoop;
using ev1sim::DashGear;
using Catch::Matchers::WithinAbs;

TEST_CASE("BusSnoop: PRND codes map to gears; undefined codes are unknown", "[bus_snoop]") {
    REQUIRE(ev1sim::DecodePrndCode(0x00u) == DashGear::kError);
    REQUIRE(ev1sim::DecodePrndCode(0x1Au) == DashGear::kPark);
    REQUIRE(ev1sim::DecodePrndCode(0x0Eu) == DashGear::kReverse);
    REQUIRE(ev1sim::DecodePrndCode(0x0Bu) == DashGear::kNeutral);
    REQUIRE(ev1sim::DecodePrndCode(0x15u) == DashGear::kDrive);
    REQUIRE(ev1sim::DecodePrndCode(0x01u) == DashGear::kUnknown);
    REQUIRE(ev1sim::DecodePrndCode(0x1Fu) == DashGear::kUnknown);
}

TEST_CASE("BusSnoop: nothing decoded reads as an empty snapshot", "[bus_snoop]") {
    BusSnoop snoop;
    const auto snap = snoop.Snapshot(0.0);
    REQUIRE_FALSE(snap.pcm);
    REQUIRE_FALSE(snap.pack);
    REQUIRE_FALSE(snap.display);
    REQUIRE_FALSE(snap.power_bars);
    REQUIRE_FALSE(snap.brake);
    REQUIRE_FALSE(snap.odometer);
    REQUIRE_FALSE(snap.lamps);  // never filled by the snoop
}

TEST_CASE("BusSnoop: $41 PCM Data Response gives speed, gear and PCM requests", "[bus_snoop]") {
    BusSnoop snoop;
    // [2] flags: Service Soon (b0) + PCM Temp (b2) + Charge Mode Display (b4)
    // [3] PRND Drive 0b10101 in b7-3, PCM Service Now (b1)
    // [4] PIM temp, [5] retard achieved, [6] speed 88 km/h, [7] seed, [8] stator temp
    const std::uint8_t p[7] = {0x15u, static_cast<std::uint8_t>((0x15u << 3) | 0x02u),
                               0x50u, 0x19u, 88u, 0x00u, 0x60u};
    REQUIRE(snoop.DecodeFrame(0x41u, p, 7u, 1.0));

    const auto pcm = snoop.Snapshot(1.0).pcm;
    REQUIRE(pcm);
    REQUIRE(pcm->speed_kph == 88u);
    REQUIRE(pcm->gear == DashGear::kDrive);
    REQUIRE(pcm->prnd_code == 0x15u);
    REQUIRE(pcm->service_soon_tt);
    REQUIRE_FALSE(pcm->reduced_perf_tt);
    REQUIRE(pcm->pcm_temp_tt);
    REQUIRE(pcm->charge_mode_display);
    REQUIRE(pcm->pcm_service_now_tt);
    REQUIRE_FALSE(pcm->pcm_connection_detect);
}

TEST_CASE("BusSnoop: $40 PCM Data Poll gives the pack readout", "[bus_snoop]") {
    BusSnoop snoop;
    // [2] 156 counts × 2 V = 312 V
    // [3..4] b15 charger + b11 AD open; current 709 counts → 0.5×709 − 400 = −45.5 A
    const std::uint16_t status = (1u << 15) | (1u << 11) | 709u;
    const std::uint8_t p[6] = {156u, static_cast<std::uint8_t>(status >> 8),
                               static_cast<std::uint8_t>(status & 0xFFu), 180u, 82u, 0u};
    REQUIRE(snoop.DecodeFrame(0x40u, p, 6u, 0.0));

    const auto pack = snoop.Snapshot(0.0).pack;
    REQUIRE(pack);
    REQUIRE(pack->pack_voltage_v == 312u);
    REQUIRE_THAT(pack->pack_current_a, WithinAbs(-45.5, 1e-6));
    REQUIRE(pack->charger_connected);
    REQUIRE_FALSE(pack->pack_malfunction);
    REQUIRE_FALSE(pack->pack_voltage_fault);
    REQUIRE_FALSE(pack->pack_current_fault);
    REQUIRE(pack->auto_disconnect_open);
    REQUIRE(pack->pack_soc_pct == 82u);
}

TEST_CASE("BusSnoop: $20 DSCM Data Poll gives SOC, range, time to full, BPM requests", "[bus_snoop]") {
    BusSnoop snoop;
    // [5..6] b15 Battery Life + b13 Service Now, capability level 5, range 999 km
    const std::uint16_t word = (1u << 15) | (1u << 13) | (5u << 10) | 999u;
    const std::uint8_t p[5] = {64u, 2u, 5u, static_cast<std::uint8_t>(word >> 8),
                               static_cast<std::uint8_t>(word & 0xFFu)};
    REQUIRE(snoop.DecodeFrame(0x20u, p, 5u, 0.0));

    const auto d = snoop.Snapshot(0.0).display;
    REQUIRE(d);
    REQUIRE(d->user_soc_pct == 64u);
    REQUIRE(d->ttf_hours == 2u);
    REQUIRE(d->ttf_minutes == 5u);
    REQUIRE(d->battery_life_tt);
    REQUIRE_FALSE(d->bpm_temp_tt);
    REQUIRE(d->bpm_service_now_tt);
    REQUIRE(d->power_capability == 5u);
    REQUIRE(d->range_km == 999u);
}

TEST_CASE("BusSnoop: $90 bargraph counts the highest lit bar", "[bus_snoop]") {
    BusSnoop snoop;
    // Bars #1..#6 lit (bits 0-5) with Current Available (b15) set alongside.
    const std::uint16_t word = (1u << 15) | 0x003Fu;
    const std::uint8_t p[2] = {static_cast<std::uint8_t>(word >> 8),
                               static_cast<std::uint8_t>(word & 0xFFu)};
    REQUIRE(snoop.DecodeFrame(0x90u, p, 2u, 0.0));
    REQUIRE(snoop.Snapshot(0.0).power_bars == std::optional<std::uint8_t>(6u));

    const std::uint8_t full[2] = {0x07u, 0xFFu};  // all 11 bars
    REQUIRE(snoop.DecodeFrame(0x90u, full, 2u, 0.0));
    REQUIRE(snoop.Snapshot(0.0).power_bars == std::optional<std::uint8_t>(11u));
}

TEST_CASE("BusSnoop: $91 BTCM Data Response gives the brake status bits", "[bus_snoop]") {
    BusSnoop snoop;
    // [2] Park Brake (b0) + Service Brake Telltale (b2) + ABS/TCS Active (b5)
    std::uint8_t p[11] = {};
    p[0] = 0x25u;
    REQUIRE(snoop.DecodeFrame(0x91u, p, 11u, 0.0));

    const auto b = snoop.Snapshot(0.0).brake;
    REQUIRE(b);
    REQUIRE(b->park_brake);
    REQUIRE_FALSE(b->service_brake);
    REQUIRE(b->brake_tt);
    REQUIRE_FALSE(b->antilock_tt);
    REQUIRE(b->abs_tcs_active);
}

TEST_CASE("BusSnoop: $21 DSCM Data Response gives the 22-bit season odometer", "[bus_snoop]") {
    BusSnoop snoop;
    // 0x2ABCDE = 2,800,862 counts of 0.1 km; b6 Power Use mode, b7 fault clear.
    const std::uint8_t p[3] = {static_cast<std::uint8_t>(0x40u | 0x2Au), 0xBCu, 0xDEu};
    REQUIRE(snoop.DecodeFrame(0x21u, p, 3u, 0.0));

    const auto o = snoop.Snapshot(0.0).odometer;
    REQUIRE(o);
    REQUIRE(o->season_odometer_dkm == 0x2ABCDEu);
    REQUIRE(o->bar_gage_power_use);
    REQUIRE_FALSE(o->odometer_fault);

    const std::uint8_t fault[3] = {0x80u, 0x00u, 0x01u};
    REQUIRE(snoop.DecodeFrame(0x21u, fault, 3u, 0.0));
    REQUIRE(snoop.Snapshot(0.0).odometer->odometer_fault);
    REQUIRE(snoop.Snapshot(0.0).odometer->season_odometer_dkm == 1u);
}

TEST_CASE("BusSnoop: a known ID with the wrong length is rejected, unknown IDs ignored", "[bus_snoop]") {
    BusSnoop snoop;
    const std::uint8_t p[8] = {};
    REQUIRE_FALSE(snoop.DecodeFrame(0x41u, p, 6u, 0.0));  // $41 is N=7
    REQUIRE(snoop.frames_rejected() == 1u);
    REQUIRE_FALSE(snoop.DecodeFrame(0x30u, p, 0u, 0.0));  // $30 poll: not a cluster frame
    REQUIRE_FALSE(snoop.DecodeFrame(0xF0u, p, 1u, 0.0));  // ALDL presence check
    REQUIRE(snoop.frames_rejected() == 1u);
    REQUIRE(snoop.frames_decoded() == 0u);
    REQUIRE_FALSE(snoop.Snapshot(0.0).pcm);
}

TEST_CASE("BusSnoop: a group goes blank once its frame is stale and returns on the next frame", "[bus_snoop]") {
    BusSnoop snoop;
    const std::uint8_t p[7] = {0u, 0u, 0u, 0u, 40u, 0u, 0u};
    REQUIRE(snoop.DecodeFrame(0x41u, p, 7u, 10.0));

    REQUIRE(snoop.Snapshot(10.0 + BusSnoop::kDefaultStaleAfterS).pcm);
    REQUIRE_FALSE(snoop.Snapshot(10.0 + BusSnoop::kDefaultStaleAfterS + 0.01).pcm);

    REQUIRE(snoop.DecodeFrame(0x41u, p, 7u, 13.0));
    REQUIRE(snoop.Snapshot(13.5).pcm);

    snoop.set_stale_after_s(0.2);
    REQUIRE_FALSE(snoop.Snapshot(13.5).pcm);
}

#if defined(EV1SIM_HAVE_WIRE_TRUTH) && EV1SIM_HAVE_WIRE_TRUTH

namespace {

using electricsim::io::WireTable;
using electricsim::io::WireTableOptions;
namespace topo = electricsim::topology;

constexpr std::uint64_t kBitPeriodNs = 122070;  // 8192 baud

std::unique_ptr<WireTable> CreateFleetTable(const char* tag) {
    WireTableOptions opts;
    opts.name = std::string("ev1sim_snoop_") + tag + "_" +
                std::to_string(static_cast<long>(::getpid()));
    opts.topology_hash = topo::kTopologyHash;
    return WireTable::create(opts, [](WireTable& t) { return topo::declare_all(t); });
}

std::vector<std::uint8_t> Encode(std::uint8_t id, const std::uint8_t* payload, std::uint8_t n) {
    std::vector<std::uint8_t> out(GM8192_MAX_FRAME_LEN);
    std::size_t len = 0;
    REQUIRE(gm8192_encode(id, payload, n, out.data(), out.size(), &len) == GM8192_OK);
    out.resize(len);
    return out;
}

// Pump every transmitter and the snoop in lockstep, one bit period per step,
// long enough for `bytes` UART bytes plus idle slack.
void Pump(std::vector<electricsim::io::UartTx*> txs, BusSnoop& snoop,
          std::uint64_t& now_ns, std::size_t bytes) {
    const std::size_t steps = bytes * 10u + 40u;
    for (std::size_t i = 0; i < steps; ++i) {
        now_ns += kBitPeriodNs;
        for (auto* tx : txs) tx->tick(now_ns);
        snoop.Step(static_cast<double>(now_ns) / 1.0e9);
    }
}

}  // namespace

TEST_CASE("BusSnoop: decodes the cluster frames off the four TX bit-stream cells",
          "[bus_snoop][wire_truth]") {
    auto table = CreateFleetTable("four_cells");
    REQUIRE(table != nullptr);

    BusSnoop snoop;
    REQUIRE(snoop.Attach(table.get()));
    REQUIRE(snoop.attached());

    electricsim::io::UartTx pim(table.get(), topo::kWireGM8192_PIM_TX, kBitPeriodNs);
    electricsim::io::UartTx bpm(table.get(), topo::kWireGM8192_BPM_TX, kBitPeriodNs);
    electricsim::io::UartTx btcm(table.get(), topo::kWireGM8192_BTCM_TX, kBitPeriodNs);
    electricsim::io::UartTx ipc(table.get(), topo::kWireGM8192_IPC_TX, kBitPeriodNs);

    const std::uint8_t p41[7] = {0u, static_cast<std::uint8_t>(0x1Au << 3), 0u, 0u, 72u, 0u, 0u};
    const std::uint8_t p40[6] = {150u, 0x03u, 0x20u, 0u, 77u, 0u};  // 300 V, 800 counts → 0 A
    const std::uint8_t p30[1] = {0u};                              // $30 trigger rides along
    const std::uint8_t p20[5] = {75u, 0u, 0u, 0x00u, 160u};         // 75 %, 160 km
    const std::uint8_t p91[11] = {0x08u};                          // anti-lock telltale
    const std::uint8_t p21[3] = {0x00u, 0x30u, 0x39u};              // 12345 → 1234.5 km

    const auto f41 = Encode(0x41u, p41, 7u);
    const auto f40 = Encode(0x40u, p40, 6u);
    const auto f30 = Encode(0x30u, p30, 0u);
    const auto f20 = Encode(0x20u, p20, 5u);
    const auto f91 = Encode(0x91u, p91, 11u);
    const auto f21 = Encode(0x21u, p21, 3u);

    // Each enqueue() starts a frame after an Idle Line, as the producers do.
    pim.enqueue(f41.data(), f41.size());
    bpm.enqueue(f40.data(), f40.size());
    bpm.enqueue(f30.data(), f30.size());
    bpm.enqueue(f20.data(), f20.size());
    btcm.enqueue(f91.data(), f91.size());
    ipc.enqueue(f21.data(), f21.size());

    std::uint64_t now_ns = 1000u;
    const std::size_t bpm_bytes = f40.size() + f30.size() + f20.size() + 4u;  // + idle gaps
    Pump({&pim, &bpm, &btcm, &ipc}, snoop, now_ns, bpm_bytes);

    const auto snap = snoop.Snapshot(static_cast<double>(now_ns) / 1.0e9);
    REQUIRE(snap.pcm);
    REQUIRE(snap.pcm->speed_kph == 72u);
    REQUIRE(snap.pcm->gear == DashGear::kPark);
    REQUIRE(snap.pack);
    REQUIRE(snap.pack->pack_voltage_v == 300u);
    REQUIRE_THAT(snap.pack->pack_current_a, WithinAbs(0.0, 1e-6));
    REQUIRE(snap.pack->pack_soc_pct == 77u);
    REQUIRE(snap.display);
    REQUIRE(snap.display->user_soc_pct == 75u);
    REQUIRE(snap.display->range_km == 160u);
    REQUIRE(snap.brake);
    REQUIRE(snap.brake->antilock_tt);
    REQUIRE(snap.odometer);
    REQUIRE(snap.odometer->season_odometer_dkm == 12345u);
    REQUIRE(snoop.frames_decoded() == 5u);  // the $30 trigger is not a cluster frame
    REQUIRE(snoop.frames_rejected() == 0u);

    // A silent bus goes stale: no more frames, sim time moves past the horizon.
    const double later = static_cast<double>(now_ns) / 1.0e9 + BusSnoop::kDefaultStaleAfterS + 0.5;
    snoop.Step(later);
    const auto stale = snoop.Snapshot(later);
    REQUIRE_FALSE(stale.pcm);
    REQUIRE_FALSE(stale.pack);
    REQUIRE_FALSE(stale.odometer);
}

TEST_CASE("BusSnoop: Attach(nullptr) leaves the snoop detached", "[bus_snoop][wire_truth]") {
    BusSnoop snoop;
    REQUIRE_FALSE(snoop.Attach(nullptr));
    REQUIRE_FALSE(snoop.attached());
    snoop.Step(1.0);  // harmless
    REQUIRE(snoop.frames_decoded() == 0u);
}

#else

TEST_CASE("BusSnoop: without the substrate Attach() reports unavailable", "[bus_snoop]") {
    BusSnoop snoop;
    REQUIRE_FALSE(snoop.Attach(nullptr));
    REQUIRE_FALSE(snoop.attached());
    snoop.Step(1.0);
    REQUIRE(snoop.frames_decoded() == 0u);
}

#endif
