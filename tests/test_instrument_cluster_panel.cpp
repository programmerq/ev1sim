// InstrumentClusterPanel — the IPC cluster HUD panel (docs/ipc_rsa_display_plan.md
// Phase 1): pure label helpers, the snapshot → view model, a null-driver draw,
// and one [e2e] case from TX bit-stream cells through BusSnoop to the panel.

#include <catch2/catch_test_macros.hpp>

#include "BusSnoop.h"
#include "InstrumentClusterPanel.h"

#include <irrlicht.h>

#include <cstdint>
#include <string>

#if defined(EV1SIM_HAVE_WIRE_TRUTH) && EV1SIM_HAVE_WIRE_TRUTH
#include "gm8192/gm8192_frame.h"
#include "topology/topology_generated.h"
#include "uart/uart_tx.hpp"
#include "wire_table.hpp"

#include <unistd.h>
#include <vector>
#endif

using namespace ev1sim;

namespace {

DashPcmStatus Pcm(std::uint8_t kph, DashGear gear) {
    DashPcmStatus p;
    p.speed_kph = kph;
    p.gear = gear;
    return p;
}

DashBpmDisplay Display(std::uint8_t soc, std::uint16_t range_km, std::uint8_t hh = 0,
                       std::uint8_t mm = 0) {
    DashBpmDisplay d;
    d.user_soc_pct = soc;
    d.range_km = range_km;
    d.ttf_hours = hh;
    d.ttf_minutes = mm;
    return d;
}

}  // namespace

TEST_CASE("Cluster speed: km/h from the $41, MPH converted at the display", "[cluster]") {
    REQUIRE(FormatClusterSpeed(std::nullopt, ClusterUnits::kMetric) == L"--- km/h");
    REQUIRE(FormatClusterSpeed(std::nullopt, ClusterUnits::kImperial) == L"--- MPH");
    REQUIRE(FormatClusterSpeed(Pcm(88, DashGear::kDrive), ClusterUnits::kMetric) == L"88 km/h");
    // 88 km/h = 54.68 mi/h → 55; 162 km/h (the field's max) = 100.66 → 101.
    REQUIRE(FormatClusterSpeed(Pcm(88, DashGear::kDrive), ClusterUnits::kImperial) == L"55 MPH");
    REQUIRE(FormatClusterSpeed(Pcm(162, DashGear::kDrive), ClusterUnits::kImperial) == L"101 MPH");
    REQUIRE(FormatClusterSpeed(Pcm(0, DashGear::kPark), ClusterUnits::kImperial) == L"0 MPH");
}

TEST_CASE("Cluster gear: PRND letters; Error, undefined and missing show a dash", "[cluster]") {
    REQUIRE(FormatClusterGear(Pcm(0, DashGear::kPark)) == L"P");
    REQUIRE(FormatClusterGear(Pcm(0, DashGear::kReverse)) == L"R");
    REQUIRE(FormatClusterGear(Pcm(0, DashGear::kNeutral)) == L"N");
    REQUIRE(FormatClusterGear(Pcm(0, DashGear::kDrive)) == L"D");
    REQUIRE(FormatClusterGear(Pcm(0, DashGear::kError)) == L"-");
    REQUIRE(FormatClusterGear(Pcm(0, DashGear::kUnknown)) == L"-");
    REQUIRE(FormatClusterGear(std::nullopt) == L"-");
}

TEST_CASE("Cluster SOC and range come from the $20 push", "[cluster]") {
    REQUIRE(FormatClusterSoc(std::nullopt) == L"SOC ---");
    REQUIRE(FormatClusterSoc(Display(82, 0)) == L"SOC 82%");
    REQUIRE(FormatClusterRange(std::nullopt, ClusterUnits::kMetric) == L"Range ---");
    REQUIRE(FormatClusterRange(Display(0, 120), ClusterUnits::kMetric) == L"Range 120 km");
    REQUIRE(FormatClusterRange(Display(0, 120), ClusterUnits::kImperial) == L"Range 75 mi");
}

TEST_CASE("Cluster power bar draws the 11 $90 segments", "[cluster]") {
    REQUIRE(FormatClusterPowerBar(std::nullopt) == L"Power ---");
    REQUIRE(FormatClusterPowerBar(std::uint8_t{0}) == L"Power [...........]");
    REQUIRE(FormatClusterPowerBar(std::uint8_t{5}) == L"Power [#####......]");
    REQUIRE(FormatClusterPowerBar(std::uint8_t{11}) == L"Power [###########]");
    REQUIRE(FormatClusterPowerBar(std::uint8_t{200}) == L"Power [###########]");  // clamps
}

TEST_CASE("Cluster pack line and the charging row", "[cluster]") {
    DashPackStatus pack;
    pack.pack_voltage_v = 312;
    pack.pack_current_a = -45.5f;
    REQUIRE(FormatClusterPack(std::nullopt) == L"Pack ---");
    REQUIRE(FormatClusterPack(pack) == L"Pack 312 V  -45.5 A");

    // Not charging (or no $40): no row at all.
    REQUIRE(FormatClusterCharge(std::nullopt, Display(50, 0, 2, 5)).empty());
    REQUIRE(FormatClusterCharge(pack, Display(50, 0, 2, 5)).empty());

    pack.charger_connected = true;
    REQUIRE(FormatClusterCharge(pack, Display(50, 0, 2, 5)) == L"Charging: full in 2:05");
    REQUIRE(FormatClusterCharge(pack, std::nullopt) == L"Charging: --:--");
}

TEST_CASE("Cluster odometer reads the IPC's own $21", "[cluster]") {
    DashOdometer odo;
    odo.season_odometer_dkm = 12345;  // 1234.5 km
    REQUIRE(FormatClusterOdometer(std::nullopt, ClusterUnits::kMetric) == L"ODO ---");
    REQUIRE(FormatClusterOdometer(odo, ClusterUnits::kMetric) == L"ODO 1234.5 km");
    REQUIRE(FormatClusterOdometer(odo, ClusterUnits::kImperial) == L"ODO 767.1 mi");
    odo.odometer_fault = true;
    REQUIRE(FormatClusterOdometer(odo, ClusterUnits::kMetric) == L"ODO FAULT");
}

TEST_CASE("Cluster bus line says which modules are being heard", "[cluster]") {
    DashSnapshot snap;
    REQUIRE(FormatClusterBusLine(snap) == L"Bus: PIM --  BPM --  BTCM --  IPC --");
    snap.pcm = Pcm(0, DashGear::kPark);
    snap.power_bars = std::uint8_t{3};  // any BPM frame counts as the BPM heard
    snap.odometer = DashOdometer{};
    REQUIRE(FormatClusterBusLine(snap) == L"Bus: PIM ok  BPM ok  BTCM --  IPC ok");
}

TEST_CASE("Cluster view: lamps are unknown until their cells are written", "[cluster]") {
    const ClusterView empty = BuildClusterView(DashSnapshot{}, ClusterUnits::kMetric);
    REQUIRE(empty.lamps.size() == 13u);
    for (const auto& lamp : empty.lamps) {
        REQUIRE_FALSE(lamp.known);
        REQUIRE_FALSE(lamp.lit);
    }

    DashSnapshot snap;
    snap.lamps = DashLamps{};
    snap.lamps->antilock = true;
    snap.lamps->brake = false;
    const ClusterView v = BuildClusterView(snap, ClusterUnits::kMetric);
    REQUIRE(std::wstring(v.lamps[0].name) == L"BRAKE");
    REQUIRE(v.lamps[0].known);
    REQUIRE_FALSE(v.lamps[0].lit);
    REQUIRE(std::wstring(v.lamps[2].name) == L"ANTI-LOCK");
    REQUIRE(v.lamps[2].known);
    REQUIRE(v.lamps[2].lit);
    REQUIRE_FALSE(v.lamps[1].known);  // PARK BRAKE cell never written
}

TEST_CASE("Cluster panel: the charging row changes the height; draws on a null driver", "[cluster]") {
    InstrumentClusterPanel panel(ClusterUnits::kMetric, 10, 10, 300);
    const int idle_h = panel.Height();

    DashSnapshot snap;
    snap.pack = DashPackStatus{};
    snap.pack->charger_connected = true;
    snap.display = Display(40, 90, 3, 30);
    panel.Update(snap);
    REQUIRE(panel.view().charge == L"Charging: full in 3:30");
    REQUIRE(panel.Height() > idle_h);

    // Irrlicht's null device needs no window or GL; it proves Draw() runs the
    // whole layout against a real driver + built-in font without faulting.
    irr::IrrlichtDevice* dev = irr::createDevice(irr::video::EDT_NULL,
                                                 irr::core::dimension2d<irr::u32>(640, 480));
    REQUIRE(dev != nullptr);
    irr::video::IVideoDriver* driver = dev->getVideoDriver();
    irr::gui::IGUIFont* font = dev->getGUIEnvironment()->getBuiltInFont();
    driver->beginScene(true, true, irr::video::SColor(255, 0, 0, 0));
    panel.Draw(driver, font);
    panel.Draw(driver, nullptr);
    panel.SetVisible(false);
    panel.Draw(driver, font);
    driver->endScene();
    panel.Draw(nullptr, font);
    dev->drop();
}

#if defined(EV1SIM_HAVE_WIRE_TRUTH) && EV1SIM_HAVE_WIRE_TRUTH

TEST_CASE("Cluster panel from the bus: TX cells → BusSnoop → panel view", "[cluster][e2e][wire_truth]") {
    namespace topo = electricsim::topology;
    using electricsim::io::WireTable;
    constexpr std::uint64_t kBitPeriodNs = 122070;

    electricsim::io::WireTableOptions opts;
    opts.name = "ev1sim_cluster_e2e_" + std::to_string(static_cast<long>(::getpid()));
    opts.topology_hash = topo::kTopologyHash;
    auto table = WireTable::create(opts, [](WireTable& t) { return topo::declare_all(t); });
    REQUIRE(table != nullptr);

    BusSnoop snoop;
    REQUIRE(snoop.Attach(table.get()));

    auto encode = [](std::uint8_t id, const std::uint8_t* p, std::uint8_t n) {
        std::vector<std::uint8_t> out(GM8192_MAX_FRAME_LEN);
        std::size_t len = 0;
        REQUIRE(gm8192_encode(id, p, n, out.data(), out.size(), &len) == GM8192_OK);
        out.resize(len);
        return out;
    };
    // $41: Drive, 100 km/h. $20: 64 %, 150 km. $21: 1000.0 km.
    const std::uint8_t p41[7] = {0u, static_cast<std::uint8_t>(0x15u << 3), 0u, 0u, 100u, 0u, 0u};
    const std::uint8_t p20[5] = {64u, 0u, 0u, 0x00u, 150u};
    const std::uint8_t p21[3] = {0x00u, 0x27u, 0x10u};
    const auto f41 = encode(0x41u, p41, 7u);
    const auto f20 = encode(0x20u, p20, 5u);
    const auto f21 = encode(0x21u, p21, 3u);

    electricsim::io::UartTx pim(table.get(), topo::kWireGM8192_PIM_TX, kBitPeriodNs);
    electricsim::io::UartTx bpm(table.get(), topo::kWireGM8192_BPM_TX, kBitPeriodNs);
    electricsim::io::UartTx ipc(table.get(), topo::kWireGM8192_IPC_TX, kBitPeriodNs);
    pim.enqueue(f41.data(), f41.size());
    bpm.enqueue(f20.data(), f20.size());
    ipc.enqueue(f21.data(), f21.size());

    std::uint64_t now = 1000u;
    for (int i = 0; i < 160; ++i) {
        now += kBitPeriodNs;
        pim.tick(now);
        bpm.tick(now);
        ipc.tick(now);
        snoop.Step(static_cast<double>(now) / 1.0e9);
    }

    InstrumentClusterPanel panel(ClusterUnits::kImperial);
    panel.Update(snoop.Snapshot(static_cast<double>(now) / 1.0e9));
    const ClusterView& v = panel.view();
    REQUIRE(v.speed == L"62 MPH");  // 100 km/h = 62.14 mi/h
    REQUIRE(v.gear == L"D");
    REQUIRE(v.soc == L"SOC 64%");
    REQUIRE(v.range == L"Range 93 mi");
    REQUIRE(v.odometer == L"ODO 621.4 mi");
    REQUIRE(v.pack == L"Pack ---");  // no $40 sent
    REQUIRE(v.bus == L"Bus: PIM ok  BPM ok  BTCM --  IPC ok");
}

#endif
