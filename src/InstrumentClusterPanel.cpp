#include "InstrumentClusterPanel.h"

#include <irrlicht.h>

#include <cmath>
#include <cstdio>
#include <cwchar>

namespace ev1sim {

namespace {

// 1 mile = 1.609344 km exactly (international mile).
constexpr double kKmPerMile = 1.609344;

unsigned KmToMilesRounded(double km) {
    return static_cast<unsigned>(std::lround(km / kKmPerMile));
}

const wchar_t* UnitsSpeed(ClusterUnits u) { return u == ClusterUnits::kImperial ? L"MPH" : L"km/h"; }
const wchar_t* UnitsDistance(ClusterUnits u) { return u == ClusterUnits::kImperial ? L"mi" : L"km"; }

constexpr int kSegments = 11;  // $90 bargraph bars #1..#11

}  // namespace

std::wstring FormatClusterSpeed(const std::optional<DashPcmStatus>& pcm, ClusterUnits units) {
    wchar_t buf[32];
    if (!pcm) {
        std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"--- %ls", UnitsSpeed(units));
        return buf;
    }
    const unsigned v = units == ClusterUnits::kImperial
                           ? KmToMilesRounded(static_cast<double>(pcm->speed_kph))
                           : static_cast<unsigned>(pcm->speed_kph);
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"%u %ls", v, UnitsSpeed(units));
    return buf;
}

std::wstring FormatClusterGear(const std::optional<DashPcmStatus>& pcm) {
    if (!pcm) return L"-";
    switch (pcm->gear) {
        case DashGear::kPark:    return L"P";
        case DashGear::kReverse: return L"R";
        case DashGear::kNeutral: return L"N";
        case DashGear::kDrive:   return L"D";
        default:                 return L"-";
    }
}

std::wstring FormatClusterSoc(const std::optional<DashBpmDisplay>& display) {
    if (!display) return L"SOC ---";
    wchar_t buf[32];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"SOC %u%%",
                  static_cast<unsigned>(display->user_soc_pct));
    return buf;
}

std::wstring FormatClusterRange(const std::optional<DashBpmDisplay>& display, ClusterUnits units) {
    if (!display) return L"Range ---";
    const unsigned v = units == ClusterUnits::kImperial
                           ? KmToMilesRounded(static_cast<double>(display->range_km))
                           : static_cast<unsigned>(display->range_km);
    wchar_t buf[32];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"Range %u %ls", v, UnitsDistance(units));
    return buf;
}

std::wstring FormatClusterPowerBar(const std::optional<std::uint8_t>& bars) {
    if (!bars) return L"Power ---";
    const int lit = *bars > kSegments ? kSegments : static_cast<int>(*bars);
    std::wstring s = L"Power [";
    s.append(static_cast<std::size_t>(lit), L'#');
    s.append(static_cast<std::size_t>(kSegments - lit), L'.');
    s += L']';
    return s;
}

std::wstring FormatClusterPack(const std::optional<DashPackStatus>& pack) {
    if (!pack) return L"Pack ---";
    wchar_t buf[48];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"Pack %u V  %.1f A",
                  static_cast<unsigned>(pack->pack_voltage_v),
                  static_cast<double>(pack->pack_current_a));
    return buf;
}

std::wstring FormatClusterCharge(const std::optional<DashPackStatus>& pack,
                                 const std::optional<DashBpmDisplay>& display) {
    if (!pack || !pack->charger_connected) return L"";
    if (!display) return L"Charging: --:--";
    wchar_t buf[48];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"Charging: full in %u:%02u",
                  static_cast<unsigned>(display->ttf_hours),
                  static_cast<unsigned>(display->ttf_minutes));
    return buf;
}

std::wstring FormatClusterOdometer(const std::optional<DashOdometer>& odo, ClusterUnits units) {
    if (!odo) return L"ODO ---";
    if (odo->odometer_fault) return L"ODO FAULT";
    double v = static_cast<double>(odo->season_odometer_dkm) / 10.0;  // km
    if (units == ClusterUnits::kImperial) v /= kKmPerMile;
    wchar_t buf[48];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"ODO %.1f %ls", v, UnitsDistance(units));
    return buf;
}

std::wstring FormatClusterBusLine(const DashSnapshot& snap) {
    const bool bpm = snap.pack || snap.display || snap.power_bars;
    std::wstring s = L"Bus:";
    s += snap.pcm ? L" PIM ok " : L" PIM -- ";
    s += bpm ? L" BPM ok " : L" BPM -- ";
    s += snap.brake ? L" BTCM ok " : L" BTCM -- ";
    s += snap.odometer ? L" IPC ok" : L" IPC --";
    return s;
}

ClusterView BuildClusterView(const DashSnapshot& snap, ClusterUnits units) {
    ClusterView v;
    v.speed     = FormatClusterSpeed(snap.pcm, units);
    v.gear      = FormatClusterGear(snap.pcm);
    v.soc       = FormatClusterSoc(snap.display);
    v.range     = FormatClusterRange(snap.display, units);
    v.power_bar = FormatClusterPowerBar(snap.power_bars);
    v.pack      = FormatClusterPack(snap.pack);
    v.charge    = FormatClusterCharge(snap.pack, snap.display);
    v.odometer  = FormatClusterOdometer(snap.odometer, units);
    v.bus       = FormatClusterBusLine(snap);

    const DashLamps none;
    const DashLamps& l = snap.lamps ? *snap.lamps : none;
    auto lamp = [](const wchar_t* name, const std::optional<bool>& cell) {
        return ClusterLamp{name, cell.has_value(), cell.value_or(false)};
    };
    v.lamps = {
        lamp(L"BRAKE", l.brake),
        lamp(L"PARK BRAKE", l.park_brake),
        lamp(L"ANTI-LOCK", l.antilock),
        lamp(L"LOW TRAC", l.low_traction),
        lamp(L"AIRBAG", l.airbag),
        lamp(L"SEAT BELT D", l.seatbelt_driver),
        lamp(L"SEAT BELT P", l.seatbelt_passenger),
        lamp(L"SERVICE NOW", l.service_now),
        lamp(L"CHECK MSGS", l.check_messages),
        lamp(L"TEMP", l.temp),
        lamp(L"BATTERY LIFE", l.battery_life),
        lamp(L"REDUCED PERF", l.reduced_perf),
        lamp(L"CHECK TIRE", l.check_tire),
    };
    return v;
}

// --- Panel -----------------------------------------------------------------

namespace {

// Layout constants (pixels). @design 2026-10-13 claude — HUD layout only.
constexpr int kPad = 8;
constexpr int kRowH = 16;
constexpr int kBigRowH = 28;
constexpr int kLampCols = 3;
constexpr int kLampH = 16;
constexpr int kLampGap = 3;

// Palm-dashboard flavoured palette: dark glass, teal text, amber lamps.
const irr::video::SColor kBg(200, 12, 20, 24);
const irr::video::SColor kText(255, 120, 220, 200);
const irr::video::SColor kDimText(255, 70, 110, 105);
const irr::video::SColor kLampLit(255, 255, 176, 0);
const irr::video::SColor kLampOff(255, 40, 52, 56);
const irr::video::SColor kLampUnknown(255, 26, 32, 34);
const irr::video::SColor kLampLitText(255, 20, 16, 0);

int LampRows(const ClusterView& v) {
    const int n = static_cast<int>(v.lamps.size());
    return (n + kLampCols - 1) / kLampCols;
}

int TextRows(const ClusterView& v) {
    // soc+range, power bar, pack, [charge], odometer, bus
    return 5 + (v.charge.empty() ? 0 : 1);
}

}  // namespace

InstrumentClusterPanel::InstrumentClusterPanel(ClusterUnits units, int anchor_x,
                                               int anchor_y, int width)
    : units_(units), x_(anchor_x), y_(anchor_y), w_(width),
      view_(BuildClusterView(DashSnapshot{}, units)) {}

void InstrumentClusterPanel::Update(const DashSnapshot& snap) {
    view_ = BuildClusterView(snap, units_);
}

int InstrumentClusterPanel::Height() const {
    return kPad + kBigRowH + TextRows(view_) * kRowH + kPad +
           LampRows(view_) * (kLampH + kLampGap) + kPad;
}

void InstrumentClusterPanel::Draw(irr::video::IVideoDriver* driver,
                                  irr::gui::IGUIFont* font) const {
    using irr::core::recti;
    if (!visible_ || driver == nullptr) return;

    driver->draw2DRectangle(kBg, recti(x_, y_, x_ + w_, y_ + Height()));

    auto text = [&](const std::wstring& s, int x, int y, int w, int h,
                    irr::video::SColor c, bool center = false) {
        if (font == nullptr || s.empty()) return;
        font->draw(s.c_str(), recti(x, y, x + w, y + h), c, center, true);
    };

    const int inner = w_ - 2 * kPad;
    int y = y_ + kPad;

    // Speed + gear: the big row. The gear sits right-aligned in its own cell.
    const int gear_w = 32;
    text(view_.speed, x_ + kPad, y, inner - gear_w, kBigRowH, kText);
    text(view_.gear, x_ + w_ - kPad - gear_w, y, gear_w, kBigRowH, kText, true);
    y += kBigRowH;

    const int half = inner / 2;
    text(view_.soc, x_ + kPad, y, half, kRowH, kText);
    text(view_.range, x_ + kPad + half, y, half, kRowH, kText);
    y += kRowH;
    text(view_.power_bar, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    text(view_.pack, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    if (!view_.charge.empty()) {
        text(view_.charge, x_ + kPad, y, inner, kRowH, kText);
        y += kRowH;
    }
    text(view_.odometer, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    text(view_.bus, x_ + kPad, y, inner, kRowH, kDimText);
    y += kRowH + kPad;

    const int lamp_w = (inner - (kLampCols - 1) * kLampGap) / kLampCols;
    for (std::size_t i = 0; i < view_.lamps.size(); ++i) {
        const ClusterLamp& lamp = view_.lamps[i];
        const int col = static_cast<int>(i) % kLampCols;
        const int row = static_cast<int>(i) / kLampCols;
        const int lx = x_ + kPad + col * (lamp_w + kLampGap);
        const int ly = y + row * (kLampH + kLampGap);
        const irr::video::SColor fill =
            !lamp.known ? kLampUnknown : (lamp.lit ? kLampLit : kLampOff);
        driver->draw2DRectangle(fill, recti(lx, ly, lx + lamp_w, ly + kLampH));
        const irr::video::SColor ink =
            lamp.known && lamp.lit ? kLampLitText : kDimText;
        text(lamp.name, lx, ly, lamp_w, kLampH, ink, true);
    }
}

}  // namespace ev1sim
