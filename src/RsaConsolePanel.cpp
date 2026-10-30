#include "RsaConsolePanel.h"

#include <irrlicht.h>

#include <algorithm>
#include <cstdio>
#include <cwchar>

namespace ev1sim {

namespace {

std::wstring Labelled(const wchar_t* label, const std::optional<bool>& v,
                      const wchar_t* on, const wchar_t* off) {
    std::wstring s = label;
    s += !v ? L"---" : (*v ? on : off);
    return s;
}

RsaLed Led(const wchar_t* name, const std::optional<bool>& cell, RsaLedColor color,
           RsaConsoleControl control = RsaConsoleControl::kNone) {
    return RsaLed{name, cell.has_value(), cell.value_or(false), color, control, true};
}

RsaLed Button(const wchar_t* name, RsaConsoleControl control) {
    return RsaLed{name, true, false, RsaLedColor::kGreen, control, false};
}

}  // namespace

std::wstring FormatRsaSecurity(const std::optional<bool>& security) {
    return Labelled(L"Security: ", security, L"ON", L"off");
}

std::wstring FormatRsaShiftLock(const std::optional<bool>& shift_blocked) {
    return Labelled(L"Shift lock: ", shift_blocked, L"BLOCKED", L"free");
}

std::wstring FormatRsaBtsi(const std::optional<bool>& btsi_solenoid) {
    return Labelled(L"BTSI solenoid: ", btsi_solenoid, L"ENERGIZED", L"off");
}

std::wstring FormatRsaBacklight(const std::optional<std::uint16_t>& dim_level_permille) {
    if (!dim_level_permille) return L"Backlight ---";
    // The RSA's DIM LEVEL OUT is permille of 12 V; shown in percent the way the
    // external sim's wire-mode panel scales it (permille / 10).
    const unsigned pct = std::min<unsigned>(*dim_level_permille, 1000u) / 10u;
    wchar_t buf[32];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"Backlight %u%%", pct);
    return buf;
}

std::wstring FormatRsaDoorLocks(const std::optional<std::uint8_t>& driver,
                                const std::optional<std::uint8_t>& passenger) {
    if (!driver || !passenger) return L"Door locks: ---";
    const bool d = *driver != 0u;
    const bool p = *passenger != 0u;
    if (d == p) return d ? L"Door locks: LOCKED" : L"Door locks: unlocked";
    std::wstring s = L"Door locks: DRV ";
    s += d ? L"locked" : L"unlocked";
    s += L", PAS ";
    s += p ? L"locked" : L"unlocked";
    return s;
}

RsaConsoleView BuildRsaConsoleView(const std::optional<DashRsaConsole>& rsa,
                                   const std::optional<bool>& coast_down_led) {
    const DashRsaConsole none;
    const DashRsaConsole& c = rsa ? *rsa : none;

    // OFF/ACC = ACC asserted and not RUN (RUN wins), known once both cells are.
    std::optional<bool> off_acc;
    if (c.acc && c.run) off_acc = *c.acc && !*c.run;

    RsaConsoleView v;
    v.mode = {
        Led(L"OFF/ACC", off_acc, RsaLedColor::kGreen, RsaConsoleControl::kOffAcc),
        Led(L"RUN", c.run, RsaLedColor::kGreen, RsaConsoleControl::kRun),
    };
    v.selector = {
        Led(L"P", c.led_park, RsaLedColor::kGreen, RsaConsoleControl::kPark),
        Led(L"R", c.led_reverse, RsaLedColor::kGreen, RsaConsoleControl::kReverse),
        Led(L"N", c.led_neutral, RsaLedColor::kGreen, RsaConsoleControl::kNeutral),
        Led(L"D", c.led_drive, RsaLedColor::kGreen, RsaConsoleControl::kDrive),
    };
    // Keypad indicators are amber, as on the external sim's panel UI.
    v.keypad = {
        Led(L"VALID CODE", c.valid_code, RsaLedColor::kAmber),
        Led(L"KEYPAD", c.valid_code, RsaLedColor::kAmber),
    };
    v.buttons = {
        Button(L"LOCK", RsaConsoleControl::kLock),
        Led(L"COAST DOWN", coast_down_led, RsaLedColor::kAmber, RsaConsoleControl::kCoastDown),
    };
    v.dimmer = {
        Button(L"DIM -", RsaConsoleControl::kDimDown),
        Button(L"DIM +", RsaConsoleControl::kDimUp),
    };
    // Keycaps as on the console keypad; a long press enters the higher digit.
    v.digits = {
        Button(L"1/2", RsaConsoleControl::kKey1), Button(L"3/4", RsaConsoleControl::kKey2),
        Button(L"5/6", RsaConsoleControl::kKey3), Button(L"7/8", RsaConsoleControl::kKey4),
        Button(L"9/0", RsaConsoleControl::kKey5),
    };
    v.security   = FormatRsaSecurity(c.security);
    v.shift_lock = FormatRsaShiftLock(c.shift_blocked);
    v.btsi       = FormatRsaBtsi(c.btsi_solenoid);
    v.backlight  = FormatRsaBacklight(c.dim_level_permille);
    v.door_locks = FormatRsaDoorLocks(c.lock_driver, c.lock_passenger);
    return v;
}

namespace {

// Layout constants (pixels). @design 2026-10-17 claude — HUD layout only.
constexpr int kPad = 8;
constexpr int kRowH = 16;
constexpr int kLedH = 16;
constexpr int kLedGap = 3;
constexpr int kLedRows = 6;   // mode, selector, keypad lamps, buttons, dimmer, digits
constexpr int kTextRows = 6;  // title, security, shift lock, BTSI, backlight, door locks

// Same palette family as the cluster panel; LEDs lit green or amber.
const irr::video::SColor kBg(200, 12, 20, 24);
const irr::video::SColor kText(255, 120, 220, 200);
const irr::video::SColor kDimText(255, 70, 110, 105);
const irr::video::SColor kLedGreen(255, 60, 220, 90);
const irr::video::SColor kLedAmber(255, 255, 176, 0);
const irr::video::SColor kLedOff(255, 40, 52, 56);
const irr::video::SColor kLedUnknown(255, 26, 32, 34);
const irr::video::SColor kLedLitText(255, 20, 16, 0);
const irr::video::SColor kButton(255, 58, 72, 78);  // a key with no lamp

}  // namespace

std::vector<RsaConsoleCell> LayoutRsaConsole(const RsaConsoleView& view, int x, int y,
                                             int w) {
    std::vector<RsaConsoleCell> cells;
    const int inner = w - 2 * kPad;
    int row_y = y + kPad + kRowH;  // below the title row
    // One row of LEDs, sharing the row width equally.
    auto row = [&](const std::vector<RsaLed>& leds) {
        const int n = static_cast<int>(leds.size());
        if (n == 0) return;
        const int led_w = (inner - (n - 1) * kLedGap) / n;
        for (int i = 0; i < n; ++i) {
            const int lx = x + kPad + i * (led_w + kLedGap);
            cells.push_back({&leds[static_cast<std::size_t>(i)], lx, row_y, lx + led_w,
                             row_y + kLedH});
        }
        row_y += kLedH + kLedGap;
    };
    row(view.mode);
    row(view.selector);
    row(view.keypad);
    row(view.buttons);
    row(view.dimmer);
    row(view.digits);
    return cells;
}

RsaConsolePanel::RsaConsolePanel(int anchor_x, int anchor_y, int width)
    : x_(anchor_x), y_(anchor_y), w_(width), view_(BuildRsaConsoleView(std::nullopt)) {}

void RsaConsolePanel::Update(const DashSnapshot& snap) {
    std::optional<bool> coast_led;
    if (snap.pcm) coast_led = snap.pcm->coast_down_led;
    view_ = BuildRsaConsoleView(snap.rsa, coast_led);
}

RsaConsoleControl RsaConsolePanel::HitTest(int px, int py) const {
    if (!visible_) return RsaConsoleControl::kNone;
    for (const RsaConsoleCell& c : LayoutRsaConsole(view_, x_, y_, w_)) {
        if (px >= c.x0 && px < c.x1 && py >= c.y0 && py < c.y1) return c.led->control;
    }
    return RsaConsoleControl::kNone;
}

int RsaConsolePanel::Height() const {
    // The last LED row's gap folds into the pad before the text rows (Draw).
    return kPad + kTextRows * kRowH + kPad + kLedRows * (kLedH + kLedGap) - kLedGap + kPad;
}

void RsaConsolePanel::Draw(irr::video::IVideoDriver* driver,
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

    text(L"RSA", x_ + kPad, y, inner, kRowH, kDimText);

    int last_y1 = y + kRowH;
    for (const RsaConsoleCell& c : LayoutRsaConsole(view_, x_, y_, w_)) {
        const RsaLed& led = *c.led;
        const irr::video::SColor lit =
            led.color == RsaLedColor::kAmber ? kLedAmber : kLedGreen;
        const irr::video::SColor fill =
            !led.lamp ? kButton : (!led.known ? kLedUnknown : (led.lit ? lit : kLedOff));
        driver->draw2DRectangle(fill, recti(c.x0, c.y0, c.x1, c.y1));
        const irr::video::SColor ink =
            !led.lamp ? kText : (led.known && led.lit ? kLedLitText : kDimText);
        text(led.name, c.x0, c.y0, c.x1 - c.x0, c.y1 - c.y0, ink, true);
        last_y1 = c.y1;
    }
    y = last_y1 + kPad;

    text(view_.security, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    text(view_.shift_lock, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    text(view_.btsi, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    text(view_.backlight, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    text(view_.door_locks, x_ + kPad, y, inner, kRowH, kText);
}

}  // namespace ev1sim
