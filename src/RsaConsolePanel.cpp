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

RsaLed Led(const wchar_t* name, const std::optional<bool>& cell, RsaLedColor color) {
    return RsaLed{name, cell.has_value(), cell.value_or(false), color};
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

std::wstring FormatRsaBacklight(const std::optional<std::uint8_t>& dim_step) {
    if (!dim_step) return L"Backlight ---";
    // The RSA's interior-dimmer step is 0..10; shown as a percentage the way
    // the external sim's wire-mode panel scales it (step * 10).
    const unsigned pct = std::min<unsigned>(*dim_step, 10u) * 10u;
    wchar_t buf[32];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"Backlight %u%%", pct);
    return buf;
}

RsaConsoleView BuildRsaConsoleView(const std::optional<DashRsaConsole>& rsa) {
    const DashRsaConsole none;
    const DashRsaConsole& c = rsa ? *rsa : none;

    // OFF/ACC = ACC asserted and not RUN (RUN wins), known once both cells are.
    std::optional<bool> off_acc;
    if (c.acc && c.run) off_acc = *c.acc && !*c.run;

    RsaConsoleView v;
    v.mode = {
        Led(L"OFF/ACC", off_acc, RsaLedColor::kGreen),
        Led(L"RUN", c.run, RsaLedColor::kGreen),
    };
    v.selector = {
        Led(L"P", c.led_park, RsaLedColor::kGreen),
        Led(L"R", c.led_reverse, RsaLedColor::kGreen),
        Led(L"N", c.led_neutral, RsaLedColor::kGreen),
        Led(L"D", c.led_drive, RsaLedColor::kGreen),
    };
    // Keypad indicators are amber, as on the external sim's panel UI.
    v.keypad = {
        Led(L"VALID CODE", c.valid_code, RsaLedColor::kAmber),
        Led(L"KEYPAD", c.valid_code, RsaLedColor::kAmber),
    };
    v.security   = FormatRsaSecurity(c.security);
    v.shift_lock = FormatRsaShiftLock(c.shift_blocked);
    v.btsi       = FormatRsaBtsi(c.btsi_solenoid);
    v.backlight  = FormatRsaBacklight(c.dim_step);
    return v;
}

namespace {

// Layout constants (pixels). @design 2026-10-17 claude — HUD layout only.
constexpr int kPad = 8;
constexpr int kRowH = 16;
constexpr int kLedH = 16;
constexpr int kLedGap = 3;
constexpr int kLedRows = 3;   // mode, selector, keypad
constexpr int kTextRows = 5;  // title, security, shift lock, BTSI, backlight

// Same palette family as the cluster panel; LEDs lit green or amber.
const irr::video::SColor kBg(200, 12, 20, 24);
const irr::video::SColor kText(255, 120, 220, 200);
const irr::video::SColor kDimText(255, 70, 110, 105);
const irr::video::SColor kLedGreen(255, 60, 220, 90);
const irr::video::SColor kLedAmber(255, 255, 176, 0);
const irr::video::SColor kLedOff(255, 40, 52, 56);
const irr::video::SColor kLedUnknown(255, 26, 32, 34);
const irr::video::SColor kLedLitText(255, 20, 16, 0);

}  // namespace

RsaConsolePanel::RsaConsolePanel(int anchor_x, int anchor_y, int width)
    : x_(anchor_x), y_(anchor_y), w_(width), view_(BuildRsaConsoleView(std::nullopt)) {}

void RsaConsolePanel::Update(const DashSnapshot& snap) {
    view_ = BuildRsaConsoleView(snap.rsa);
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
    y += kRowH;

    // One row of LEDs, sharing the row width equally.
    auto led_row = [&](const std::vector<RsaLed>& leds) {
        const int n = static_cast<int>(leds.size());
        const int led_w = (inner - (n - 1) * kLedGap) / n;
        for (int i = 0; i < n; ++i) {
            const RsaLed& led = leds[static_cast<std::size_t>(i)];
            const int lx = x_ + kPad + i * (led_w + kLedGap);
            const irr::video::SColor lit =
                led.color == RsaLedColor::kAmber ? kLedAmber : kLedGreen;
            const irr::video::SColor fill =
                !led.known ? kLedUnknown : (led.lit ? lit : kLedOff);
            driver->draw2DRectangle(fill, recti(lx, y, lx + led_w, y + kLedH));
            const irr::video::SColor ink = led.known && led.lit ? kLedLitText : kDimText;
            text(led.name, lx, y, led_w, kLedH, ink, true);
        }
        y += kLedH + kLedGap;
    };
    led_row(view_.mode);
    led_row(view_.selector);
    led_row(view_.keypad);
    y += kPad - kLedGap;

    text(view_.security, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    text(view_.shift_lock, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    text(view_.btsi, x_ + kPad, y, inner, kRowH, kText);
    y += kRowH;
    text(view_.backlight, x_ + kPad, y, inner, kRowH, kText);
}

}  // namespace ev1sim
