// RsaConsolePanel — the RSA center console HUD panel (docs/ipc_rsa_display_plan.md
// Phase 2): pure label helpers, the RSA cells → LED view model (the external
// sim's wire-mode panel derivations), and a null-driver draw.

#include <catch2/catch_test_macros.hpp>

#include "RsaConsolePanel.h"

#include <irrlicht.h>

#include <cstdint>
#include <string>

using namespace ev1sim;

TEST_CASE("RSA console labels: written cells read ON/off, unwritten read ---", "[rsa_console]") {
    REQUIRE(FormatRsaSecurity(std::nullopt) == L"Security: ---");
    REQUIRE(FormatRsaSecurity(true) == L"Security: ON");
    REQUIRE(FormatRsaSecurity(false) == L"Security: off");
    REQUIRE(FormatRsaShiftLock(true) == L"Shift lock: BLOCKED");
    REQUIRE(FormatRsaShiftLock(false) == L"Shift lock: free");
    REQUIRE(FormatRsaShiftLock(std::nullopt) == L"Shift lock: ---");
    REQUIRE(FormatRsaBtsi(true) == L"BTSI solenoid: ENERGIZED");
    REQUIRE(FormatRsaBtsi(std::nullopt) == L"BTSI solenoid: ---");
}

TEST_CASE("RSA console backlight: dimmer step 0..10 as a percentage", "[rsa_console]") {
    REQUIRE(FormatRsaBacklight(std::nullopt) == L"Backlight ---");
    REQUIRE(FormatRsaBacklight(std::uint8_t{0}) == L"Backlight 0%");
    REQUIRE(FormatRsaBacklight(std::uint8_t{7}) == L"Backlight 70%");
    REQUIRE(FormatRsaBacklight(std::uint8_t{200}) == L"Backlight 100%");  // clamped
}

TEST_CASE("RSA console view: every LED unknown before the RSA writes", "[rsa_console]") {
    const RsaConsoleView v = BuildRsaConsoleView(std::nullopt);
    REQUIRE(v.mode.size() == 2u);
    REQUIRE(v.selector.size() == 4u);
    REQUIRE(v.keypad.size() == 2u);
    for (const auto* row : {&v.mode, &v.selector, &v.keypad}) {
        for (const RsaLed& led : *row) REQUIRE_FALSE(led.known);
    }
    // COAST DOWN's lamp waits on the PCM; LOCK and the keys have no lamp.
    REQUIRE(v.buttons.size() == 2u);
    REQUIRE_FALSE(v.buttons[0].lamp);
    REQUIRE(v.buttons[1].lamp);
    REQUIRE_FALSE(v.buttons[1].known);
    REQUIRE(v.digits.size() == 5u);
    for (const RsaLed& key : v.digits) REQUIRE_FALSE(key.lamp);
    REQUIRE(v.backlight == L"Backlight ---");
}

TEST_CASE("RSA console COAST DOWN lamp is the PCM's $41 LED request", "[rsa_console]") {
    REQUIRE(BuildRsaConsoleView(std::nullopt, true).buttons[1].lit);
    REQUIRE_FALSE(BuildRsaConsoleView(std::nullopt, false).buttons[1].lit);
    REQUIRE(BuildRsaConsoleView(std::nullopt, false).buttons[1].known);

    // Through the panel: the snapshot's $41 group feeds it.
    RsaConsolePanel panel;
    DashSnapshot snap;
    snap.pcm = DashPcmStatus{};
    snap.pcm->coast_down_led = true;
    panel.Update(snap);
    REQUIRE(panel.view().buttons[1].lit);
}

TEST_CASE("RSA console hit test finds each control where it is drawn", "[rsa_console]") {
    RsaConsolePanel panel(100, 50, 220);
    const auto cells = LayoutRsaConsole(panel.view(), 100, 50, 220);
    REQUIRE(cells.size() == 2u + 4u + 2u + 2u + 5u);
    for (const RsaConsoleCell& c : cells) {
        const int cx = (c.x0 + c.x1) / 2;
        const int cy = (c.y0 + c.y1) / 2;
        REQUIRE(panel.HitTest(cx, cy) == c.led->control);
        // Every cell lies inside the panel box Draw() fills.
        REQUIRE(c.x0 >= 100);
        REQUIRE(c.x1 <= 100 + 220);
        REQUIRE(c.y1 <= 50 + panel.Height());
    }
    // Spot checks by name: the rows run mode, selector, lamps, buttons, keys.
    REQUIRE(cells[1].led->control == RsaConsoleControl::kRun);
    REQUIRE(cells[5].led->control == RsaConsoleControl::kDrive);
    REQUIRE(cells[6].led->control == RsaConsoleControl::kNone);  // VALID CODE lamp
    REQUIRE(cells[9].led->control == RsaConsoleControl::kCoastDown);
    REQUIRE(cells[14].led->control == RsaConsoleControl::kKey5);

    // Outside, in a gap, or hidden: nothing.
    REQUIRE(panel.HitTest(99, cells[0].y0 + 2) == RsaConsoleControl::kNone);
    REQUIRE(panel.HitTest(cells[0].x1 + 1, cells[0].y0 + 2) == RsaConsoleControl::kNone);
    REQUIRE(panel.HitTest(cells[0].x0 + 2, cells[0].y1 + 1) == RsaConsoleControl::kNone);
    panel.SetVisible(false);
    REQUIRE(panel.HitTest(cells[1].x0 + 2, cells[1].y0 + 2) == RsaConsoleControl::kNone);
}

TEST_CASE("RSA console mode LEDs: RUN wins over ACC", "[rsa_console]") {
    DashRsaConsole c;
    c.acc = true;
    c.run = false;
    RsaConsoleView v = BuildRsaConsoleView(c);
    REQUIRE(std::wstring(v.mode[0].name) == L"OFF/ACC");
    REQUIRE(v.mode[0].known);
    REQUIRE(v.mode[0].lit);        // ACC without RUN
    REQUIRE_FALSE(v.mode[1].lit);  // RUN dark

    c.run = true;
    v = BuildRsaConsoleView(c);
    REQUIRE_FALSE(v.mode[0].lit);  // RUN suppresses OFF/ACC
    REQUIRE(v.mode[1].lit);

    // OFF/ACC derives from both cells, so it stays unknown until both are written.
    DashRsaConsole acc_only;
    acc_only.acc = true;
    REQUIRE_FALSE(BuildRsaConsoleView(acc_only).mode[0].known);
}

TEST_CASE("RSA console selector and keypad LEDs follow their cells", "[rsa_console]") {
    DashRsaConsole c;
    c.led_park = false;
    c.led_drive = true;
    c.valid_code = true;
    const RsaConsoleView v = BuildRsaConsoleView(c);
    REQUIRE(std::wstring(v.selector[3].name) == L"D");
    REQUIRE(v.selector[3].lit);
    REQUIRE(v.selector[0].known);
    REQUIRE_FALSE(v.selector[0].lit);
    REQUIRE_FALSE(v.selector[1].known);  // REVERSE never written
    // VALID CODE lights the exterior keypad lamp too, both amber.
    REQUIRE(v.keypad[0].lit);
    REQUIRE(v.keypad[1].lit);
    REQUIRE(v.keypad[1].color == RsaLedColor::kAmber);
}

TEST_CASE("RSA console panel: updates from a snapshot; draws on a null driver", "[rsa_console]") {
    RsaConsolePanel panel(10, 10, 220);
    // Box ends one pad below the last text row: 3 pads, 5 text rows, 5 LED rows
    // with the gap between them (8*3 + 16*5 + 16*5 + 3*4).
    REQUIRE(panel.Height() == 196);

    DashSnapshot snap;
    snap.rsa = DashRsaConsole{};
    snap.rsa->shift_blocked = true;
    snap.rsa->dim_step = std::uint8_t{5};
    panel.Update(snap);
    REQUIRE(panel.view().shift_lock == L"Shift lock: BLOCKED");
    REQUIRE(panel.view().backlight == L"Backlight 50%");

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
