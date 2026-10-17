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
    REQUIRE(v.backlight == L"Backlight ---");
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
    // Box ends one pad below the last text row: 3 pads, 5 text rows, 3 LED rows
    // with the gap between them (8*3 + 16*5 + 16*3 + 3*2).
    REQUIRE(panel.Height() == 158);

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
