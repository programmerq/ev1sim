// RsaConsoleInput — console clicks (docs/ipc_rsa_display_plan.md Phase 4):
// the press/release timing, each control applied to a real PhysicalWorld and
// read back through the producers SimApp publishes from, and the Irrlicht
// receiver fed real mouse events.

#include <catch2/catch_test_macros.hpp>

#include "PhysicalWorld.h"
#include "RsaConsoleInput.h"

#include <irrlicht.h>

using namespace ev1sim;
using C = RsaConsoleControl;

TEST_CASE("Console pointer: buttons act on press, keypad keys on release", "[rsa_console_input]") {
    RsaConsolePointer p;
    const auto run = p.Press(C::kRun, 1.0);
    REQUIRE(run);
    REQUIRE(run->control == C::kRun);
    REQUIRE_FALSE(p.Release(1.1));  // nothing more for a button

    REQUIRE_FALSE(p.Press(C::kKey3, 2.0));  // waits for the hold time
    const auto tap = p.Release(2.0 + kRsaKeypadLongPressS - 0.01);
    REQUIRE(tap);
    REQUIRE(tap->control == C::kKey3);
    REQUIRE_FALSE(tap->long_press);

    REQUIRE_FALSE(p.Press(C::kKey1, 5.0));
    const auto hold = p.Release(5.0 + kRsaKeypadLongPressS);
    REQUIRE(hold);
    REQUIRE(hold->long_press);

    REQUIRE_FALSE(p.Press(C::kNone, 6.0));
    REQUIRE_FALSE(p.Release(6.5));
}

TEST_CASE("Console mode buttons reach the keypad driver's mode switches", "[rsa_console_input]") {
    PhysicalWorld w;
    RsaKeypadDriver& k = w.rsa_keypad();
    REQUIRE_FALSE(ApplyRsaConsoleAction(w, {C::kOffAcc, false}));
    k.update(0.0);
    REQUIRE(k.consume_fires_now().mode_switches == RsaKeypadDriver::kModeSwOffAcc);
    REQUIRE(k.expected_state() == RsaKeypadDriver::ExpectedState::ACC);

    ApplyRsaConsoleAction(w, {C::kRun, false});
    k.update(0.0);
    REQUIRE(k.consume_fires_now().mode_switches == RsaKeypadDriver::kModeSwRun);
    REQUIRE(k.expected_state() == RsaKeypadDriver::ExpectedState::RUN);

    // OFF/ACC tapped from RUN: the press is sent, the RSA keeps RUN.
    ApplyRsaConsoleAction(w, {C::kOffAcc, false});
    k.update(0.0);
    REQUIRE(k.consume_fires_now().mode_switches == RsaKeypadDriver::kModeSwOffAcc);
    REQUIRE(k.expected_state() == RsaKeypadDriver::ExpectedState::RUN);

    ApplyRsaConsoleAction(w, {C::kLock, false});
    k.update(0.0);
    REQUIRE(k.consume_fires_now().mode_switches == RsaKeypadDriver::kModeSwLock);
    REQUIRE(k.expected_state() == RsaKeypadDriver::ExpectedState::OFF);

    // The K key carries on from where the console left it: OFF -> ACC.
    k.cycle_k();
    REQUIRE(k.expected_state() == RsaKeypadDriver::ExpectedState::ACC);
}

TEST_CASE("Console keypad keys send one digit; a long press the higher one", "[rsa_console_input]") {
    PhysicalWorld w;
    ApplyRsaConsoleAction(w, {C::kKey2, false});
    auto f = w.rsa_keypad().consume_fires_now();
    REQUIRE(f.button_value[1] == 1u);  // tap: lower digit (3)
    REQUIRE(f.button_value[0] == 0u);
    REQUIRE(f.mode_switches == 0u);
    REQUIRE(w.rsa_keypad().consume_fires_now().button_value[1] == 0u);  // one tick

    ApplyRsaConsoleAction(w, {C::kKey5, true});
    REQUIRE(w.rsa_keypad().consume_fires_now().button_value[4] == 2u);  // hold: 0
}

TEST_CASE("Console P R N D select the position directly, pins follow", "[rsa_console_input]") {
    PhysicalWorld w;
    using P = PrndSelector::Position;
    ApplyRsaConsoleAction(w, {C::kDrive, false});
    REQUIRE(w.prnd_selector().position() == P::D);
    PrndSelector d;  // the same position reached by the ',' '.' keys
    d.cycle_up(); d.cycle_up(); d.cycle_up();
    REQUIRE(w.prnd_selector().pin_a() == d.pin_a());
    REQUIRE(w.prnd_selector().pin_b() == d.pin_b());
    REQUIRE(w.prnd_selector().pin_c() == d.pin_c());
    REQUIRE(w.prnd_selector().pin_d() == d.pin_d());
    ApplyRsaConsoleAction(w, {C::kReverse, false});
    REQUIRE(w.prnd_selector().position() == P::R);
    ApplyRsaConsoleAction(w, {C::kPark, false});
    REQUIRE(w.prnd_selector().position() == P::P);
    ApplyRsaConsoleAction(w, {C::kNeutral, false});
    REQUIRE(w.prnd_selector().position() == P::N);
}

TEST_CASE("Console COAST DOWN toggles the button and reports the change", "[rsa_console_input]") {
    PhysicalWorld w;
    REQUIRE_FALSE(w.coast_down_button().on());
    REQUIRE(ApplyRsaConsoleAction(w, {C::kCoastDown, false}));
    REQUIRE(w.coast_down_button().on());
    REQUIRE(ApplyRsaConsoleAction(w, {C::kCoastDown, false}));
    REQUIRE_FALSE(w.coast_down_button().on());
    REQUIRE_FALSE(ApplyRsaConsoleAction(w, {C::kNone, false}));
}

TEST_CASE("Console DIM - / DIM + queue one click each", "[rsa_console_input]") {
    PhysicalWorld w;
    REQUIRE_FALSE(ApplyRsaConsoleAction(w, {C::kDimUp, false}));
    REQUIRE_FALSE(ApplyRsaConsoleAction(w, {C::kDimUp, false}));
    REQUIRE_FALSE(ApplyRsaConsoleAction(w, {C::kDimDown, false}));
    REQUIRE(w.rsa_dimmer().take(true) == 2);
    REQUIRE(w.rsa_dimmer().take(false) == 1);
    REQUIRE(w.rsa_dimmer().take(true) == 0);  // taken once
}

namespace {
irr::SEvent Mouse(irr::EMOUSE_INPUT_EVENT e, int x, int y) {
    irr::SEvent ev{};
    ev.EventType = irr::EET_MOUSE_INPUT_EVENT;
    ev.MouseInput.Event = e;
    ev.MouseInput.X = x;
    ev.MouseInput.Y = y;
    return ev;
}
}  // namespace

TEST_CASE("Console mouse: clicks on the real panel layout become actions", "[rsa_console_input]") {
    RsaConsolePanel panel(100, 50, 220);
    bool ui_mode = true;
    double now = 0.0;
    RsaConsoleMouse mouse([&]() { return ui_mode; },
                          [&](int x, int y) { return panel.HitTest(x, y); },
                          [&]() { return now; });
    const auto cells = LayoutRsaConsole(panel.view(), 100, 50, 220);
    auto centre = [&](std::size_t i, irr::EMOUSE_INPUT_EVENT e) {
        return Mouse(e, (cells[i].x0 + cells[i].x1) / 2, (cells[i].y0 + cells[i].y1) / 2);
    };

    // RUN: absorbed on press, acted on at once.
    REQUIRE(mouse.OnEvent(centre(1, irr::EMIE_LMOUSE_PRESSED_DOWN)));
    REQUIRE_FALSE(mouse.OnEvent(centre(1, irr::EMIE_LMOUSE_LEFT_UP)));
    // A keypad key held 0.6 s: the higher digit, on release.
    REQUIRE(mouse.OnEvent(centre(12, irr::EMIE_LMOUSE_PRESSED_DOWN)));
    now = 0.6;
    REQUIRE(mouse.OnEvent(centre(12, irr::EMIE_LMOUSE_LEFT_UP)));
    // A click on the VALID CODE lamp or off the panel is not the console's.
    REQUIRE_FALSE(mouse.OnEvent(centre(6, irr::EMIE_LMOUSE_PRESSED_DOWN)));
    REQUIRE_FALSE(mouse.OnEvent(Mouse(irr::EMIE_LMOUSE_PRESSED_DOWN, 5, 5)));
    // Out of UI mode the mouse belongs to the camera.
    ui_mode = false;
    REQUIRE_FALSE(mouse.OnEvent(centre(1, irr::EMIE_LMOUSE_PRESSED_DOWN)));

    const auto actions = mouse.TakeActions();
    REQUIRE(actions.size() == 2u);
    REQUIRE(actions[0].control == C::kRun);
    REQUIRE(actions[1].control == C::kKey1);
    REQUIRE(actions[1].long_press);
    REQUIRE(mouse.TakeActions().empty());
}

TEST_CASE("Console click picker: 3D screen only in the Cabin view, under no GUI widget",
          "[rsa_console_input]") {
    using C = RsaConsoleControl;
    int ray_tests = 0;
    const auto screen = [&]() { ++ray_tests; return C::kDrive; };

    // The 2D panel wins wherever it is drawn, in any view.
    REQUIRE(PickRsaConsoleControl(C::kRun, false, false, screen) == C::kRun);
    REQUIRE(ray_tests == 0);
    // Off the panel, in the Cabin view with nothing over the point: the screen.
    REQUIRE(PickRsaConsoleControl(C::kNone, true, false, screen) == C::kDrive);
    REQUIRE(ray_tests == 1);
    // Chase (or any other) view: the console quad may lie under the cursor in
    // chassis space but is not what the driver sees, so no click.
    REQUIRE(PickRsaConsoleControl(C::kNone, false, false, screen) == C::kNone);
    // A floating-UI widget over the point takes the click.
    REQUIRE(PickRsaConsoleControl(C::kNone, true, true, screen) == C::kNone);
    REQUIRE(ray_tests == 1);
    REQUIRE(PickRsaConsoleControl(C::kNone, true, false, nullptr) == C::kNone);
}
