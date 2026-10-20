#include "RsaConsoleInput.h"

namespace ev1sim {

namespace {

bool IsKey(RsaConsoleControl c) {
    return c >= RsaConsoleControl::kKey1 && c <= RsaConsoleControl::kKey5;
}

}  // namespace

std::optional<RsaConsoleAction> RsaConsolePointer::Press(RsaConsoleControl control,
                                                         double t_s) {
    held_ = control;
    pressed_at_s_ = t_s;
    if (control == RsaConsoleControl::kNone || IsKey(control)) return std::nullopt;
    return RsaConsoleAction{control, false};
}

std::optional<RsaConsoleAction> RsaConsolePointer::Release(double t_s) {
    const RsaConsoleControl was = held_;
    held_ = RsaConsoleControl::kNone;
    if (!IsKey(was)) return std::nullopt;
    return RsaConsoleAction{was, t_s - pressed_at_s_ >= kRsaKeypadLongPressS};
}

bool ApplyRsaConsoleAction(PhysicalWorld& world, const RsaConsoleAction& action) {
    using C = RsaConsoleControl;
    using P = PrndSelector::Position;
    RsaKeypadDriver& keypad = world.rsa_keypad();
    switch (action.control) {
        case C::kNone: return false;
        case C::kOffAcc: keypad.press_mode_button(RsaKeypadDriver::kModeSwOffAcc); return false;
        case C::kRun:    keypad.press_mode_button(RsaKeypadDriver::kModeSwRun);    return false;
        case C::kLock:   keypad.press_mode_button(RsaKeypadDriver::kModeSwLock);   return false;
        case C::kPark:    world.prnd_selector().set_position(P::P); return false;
        case C::kReverse: world.prnd_selector().set_position(P::R); return false;
        case C::kNeutral: world.prnd_selector().set_position(P::N); return false;
        case C::kDrive:   world.prnd_selector().set_position(P::D); return false;
        case C::kCoastDown: world.coast_down_button().toggle(); return true;
        case C::kKey1: case C::kKey2: case C::kKey3: case C::kKey4: case C::kKey5:
            keypad.press_digit(static_cast<int>(action.control) - static_cast<int>(C::kKey1),
                               action.long_press);
            return false;
    }
    return false;
}

RsaConsoleMouse::RsaConsoleMouse(std::function<bool()> enabled,
                                 std::function<RsaConsoleControl(int, int)> hit,
                                 std::function<double()> now_s)
    : enabled_(std::move(enabled)), hit_(std::move(hit)), now_s_(std::move(now_s)) {}

bool RsaConsoleMouse::OnEvent(const irr::SEvent& event) {
    if (event.EventType != irr::EET_MOUSE_INPUT_EVENT) return false;
    const auto& m = event.MouseInput;
    if (m.Event == irr::EMIE_LMOUSE_PRESSED_DOWN) {
        if (!enabled_ || !enabled_() || !hit_) return false;
        const RsaConsoleControl c = hit_(m.X, m.Y);
        if (c == RsaConsoleControl::kNone) return false;  // not ours: pass it on
        if (auto a = pointer_.Press(c, now_s_ ? now_s_() : 0.0)) actions_.push_back(*a);
        return true;
    }
    if (m.Event == irr::EMIE_LMOUSE_LEFT_UP) {
        if (auto a = pointer_.Release(now_s_ ? now_s_() : 0.0)) {
            actions_.push_back(*a);
            return true;
        }
    }
    return false;
}

std::vector<RsaConsoleAction> RsaConsoleMouse::TakeActions() {
    std::vector<RsaConsoleAction> out;
    out.swap(actions_);
    return out;
}

}  // namespace ev1sim
