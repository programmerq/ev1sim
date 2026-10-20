#pragma once

#include "PhysicalWorld.h"
#include "RsaConsolePanel.h"

#include <irrlicht.h>

#include <functional>
#include <optional>
#include <vector>

// ---------------------------------------------------------------------------
// RsaConsoleInput — clicks on the RSA console (docs/ipc_rsa_display_plan.md
// Phase 4) become the same driver inputs the keyboard produces.
//
// Every control goes through a PhysicalWorld component ev1sim already
// publishes from (the K-key keypad driver, the PRND selector), plus the new
// coast-down button, so the console adds no new writer to the RSA's input
// cells.
//
// Split for testing: RsaConsolePointer turns press/release times into actions
// (pure), ApplyRsaConsoleAction() applies one to a PhysicalWorld, and
// RsaConsoleMouse is the thin Irrlicht event receiver SimApp registers.
// ---------------------------------------------------------------------------

namespace ev1sim {

struct RsaConsoleAction {
    RsaConsoleControl control = RsaConsoleControl::kNone;
    bool long_press = false;  // keypad keys only: the higher digit
};

/// Hold time that makes a keypad press the higher digit. The interior keypad
/// enters the lower digit on a tap and the higher on a hold (EV1 community
/// notation, RsaKeypadDriver); the hold length is not printed.
/// @design 2026-10-20 claude: 500 ms, the threshold RsaKeypadDriver's TODO names.
inline constexpr double kRsaKeypadLongPressS = 0.5;

/// Press/release bookkeeping for one pointer. Buttons act on press; keypad
/// keys act on release, once the hold time is known. Pure.
class RsaConsolePointer {
public:
    /// The pointer went down on `control` at `t_s`. Returns the action to apply
    /// now (non-keypad controls), or nothing (keypad keys wait for release,
    /// kNone does nothing).
    std::optional<RsaConsoleAction> Press(RsaConsoleControl control, double t_s);

    /// The pointer came up at `t_s`. Returns the keypad action whose press
    /// this ends, if any.
    std::optional<RsaConsoleAction> Release(double t_s);

private:
    RsaConsoleControl held_ = RsaConsoleControl::kNone;
    double pressed_at_s_ = 0.0;
};

/// Apply one console action to the physical world. Returns true when it
/// changed the coast-down button, which the caller publishes.
bool ApplyRsaConsoleAction(PhysicalWorld& world, const RsaConsoleAction& action);

/// Irrlicht receiver: left-button press/release while `enabled()` is true,
/// hit-tested by `hit(x, y)`. Queues actions for SimApp to apply once per frame.
class RsaConsoleMouse : public irr::IEventReceiver {
public:
    RsaConsoleMouse(std::function<bool()> enabled,
                    std::function<RsaConsoleControl(int, int)> hit,
                    std::function<double()> now_s);

    bool OnEvent(const irr::SEvent& event) override;

    /// Actions since the last call, oldest first.
    std::vector<RsaConsoleAction> TakeActions();

private:
    std::function<bool()> enabled_;
    std::function<RsaConsoleControl(int, int)> hit_;
    std::function<double()> now_s_;
    RsaConsolePointer pointer_;
    std::vector<RsaConsoleAction> actions_;
};

}  // namespace ev1sim
