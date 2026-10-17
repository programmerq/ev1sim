#pragma once

#include "DashSnapshot.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// RsaConsolePanel — the RSA center console as a 2D HUD panel.
//
// Phase 2 of docs/ipc_rsa_display_plan.md: display-only, fed by the RSA output
// cells in DashSnapshot::rsa. It shows the console's indicators that the
// external sim actually drives, using the same derivations as its wire-mode RSA
// panel (examples/ev1_rsa_panel/wire_panel_sink.cpp):
//
//   RUN LED      = RSA_RUN1_OUT
//   OFF/ACC LED  = CHASSIS_RSA_ACC_OUT and not RUN
//   VALID CODE   = CHASSIS_RSA_VALID_CODE (also lights the exterior KEYPAD lamp)
//   P R N D      = the four CHASSIS_RSA_SELECTOR_LED_* cells
//
// The HVAC, fan, clock and trip LEDs in rsa_panel_catalog.yaml are marked
// "stub" there (no RSA behaviour modelled), so the panel leaves them out rather
// than draw a state nobody produces.
//
// Same split as InstrumentClusterPanel: pure Format* / BuildRsaConsoleView
// helpers (no Irrlicht) and a panel class that only lays out and draws.
// ---------------------------------------------------------------------------

namespace irr {
namespace video { class IVideoDriver; }
namespace gui { class IGUIFont; }
}  // namespace irr

namespace ev1sim {

// --- Pure label helpers ----------------------------------------------------

/// "Security: ON" / "Security: off", or "Security: ---" before the cell is written.
std::wstring FormatRsaSecurity(const std::optional<bool>& security);
/// "Shift lock: BLOCKED" / "Shift lock: free" / "Shift lock: ---".
std::wstring FormatRsaShiftLock(const std::optional<bool>& shift_blocked);
/// "BTSI solenoid: ENERGIZED" / "BTSI solenoid: off" / "BTSI solenoid: ---".
std::wstring FormatRsaBtsi(const std::optional<bool>& btsi_solenoid);
/// "Backlight 70%" from the 0..10 dimmer step (clamped), or "Backlight ---".
std::wstring FormatRsaBacklight(const std::optional<std::uint8_t>& dim_step);

// --- View model ------------------------------------------------------------

enum class RsaLedColor : std::uint8_t { kGreen, kAmber };

struct RsaLed {
    const wchar_t* name;  // static label, e.g. L"RUN"
    bool known;           // every cell the LED derives from has been written
    bool lit;             // meaningful only when known
    RsaLedColor color;
};

struct RsaConsoleView {
    std::vector<RsaLed> mode;      // OFF/ACC, RUN
    std::vector<RsaLed> selector;  // P, R, N, D
    std::vector<RsaLed> keypad;    // VALID CODE, KEYPAD (exterior)
    std::wstring security;
    std::wstring shift_lock;
    std::wstring btsi;
    std::wstring backlight;
};

/// Everything the panel shows, derived from the snapshot's RSA group. Pure.
RsaConsoleView BuildRsaConsoleView(const std::optional<DashRsaConsole>& rsa);

// --- Panel -----------------------------------------------------------------

class RsaConsolePanel {
public:
    explicit RsaConsolePanel(int anchor_x = 10, int anchor_y = 10, int width = 220);

    void SetVisible(bool visible) { visible_ = visible; }
    void SetAnchor(int x, int y) { x_ = x; y_ = y; }
    int Width() const { return w_; }
    bool IsVisible() const { return visible_; }

    void Update(const DashSnapshot& snap);
    const RsaConsoleView& view() const { return view_; }

    // Paint the latest view. No-op when hidden or `driver` is null; a null
    // `font` draws the panel and LED boxes without text.
    void Draw(irr::video::IVideoDriver* driver, irr::gui::IGUIFont* font) const;

    // Fixed height (the layout has no optional rows).
    int Height() const;

private:
    int x_;
    int y_;
    int w_;
    bool visible_ = true;
    RsaConsoleView view_;
};

}  // namespace ev1sim
