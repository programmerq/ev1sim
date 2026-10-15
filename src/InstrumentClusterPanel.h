#pragma once

#include "DashSnapshot.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// InstrumentClusterPanel — the IPC instrument cluster as a 2D HUD panel.
//
// Phase 1 of docs/ipc_rsa_display_plan.md: display-only, fed by a DashSnapshot
// (BusSnoop's frame decode plus the IPC lamp cells). Split the same way as
// FloatingUiPanel:
//
//   * Format* helpers and BuildClusterView() are pure (no Irrlicht), so every
//     string and lamp state the panel shows is unit-testable without a window.
//   * InstrumentClusterPanel owns layout + drawing only: it keeps the latest
//     ClusterView and paints it with the Irrlicht 2D driver each frame.
//
// A field whose frame is missing or stale shows "---"; a lamp whose cell has
// never been written is drawn dim with no lit state, never as "off".
// ---------------------------------------------------------------------------

namespace irr {
namespace video { class IVideoDriver; }
namespace gui { class IGUIFont; }
}  // namespace irr

namespace ev1sim {

// Display units. The bus carries metric (km/h, 0.1 km odometer counts, km
// range); imperial is a conversion at the display only.
enum class ClusterUnits : std::uint8_t { kMetric, kImperial };

// --- Pure label helpers ----------------------------------------------------

/// "88 km/h" / "55 MPH", or "--- km/h" / "--- MPH" with no fresh $41.
std::wstring FormatClusterSpeed(const std::optional<DashPcmStatus>& pcm, ClusterUnits units);
/// "P" "R" "N" "D"; "-" for the Error code, an undefined code, or no fresh $41.
std::wstring FormatClusterGear(const std::optional<DashPcmStatus>& pcm);
/// "SOC 82%" from the $20 User State of Charge, or "SOC ---".
std::wstring FormatClusterSoc(const std::optional<DashBpmDisplay>& display);
/// "Range 120 km" / "Range 75 mi", or "Range ---".
std::wstring FormatClusterRange(const std::optional<DashBpmDisplay>& display, ClusterUnits units);
/// "Power [#####......]" — the $90 bargraph, 11 segments lit from the left; "Power ---".
std::wstring FormatClusterPowerBar(const std::optional<std::uint8_t>& bars);
/// "Pack 312 V  -45.5 A" from the $40 poll, or "Pack ---".
std::wstring FormatClusterPack(const std::optional<DashPackStatus>& pack);
/// "Charging: full in 2:05" while the $40 says the charger is connected;
/// "Charging: --:--" when the $20 time-to-full is missing; "" when not charging
/// (or the $40 is missing), so the panel drops the row.
std::wstring FormatClusterCharge(const std::optional<DashPackStatus>& pack,
                                 const std::optional<DashBpmDisplay>& display);
/// "ODO 1234.5 km" / "ODO 767.1 mi" from the IPC's own $21, "ODO FAULT" when
/// the frame flags the odometer faulted, or "ODO ---".
std::wstring FormatClusterOdometer(const std::optional<DashOdometer>& odo, ClusterUnits units);
/// Which modules are being heard: "Bus: PIM ok  BPM ok  BTCM --  IPC ok".
std::wstring FormatClusterBusLine(const DashSnapshot& snap);

// --- View model ------------------------------------------------------------

struct ClusterLamp {
    const wchar_t* name;  // static label, e.g. L"BRAKE"
    bool known;           // the lamp's cell has been written
    bool lit;             // meaningful only when known
};

struct ClusterView {
    std::wstring speed;
    std::wstring gear;
    std::wstring soc;
    std::wstring range;
    std::wstring power_bar;
    std::wstring pack;
    std::wstring charge;    // empty = not charging; the row is not drawn
    std::wstring odometer;
    std::wstring bus;
    std::vector<ClusterLamp> lamps;  // fixed order, one entry per DashLamps member
};

/// Everything the panel shows, derived from one snapshot. Pure.
ClusterView BuildClusterView(const DashSnapshot& snap, ClusterUnits units);

// --- Panel -----------------------------------------------------------------

class InstrumentClusterPanel {
public:
    // anchor_x / anchor_y: top-left corner in screen pixels; width in pixels.
    explicit InstrumentClusterPanel(ClusterUnits units = ClusterUnits::kMetric,
                                    int anchor_x = 10, int anchor_y = 10,
                                    int width = 300);

    void SetVisible(bool visible) { visible_ = visible; }
    // Move the panel (e.g. to follow a window resize); width is fixed.
    void SetAnchor(int x, int y) { x_ = x; y_ = y; }
    int Width() const { return w_; }
    bool IsVisible() const { return visible_; }

    // Rebuild the view from this tick's snapshot (cheap; strings only).
    void Update(const DashSnapshot& snap);
    const ClusterView& view() const { return view_; }

    // Paint the latest view. No-op when hidden or `driver` is null; a null
    // `font` draws the panel and lamp boxes without text.
    void Draw(irr::video::IVideoDriver* driver, irr::gui::IGUIFont* font) const;

    // Panel height for the current view, so callers can stack other HUD panels.
    int Height() const;

private:
    ClusterUnits units_;
    int x_;
    int y_;
    int w_;
    bool visible_ = true;
    ClusterView view_;
};

}  // namespace ev1sim
