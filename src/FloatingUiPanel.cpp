#include "FloatingUiPanel.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cwchar>
#include <cstdio>

// ---------------------------------------------------------------------------
// Pure label-format helpers
// ---------------------------------------------------------------------------

std::wstring FormatHazardLabel(bool on) {
    return on ? L"Hazard: ON" : L"Hazard: OFF";
}

std::wstring FormatLockAllLabel(bool any_locked) {
    // If any door is locked the batch-action is "Unlock All"; otherwise "Lock All".
    return any_locked ? L"Unlock All" : L"Lock All";
}

std::wstring FormatDoorLabel(const wchar_t* door_name, bool locked) {
    std::wstring label = door_name;
    label += locked ? L": locked" : L": unlocked";
    return label;
}

std::wstring FormatCouplerLabel(bool present) {
    return present ? L"Coupler: PLUGGED" : L"Coupler: UNPLUGGED";
}

std::wstring FormatExtKeypadButtonLabel(int button_idx) {
    // Labels mirror EV1 exterior pillar keypad keycaps.
    static const wchar_t* kLabels[] = { L"[1/2]", L"[3/4]", L"[5/6]", L"[7/8]", L"[9/0]" };
    if (button_idx < 0 || button_idx >= 5) return L"[?]";
    return kLabels[button_idx];
}

std::wstring FormatDoorHandleLabel(const wchar_t* door_name) {
    std::wstring label = L"Handle: ";
    label += door_name;
    return label;
}

std::wstring FormatWiperLabel(int pos) {
    const wchar_t* names[] = { L"OFF", L"INT", L"LOW", L"HIGH" };
    std::wstring label = L"Wiper: ";
    label += (pos >= 0 && pos <= 3) ? names[pos] : L"?";
    return label;
}

std::wstring FormatWashLabel() {
    return L"Wash";
}

std::wstring FormatCruiseLabel(const wchar_t* action) {
    std::wstring label = L"Cruise: ";
    label += action;
    return label;
}

std::wstring FormatTripResetLabel() {
    return L"Trip Reset";
}

std::wstring FormatSeatbeltLabel(const wchar_t* seat_name, bool buckled) {
    std::wstring label = L"Seatbelt ";
    label += seat_name;
    label += buckled ? L": BUCKLED" : L": UNBUCKLED";
    return label;
}

std::wstring FormatHvacBlowerLabel(std::uint8_t level) {
    // Encoding per HTCM supervisor: 0=OFF, 1=LOW, 2=MED, 3=HIGH.
    // 0xFF = never received from bus (show placeholder).
    switch (level) {
        case 0:    return L"Blower: OFF";
        case 1:    return L"Blower: LOW";
        case 2:    return L"Blower: MED";
        case 3:    return L"Blower: HIGH";
        case 0xFF: return L"Blower: ---";
        default:   return L"Blower: ?";
    }
}

std::wstring FormatDefrostGridLabel(bool active) {
    return active ? L"Defrost: ON" : L"Defrost: OFF";
}

// --- HVAC driver-control labels (the requests the driver sends to HTCM,
//     distinct from the blower/defrost-grid *feedback* helpers above). ---
std::wstring FormatHvacSetpointLabel(double setpoint_c) {
    wchar_t buf[64];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]),
                  L"HVAC Temp: %.1f C", setpoint_c);
    return buf;
}

std::wstring FormatHvacFanLabel(std::uint8_t fan) {
    // Mirrors HvacControls::Fan: 0=OFF, 1=LOW, 2=MED, 3=HIGH.
    switch (fan) {
        case 0:  return L"HVAC Fan: OFF";
        case 1:  return L"HVAC Fan: LOW";
        case 2:  return L"HVAC Fan: MED";
        case 3:  return L"HVAC Fan: HIGH";
        default: return L"HVAC Fan: ?";
    }
}

std::wstring FormatHvacModeLabel(std::uint8_t mode) {
    // Mirrors HvacControls::Mode: 0=FACE, 1=BILEVEL, 2=FEET, 3=DEFROST.
    switch (mode) {
        case 0:  return L"HVAC Mode: FACE";
        case 1:  return L"HVAC Mode: BILEVEL";
        case 2:  return L"HVAC Mode: FEET";
        case 3:  return L"HVAC Mode: DEFROST";
        default: return L"HVAC Mode: ?";
    }
}

std::wstring FormatHvacAcLabel(bool on) {
    return on ? L"HVAC A/C: ON" : L"HVAC A/C: OFF";
}

std::wstring FormatHvacDefrostLabel(bool on) {
    return on ? L"HVAC Defrost: ON" : L"HVAC Defrost: OFF";
}

std::wstring FormatIpcSeatbeltTelltaleLabel(const wchar_t* seat_name,
                                            bool lamp_on,
                                            bool ever_received) {
    std::wstring label = L"Seatbelt (";
    label += seat_name;
    label += L"): ";
    if (!ever_received) {
        label += L"---";
    } else {
        label += lamp_on ? L"ON" : L"OFF";
    }
    return label;
}

std::wstring FormatPrndGearLabel(int pos) {
    const wchar_t* names[] = { L"P", L"R", L"N", L"D" };
    std::wstring label = L"Gear: ";
    label += (pos >= 0 && pos <= 3) ? names[pos] : L"?";
    return label;
}

std::wstring FormatRsaShiftBlockedLabel(bool blocked, bool ever_received) {
    if (!ever_received) {
        return L"Shift: ---";
    }
    return blocked ? L"Shift: BRAKE TO SHIFT" : L"Shift: OK";
}

std::wstring FormatRsaRunModeLabel(std::uint8_t mode, bool ever_received) {
    if (!ever_received) {
        return L"Mode: ---";
    }
    // kSigRunModeBroadcast (5711) encoding per rsa_scan.h: 0=OFF, 1=ACC, 2=RUN.
    // START (an internal RSA precharge phase, not a key) is never broadcast on 5711;
    // RSA transitions directly from START press → RUN state on the broadcast.
    switch (mode) {
        case 0: return L"Mode: OFF";
        case 1: return L"Mode: ACC";
        case 2: return L"Mode: RUN";
        default: {
            wchar_t buf[32];
            std::swprintf(buf, sizeof(buf) / sizeof(buf[0]),
                          L"Mode: ?(%u)", static_cast<unsigned>(mode));
            return buf;
        }
    }
}

std::wstring FormatPimCruiseStatusLabel(bool ever_received_active,
                                        bool active,
                                        float setpoint_mps) {
    if (!ever_received_active) {
        return L"Cruise: ---";
    }
    if (!active) {
        return L"Cruise: OFF";
    }
    // "Cruise: 23.5 m/s ON"
    wchar_t buf[64];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]),
                  L"Cruise: %.1f m/s ON", static_cast<double>(setpoint_mps));
    return buf;
}

std::wstring FormatIpcTripDistanceLabel(bool ever_received, float distance_m) {
    if (!ever_received) {
        return L"Trip: ---";
    }
    // Convert metres to kilometres, show one decimal place.
    // "Trip: 12.3 km"
    wchar_t buf[64];
    const double km = static_cast<double>(distance_m) / 1000.0;
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"Trip: %.1f km", km);
    return buf;
}

std::wstring FormatIpcTelltaleLampLabel(const wchar_t* name,
                                        bool lamp_on,
                                        bool ever_received) {
    std::wstring label = name;
    label += L": ";
    if (!ever_received) {
        label += L"---";
    } else {
        label += lamp_on ? L"ON" : L"OFF";
    }
    return label;
}

std::wstring FormatHeadlampStatusLabel(bool low, bool high, bool ever_received) {
    if (!ever_received) return L"Headlamps: ---";
    if (high)          return L"Headlamps: HIGH";
    if (low)           return L"Headlamps: LOW";
    return L"Headlamps: OFF";
}

std::wstring FormatTurnSignalStatusLabel(const wchar_t* side, bool active,
                                         bool ever_received) {
    std::wstring label = side;
    label += L" Turn: ";
    if (!ever_received) {
        label += L"---";
    } else {
        label += active ? L"ON" : L"OFF";
    }
    return label;
}

std::wstring FormatVehicleSpeedLabel(bool ever_received, float speed_mps) {
    if (!ever_received) {
        return L"Speed: ---";
    }
    // km/h = m/s × 3.6; round to nearest integer.
    const double mps  = static_cast<double>(speed_mps);
    const double kmh  = mps * 3.6;
    const int    kmh_int = static_cast<int>(kmh + 0.5);
    // "Speed: 12.3 m/s (44 km/h)"
    wchar_t buf[80];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]),
                  L"Speed: %.1f m/s (%d km/h)",
                  mps, kmh_int);
    return buf;
}

std::wstring FormatPedalPercentLabel(const char* name, double value_0_to_1) {
    // Build a wide-character version of the name.
    wchar_t wname[64] = {};
    for (int i = 0; name[i] && i < 63; ++i)
        wname[i] = static_cast<wchar_t>(name[i]);

    // Guard against NaN / infinity — show placeholder.
    if (!std::isfinite(value_0_to_1)) {
        wchar_t buf[80];
        std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"%ls: ---", wname);
        return buf;
    }

    // Clamp to [0, 1] and convert to integer percent (round to nearest).
    if (value_0_to_1 < 0.0) value_0_to_1 = 0.0;
    if (value_0_to_1 > 1.0) value_0_to_1 = 1.0;
    const int pct = static_cast<int>(value_0_to_1 * 100.0 + 0.5);

    wchar_t buf[80];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"%ls: %d%%", wname, pct);
    return buf;
}

std::wstring FormatHoodStateLabel(int state) {
    switch (state) {
        case 0:  return L"Hood: CLOSED";
        case 1:  return L"Hood: POPPED (safety catch)";
        case 2:  return L"Hood: OPEN";
        default: return L"Hood: ?";
    }
}

std::wstring FormatPowerWindowButtonLabel(const wchar_t* text, bool active) {
    std::wstring s = text ? text : L"";
    if (active) s += L" [ON]";
    return s;
}

std::wstring FormatSteeringAngleLabel(double angle_deg) {
    // Guard against NaN / infinity (also the caller's "no vehicle state yet"
    // signal) — show a placeholder.
    if (!std::isfinite(angle_deg)) return L"Steering: ---";

    const double mag = (angle_deg < 0.0) ? -angle_deg : angle_deg;
    wchar_t buf[64];
    if (mag < 0.05) {
        // Within rounding distance of centered — no left/right side.
        std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"Steering: 0.0 deg");
    } else {
        const wchar_t* side = (angle_deg > 0.0) ? L"L" : L"R";
        std::swprintf(buf, sizeof(buf) / sizeof(buf[0]),
                      L"Steering: %.1f deg %ls", mag, side);
    }
    return buf;
}

std::wstring FormatTrunkLidLabel(bool open) {
    return open ? L"Trunk lid: OPEN" : L"Trunk lid: CLOSED";
}

std::wstring FormatPedalSliderLabel(const wchar_t* name, double value_0_to_1) {
    std::wstring label = name ? name : L"";
    label += L" slider: ";
    if (!std::isfinite(value_0_to_1) || value_0_to_1 <= 0.0) return label + L"off";
    if (value_0_to_1 > 1.0) value_0_to_1 = 1.0;
    const int pct = static_cast<int>(value_0_to_1 * 100.0 + 0.5);
    wchar_t buf[16];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"%d%%", pct);
    return label + buf;
}

std::wstring FormatSteerSliderLabel(double steering) {
    if (!std::isfinite(steering) || steering == 0.0) return L"Steer slider: off";
    double mag = steering < 0.0 ? -steering : steering;
    if (mag > 1.0) mag = 1.0;
    const int pct = static_cast<int>(mag * 100.0 + 0.5);
    wchar_t buf[48];
    std::swprintf(buf, sizeof(buf) / sizeof(buf[0]), L"Steer slider: %d%% %ls",
                  pct, steering > 0.0 ? L"L" : L"R");
    return buf;
}

// ---------------------------------------------------------------------------
// Panel layout
// ---------------------------------------------------------------------------

int FloatingUiRowsPerColumn(int screen_h, int anchor_y, int row_h, int row_gap,
                            int pad_top, int pad_bot) {
    // n rows take n*row_h + (n-1)*row_gap; solve for the largest n that fits.
    const int room = screen_h - anchor_y - pad_top - pad_bot + row_gap;
    const int n = room / (row_h + row_gap);
    return n < 1 ? 1 : n;
}

FloatingUiRect FloatingUiRowRect(int idx, int rows_per_col, int origin_x,
                                 int origin_y, int row_w, int row_h,
                                 int row_gap, int col_gap) {
    if (rows_per_col < 1) rows_per_col = 1;
    const int col = idx / rows_per_col;
    const int row = idx % rows_per_col;
    const int x = origin_x + col * (row_w + col_gap);
    const int y = origin_y + row * (row_h + row_gap);
    return {x, y, x + row_w, y + row_h};
}

FloatingUiRect MagnifyRect(const FloatingUiRect& r, double scale) {
    const double cx = (r.x0 + r.x1) / 2.0;
    const double cy = (r.y0 + r.y1) / 2.0;
    const double hw = (r.x1 - r.x0) * scale / 2.0;
    const double hh = (r.y1 - r.y0) * scale / 2.0;
    return {static_cast<int>(std::lround(cx - hw)), static_cast<int>(std::lround(cy - hh)),
            static_cast<int>(std::lround(cx + hw)), static_cast<int>(std::lround(cy + hh))};
}

namespace {
irr::core::recti ToIrr(const FloatingUiRect& r) { return {r.x0, r.y0, r.x1, r.y1}; }
}  // namespace

// ---------------------------------------------------------------------------
// FloatingUiPanel
// ---------------------------------------------------------------------------

FloatingUiPanel::FloatingUiPanel(irr::gui::IGUIEnvironment* gui,
                                 int anchor_x, int anchor_y,
                                 int btn_w,    int btn_h)
    : m_gui(gui),
      m_anchor_x(anchor_x),
      m_anchor_y(anchor_y),
      m_btn_w(btn_w),
      m_btn_h(btn_h)
{
    assert(gui && "FloatingUiPanel requires a valid IGUIEnvironment*");

    // The panel holds more rows than one screen-height column fits, so it
    // wraps into further columns at the screen's bottom edge.
    int screen_h = 720;
    if (irr::video::IVideoDriver* driver = gui->getVideoDriver()) {
        const int h = static_cast<int>(driver->getScreenSize().Height);
        if (h > 0) screen_h = h;
    }
    m_rows_per_col = FloatingUiRowsPerColumn(screen_h, anchor_y, btn_h, kPadding,
                                             kBgPadTop, kBgPadBot);

    // Background panel — will be sized when first button is added.
    // Start hidden.
    m_bg = gui->addStaticText(L"",
        irr::core::recti(anchor_x, anchor_y,
                         anchor_x + btn_w + kBgPadLeft + kBgPadRight,
                         anchor_y + kBgPadTop + kBgPadBot),
        /*border=*/false, /*wordWrap=*/false,
        /*parent=*/nullptr, /*id=*/-1);
    if (m_bg) {
        m_bg->setBackgroundColor(irr::video::SColor(180, 20, 20, 30));
        m_bg->setVisible(false);
        m_bg->setNotClipped(true);
    }
}

FloatingUiPanel::~FloatingUiPanel() {
    // Irrlicht GUI elements are reference-counted; remove() decrements the
    // count.  We do not delete — Irrlicht owns the allocations.
    for (auto& b : m_buttons) {
        if (b.widget)
            b.widget->remove();
    }
    for (auto& sl : m_sliders) {
        if (sl.label) sl.label->remove();
        if (sl.bar)   sl.bar->remove();
    }
    if (m_bg)
        m_bg->remove();
}

// ---------------------------------------------------------------------------
FloatingUiRect FloatingUiPanel::NextRow() {
    const FloatingUiRect r = FloatingUiRowRect(
        m_rows, m_rows_per_col, m_anchor_x + kBgPadLeft, m_anchor_y + kBgPadTop,
        m_btn_w, m_btn_h, kPadding, kColumnGap);
    ++m_rows;

    // Resize background to cover every row so far, across all columns.
    if (m_bg) {
        const int cols = (m_rows + m_rows_per_col - 1) / m_rows_per_col;
        const int rows = m_rows < m_rows_per_col ? m_rows : m_rows_per_col;
        const int total_h = kBgPadTop + rows * (m_btn_h + kPadding) - kPadding + kBgPadBot;
        const int total_w = kBgPadLeft + cols * (m_btn_w + kColumnGap) - kColumnGap
                            + kBgPadRight;
        m_bg->setRelativePosition(
            irr::core::recti(m_anchor_x, m_anchor_y,
                             m_anchor_x + total_w, m_anchor_y + total_h));
    }
    return r;
}

irr::gui::IGUIButton* FloatingUiPanel::CreateButtonWidget(
        const std::function<std::wstring()>& label_fn, FloatingUiRect* rect_out) {
    const int idx = m_rows;
    const FloatingUiRect r = NextRow();
    if (rect_out) *rect_out = r;

    // Compute the initial label.
    std::wstring init_label = label_fn ? label_fn() : L"";

    auto* btn = m_gui->addButton(ToIrr(r), nullptr, idx, init_label.c_str());
    if (btn) {
        btn->setVisible(false);  // hidden until panel is shown
        btn->setNotClipped(true);
    }
    return btn;
}

void FloatingUiPanel::AddButton(std::function<std::wstring()> label_fn,
                                std::function<void()>         on_click) {
    ButtonEntry e;
    e.widget   = CreateButtonWidget(label_fn, &e.rect);
    e.label_fn = std::move(label_fn);
    e.on_click = std::move(on_click);
    m_buttons.push_back(std::move(e));
}

void FloatingUiPanel::AddHoldButton(std::function<std::wstring()> label_fn,
                                    std::function<void(bool)>     on_hold) {
    ButtonEntry e;
    e.widget   = CreateButtonWidget(label_fn, &e.rect);
    e.label_fn = std::move(label_fn);
    e.on_hold  = std::move(on_hold);
    e.is_hold  = true;
    m_buttons.push_back(std::move(e));
}

void FloatingUiPanel::AddSlider(std::function<std::wstring()> label_fn,
                                int min, int max, int rest,
                                std::function<void(int)> on_change) {
    const FloatingUiRect r = NextRow();
    // Label takes the left 45 % of the row, the bar the rest.
    const int split = r.x0 + (r.x1 - r.x0) * 45 / 100;

    SliderEntry e;
    const std::wstring init_label = label_fn ? label_fn() : L"";
    e.label = m_gui->addStaticText(init_label.c_str(),
                                   irr::core::recti(r.x0 + 2, r.y0, split, r.y1),
                                   /*border=*/false, /*wordWrap=*/false);
    if (e.label) {
        e.label->setOverrideColor(irr::video::SColor(255, 230, 230, 230));
        e.label->setTextAlignment(irr::gui::EGUIA_UPPERLEFT, irr::gui::EGUIA_CENTER);
        e.label->setVisible(false);
        e.label->setNotClipped(true);
    }
    e.bar = m_gui->addScrollBar(/*horizontal=*/true,
                                irr::core::recti(split + 2, r.y0 + 3, r.x1, r.y1 - 3));
    if (e.bar) {
        e.bar->setMin(min);
        e.bar->setMax(max);
        e.bar->setSmallStep(1);
        e.bar->setLargeStep((max - min) / 10 > 0 ? (max - min) / 10 : 1);
        e.bar->setPos(rest);
        e.bar->setVisible(false);
        e.bar->setNotClipped(true);
    }
    e.label_fn  = std::move(label_fn);
    e.on_change = std::move(on_change);
    e.rest      = rest;
    m_sliders.push_back(std::move(e));
}

void FloatingUiPanel::MagnifyLastButtonOnHover() {
    if (!m_buttons.empty()) m_buttons.back().magnify = true;
}

// ---------------------------------------------------------------------------
void FloatingUiPanel::SetVisible(bool visible) {
    m_visible = visible;
    if (m_bg)   m_bg->setVisible(visible);
    for (auto& b : m_buttons) {
        if (b.widget)
            b.widget->setVisible(visible);
        // Hiding the panel must release any held momentary switch so it can't
        // stay asserted (and keep publishing) while the panel is invisible.
        if (!visible && b.is_hold && b.on_hold)
            b.on_hold(false);
    }
    for (auto& sl : m_sliders) {
        if (sl.label) sl.label->setVisible(visible);
        if (sl.bar)   sl.bar->setVisible(visible);
        // Same rule for analog overrides: hiding the panel lets go of them.
        if (!visible) {
            if (sl.bar) sl.bar->setPos(sl.rest);
            if (sl.on_change) sl.on_change(sl.rest);
        }
    }
}

// ---------------------------------------------------------------------------
void FloatingUiPanel::UpdateLabels() {
    if (!m_visible)
        return;
    for (auto& b : m_buttons) {
        // Momentary hold buttons: drive the callback from the live pressed
        // state before refreshing the label, so the label reflects this tick.
        if (b.is_hold && b.widget && b.on_hold)
            b.on_hold(b.widget->isPressed());
        if (b.widget && b.label_fn) {
            std::wstring label = b.label_fn();
            b.widget->setText(label.c_str());
        }
    }

    // Hover magnify: grow the hovered key in place and draw it above its
    // neighbours; every other magnifiable key goes back to its own rect.
    const irr::gui::IGUIElement* hovered = m_gui->getHovered();
    for (auto& b : m_buttons) {
        if (!b.magnify || !b.widget) continue;
        const bool over = (b.widget == hovered);
        b.widget->setRelativePosition(ToIrr(over ? MagnifyRect(b.rect, kMagnifyScale) : b.rect));
        if (over) m_gui->getRootGUIElement()->bringToFront(b.widget);
    }

    for (auto& sl : m_sliders) {
        if (sl.label && sl.label_fn) sl.label->setText(sl.label_fn().c_str());
    }
}

// ---------------------------------------------------------------------------
bool FloatingUiPanel::OnEvent(const irr::SEvent& event) {
    if (!m_visible)
        return false;
    if (event.EventType != irr::EET_GUI_EVENT)
        return false;

    // Match by widget pointer — button IDs may be reused in the GUI environment.
    const irr::gui::IGUIElement* caller = event.GUIEvent.Caller;

    if (event.GUIEvent.EventType == irr::gui::EGET_SCROLL_BAR_CHANGED) {
        for (auto& sl : m_sliders) {
            if (sl.bar == caller) {
                if (sl.on_change) sl.on_change(sl.bar->getPos());
                return true;
            }
        }
        return false;
    }
    if (event.GUIEvent.EventType != irr::gui::EGET_BUTTON_CLICKED)
        return false;

    for (auto& b : m_buttons) {
        if (b.widget == caller) {
            if (b.on_click)
                b.on_click();
            return true;
        }
    }
    return false;
}
