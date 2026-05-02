#include "FloatingUiPanel.h"

#include <cassert>
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

std::wstring FormatRsaRunModeLabel(std::uint8_t mode, bool ever_received) {
    if (!ever_received) {
        return L"Mode: ---";
    }
    // kSigRunModeBroadcast (5711) encoding per rsa_scan.h: 0=OFF, 1=ACC, 2=RUN.
    // START (enum 4 on the mode-button input 6971) is never broadcast on 5711;
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
    if (m_bg)
        m_bg->remove();
}

// ---------------------------------------------------------------------------
void FloatingUiPanel::AddButton(std::function<std::wstring()> label_fn,
                                std::function<void()>         on_click) {
    const int idx = static_cast<int>(m_buttons.size());
    const int by  = m_anchor_y + kBgPadTop
                    + idx * (m_btn_h + kPadding);
    const int bx  = m_anchor_x + kBgPadLeft;

    irr::core::recti rect(bx, by, bx + m_btn_w, by + m_btn_h);

    // Compute the initial label.
    std::wstring init_label = label_fn ? label_fn() : L"";

    auto* btn = m_gui->addButton(rect, nullptr, idx, init_label.c_str());
    if (btn) {
        btn->setVisible(false);  // hidden until panel is shown
        btn->setNotClipped(true);
    }

    // Resize background to cover all buttons.
    if (m_bg) {
        const int total_h = kBgPadTop
                            + static_cast<int>(m_buttons.size() + 1)
                              * (m_btn_h + kPadding)
                            - kPadding
                            + kBgPadBot;
        const int total_w = kBgPadLeft + m_btn_w + kBgPadRight;
        m_bg->setRelativePosition(
            irr::core::recti(m_anchor_x, m_anchor_y,
                             m_anchor_x + total_w,
                             m_anchor_y + total_h));
    }

    m_buttons.push_back({ btn, std::move(label_fn), std::move(on_click) });
}

// ---------------------------------------------------------------------------
void FloatingUiPanel::SetVisible(bool visible) {
    m_visible = visible;
    if (m_bg)   m_bg->setVisible(visible);
    for (auto& b : m_buttons) {
        if (b.widget)
            b.widget->setVisible(visible);
    }
}

// ---------------------------------------------------------------------------
void FloatingUiPanel::UpdateLabels() {
    if (!m_visible)
        return;
    for (auto& b : m_buttons) {
        if (b.widget && b.label_fn) {
            std::wstring label = b.label_fn();
            b.widget->setText(label.c_str());
        }
    }
}

// ---------------------------------------------------------------------------
bool FloatingUiPanel::OnEvent(const irr::SEvent& event) {
    if (!m_visible)
        return false;
    if (event.EventType != irr::EET_GUI_EVENT)
        return false;
    if (event.GUIEvent.EventType != irr::gui::EGET_BUTTON_CLICKED)
        return false;

    // Match by widget pointer — button IDs may be reused in the GUI environment.
    const irr::gui::IGUIElement* caller = event.GUIEvent.Caller;
    for (auto& b : m_buttons) {
        if (b.widget == caller) {
            if (b.on_click)
                b.on_click();
            return true;
        }
    }
    return false;
}
