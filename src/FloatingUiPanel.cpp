#include "FloatingUiPanel.h"

#include <cassert>

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
