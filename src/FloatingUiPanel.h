#pragma once

#include "PhysicalWorld.h"

#include <irrlicht.h>

#include <functional>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// FloatingUiPanel — a translucent vertical column of clickable buttons
// anchored to the top-left of the Irrlicht window.
//
// Usage pattern:
//   1. Construct with an IGUIEnvironment* and screen dimensions.
//   2. Call AddButton() to register each button with a label-provider and
//      a click callback.
//   3. Call SetVisible(true) when the user presses TAB to enter UI mode;
//      SetVisible(false) to hide it.
//   4. Call UpdateLabels() each frame to refresh button text from state.
//   5. Irrlicht's own GUI event system handles clicks — no extra
//      ProcessClicks() call needed; callbacks fire automatically on
//      EGET_BUTTON_CLICKED events via the GUIEventDelegate added to the
//      IEventReceiver chain.  Wrap this class's OnGuiEvent() in a thin
//      IEventReceiver shim if Irrlicht version requires it; see SimApp.
//
// Label helpers (pure, testable, no Irrlicht dependency):
//   FormatHazardLabel, FormatLockAllLabel, FormatDoorLabel,
//   FormatCouplerLabel
// These are free functions so unit tests can call them without an Irrlicht
// window.
// ---------------------------------------------------------------------------

namespace ev1sim { class PhysicalWorld; }

// ---------------------------------------------------------------------------
// Pure label-format helpers (no Irrlicht, fully unit-testable)
// ---------------------------------------------------------------------------
std::wstring FormatHazardLabel(bool on);
std::wstring FormatLockAllLabel(bool any_locked);
std::wstring FormatDoorLabel(const wchar_t* door_name, bool locked);
std::wstring FormatCouplerLabel(bool present);
std::wstring FormatExtKeypadButtonLabel(int button_idx);
std::wstring FormatDoorHandleLabel(const wchar_t* door_name);

// New label helpers — Wave 2 buttons.
/// Format wiper stalk cycle button label.  pos: 0=OFF, 1=INT, 2=LOW, 3=HIGH.
std::wstring FormatWiperLabel(int pos);
/// Format wiper wash button label (always static "Wash").
std::wstring FormatWashLabel();
/// Format cruise stalk button labels.
std::wstring FormatCruiseLabel(const wchar_t* action);
/// Format IPC trip-reset button label (always static "Trip Reset").
std::wstring FormatTripResetLabel();
/// Format seatbelt toggle button label.  door_name: e.g. L"D" or L"P".
std::wstring FormatSeatbeltLabel(const wchar_t* seat_name, bool buckled);

// ---------------------------------------------------------------------------
class FloatingUiPanel : public irr::IEventReceiver {
public:
    // anchor_x / anchor_y: top-left corner of the panel in screen pixels.
    // btn_w / btn_h: width and height of each button.
    FloatingUiPanel(irr::gui::IGUIEnvironment* gui,
                    int anchor_x = 10, int anchor_y = 10,
                    int btn_w    = 220, int btn_h   = 20);
    ~FloatingUiPanel();

    // Non-copyable.
    FloatingUiPanel(const FloatingUiPanel&)            = delete;
    FloatingUiPanel& operator=(const FloatingUiPanel&) = delete;

    // Register a button.  label_fn returns the current label text each frame;
    // on_click is called when the button is clicked.
    void AddButton(std::function<std::wstring()> label_fn,
                   std::function<void()>         on_click);

    // Show or hide the whole panel (all buttons + background).
    void SetVisible(bool visible);
    bool IsVisible() const { return m_visible; }

    // Refresh all button labels from the current PhysicalWorld state.
    // Must be called each frame inside BeginScene…EndScene.
    void UpdateLabels();

    // IEventReceiver — route GUI button-click events to callbacks.
    // Register this with m_vis->AddUserEventReceiver() in SimApp.
    bool OnEvent(const irr::SEvent& event) override;

private:
    struct ButtonEntry {
        irr::gui::IGUIButton*     widget   = nullptr;
        std::function<std::wstring()> label_fn;
        std::function<void()>         on_click;
    };

    irr::gui::IGUIEnvironment* m_gui;
    irr::gui::IGUIStaticText*  m_bg       = nullptr;
    std::vector<ButtonEntry>   m_buttons;
    int   m_anchor_x;
    int   m_anchor_y;
    int   m_btn_w;
    int   m_btn_h;
    bool  m_visible = false;

    static constexpr int kPadding    = 4;  // pixels between buttons
    static constexpr int kBgPadLeft  = 4;
    static constexpr int kBgPadTop   = 4;
    static constexpr int kBgPadRight = 4;
    static constexpr int kBgPadBot   = 4;
};
