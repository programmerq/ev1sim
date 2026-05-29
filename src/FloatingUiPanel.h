#pragma once

#include "PhysicalWorld.h"

#include <irrlicht.h>

#include <cstdint>
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

// HVAC status label helpers (display-only, not buttons).
/// Format HVAC blower level label.
/// level: 0=OFF, 1=LOW, 2=MED, 3=HIGH, 0xFF=no data (shows "---").
std::wstring FormatHvacBlowerLabel(std::uint8_t level);
/// Format defrost grid status label.  active: true=ON, false=OFF.
std::wstring FormatDefrostGridLabel(bool active);

// HVAC driver-control labels (the requests the driver sends; distinct from the
// blower/defrost-grid *feedback* helpers above).  Mirror ev1sim::HvacControls.
/// Format the HVAC temperature setpoint (°C, one decimal): "HVAC Temp: 21.0 C".
std::wstring FormatHvacSetpointLabel(double setpoint_c);
/// Format the HVAC fan request.  fan: 0=OFF, 1=LOW, 2=MED, 3=HIGH.
std::wstring FormatHvacFanLabel(std::uint8_t fan);
/// Format the HVAC mode request.  mode: 0=FACE, 1=BILEVEL, 2=FEET, 3=DEFROST.
std::wstring FormatHvacModeLabel(std::uint8_t mode);
/// Format the A/C-compressor request.  on: true=ON, false=OFF.
std::wstring FormatHvacAcLabel(bool on);
/// Format the front-defrost request.  on: true=ON, false=OFF.
std::wstring FormatHvacDefrostLabel(bool on);

// IPC LCD telltale status label helpers (display-only, not buttons).
/// Format IPC seatbelt telltale label.
/// seat_name: e.g. L"D" (driver) or L"P" (passenger).
/// lamp_on: true=lamp on (unbuckled at speed), false=lamp off.
/// ever_received: false if no data from IPC yet (shows "---").
std::wstring FormatIpcSeatbeltTelltaleLabel(const wchar_t* seat_name,
                                            bool lamp_on,
                                            bool ever_received);

// PRND gear selector status label helper (display-only, not a button).
/// Format PRND gear selector label from PrndSelector::Position enum index.
/// pos: 0=P, 1=R, 2=N, 3=D (matches PrndSelector::Position cast to int).
/// Shows "Gear: P / R / N / D" accordingly.
std::wstring FormatPrndGearLabel(int pos);

// RSA shift-blocked cue label helper (display-only, not a button).
/// Format RSA shift-blocked cue label.
/// blocked:       true = shift refused this tick (brake not pressed in PARK).
/// ever_received: false if no frame from RSA yet (shows "Shift: ---").
/// Shows "Shift: BRAKE TO SHIFT" when blocked, "Shift: OK" otherwise.
std::wstring FormatRsaShiftBlockedLabel(bool blocked, bool ever_received);

// RSA run-mode status label helper (display-only, not a button).
/// Format RSA run-mode label from the kSigRunModeBroadcast (5711) uint8 value.
/// mode: 0=OFF, 1=ACC, 2=RUN (per rsa_scan.h; the broadcast signal does not
/// carry START — that enum value exists only on the mode-button input (6971)).
/// ever_received: false if no frame has arrived yet (shows "Mode: ---").
/// Unknown enum values show "Mode: ?(N)" where N is the raw byte.
std::wstring FormatRsaRunModeLabel(std::uint8_t mode, bool ever_received);

// PIM cruise-control status label helper (display-only, not a button).
/// Format PIM cruise active + setpoint label.
/// ever_received_active:  false if no cruise-active frame received yet (shows "---").
/// active: true = cruise ACTIVE (engaged); false = OFF or STANDBY.
/// setpoint_mps: target speed in m/s (shown only when active=true).
std::wstring FormatPimCruiseStatusLabel(bool ever_received_active,
                                        bool active,
                                        float setpoint_mps);

// IPC trip distance label helper (display-only, not a button).
/// Format IPC trip distance label.
/// ever_received: false if no trip-distance frame received yet (shows "Trip: ---").
/// distance_m: accumulated trip distance in metres; converted to km for display.
/// Shows "Trip: 12.3 km" (one decimal place).
std::wstring FormatIpcTripDistanceLabel(bool ever_received, float distance_m);

// IPC BTCM / airbag telltale label helpers (display-only, not buttons).
/// Format an IPC telltale label using a short display name.
/// name: e.g. L"Brake", L"ParkBrake", L"ABS", L"LowTrac", L"AirBag".
/// lamp_on: true=lamp on, false=lamp off.
/// ever_received: false if no frame received yet (shows "---").
/// Shows e.g. "Brake: ON" / "Brake: OFF" / "Brake: ---".
std::wstring FormatIpcTelltaleLampLabel(const wchar_t* name,
                                        bool lamp_on,
                                        bool ever_received);

// Pedal percent label helpers (display-only, not buttons).
/// Format throttle or brake pedal position as an integer percent.
/// name: label prefix, e.g. "Throttle" or "Brake".
/// value_0_to_1: pedal travel in [0, 1]; clamped and rounded to nearest int.
/// NaN / inf are rendered as "---".
/// Shows e.g. "Throttle: 47%" (no trailing decimal, locale-independent).
std::wstring FormatPedalPercentLabel(const char* name, double value_0_to_1);

// Headlamp status label helper (display-only, not a button).
/// Format headlamp state label from low-beam and high-beam bulb states.
/// low:  true if either LLBH or RLBH (low-beam bulbs) is on.
/// high: true if either LHBH or RHBH (high-beam bulbs) is on.
/// ever_received: false if no bulb data has arrived yet (shows "Headlamps: ---").
/// Priority: HIGH > LOW > OFF.  Shows "Headlamps: OFF / LOW / HIGH".
std::wstring FormatHeadlampStatusLabel(bool low, bool high, bool ever_received);

// Turn-signal status label helper (display-only, not a button).
/// Format turn-signal state label for one side.
/// side: display prefix, e.g. L"L" (left) or L"R" (right).
/// active: true if either the front or rear turn-signal bulb is on this tick.
/// ever_received: false if no bulb data has arrived yet (shows e.g. "L Turn: ---").
/// Shows "L Turn: ON" or "L Turn: OFF" once data is received.
std::wstring FormatTurnSignalStatusLabel(const wchar_t* side, bool active,
                                         bool ever_received);

// Vehicle speed label helper (display-only, not a button).
/// Format vehicle speed as "Speed: N.N m/s (NN km/h)" or "Speed: ---" if not received.
/// speed_mps: forward vehicle speed in m/s (from ev1sim physics, VehicleState snapshot).
/// ever_received: false before the first SetVehicleState() call (shows "Speed: ---").
/// Locale-independent formatting (period decimal, no thousand separators).
std::wstring FormatVehicleSpeedLabel(bool ever_received, float speed_mps);

// Steering-angle indicator label helper (display-only, not a button).
/// Format the front road-wheel steering angle (degrees) for the HUD.
/// angle_deg: front-left road-wheel angle (positive = left, per Chrono's
///            GetSteeringAngle convention).  NaN/inf → "Steering: ---".
/// Shows "Steering: 0.0 deg", "Steering: 12.3 deg L", or "Steering: 12.3 deg R".
std::wstring FormatSteeringAngleLabel(double angle_deg);

// BPM pack voltage label helper (display-only, not a button).
/// Format BPM pack voltage as "PackVolt: NNN.N V" or "PackVolt: ---" if not received.
/// pack_voltage_mv: raw millivolt value from kSigChassisBpmPackVoltageMv (4139).
/// ever_received: false before the first BPM publish (shows "PackVolt: ---").
/// Converts mV → V with one decimal place.  Locale-independent formatting.
std::wstring FormatBpmPackVoltageLabel(bool ever_received, std::uint32_t pack_voltage_mv);

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
