#pragma once

#include <cstddef>
#include <optional>
#include <string_view>

namespace ev1sim {

// Named, configurable input actions that a wheel button — or, later, an Arduino
// contact — can be bound to via JSON (config/input_bindings.json).  This is the
// single registry the hardware-input layer shares with the keyboard: the
// binding config names an action by string, the registry maps it to this enum,
// and `SimApp::DispatchAction` performs the effect by reusing the very same
// PhysicalWorld / sim handlers the keyboard already calls.
//
// `Horn` is a *level* input (asserted while the button is held); every other
// action is *momentary* (fires once on the button's rising edge).  Continuous
// axes (steering, throttle, brake) are NOT actions — they are axis bindings
// handled separately by WheelInputController.
enum class InputAction {
    None = 0,

    Horn,                 // held: drives the horn-button contact (circuit 28)

    // Steering-column switches (map onto real-car connector cavities).
    HeadlightCycle,
    TurnSignalLeft,
    TurnSignalRight,
    HazardToggle,
    WiperCycle,
    WiperWash,

    // Cruise stalk.
    CruiseSet,
    CruiseResume,
    CruiseCancel,
    CruiseSpeedUp,
    CruiseSpeedDown,

    // Transmission selector.
    PrndUp,
    PrndDown,

    // Propulsion / security.
    KeyOnCycle,
    IpcTripReset,

    // Vehicle.
    ResetVehicle,
    ParkingBrakeToggle,

    // Sim / debug (no real-car equivalent).
    CameraCycle,
    PauseToggle,
    SnapshotToggle,
    UiModeToggle,
    HelpToggle,
    Quit,
};

namespace detail {
struct ActionName {
    InputAction      action;
    std::string_view name;
};

// Canonical snake_case names used in config/input_bindings.json.  Keep in sync
// with InputAction above; the unit test asserts every enumerator (except None)
// has exactly one name and round-trips.
inline constexpr ActionName kInputActionNames[] = {
    {InputAction::Horn,               "horn"},
    {InputAction::HeadlightCycle,     "headlight_cycle"},
    {InputAction::TurnSignalLeft,     "turn_left"},
    {InputAction::TurnSignalRight,    "turn_right"},
    {InputAction::HazardToggle,       "hazard_toggle"},
    {InputAction::WiperCycle,         "wiper_cycle"},
    {InputAction::WiperWash,          "wiper_wash"},
    {InputAction::CruiseSet,          "cruise_set"},
    {InputAction::CruiseResume,       "cruise_resume"},
    {InputAction::CruiseCancel,       "cruise_cancel"},
    {InputAction::CruiseSpeedUp,      "cruise_speed_up"},
    {InputAction::CruiseSpeedDown,    "cruise_speed_down"},
    {InputAction::PrndUp,             "prnd_up"},
    {InputAction::PrndDown,           "prnd_down"},
    {InputAction::KeyOnCycle,         "key_on_cycle"},
    {InputAction::IpcTripReset,       "ipc_trip_reset"},
    {InputAction::ResetVehicle,       "reset_vehicle"},
    {InputAction::ParkingBrakeToggle, "parking_brake_toggle"},
    {InputAction::CameraCycle,        "camera_cycle"},
    {InputAction::PauseToggle,        "pause_toggle"},
    {InputAction::SnapshotToggle,     "snapshot_toggle"},
    {InputAction::UiModeToggle,       "ui_mode_toggle"},
    {InputAction::HelpToggle,         "help_toggle"},
    {InputAction::Quit,               "quit"},
};
}  // namespace detail

// Number of bindable named actions (excludes None).
inline constexpr std::size_t kInputActionCount =
    sizeof(detail::kInputActionNames) / sizeof(detail::kInputActionNames[0]);

// Look up an action by its config name.  Returns std::nullopt for an unknown
// name so config parsing can warn + skip rather than silently misbind a button.
inline std::optional<InputAction> InputActionFromString(std::string_view name) {
    for (const auto& e : detail::kInputActionNames) {
        if (e.name == name) {
            return e.action;
        }
    }
    return std::nullopt;
}

// Reverse lookup — canonical name for an action ("none" for None / unknown).
inline std::string_view InputActionToString(InputAction action) {
    for (const auto& e : detail::kInputActionNames) {
        if (e.action == action) {
            return e.name;
        }
    }
    return "none";
}

// True if the action is a held/level input (only Horn today); all others are
// momentary one-shots dispatched on the button's rising edge.
inline bool InputActionIsHeld(InputAction action) {
    return action == InputAction::Horn;
}

}  // namespace ev1sim
