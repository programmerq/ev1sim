#pragma once

#include "DriverCommand.h"

// ---------------------------------------------------------------------------
// PanelDriverOverride — the floating panel's accelerator, brake and steering
// sliders as an analog driver input (TAB opens the panel).
//
// A slider at rest (0) leaves that axis to the keyboard or wheel; a slider
// moved off rest replaces it, the same way a connected wheel replaces the
// keyboard's axes.  The brake slider is one pedal, so it sets front and rear
// alike, as the keyboard does.  A scripted driver still wins over all of it
// (SimApp applies the script after this).
// ---------------------------------------------------------------------------

struct PanelDriverOverride {
    double throttle = 0.0;  // 0..1, 0 = not overriding
    double brake    = 0.0;  // 0..1, 0 = not overriding
    double steering = 0.0;  // -1..1 (positive = left), 0 = not overriding
};

inline void ApplyPanelDriverOverride(DriverCommand& cmd, const PanelDriverOverride& o) {
    if (o.throttle > 0.0) cmd.throttle = o.throttle > 1.0 ? 1.0 : o.throttle;
    if (o.brake > 0.0) {
        const double b = o.brake > 1.0 ? 1.0 : o.brake;
        cmd.front_brake = b;
        cmd.rear_brake  = b;
    }
    if (o.steering != 0.0) {
        cmd.steering = o.steering > 1.0 ? 1.0 : (o.steering < -1.0 ? -1.0 : o.steering);
    }
}
