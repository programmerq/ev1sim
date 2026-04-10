#pragma once

#include "DriverCommand.h"

// Abstract input source.
// Keyboard is the first-pass implementation; socket-based, wheel,
// and playback controllers can be added later without touching
// physics or rendering code.
class InputController {
public:
    virtual ~InputController() = default;

    // Called once per render frame.  dt is the wall-clock frame interval.
    virtual DriverCommand Update(double dt) = 0;
};
