#pragma once

#include "FeedbackChannel.h"

#include <cstdint>

namespace ev1sim {

class SdlContext;

// Plays the feedback channels on a rumble controller's two motors through SDL,
// so motors mounted anywhere (on the pedals, a seat, a gamepad) can carry the
// ABS pulse.  Opens the first SDL device whose name contains
// device_name_match; an empty match leaves it off.  Hot-plug tolerant, like
// WheelInputController.  Compiled only when EV1SIM_HAVE_WHEEL_IO is defined.
class ControllerRumble {
public:
    ControllerRumble(SdlContext* sdl, ControllerRumbleConfig cfg);
    ~ControllerRumble();

    ControllerRumble(const ControllerRumble&)            = delete;
    ControllerRumble& operator=(const ControllerRumble&) = delete;

    bool HasDevice() const { return m_joystick != nullptr; }

    // Call once per frame with whether the ABS pulse is on right now.
    void Update(bool abs_pulse_on);

private:
    void OpenMatchingDevice();
    void CloseDevice();

    SdlContext*            m_sdl;
    ControllerRumbleConfig m_cfg;
    void*                  m_joystick = nullptr;   // SDL_Joystick*
    RumbleLevels           m_last;
};

}  // namespace ev1sim
