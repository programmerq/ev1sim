#pragma once

#include "RigSwitches.h"

#include <memory>
#include <vector>

namespace ev1sim {

class SdlContext;

// Opens each configured rig device (RigConfig) through SDL by name, hot-plug
// tolerant like WheelInputController, and polls it into a RigSample through its
// RigSwitchTracker.  Compiled only when EV1SIM_HAVE_WHEEL_IO is defined.
class RigInputDevices {
public:
    RigInputDevices(SdlContext* sdl, const RigConfig& cfg);
    ~RigInputDevices();

    RigInputDevices(const RigInputDevices&)            = delete;
    RigInputDevices& operator=(const RigInputDevices&) = delete;

    // Polls every open device (opening any that appeared) and merges their
    // samples.  Call after SdlContext::Pump().
    RigSample Update();

private:
    struct Device {
        RigSwitchTracker tracker;
        void*            joystick = nullptr;   // SDL_Joystick*
    };
    void TryOpen(Device& d);
    static void Close(Device& d);

    SdlContext*         m_sdl;
    std::vector<Device> m_devices;
};

}  // namespace ev1sim
