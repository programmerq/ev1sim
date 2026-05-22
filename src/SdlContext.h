#pragma once

// RAII guard for the SDL3 joystick + haptic subsystems used by the
// force-feedback wheel + pedals.  Deliberately does NOT initialize SDL_VIDEO:
// SDL only opens the wheel/haptic *device*; Irrlicht owns the window and the
// GL context, so the two coexist without fighting over either.
//
// Compiled only when EV1SIM_HAVE_WHEEL_IO is defined (see CMakeLists.txt).
// Constructed once by SimApp in non-headless mode, before the wheel controller.

namespace ev1sim {

class SdlContext {
public:
    // Initializes the SDL joystick + haptic subsystems.  ok() reports whether
    // init succeeded; on failure the wheel/FFB code degrades to "no device"
    // and the sim keeps running on keyboard input.
    SdlContext();
    ~SdlContext();

    SdlContext(const SdlContext&)            = delete;
    SdlContext& operator=(const SdlContext&) = delete;

    bool ok() const { return m_ok; }

    // Pump SDL's event queue once per render frame so device hot-plug and
    // axis/button state stay current.  Joystick state is then read by polling
    // (SDL_GetJoystickAxis/Button) rather than by consuming events, so this
    // simply keeps SDL's internal state fresh and the queue bounded.
    void Pump();

private:
    bool m_ok = false;
};

}  // namespace ev1sim
