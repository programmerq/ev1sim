#pragma once

#include <memory>

// Real-time piezo "sounder" audio — the LHJB turn/hazard tick and the
// door-lock solenoid click.  Uses CoreAudio (AudioUnit) on macOS; no-op on
// other platforms.  Mirrors HornAudio's structure.
//
// The EV1 piezo is a square-wave-driven buzzer: while energised it buzzes, and
// the LHJB flasher gates it on/off each flash half-cycle — that toggling IS the
// audible TURN/HAZ "tick".  Drive SetSounding() from PhysicalWorld::Sounder
// (chassis 4096) each frame; fire PlayClick() for a one-shot actuation click
// (e.g. a door-lock motor reaching end-of-travel).
class SounderAudio {
public:
    SounderAudio();
    ~SounderAudio();

    // Continuous piezo buzz while `sounding` is true.  Call once per frame.
    void SetSounding(bool sounding);

    // One-shot short buzz (~40 ms) — e.g. the door-lock solenoid click.
    void PlayClick();

    // Non-copyable.
    SounderAudio(const SounderAudio&) = delete;
    SounderAudio& operator=(const SounderAudio&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
