#pragma once

#include <memory>

// Real-time piezo "sounder" audio: the turn-signal / hazard tick (sounder X56
// on the LHJB board) and the door-lock solenoid click.  Uses CoreAudio
// (AudioUnit) on macOS; no-op on other platforms.  Mirrors HornAudio's
// structure.
//
// Both are one-shot square-wave bursts at the piezo's fixed pitch.  SimApp
// calls PlayTick() once per rising edge of electricsim's TURN_TICK_SOUNDER_DRIVE
// (ev1sim::Sounder, chassis 4096), and PlayClick() when a door-lock motor
// reaches end of travel.
class SounderAudio {
public:
    SounderAudio();
    ~SounderAudio();

    // One short tick (~12 ms) — one turn-signal / hazard flash.
    void PlayTick();

    // One-shot short buzz (~40 ms) — e.g. the door-lock solenoid click.
    void PlayClick();

    // Non-copyable.
    SounderAudio(const SounderAudio&) = delete;
    SounderAudio& operator=(const SounderAudio&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
