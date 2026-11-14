#pragma once

#include <memory>

#include "CabinAudioSynth.h"

// The cabin's audio output: the horn, the turn-signal / hazard tick and the
// door-lock click, mixed by CabinAudioSynth into one mono stream.
//
// Plays through SDL3's audio subsystem, so it sounds on macOS, Windows and
// Linux alike.  Built without SDL3 (EV1SIM_HAVE_WHEEL_IO=OFF) it keeps the
// synth but opens no device.  With `open_device` false (headless runs) it
// never touches an audio device, so a nightly run on a server neither needs one
// nor makes a sound.
class CabinAudio {
public:
    explicit CabinAudio(bool open_device);
    ~CabinAudio();

    CabinAudio(const CabinAudio&)            = delete;
    CabinAudio& operator=(const CabinAudio&) = delete;

    // Which horn tones sound.  Call once per frame.
    void SetHorn(bool low_on, bool high_on) { m_synth.SetHorn(low_on, high_on); }

    // One turn-signal / hazard tick (one rising edge of the sounder drive).
    void PlayTick() { m_synth.PlayTick(); }

    // One door-lock solenoid click.
    void PlayClick() { m_synth.PlayClick(); }

    // true when an output device is open and playing.
    bool playing() const;

    // The output device's state (SDL's stream); public only so the audio
    // callback in CabinAudio.cpp can name it.
    struct Device;

private:
    ev1sim::CabinAudioSynth m_synth;
    std::unique_ptr<Device> m_device;
};
