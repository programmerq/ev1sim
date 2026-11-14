#pragma once

#include <atomic>
#include <cstddef>

// The cabin's sounds as samples: the two-tone horn and the piezo sounder's
// bursts (the turn-signal / hazard tick and the door-lock click).  Pure: no
// audio device, so it is unit-tested; CabinAudio feeds it to the platform's
// output.  The setters are called from the sim thread and Render() from the
// audio thread, so the shared state is atomic.
//
// Pitches and levels:
//   - Horn: 400 Hz (low) and 500 Hz (high) sine tones, mixed when both sound.
//     LHJB drives both horns together (HORN_DRIVE_LINE_LOW/HIGH).
//   - Piezo: a 2.7 kHz square wave, the typical pitch of a piezo buzzer
//     element.  Not measured from the EV1's sounder X56.
//   - Burst lengths (tick 12 ms, click 40 ms) and every level are by-ear
//     estimates: no recording of the car's horn, tick or lock click is in hand.
namespace ev1sim {

class CabinAudioSynth {
public:
    static constexpr int    kSampleRate    = 44100;
    static constexpr double kHornLowHz     = 400.0;
    static constexpr double kHornHighHz    = 500.0;
    static constexpr double kPiezoHz       = 2700.0;
    static constexpr float  kHornLevel     = 0.25f;   // per tone
    static constexpr float  kPiezoLevel    = 0.20f;
    static constexpr int    kTickFrames    = kSampleRate * 12 / 1000;   // 12 ms
    static constexpr int    kClickFrames   = kSampleRate * 40 / 1000;   // 40 ms

    // Which horn tones sound.  Call once per frame.
    void SetHorn(bool low_on, bool high_on);

    // One turn-signal / hazard tick.
    void PlayTick() { StartBurst(kTickFrames); }

    // One door-lock solenoid click.
    void PlayClick() { StartBurst(kClickFrames); }

    // Fill `frames` mono float samples at kSampleRate.  Audio thread.
    void Render(float* out, std::size_t frames);

private:
    void StartBurst(int frames);

    std::atomic<bool> m_low_on{false};
    std::atomic<bool> m_high_on{false};
    std::atomic<int>  m_burst_frames{0};

    // Phase accumulators, audio thread only.  They always advance, so a tone
    // that restarts does not click.
    double m_phase_low   = 0.0;
    double m_phase_high  = 0.0;
    double m_phase_piezo = 0.0;
};

}  // namespace ev1sim
