#include "CabinAudioSynth.h"

#include <cmath>

namespace ev1sim {

namespace {
constexpr double kTwoPi = 6.283185307179586;

void Advance(double& phase, double inc) {
    phase += inc;
    if (phase >= kTwoPi) phase -= kTwoPi;
}
}  // namespace

void CabinAudioSynth::SetHorn(bool low_on, bool high_on) {
    m_low_on.store(low_on, std::memory_order_relaxed);
    m_high_on.store(high_on, std::memory_order_relaxed);
}

void CabinAudioSynth::StartBurst(int frames) {
    // A new burst restarts the countdown; a tick and a click landing in the
    // same frame play as the longer of the two.
    int cur = m_burst_frames.load(std::memory_order_relaxed);
    while (cur < frames &&
           !m_burst_frames.compare_exchange_weak(cur, frames, std::memory_order_relaxed)) {
    }
}

void CabinAudioSynth::Render(float* out, std::size_t frames) {
    const bool lo = m_low_on.load(std::memory_order_relaxed);
    const bool hi = m_high_on.load(std::memory_order_relaxed);

    // Take the whole countdown, use what this buffer needs, and hand back the
    // rest unless a new burst started meanwhile.
    const int taken = m_burst_frames.exchange(0, std::memory_order_relaxed);
    int burst = taken;

    const double inc_lo    = kTwoPi * kHornLowHz  / kSampleRate;
    const double inc_hi    = kTwoPi * kHornHighHz / kSampleRate;
    const double inc_piezo = kTwoPi * kPiezoHz    / kSampleRate;

    for (std::size_t i = 0; i < frames; ++i) {
        float s = 0.0f;
        if (lo) s += kHornLevel * static_cast<float>(std::sin(m_phase_low));
        if (hi) s += kHornLevel * static_cast<float>(std::sin(m_phase_high));
        if (burst > 0) {
            s += (m_phase_piezo < kTwoPi / 2.0) ? kPiezoLevel : -kPiezoLevel;
            --burst;
        }
        out[i] = s;
        Advance(m_phase_low, inc_lo);
        Advance(m_phase_high, inc_hi);
        Advance(m_phase_piezo, inc_piezo);
    }

    if (burst > 0) {
        int expected = 0;
        m_burst_frames.compare_exchange_strong(expected, burst, std::memory_order_relaxed);
    }
}

}  // namespace ev1sim
