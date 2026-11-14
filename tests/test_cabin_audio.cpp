// CabinAudioSynth: the cabin's horn, turn-signal / hazard tick and door-lock
// click as samples.  No audio device involved.

#include <catch2/catch_test_macros.hpp>

#include "CabinAudio.h"
#include "CabinAudioSynth.h"

#include <cmath>
#include <cstddef>
#include <vector>

using ev1sim::CabinAudioSynth;

namespace {

std::vector<float> Render(CabinAudioSynth& s, std::size_t frames) {
    std::vector<float> out(frames, 99.0f);
    s.Render(out.data(), frames);
    return out;
}

std::size_t NonZero(const std::vector<float>& v) {
    std::size_t n = 0;
    for (float x : v) n += (x != 0.0f);
    return n;
}

// Upward zero crossings per second: the tone's frequency.
double CrossingsHz(const std::vector<float>& v) {
    int n = 0;
    for (std::size_t i = 1; i < v.size(); ++i) n += (v[i - 1] < 0.0f && v[i] >= 0.0f);
    return n * static_cast<double>(CabinAudioSynth::kSampleRate) / v.size();
}

}  // namespace

TEST_CASE("CabinAudioSynth: silent by default", "[CabinAudio]") {
    CabinAudioSynth s;
    const auto out = Render(s, 4096);
    CHECK(NonZero(out) == 0);
}

TEST_CASE("CabinAudioSynth: one tick is a 12 ms piezo burst", "[CabinAudio]") {
    CabinAudioSynth s;
    s.PlayTick();
    const auto out = Render(s, CabinAudioSynth::kSampleRate / 10);   // 100 ms
    CHECK(NonZero(out) == static_cast<std::size_t>(CabinAudioSynth::kTickFrames));
    // The burst is at the start and is a square wave at the piezo level.
    for (int i = 0; i < CabinAudioSynth::kTickFrames; ++i)
        REQUIRE(std::fabs(out[i]) == CabinAudioSynth::kPiezoLevel);
    // About 2.7 kHz: count sign flips inside the burst.
    int flips = 0;
    for (int i = 1; i < CabinAudioSynth::kTickFrames; ++i)
        flips += ((out[i] > 0) != (out[i - 1] > 0));
    const double hz = flips / 2.0 / (CabinAudioSynth::kTickFrames /
                                     static_cast<double>(CabinAudioSynth::kSampleRate));
    CHECK(std::fabs(hz - CabinAudioSynth::kPiezoHz) < 100.0);
    // Nothing more until the next tick.
    CHECK(NonZero(Render(s, 4096)) == 0);
}

TEST_CASE("CabinAudioSynth: a burst carries across render buffers", "[CabinAudio]") {
    // Devices pull small buffers; a tick longer than one buffer must finish in
    // the next one, not be cut.
    CabinAudioSynth s;
    s.PlayTick();
    std::size_t total = 0;
    for (int b = 0; b < 10; ++b) total += NonZero(Render(s, 128));
    CHECK(total == static_cast<std::size_t>(CabinAudioSynth::kTickFrames));
}

TEST_CASE("CabinAudioSynth: the door-lock click is longer than a tick", "[CabinAudio]") {
    CabinAudioSynth s;
    s.PlayClick();
    CHECK(NonZero(Render(s, 4096)) == static_cast<std::size_t>(CabinAudioSynth::kClickFrames));
    // A tick landing during a click does not cut the click short.
    s.PlayClick();
    s.PlayTick();
    CHECK(NonZero(Render(s, 4096)) == static_cast<std::size_t>(CabinAudioSynth::kClickFrames));
}

TEST_CASE("CabinAudioSynth: horn tones sound while held", "[CabinAudio]") {
    CabinAudioSynth s;
    s.SetHorn(true, false);
    const auto low = Render(s, CabinAudioSynth::kSampleRate);       // 1 s
    CHECK(std::fabs(CrossingsHz(low) - CabinAudioSynth::kHornLowHz) <= 2.0);

    s.SetHorn(false, true);
    const auto high = Render(s, CabinAudioSynth::kSampleRate);
    CHECK(std::fabs(CrossingsHz(high) - CabinAudioSynth::kHornHighHz) <= 2.0);

    s.SetHorn(true, true);
    float peak = 0.0f;
    for (float x : Render(s, CabinAudioSynth::kSampleRate)) peak = std::max(peak, std::fabs(x));
    CHECK(peak > CabinAudioSynth::kHornLevel);              // both tones mixed
    CHECK(peak <= 2.0f * CabinAudioSynth::kHornLevel);

    s.SetHorn(false, false);
    CHECK(NonZero(Render(s, 4096)) == 0);
}

TEST_CASE("CabinAudio: a headless run opens no device", "[CabinAudio]") {
    CabinAudio audio(/*open_device=*/false);
    CHECK_FALSE(audio.playing());
    audio.PlayTick();               // harmless without a device
    audio.SetHorn(true, true);
}
