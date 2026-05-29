#include "SounderAudio.h"

#ifdef __APPLE__

#include <AudioToolbox/AudioToolbox.h>
#include <atomic>
#include <cmath>
#include <iostream>

// ── Constants ──────────────────────────────────────────────────────────────

static constexpr double kSampleRate  = 44100.0;
static constexpr double kPiezoFreq   = 2700.0;   // typical piezo buzzer pitch
static constexpr float  kVolume      = 0.20f;
// One-shot click duration (~40 ms) for the door-lock solenoid actuation.
static const int        kClickFrames = static_cast<int>(0.04 * kSampleRate);

// ── Implementation ─────────────────────────────────────────────────────────

struct SounderAudioImpl {
    AudioComponentInstance outputUnit = nullptr;

    std::atomic<bool> sounding{false};
    std::atomic<int>  click_frames{0};   // one-shot countdown set by PlayClick()

    // Phase accumulator (audio thread only).
    double phase = 0.0;
};

// Pimpl bridge — forward-declared in the header as SounderAudio::Impl.
struct SounderAudio::Impl : SounderAudioImpl {};

static OSStatus renderCallback(void* refCon,
                               AudioUnitRenderActionFlags* /*flags*/,
                               const AudioTimeStamp* /*timestamp*/,
                               UInt32 /*busNumber*/,
                               UInt32 numFrames,
                               AudioBufferList* bufferList) {
    auto* impl = static_cast<SounderAudioImpl*>(refCon);
    auto* buf  = static_cast<float*>(bufferList->mBuffers[0].mData);

    const bool sounding = impl->sounding.load(std::memory_order_relaxed);
    // Best-effort one-shot: read the click countdown, decrement locally, store
    // back.  A PlayClick() racing this callback may rarely lose/truncate a
    // click by a few ms — inaudible for a sounder.
    int click = impl->click_frames.load(std::memory_order_relaxed);

    constexpr double TWO_PI = 2.0 * M_PI;
    const double inc = TWO_PI * kPiezoFreq / kSampleRate;

    for (UInt32 i = 0; i < numFrames; ++i) {
        const bool on = sounding || click > 0;
        // Square wave — a piezo element is square-driven.
        buf[i] = on ? (std::sin(impl->phase) >= 0.0 ? kVolume : -kVolume) : 0.0f;

        // Always advance phase so restarts are click-free.
        impl->phase += inc;
        if (impl->phase >= TWO_PI) impl->phase -= TWO_PI;
        if (click > 0) --click;
    }

    impl->click_frames.store(click, std::memory_order_relaxed);
    return noErr;
}

// ── Public API ─────────────────────────────────────────────────────────────

SounderAudio::SounderAudio() : m_impl(std::make_unique<Impl>()) {
    // Find the default output AudioUnit.
    AudioComponentDescription desc{};
    desc.componentType         = kAudioUnitType_Output;
    desc.componentSubType      = kAudioUnitSubType_DefaultOutput;
    desc.componentManufacturer = kAudioUnitManufacturer_Apple;

    AudioComponent comp = AudioComponentFindNext(nullptr, &desc);
    if (!comp) {
        std::cerr << "[SounderAudio] No default audio output found.\n";
        return;
    }

    OSStatus err = AudioComponentInstanceNew(comp, &m_impl->outputUnit);
    if (err != noErr) {
        std::cerr << "[SounderAudio] Failed to create AudioUnit (" << err << ").\n";
        return;
    }

    // Set render callback.
    AURenderCallbackStruct cb{};
    cb.inputProc       = renderCallback;
    cb.inputProcRefCon = m_impl.get();
    AudioUnitSetProperty(m_impl->outputUnit,
                         kAudioUnitProperty_SetRenderCallback,
                         kAudioUnitScope_Input, 0,
                         &cb, sizeof(cb));

    // Configure stream format: mono float32 @ 44.1 kHz.
    AudioStreamBasicDescription fmt{};
    fmt.mSampleRate       = kSampleRate;
    fmt.mFormatID         = kAudioFormatLinearPCM;
    fmt.mFormatFlags      = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
    fmt.mBytesPerPacket   = sizeof(float);
    fmt.mFramesPerPacket  = 1;
    fmt.mBytesPerFrame    = sizeof(float);
    fmt.mChannelsPerFrame = 1;
    fmt.mBitsPerChannel   = 32;
    AudioUnitSetProperty(m_impl->outputUnit,
                         kAudioUnitProperty_StreamFormat,
                         kAudioUnitScope_Input, 0,
                         &fmt, sizeof(fmt));

    AudioUnitInitialize(m_impl->outputUnit);
    AudioOutputUnitStart(m_impl->outputUnit);
}

SounderAudio::~SounderAudio() {
    if (m_impl && m_impl->outputUnit) {
        AudioOutputUnitStop(m_impl->outputUnit);
        AudioUnitUninitialize(m_impl->outputUnit);
        AudioComponentInstanceDispose(m_impl->outputUnit);
    }
}

void SounderAudio::SetSounding(bool sounding) {
    m_impl->sounding.store(sounding, std::memory_order_relaxed);
}

void SounderAudio::PlayClick() {
    m_impl->click_frames.store(kClickFrames, std::memory_order_relaxed);
}

#else  // !__APPLE__

struct SounderAudio::Impl {};

SounderAudio::SounderAudio() : m_impl(std::make_unique<Impl>()) {}
SounderAudio::~SounderAudio() = default;
void SounderAudio::SetSounding(bool) {}
void SounderAudio::PlayClick() {}

#endif
