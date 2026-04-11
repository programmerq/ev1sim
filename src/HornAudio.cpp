#include "HornAudio.h"

#ifdef __APPLE__

#include <AudioToolbox/AudioToolbox.h>
#include <atomic>
#include <cmath>
#include <iostream>

// ── Constants ──────────────────────────────────────────────────────────────

static constexpr double kSampleRate = 44100.0;
static constexpr double kLowFreq   = 400.0;
static constexpr double kHighFreq  = 500.0;
static constexpr float  kVolume    = 0.25f;   // per-tone amplitude

// ── Implementation ─────────────────────────────────────────────────────────

struct HornAudioImpl {
    AudioComponentInstance outputUnit = nullptr;

    std::atomic<bool> low_on{false};
    std::atomic<bool> high_on{false};

    // Phase accumulators (audio thread only — no atomics needed).
    double phase_low  = 0.0;
    double phase_high = 0.0;
};

// Pimpl bridge — forward-declared in the header as HornAudio::Impl.
struct HornAudio::Impl : HornAudioImpl {};

static OSStatus renderCallback(void* refCon,
                                AudioUnitRenderActionFlags* /*flags*/,
                                const AudioTimeStamp* /*timestamp*/,
                                UInt32 /*busNumber*/,
                                UInt32 numFrames,
                                AudioBufferList* bufferList) {
    auto* impl = static_cast<HornAudioImpl*>(refCon);
    auto* buf  = static_cast<float*>(bufferList->mBuffers[0].mData);

    const bool lo = impl->low_on.load(std::memory_order_relaxed);
    const bool hi = impl->high_on.load(std::memory_order_relaxed);

    constexpr double TWO_PI = 2.0 * M_PI;
    const double incLo = TWO_PI * kLowFreq  / kSampleRate;
    const double incHi = TWO_PI * kHighFreq / kSampleRate;

    for (UInt32 i = 0; i < numFrames; ++i) {
        float sample = 0.0f;
        if (lo) sample += kVolume * static_cast<float>(sin(impl->phase_low));
        if (hi) sample += kVolume * static_cast<float>(sin(impl->phase_high));
        buf[i] = sample;

        // Always advance phase so restarts are click-free.
        impl->phase_low  += incLo;
        impl->phase_high += incHi;
        if (impl->phase_low  >= TWO_PI) impl->phase_low  -= TWO_PI;
        if (impl->phase_high >= TWO_PI) impl->phase_high -= TWO_PI;
    }

    return noErr;
}

// ── Public API ─────────────────────────────────────────────────────────────

HornAudio::HornAudio() : m_impl(std::make_unique<Impl>()) {
    // Find the default output AudioUnit.
    AudioComponentDescription desc{};
    desc.componentType         = kAudioUnitType_Output;
    desc.componentSubType      = kAudioUnitSubType_DefaultOutput;
    desc.componentManufacturer = kAudioUnitManufacturer_Apple;

    AudioComponent comp = AudioComponentFindNext(nullptr, &desc);
    if (!comp) {
        std::cerr << "[HornAudio] No default audio output found.\n";
        return;
    }

    OSStatus err = AudioComponentInstanceNew(comp, &m_impl->outputUnit);
    if (err != noErr) {
        std::cerr << "[HornAudio] Failed to create AudioUnit (" << err << ").\n";
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

HornAudio::~HornAudio() {
    if (m_impl && m_impl->outputUnit) {
        AudioOutputUnitStop(m_impl->outputUnit);
        AudioUnitUninitialize(m_impl->outputUnit);
        AudioComponentInstanceDispose(m_impl->outputUnit);
    }
}

void HornAudio::SetTones(bool low_on, bool high_on) {
    m_impl->low_on.store(low_on,  std::memory_order_relaxed);
    m_impl->high_on.store(high_on, std::memory_order_relaxed);
}

#else  // !__APPLE__

struct HornAudio::Impl {};

HornAudio::HornAudio() : m_impl(std::make_unique<Impl>()) {}
HornAudio::~HornAudio() = default;
void HornAudio::SetTones(bool, bool) {}

#endif
