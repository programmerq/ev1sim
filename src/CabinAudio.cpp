#include "CabinAudio.h"

#include <iostream>

#ifdef EV1SIM_HAVE_WHEEL_IO

#include <SDL3/SDL.h>

#include <vector>

// The SDL side: a playback stream whose callback pulls samples from the synth
// whenever the device wants more.  The Device is the callback's userdata, so
// it owns everything the audio thread touches besides the synth.
struct CabinAudio::Device {
    ev1sim::CabinAudioSynth* synth        = nullptr;
    SDL_AudioStream*         stream       = nullptr;
    std::vector<float>       buf;          // audio thread only
    bool                     subsystem_up = false;
};

namespace {
void SDLCALL FeedStream(void* userdata, SDL_AudioStream* stream,
                        int additional_amount, int /*total_amount*/) {
    auto* dev = static_cast<CabinAudio::Device*>(userdata);
    if (additional_amount <= 0) return;
    const auto frames = static_cast<std::size_t>(additional_amount) / sizeof(float);
    if (dev->buf.size() < frames) dev->buf.resize(frames);
    dev->synth->Render(dev->buf.data(), frames);
    SDL_PutAudioStreamData(stream, dev->buf.data(), static_cast<int>(frames * sizeof(float)));
}
}  // namespace

CabinAudio::CabinAudio(bool open_device) {
    if (!open_device) return;
    m_device = std::make_unique<Device>();
    m_device->synth = &m_synth;
    if (!SDL_InitSubSystem(SDL_INIT_AUDIO)) {
        std::cerr << "[CabinAudio] SDL audio init failed: " << SDL_GetError()
                  << " — cabin audio off\n";
        m_device.reset();
        return;
    }
    m_device->subsystem_up = true;

    SDL_AudioSpec spec{};
    spec.format   = SDL_AUDIO_F32;
    spec.channels = 1;
    spec.freq     = ev1sim::CabinAudioSynth::kSampleRate;
    m_device->stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec,
                                                 FeedStream, m_device.get());
    if (!m_device->stream) {
        std::cerr << "[CabinAudio] no audio output: " << SDL_GetError()
                  << " — cabin audio off\n";
        return;
    }
    SDL_ResumeAudioStreamDevice(m_device->stream);
}

CabinAudio::~CabinAudio() {
    if (!m_device) return;
    // Destroying the stream stops the callback before the synth goes away.
    if (m_device->stream) SDL_DestroyAudioStream(m_device->stream);
    if (m_device->subsystem_up) SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

bool CabinAudio::playing() const {
    return m_device && m_device->stream != nullptr;
}

#else  // !EV1SIM_HAVE_WHEEL_IO

struct CabinAudio::Device {};

CabinAudio::CabinAudio(bool open_device) {
    if (open_device)
        std::cerr << "[CabinAudio] built without SDL3 (EV1SIM_HAVE_WHEEL_IO=OFF)"
                     " — cabin audio off\n";
}

CabinAudio::~CabinAudio() = default;

bool CabinAudio::playing() const { return false; }

#endif
