// CabinAudio through a real SDL3 output: SDL's "disk" audio driver writes what
// the device would play to a file, so the test hears the tick without speakers.
// Built only with SDL3 (EV1SIM_HAVE_WHEEL_IO).

#include <catch2/catch_test_macros.hpp>

#include <SDL3/SDL.h>

#include "CabinAudio.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <process.h>   // _getpid
#define EV1SIM_GETPID _getpid
#else
#include <unistd.h>    // getpid
#define EV1SIM_GETPID ::getpid
#endif

TEST_CASE("CabinAudio: a tick reaches the SDL output device", "[CabinAudio][sdl]") {
    const std::string path = "ev1sim_cabin_audio_" + std::to_string(static_cast<long>(EV1SIM_GETPID())) + ".raw";
    SDL_SetHint(SDL_HINT_AUDIO_DRIVER, "disk");
    SDL_SetHint(SDL_HINT_AUDIO_DISK_OUTPUT_FILE, path.c_str());
    SDL_SetHint(SDL_HINT_AUDIO_DISK_TIMESCALE, "1.0");
    {
        CabinAudio audio(/*open_device=*/true);
        REQUIRE(audio.playing());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        audio.PlayTick();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    SDL_ResetHint(SDL_HINT_AUDIO_DRIVER);
    SDL_ResetHint(SDL_HINT_AUDIO_DISK_OUTPUT_FILE);
    SDL_ResetHint(SDL_HINT_AUDIO_DISK_TIMESCALE);

    std::FILE* f = std::fopen(path.c_str(), "rb");
    REQUIRE(f != nullptr);
    std::vector<float> samples;
    float x = 0.0f;
    while (std::fread(&x, sizeof x, 1, f) == 1) samples.push_back(x);
    std::fclose(f);
    std::remove(path.c_str());

    // The device may resample, so count loud samples rather than exact frames:
    // a 12 ms burst at the device rate, give or take a buffer edge.
    std::size_t loud = 0;
    for (float s : samples) loud += (std::fabs(s) > 0.1f);
    CHECK(samples.size() > 0);
    CHECK(loud > 0);
    CHECK(loud < samples.size() / 4);   // one short tick, not a continuous buzz
}
