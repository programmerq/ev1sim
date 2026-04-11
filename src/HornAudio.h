#pragma once

#include <memory>

// Real-time two-tone horn audio generator.
// Uses CoreAudio (AudioUnit) on macOS.  No-op on other platforms.
class HornAudio {
public:
    HornAudio();
    ~HornAudio();

    // Set which tones are currently active.  Call once per frame.
    // Tones sound while true; silence resumes immediately on false.
    void SetTones(bool low_on, bool high_on);

    // Non-copyable.
    HornAudio(const HornAudio&) = delete;
    HornAudio& operator=(const HornAudio&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
