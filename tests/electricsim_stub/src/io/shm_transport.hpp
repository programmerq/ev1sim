#pragma once
// CI STUB — minimal stand-in for electricsim's src/io/shm_transport.hpp.
// Header-only no-op transport: enough for the connector's guarded path to
// compile + link in CI.  publish_frame() succeeds and poll_frame() always
// times out, so an enabled connector reaches "connecting"/"connected" without
// any real shared-memory bus.  Not the real transport.
#include "protocol.hpp"

#include <chrono>
#include <string>

namespace electricsim { namespace io {

struct SharedMemoryTransportOptions {
    std::string name;
    bool        create = false;
};

class SharedMemoryTransport {
public:
    explicit SharedMemoryTransport(const SharedMemoryTransportOptions&) {}
    bool publish_frame(const Frame&) { return true; }
    PollResult poll_frame(std::chrono::milliseconds) {
        return PollResult{PollStatus::Timeout, Frame{}};
    }
};

}}  // namespace electricsim::io
