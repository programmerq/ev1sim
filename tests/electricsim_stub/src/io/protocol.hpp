#pragma once
// CI STUB — minimal stand-in for electricsim's src/io/protocol.hpp.
// Mirrors only the surface the ev1sim connector uses, so the guarded
// (EV1SIM_HAVE_EXTERNAL_SIM=1) transport/publish path compiles + links in CI
// without a real electricsim checkout.  Not the real protocol.
#include <cstdint>
#include <vector>

namespace electricsim { namespace io {

enum class SignalEncoding { Unsigned, Signed, Float, Opaque };
enum class FrameType { Heartbeat, DeltaBatch, SignalDefine };
enum class PollStatus { Ok, Timeout, Closed, Corrupt };

struct DeltaRecord {
    std::uint32_t              signal_id = 0;
    SignalEncoding             encoding  = SignalEncoding::Unsigned;
    std::uint32_t              bit_width = 0;
    std::vector<std::uint8_t>  payload;
};

struct FrameHeader {
    FrameType     type              = FrameType::Heartbeat;
    std::uint32_t stream_id         = 0;
    std::uint64_t sequence          = 0;
    std::uint64_t monotonic_time_ns = 0;
};

struct Frame {
    FrameHeader              header;
    std::vector<DeltaRecord> deltas;
};

struct PollResult {
    PollStatus status = PollStatus::Timeout;
    Frame      frame;
};

}}  // namespace electricsim::io
