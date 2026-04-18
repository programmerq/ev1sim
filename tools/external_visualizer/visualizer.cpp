// External visualizer example.
//
// Models what a 3D vehicle viewer (or a dashboard, or any "downstream consumer
// of pin states") would do: join the realtime I/O fabric as an independent
// process, subscribe to the signals it cares about, and render a picture of
// the vehicle driven entirely by what the simulated circuit publishes.
//
// The visualizer does NOT drive any circuit signals itself -- it is a pure
// consumer that owns the scene. Compile and run it alongside the programs
// in examples/realtime_harness/ to see it react live.

#include "components.hpp"

#include "protocol.hpp"
#include "shm_transport.hpp"
#include "signal_catalog.hpp"

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

using electricsim::external_visualizer::Component;
using electricsim::external_visualizer::kComponents;

constexpr std::uint32_t kStreamVisualizer = 0x5649535Au; // "VISZ"
constexpr char kBusName[] = "electricsim_harness_bus";

struct ComponentState {
  bool seen{false};
  bool digital{false};
  std::uint32_t analog_mv{0};
};

const Component* find_component(std::uint32_t signal_id) {
  for (const auto& c : kComponents) {
    if (c.signal_id == signal_id) return &c;
  }
  return nullptr;
}

std::uint32_t read_u32_le(const std::vector<std::uint8_t>& payload) {
  if (payload.size() < 4) return 0;
  return static_cast<std::uint32_t>(payload[0]) |
         (static_cast<std::uint32_t>(payload[1]) << 8u) |
         (static_cast<std::uint32_t>(payload[2]) << 16u) |
         (static_cast<std::uint32_t>(payload[3]) << 24u);
}

// Announce ourselves on the bus: publish one SignalDefine frame per component
// we claim to visualize. This is optional -- the harness peers don't require
// it -- but it makes the visualizer self-documenting to anything else watching
// the fabric (e.g. bulb_watcher).
void publish_presence(electricsim::io::SharedMemoryTransport& transport, std::uint64_t& seq) {
  using namespace electricsim::io;
  const auto now_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());

  Frame define{};
  define.header.type = FrameType::SignalDefine;
  define.header.stream_id = kStreamVisualizer;
  define.header.sequence = seq++;
  define.header.monotonic_time_ns = now_ns;
  for (const auto& c : kComponents) {
    DeltaRecord d{};
    d.signal_id = c.signal_id;
    d.encoding = SignalEncoding::Opaque;
    d.bit_width = 8;
    const std::string payload =
        std::string(c.upstream_signal) + "|visualizer:" + c.label + "@" + c.placement;
    d.payload.assign(payload.begin(), payload.end());
    define.deltas.push_back(std::move(d));
  }
  transport.publish_frame(define);
}

std::string lamp_badge(const Component& c, const ComponentState& st) {
  if (!st.seen) return "[ ? ] " + std::string(c.label);
  if (c.is_output_lamp) {
    return std::string(st.digital ? "[ * ] " : "[   ] ") + c.label;
  }
  if (c.signal_id == 3003u) { // battery_mv -- render as volts
    std::ostringstream oss;
    oss << "[" << std::fixed << std::setprecision(2)
        << (static_cast<double>(st.analog_mv) / 1000.0) << "V] " << c.label;
    return oss.str();
  }
  return std::string(st.digital ? "[ON ] " : "[off] ") + c.label;
}

void render(const std::map<std::uint32_t, ComponentState>& states) {
  std::cout << "\x1b[2J\x1b[H"; // clear + home
  std::cout << "External Vehicle Visualizer (bus: " << kBusName << ")\n";
  std::cout << "Components driven by the simulation:\n\n";

  const auto row = [&](const char* heading) {
    std::cout << "  " << heading << "\n";
    for (const auto& c : kComponents) {
      if (std::string(c.placement) != heading) continue;
      auto it = states.find(c.signal_id);
      ComponentState st = (it == states.end()) ? ComponentState{} : it->second;
      std::cout << "      " << lamp_badge(c, st) << "\n";
    }
    std::cout << "\n";
  };
  row("dashboard");
  row("front-left");
  row("rear");
  std::cout.flush();
}

} // namespace

int main() {
  using namespace electricsim::io;
  std::cout.setf(std::ios::unitbuf);

  SharedMemoryTransportOptions opts{};
  opts.name = kBusName;
  opts.create = true; // tolerate being started before the rest of the harness
  SharedMemoryTransport transport(opts);

  std::cout << "[visualizer] joined bus " << kBusName << "\n";

  std::uint64_t seq = 1;
  auto next_presence = std::chrono::steady_clock::now();

  std::map<std::uint32_t, ComponentState> states;
  for (const auto& c : kComponents) states[c.signal_id] = {};

  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_presence) {
      publish_presence(transport, seq);
      next_presence = now + std::chrono::seconds(2);
    }

    bool had_update = false;
    for (;;) {
      PollResult polled = transport.poll_frame(std::chrono::milliseconds(0));
      if (polled.status != PollStatus::Ok) break;
      if (polled.frame.header.stream_id == kStreamVisualizer) continue; // ignore our own echo
      if (polled.frame.header.type != FrameType::DeltaBatch) continue;

      for (const auto& d : polled.frame.deltas) {
        const Component* c = find_component(d.signal_id);
        if (!c) continue;
        ComponentState& st = states[d.signal_id];
        st.seen = true;
        if (c->is_output_lamp || c->signal_id == 3000u) {
          if (!d.payload.empty()) st.digital = (d.payload[0] & 1u) != 0;
        } else {
          st.analog_mv = read_u32_le(d.payload);
        }
        had_update = true;
      }
    }

    if (had_update) render(states);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  }
}
