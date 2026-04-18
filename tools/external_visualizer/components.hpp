#pragma once

#include <array>
#include <cstdint>

namespace electricsim::external_visualizer {

// A Component is the visualizer's side of the "wire": a physical thing in the
// scene (a bulb, an indicator, a sensor light) that is driven by a signal on
// the fabric.
//
// In a real 3D visualizer each entry would bind to a mesh, a light, a gauge
// needle, etc. Here we just render an ASCII badge so the example stays small.
struct Component {
  std::uint32_t signal_id;
  const char* upstream_signal;
  const char* label;
  const char* placement;
  bool is_output_lamp;
};

// Mapping for the signals produced by the realtime_harness example. Edit this
// table (or build your own) to bind additional components as the harness grows
// new signals -- e.g. {3010, "vehicle.body.left_rear.brake_light_cmd",
// "left_rear_brake", "rear-left", true}.
inline constexpr std::array<Component, 4> kComponents{{
    {3001, "avr0.portb.pb0.exterior_bulb_cmd",   "exterior_bulb",    "rear",       true},
    {3002, "avr0.portb.pb1.door_indicator_cmd",  "door_indicator",   "dashboard",  true},
    {3000, "vehicle.body.door_ajar_switch",      "door_ajar_sensor", "front-left", false},
    {3003, "vehicle.power.battery_mv",           "battery_gauge",    "dashboard",  false},
}};

} // namespace electricsim::external_visualizer
