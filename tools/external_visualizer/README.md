# External Visualizer Example

An "outside program" that participates in the simulation as a pure consumer.
Stand-in for a 3D vehicle viewer: it owns a catalog of visual components
(bulbs, indicators, gauges), each bound to a signal on the realtime I/O
fabric. Whatever circuit publishes that signal drives the corresponding
component's visual state.

This example only renders ASCII, but the wiring is the same one a real 3D
viewer would use: connect to the shared-memory bus, subscribe to signals,
react to deltas.

## What makes this different from the harness examples

The programs in `examples/realtime_harness/` all link `electricsim_core`,
which in turn pulls in `libsimavr` and `libelf` (needed to simulate MCU
firmware). A visualizer doesn't simulate anything — it just watches pins.

So this example links against a smaller static library, `electricsim_connector`,
which contains only the fabric's wire protocol, shared-memory transport,
and signal catalog. No MCU toolchain needed to build or run it.

That library is the intended integration point for any external participant
— including, eventually, a thin C++ shim that speaks HTTP/WebSocket so
non-C++ visualizers (Python, JS, Unity/Unreal editors, etc.) can join without
linking the native library directly.

## Layout

- `components.hpp` — the visualizer's own registry of components and which
  fabric signals they listen to. Edit this to bind more components.
- `visualizer.cpp` — connects to the bus, polls frames, renders the scene.
- `Makefile` — convenience targets that delegate to the top-level CMake build.

## Running

Build alongside the harness (any one of these works):

```sh
# from this directory
make build

# or from the repo root
cmake -S . -B build && cmake --build build --target ex_external_visualizer
```

Start the harness first (it owns the firmware simulation and produces signal
data), then the visualizer:

```sh
cd examples/realtime_harness
make build
make start-vehicle
make start-avr

cd ../external_visualizer
make start          # foreground, ANSI redraw
```

You should see lamps flip as the AVR firmware toggles its output pins and the
door-ajar sensor cycles.

## Adding a component

1. Pick a qualified signal name and a stable numeric ID (see
   `examples/realtime_harness/harness_signals.hpp` for the existing range —
   3000–3099 is already in use).
2. Have whatever circuit/process owns the wire publish `DeltaBatch` frames
   for that signal.
3. Add a row to the `kComponents` table in `components.hpp`:

   ```cpp
   {3010, "vehicle.body.left_rear.brake_light_cmd",
    "left_rear_brake", "rear", true},
   ```

4. Rebuild — no other code changes.

## Future: non-C++ visualizers

The `electricsim_connector` library is the contract. A Python (or Unity,
Unreal, web) visualizer has two options:

- **Re-implement the wire protocol natively.** It's a small binary format in
  `src/io/protocol.hpp` over a POSIX shared-memory ring. Doable but not free.
- **Stand up a bridge process.** A short C++ program that speaks connector on
  one side and WebSocket/HTTP/gRPC on the other. The foreign visualizer only
  ever sees JSON (or whatever); the bridge handles the binary protocol and
  late-join / snapshot mechanics. This is the expected path and is left for a
  follow-up example.
