# IPC + RSA in-app display surfaces — ev1sim HUD rework plan

**Status:** Planned (backlog). Drafted 2026-06-21.
**Tracking:** `docs/TODO.md` → "IPC + RSA display rework (planned)".

## Why

ev1sim's HUD today is an ad-hoc set of `FloatingUiPanel` rows. We want to
surface the **IPC instrument cluster** and **RSA center-console** state as
first-class in-app displays — and eventually accept input from them — in the
spirit of the period Palm EV1 dashboard app.

The immediate trigger: the BPM pack-voltage readout (chassis `4139`) went
permanently blank when electricsim retired that cell (Phase C-b — PIM, its only
consumer, moved to the HV-bus rail `4155` as the physically-correct node). The
display layer needs rework anyway, so rather than re-add a one-off cell we
re-base ev1sim's dashboard on a faithful "snoop the bus" data source.

## Decisions (ratified 2026-06-21)

| Axis | Decision |
|---|---|
| **Data source** | **Snoop & decode the GM-8192 frame cells.** Decode the per-module `*_TX` bit-stream wire cells with `gm8192_rx_framer` to recover whatever a module broadcasts — like the Palm app reading the ALDL/serial line. No per-value electricsim cell required. |
| **Render** | **2D floating panel(s) first; 3D in-scene later.** |
| **First cut** | **IPC instrument cluster.** |
| **Interactivity** | **Display-only first;** clickable input is a later phase. |
| **Aesthetic** | Palm-EV1-dashboard styling for the 2D panels; embedding/emulating the real Palm app is a far stretch goal. |

## Architecture: the "bus snoop" layer

ev1sim is a WireTable peer (post wire-truth); the GM-8192 module traffic lives
on the bus as serialized `*_TX` bit-stream cells —
`kWireGM8192_{BPM,BTCM,PIM,RSA,HTCM,IPC,APM,SCANTOOL}_TX` (+ `BPM_TX_CLASS2`).
ev1sim already peeks `GM8192_BTCM_TX` (`WireTruthChassis::btcm_tx_total_bits`)
and links `gm8192_rx_framer`, so the groundwork exists.

Add a `BusSnoop` decode layer that, each tick:
- reads the relevant `*_TX` cells and runs `gm8192_rx_framer` to recover frames;
- decodes module frames into a typed `DashSnapshot` (speed, odometer/trip, pack
  voltage, IPC telltales, key / charge / run-mode state, …);
- is `written()` / freshness-gated — a stale or never-written frame leaves its
  field blank (kHold-safe), exactly like the existing consumer overlay.

Frame field maps come from electricsim's serial-message catalog +
`docs/gm8192_protocol.md`. Where a wanted value is **not** carried in any frame,
fall back to a dedicated chassis cell (additive / MINOR on electricsim) — but the
default is "decode what's already on the wire."

**No chassis-contract bump is required for the snoop path** — it reads existing
`*_TX` cells. Any dedicated-cell fallback would be additive (MINOR), never MAJOR.

## Phases

### Phase 0 — retire the dead 4139 pack-voltage path (prereq, ev1sim-only)
Remove the now-dead BPM pack-voltage consumer:
- `kSigBpmPackVoltageMv` (4139) endpoint + its `DebugInjectU32` decode in
  `ExternalSimConnector`;
- `GetBpmPackVoltageMv()` / `HasReceivedBpmPackVoltage()` + the `m_state` fields;
- `FormatBpmPackVoltageLabel` (`FloatingUiPanel.{h,cpp}`) + its test;
- the `SimApp.cpp` floating-UI wiring (~L832).

Nothing produces 4139 since electricsim retired it, so the readout is
permanently blank. Pack voltage returns in Phase 1 via frame snoop (BPM
broadcasts it on its GM-8192 status frame).

### Phase 1 — IPC instrument cluster, 2D, snoop-fed (display-only)
- `BusSnoop` decode of the IPC / BPM / BTCM frames → `DashSnapshot` (speed,
  odo/trip, pack V, telltales, key state).
- `InstrumentClusterPanel` (2D) built from pure, unit-testable `Format*` label
  helpers (no Irrlicht dependency), following the existing `FloatingUiPanel`
  pattern. Reuse the IPC telltale cells ev1sim already consumes (4130-4145);
  snoop-decode the rest.
- Tests: frame-decode units (feed synthetic `*_TX` bit-streams → assert
  snapshot), pure label-format units, one `[e2e]` panel-from-snapshot test.

### Phase 2 — RSA center console, 2D, snoop-fed (display-only)
- `RsaConsolePanel` mirroring electricsim's `ev1/rsa/rsa_panel_catalog.yaml`:
  mode LEDs (OFF/ACC/RUN), keypad / valid-code state, PRND, clock, HVAC head.
- Source from `GM8192_RSA_TX` snoop + the RSA output cells ev1sim already reads
  (run-mode, shift-blocked 4088).

### Phase 3 — 3D in-scene models
- Promote the cluster + console to 3D objects in the Chrono/Irrlicht cabin
  (gauges, LCD, console). Same `DashSnapshot` feed; swap the render backend.

### Phase 4 — interactivity (input)
- Make RSA console controls clickable to drive the bus (keypad / mode / PRND),
  reusing ev1sim's existing driver-input producers (it already publishes these
  via the keyboard path).
- **Multi-writer arbitration:** the web `ev1_rsa_panel` (electricsim) and ev1sim
  would both be RSA input sources. Settle "who owns the input" with electricsim's
  RSA controller before enabling. This is an arbitration policy, **not** a
  contract MAJOR.

## Risks / open questions
- **Frame coverage** — confirm each wanted display value is actually carried in a
  GM-8192 frame; enumerate any that need a dedicated chassis cell fallback.
- **Telltale source of truth** — prefer the dedicated chassis telltale cells
  (4130-4145) where they exist over re-deriving from frames, to avoid two
  sources for the same lamp.
- **Decode cost on the render thread** — snoop on the existing tick; keep decode
  cheap and cache the `DashSnapshot` per poll cycle.
- **3D asset effort** (Phase 3) — cluster/console meshes are a larger lift than
  the 2D panels; sequence accordingly.
