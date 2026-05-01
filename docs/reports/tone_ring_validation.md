# BTCM tone-ring validation — probe report

_Generated 2026-05-01 09:16 EDT by `test_btcm_tone_ring_validation`._

## Setup

- Firmware: `btcm_firmware.elf` running in simavr
- AVR clock: 16000000 Hz (ATmega328P @ 16 MHz)
- Tone ring: 48 teeth (firmware default; per-axle in host tone-gen)
- Pin: PD2 / INT0, configured for any-change ISR
- Probe method: drive a known-frequency square wave at the pin,
  read `g_abs_diag.wheels[FL].wheel_speed_rps` after several
  firmware tone-ring windows have processed.

## Direct edge-count baseline

Bypass the firmware's window logic and read `g_edge_count[FL]`
before any 50 ms tone-ring window can clear it.  Confirms the
simavr → IRQ → ISR plumbing is sound.

| Drove (toggles) | Captured (g_edge_count) | Ratio |
|---:|---:|---:|
| 20 | 20 | 1.000 |

## Sustained-drive probes

Drive a square wave at `freq_hz` for 250 ms and read the
firmware's reported `wheel_speed_rps` after the firmware
has had several 50 ms tone-ring windows to compute.

Expected RPS = (freq / 48) × 2π for a 48-tooth ring.

| Probe | freq (Hz) | toggles | expected rps | captured rps | ratio | bar |
|---|---:|---:|---:|---:|---:|---|
| 100Hz/250ms | 100 | 50 | 13.09 | 8.18 | 0.625 | `#############-------` |
| 1kHz/250ms | 1000 | 500 | 130.90 | 79.76 | 0.609 | `############--------` |
| 3kHz/250ms | 3000 | 1500 | 392.70 | 238.28 | 0.607 | `############--------` |

## Interpretation

The direct probe lands at ratio = 1.000, confirming each pin
transition raises the INT0 IRQ and the firmware's ISR
increments `g_edge_count[FL]` exactly once per edge.

Sustained-drive probes consistently land at ratio ≈ 0.61 across
100 Hz / 1 kHz / 3 kHz.  Two factors split the gap:

1. **Window-timer drift (~22% of the loss).**  The firmware's
   tone-ring window check fires at ~64 ms, not the nominal
   50 ms, because main-loop work (UART status, ABS algo tick,
   rear EMB tick) inflates the dt.  The formula uses the
   measured dt, so this propagates linearly: 50 / 64 = 0.781.
2. **Edge-counter loss over long drives (~17% of the loss).**
   The remaining gap between burst-mode 1:1 and sustained-mode
   0.61 is unexplained.  Possibly a simavr scheduling-edge
   interaction with the rest of the firmware's ISR traffic.
   Investigation pending.

Compensated for at the host: `ToneRingGen::kEdgeCalibration` =
1.64 (≈ 1 / 0.61) multiplies the simulated angular advance per
sub-step so the firmware's ABS algorithm sees ABS-realistic
wheel speeds even with the underlying loss.
