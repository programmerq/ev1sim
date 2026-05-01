#!/usr/bin/env python3
"""
ABS scenario validation report generator.

Aggregates the four ABS-test summary.txt files (high_mu, low_mu,
mu_jump, split_mu) produced by run_abs_compare.sh and emits a
single markdown memo with stop-distance comparison, per-wheel slip
stats, and ABS engagement counts.

Usage:
    ./scripts/abs_report.py <output.md>

Run scripts/run_abs_compare.sh for each test first; this script
reads /tmp/ev1sim_abs_<test>/summary.txt for each.
"""

from __future__ import annotations

import datetime
import re
import sys
from pathlib import Path

TESTS = ["high_mu", "low_mu", "mu_jump", "split_mu"]
SUMMARY_DIR_TMPL = "/tmp/ev1sim_abs_{test}"


def parse_summary(text: str) -> dict:
    """Extract key fields from a summary.txt produced by compare_abs_runs.py.

    Returns a dict with keys:
        on_brake_v, on_stop_dist, on_stop_t, on_final_v,
        on_fl_locked, on_fr_locked, on_rl_locked, on_rr_locked,
        on_fl_events, on_fr_events,
        (and the same for off_*)
    Missing fields are None.
    """
    out: dict = {"on_stopped": False, "off_stopped": False}
    section = None
    for line in text.splitlines():
        s = line.strip()
        if "BTCM-on" in s and "===" in s:
            section = "on"
        elif "BTCM-off" in s and "===" in s:
            section = "off"
        elif section is None:
            continue
        # Brake-on
        m = re.search(r"brake-on\s+at\s+t=\s*([\d.]+)\s*s,\s*v=\s*([\d.]+)", s)
        if m:
            out[f"{section}_brake_t"] = float(m.group(1))
            out[f"{section}_brake_v"] = float(m.group(2))
        # Stopped
        m = re.search(r"stopped\s+at\s+t=\s*([\d.]+)\s*s\s+Δt=\s*([\d.]+)\s*s\s+distance=\s*([\d.]+)", s)
        if m:
            out[f"{section}_stop_t"] = float(m.group(2))
            out[f"{section}_stop_dist"] = float(m.group(3))
            out[f"{section}_stopped"] = True
        # Did NOT stop
        m = re.search(r"did NOT stop within scenario.\s+Final speed:\s+([\d.]+)", s)
        if m:
            out[f"{section}_final_v"] = float(m.group(1))
        # Per-wheel slip lines like "FL: peak=  0.040  mean=  0.010  locked=  0.0%"
        m = re.match(r"(FL|FR|RL|RR):\s+peak=\s*([\d.]+)\s+mean=\s*([\d.]+)\s+locked=\s*([\d.]+)%", s)
        if m:
            wheel = m.group(1).lower()
            out[f"{section}_{wheel}_peak"]   = float(m.group(2))
            out[f"{section}_{wheel}_mean"]   = float(m.group(3))
            out[f"{section}_{wheel}_locked"] = float(m.group(4))
        # ABS phase transitions
        m = re.search(r"ABS phase transitions:\s+FL=(\d+)\s+FR=(\d+)", s)
        if m:
            out[f"{section}_fl_events"] = int(m.group(1))
            out[f"{section}_fr_events"] = int(m.group(2))
    return out


def stop_str(d: dict, prefix: str) -> str:
    """Format a stop-distance/final-speed cell for the comparison table."""
    if d.get(f"{prefix}_stopped"):
        dist = d.get(f"{prefix}_stop_dist", 0)
        dt   = d.get(f"{prefix}_stop_t", 0)
        return f"{dist:.1f} m / {dt:.2f} s"
    if (v := d.get(f"{prefix}_final_v")) is not None:
        return f"didn't stop, v\\_f = {v:.2f} m/s"
    return "—"


def locked_str(d: dict, prefix: str) -> str:
    fl = d.get(f"{prefix}_fl_locked", 0)
    fr = d.get(f"{prefix}_fr_locked", 0)
    rl = d.get(f"{prefix}_rl_locked", 0)
    rr = d.get(f"{prefix}_rr_locked", 0)
    return f"FL {fl:.0f}% / FR {fr:.0f}% / RL {rl:.0f}% / RR {rr:.0f}%"


def events_str(d: dict, prefix: str) -> str:
    fl = d.get(f"{prefix}_fl_events", 0)
    fr = d.get(f"{prefix}_fr_events", 0)
    return f"{fl} / {fr}"


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 1
    out_path = Path(argv[1])

    parsed: dict[str, dict] = {}
    missing: list[str] = []
    for t in TESTS:
        p = Path(SUMMARY_DIR_TMPL.format(test=t)) / "summary.txt"
        if not p.exists():
            missing.append(str(p))
            continue
        parsed[t] = parse_summary(p.read_text())

    if missing:
        print("warning: missing summary files (run scripts/run_abs_compare.sh first):",
              file=sys.stderr)
        for m in missing:
            print(f"  - {m}", file=sys.stderr)

    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M %Z").strip()

    lines: list[str] = []
    lines.append("# EV1 ABS scenario validation — comparison report")
    lines.append("")
    lines.append(f"_Generated {ts} by `scripts/abs_report.py`._")
    lines.append("")
    lines.append("Source: `/tmp/ev1sim_abs_<test>/summary.txt` for each of the four")
    lines.append("standard validation scenarios.  Re-run with")
    lines.append("`for t in high_mu low_mu mu_jump split_mu; do ./scripts/run_abs_compare.sh $t; done`")
    lines.append("then re-run this script.")
    lines.append("")

    lines.append("## Headline: stopping distance, BTCM-on vs BTCM-off")
    lines.append("")
    lines.append("| Test | BTCM-on stop | BTCM-off stop | Δ |")
    lines.append("|---|---|---|---|")
    for t in TESTS:
        d = parsed.get(t)
        if d is None:
            lines.append(f"| {t} | (no data) | (no data) | — |")
            continue
        on  = stop_str(d, "on")
        off = stop_str(d, "off")
        # Compute delta only if both stopped.
        delta = "—"
        if d.get("on_stopped") and d.get("off_stopped"):
            delta = f"{d['on_stop_dist'] - d['off_stop_dist']:+.1f} m"
        lines.append(f"| {t} | {on} | {off} | {delta} |")
    lines.append("")

    lines.append("## ABS engagement (front phase transitions per wheel)")
    lines.append("")
    lines.append("| Test | BTCM-on FL/FR | BTCM-off FL/FR |")
    lines.append("|---|---:|---:|")
    for t in TESTS:
        d = parsed.get(t)
        if d is None:
            lines.append(f"| {t} | — | — |")
            continue
        lines.append(f"| {t} | {events_str(d, 'on')} | {events_str(d, 'off')} |")
    lines.append("")

    lines.append("## Per-wheel time-locked percentage during brake event")
    lines.append("")
    lines.append("| Test | BTCM-on | BTCM-off |")
    lines.append("|---|---|---|")
    for t in TESTS:
        d = parsed.get(t)
        if d is None:
            lines.append(f"| {t} | — | — |")
            continue
        lines.append(f"| {t} | {locked_str(d, 'on')} | {locked_str(d, 'off')} |")
    lines.append("")

    lines.append("## Per-wheel peak slip during brake event")
    lines.append("")
    lines.append("| Test | BTCM-on (FL / FR / RL / RR) | BTCM-off (FL / FR / RL / RR) |")
    lines.append("|---|---|---|")
    for t in TESTS:
        d = parsed.get(t)
        if d is None:
            lines.append(f"| {t} | — | — |")
            continue
        on_peaks = " / ".join(
            f"{d.get(f'on_{w}_peak', 0):.2f}" for w in ("fl", "fr", "rl", "rr"))
        off_peaks = " / ".join(
            f"{d.get(f'off_{w}_peak', 0):.2f}" for w in ("fl", "fr", "rl", "rr"))
        lines.append(f"| {t} | {on_peaks} | {off_peaks} |")
    lines.append("")

    lines.append("## How to interpret")
    lines.append("")
    lines.append("- **Δ stop distance** in the headline table is BTCM-on minus")
    lines.append("  BTCM-off.  Positive = ABS made it worse, negative = ABS helped.")
    lines.append("  An ideal ABS lands close to zero on dry pavement (where wheel")
    lines.append("  lock isn't a real risk) and strongly negative on slippery")
    lines.append("  surfaces (where lock-prevention pays off).")
    lines.append("- **Phase transitions** = number of times the front-wheel ABS")
    lines.append("  state machine moved between APPLY/HOLD/DUMP during the brake")
    lines.append("  event.  Zero on dry pavement is correct; double-digit counts")
    lines.append("  on slippery surfaces show the algorithm is engaging.")
    lines.append("- **Time-locked** = % of brake event the wheel was at near-zero")
    lines.append("  rotation.  ABS is supposed to keep this at 0 % (or close).")
    lines.append("  High locked % despite high engagement count means the algorithm")
    lines.append("  is firing but not effectively modulating.")
    lines.append("- **Peak slip** = highest instantaneous slip ratio seen.  Real")
    lines.append("  ABS targets the peak-grip slip range (~0.10–0.15).  Slip near")
    lines.append("  1.0 means the wheel was fully locked.  Slip near 0 means it was")
    lines.append("  rolling cleanly.")
    lines.append("")

    out_path.write_text("\n".join(lines) + "\n")
    print(f"[abs-report] wrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
