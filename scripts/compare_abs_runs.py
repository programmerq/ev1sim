#!/usr/bin/env python3
"""Compare two ABS hard-brake CSVs (BTCM-on vs BTCM-off).

Prints:
  - Stopping-distance + time-to-stop summary
  - Per-wheel slip-ratio peaks (locked = 1.0)
  - ASCII speed-vs-time plot, two lines overlaid
  - ASCII per-wheel-slip plot for the BTCM-on run

stdlib only — no matplotlib needed.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path


def load_csv(path: Path) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    with path.open() as f:
        reader = csv.DictReader(f)
        for r in reader:
            out.append({k: float(v) if v else 0.0 for k, v in r.items()})
    return out


def find_brake_event(rows: list[dict]) -> int:
    """Return index where applied_front_brake first crosses 0.5."""
    for i, r in enumerate(rows):
        if r.get("applied_front_brake", 0) > 0.5:
            return i
    return -1


def find_stop(rows: list[dict], start_idx: int, threshold: float = 0.3) -> int:
    """First row at/after start_idx where speed_mps drops below threshold."""
    for i in range(start_idx, len(rows)):
        if rows[i]["speed_mps"] < threshold:
            return i
    return -1


def stopping_distance(rows: list[dict], start_idx: int, stop_idx: int) -> float:
    """Euclidean distance from brake-on to stop (m)."""
    if start_idx < 0 or stop_idx < 0:
        return float("nan")
    x0 = rows[start_idx].get("pos_x", 0.0)
    x1 = rows[stop_idx].get("pos_x", 0.0)
    # Treat pos_y as constant in straight-line scenarios; include just in case.
    return abs(x1 - x0)


def peak_slip(rows: list[dict], start_idx: int, stop_idx: int, wheel: str) -> float:
    """Maximum |slip_ratio_<wheel>| over the brake event."""
    key = f"slip_ratio_{wheel}"
    if key not in rows[0]:
        return float("nan")
    end = stop_idx if stop_idx > 0 else len(rows)
    return max(abs(r[key]) for r in rows[start_idx:end])


def ascii_plot_overlay(
    series: list[tuple[str, list[tuple[float, float]]]],
    width: int = 60,
    height: int = 18,
    x_label: str = "t",
    y_label: str = "v",
    chars: str = "*+",
) -> str:
    """Two-series overlay plot.  Each series is (label, [(x, y), ...])."""
    if not series or all(not pts for _, pts in series):
        return "(no data)"

    xs = [x for _, pts in series for x, _ in pts]
    ys = [y for _, pts in series for _, y in pts]
    if not xs or not ys:
        return "(no data)"

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    if x_max == x_min:
        x_max = x_min + 1.0
    if y_max == y_min:
        y_max = y_min + 1.0

    grid = [[" "] * width for _ in range(height)]

    for ch, (_, pts) in zip(chars, series):
        for x, y in pts:
            col = int((x - x_min) / (x_max - x_min) * (width - 1))
            row = height - 1 - int((y - y_min) / (y_max - y_min) * (height - 1))
            if 0 <= col < width and 0 <= row < height:
                if grid[row][col] == " ":
                    grid[row][col] = ch
                elif grid[row][col] != ch:
                    grid[row][col] = "#"  # both series at this cell

    lines = []
    for r, row in enumerate(grid):
        # Y-axis labels at top, middle, bottom.
        if r == 0:
            label = f"{y_max:6.2f}"
        elif r == height - 1:
            label = f"{y_min:6.2f}"
        elif r == height // 2:
            label = f"{(y_min + y_max) / 2:6.2f}"
        else:
            label = "      "
        lines.append(f"{label} | {''.join(row)}")
    lines.append(" " * 7 + "+" + "-" * width)
    lines.append(
        " " * 7
        + " "
        + f"{x_min:.2f}".ljust(width // 2)
        + f"{x_max:.2f}".rjust(width - width // 2)
    )

    legend_parts = [f"{ch}={lbl}" for ch, (lbl, _) in zip(chars, series)]
    lines.append(f"        ({y_label} vs {x_label})  " + "  ".join(legend_parts) + "  #=both")
    return "\n".join(lines)


def fmt(x: float, prec: int = 2) -> str:
    if x != x:  # NaN check
        return "  n/a"
    return f"{x:7.{prec}f}"


def summarize(label: str, rows: list[dict]) -> tuple[int, int]:
    brake_idx = find_brake_event(rows)
    stop_idx = find_stop(rows, brake_idx) if brake_idx >= 0 else -1

    print(f"=== {label} ===")
    if brake_idx < 0:
        print("  brake event not detected")
        return brake_idx, stop_idx
    t_brake = rows[brake_idx]["sim_time_s"]
    v_brake = rows[brake_idx]["speed_mps"]
    print(f"  brake-on at t={t_brake:.2f} s, v={v_brake:.2f} m/s")
    if stop_idx >= 0:
        t_stop = rows[stop_idx]["sim_time_s"]
        dist = stopping_distance(rows, brake_idx, stop_idx)
        print(f"  stop  at t={t_stop:.2f} s  Δt={t_stop - t_brake:.2f} s  "
              f"distance={dist:.2f} m")
    else:
        v_final = rows[-1]["speed_mps"]
        print(f"  did NOT stop within scenario.  Final speed: {v_final:.2f} m/s")

    print("  peak |slip| during brake event:")
    for wheel in ("fl", "fr", "rl", "rr"):
        peak = peak_slip(rows, brake_idx, stop_idx if stop_idx > 0 else len(rows) - 1,
                         wheel)
        flag = "  ← LOCKED" if not math.isnan(peak) and peak >= 0.95 else ""
        print(f"    {wheel.upper()}: {fmt(peak, 3)}{flag}")
    print()
    return brake_idx, stop_idx


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print("usage: compare_abs_runs.py <btcm_on.csv> <btcm_off.csv>",
              file=sys.stderr)
        return 1
    on_path, off_path = Path(argv[1]), Path(argv[2])
    if not on_path.exists() or not off_path.exists():
        print(f"missing CSV: {on_path} or {off_path}", file=sys.stderr)
        return 1

    on_rows = load_csv(on_path)
    off_rows = load_csv(off_path)

    print("ABS comparison: BTCM-on vs BTCM-off, 30 m/s → full pedal\n")
    on_brake, on_stop = summarize("BTCM-on  (anti-lock + 4-wheel braking)", on_rows)
    off_brake, off_stop = summarize("BTCM-off (front-only, hydraulic passthrough)", off_rows)

    # Speed vs time during the brake window for both runs.
    def speed_pts(rows, brake_idx, stop_idx):
        if brake_idx < 0:
            return []
        end = stop_idx if stop_idx > 0 else len(rows)
        # Trim to a small window past stop for context.
        return [(r["sim_time_s"], r["speed_mps"])
                for r in rows[brake_idx:min(end + 50, len(rows))]]

    print("Speed vs time during brake event:")
    print(ascii_plot_overlay(
        [("BTCM-on", speed_pts(on_rows, on_brake, on_stop)),
         ("BTCM-off", speed_pts(off_rows, off_brake, off_stop))],
        width=60, height=15, x_label="t [s]", y_label="v [m/s]",
    ))
    print()

    # Per-wheel slip on the BTCM-on run — shows ABS doing its thing.
    if on_brake >= 0:
        end = on_stop if on_stop > 0 else len(on_rows)
        pts_fl = [(r["sim_time_s"], r.get("slip_ratio_fl", 0))
                  for r in on_rows[on_brake:end]]
        pts_rl = [(r["sim_time_s"], r.get("slip_ratio_rl", 0))
                  for r in on_rows[on_brake:end]]
        print("BTCM-on per-wheel slip ratio (FL vs RL — should stay below ~0.2):")
        print(ascii_plot_overlay(
            [("FL slip", pts_fl), ("RL slip", pts_rl)],
            width=60, height=12, x_label="t [s]", y_label="slip",
        ))
        print()

    # Same plot for BTCM-off — wheels should saturate at 1.0 (locked).
    if off_brake >= 0:
        end = off_stop if off_stop > 0 else len(off_rows)
        pts_fl = [(r["sim_time_s"], r.get("slip_ratio_fl", 0))
                  for r in off_rows[off_brake:end]]
        pts_rl = [(r["sim_time_s"], r.get("slip_ratio_rl", 0))
                  for r in off_rows[off_brake:end]]
        print("BTCM-off per-wheel slip ratio (FL vs RL — front locks, rear free-rolls):")
        print(ascii_plot_overlay(
            [("FL slip", pts_fl), ("RL slip", pts_rl)],
            width=60, height=12, x_label="t [s]", y_label="slip",
        ))

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
