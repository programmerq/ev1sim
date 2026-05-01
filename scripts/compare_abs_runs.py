#!/usr/bin/env python3
"""Compare two ABS scenario CSVs (BTCM-on vs BTCM-off) and emit a report.

Reports printed:
  - Per-run summary: stop time, stopping distance, yaw drift,
    per-wheel slip stats (peak / mean / time-locked-percent).
  - Speed-vs-time overlay (vehicle speed, both runs).
  - Wheel-speed-as-vehicle-speed: per-wheel angular velocity converted
    to "what speed would the vehicle be going if this wheel was rolling
    freely?".  When wheels lock, this drops below the actual vehicle
    speed — that gap is the slip.  Annotated with ABS phase markers
    along the bottom (`H`=HOLD, `D`=DUMP, `_`=APPLY/idle).
  - ABS phase timeline per front wheel, BTCM-on run only.

stdlib only — no matplotlib / numpy.  ASCII plots scale to the data.
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

# Sync with EV1_TMeasyTire.json "Unloaded Radius [m]".
TIRE_RADIUS_M = 0.2915

# Phase encoding (Scenario.cpp): 0=APPLY, 1=HOLD, 2=DUMP, -1=stale.
PHASE_LABEL = {0: "_", 1: "H", 2: "D", -1: " "}


def load_csv(path: Path) -> list[dict[str, float]]:
    with path.open() as f:
        return [
            {k: float(v) if v else 0.0 for k, v in row.items()}
            for row in csv.DictReader(f)
        ]


def find_brake_event(rows: list[dict]) -> int:
    """Index where applied_front_brake first crosses 0.5."""
    for i, r in enumerate(rows):
        if r.get("applied_front_brake", 0) > 0.5:
            return i
    return -1


def find_stop(rows: list[dict], start_idx: int, threshold: float = 1.0) -> int:
    """First row at/after start_idx where speed_mps drops below threshold."""
    for i in range(start_idx, len(rows)):
        if rows[i]["speed_mps"] < threshold:
            return i
    return -1


def stopping_distance(rows: list[dict], start_idx: int, stop_idx: int) -> float:
    if start_idx < 0 or stop_idx < 0:
        return float("nan")
    x0 = rows[start_idx].get("pos_x", 0.0)
    x1 = rows[stop_idx].get("pos_x", 0.0)
    y0 = rows[start_idx].get("pos_y", 0.0)
    y1 = rows[stop_idx].get("pos_y", 0.0)
    return math.hypot(x1 - x0, y1 - y0)


def yaw_drift_deg(rows: list[dict], start_idx: int, stop_idx: int) -> float:
    """Maximum |yaw_rate| × Δt accumulated heading drift (deg).

    Approximation — integrate yaw_rate over the brake event.  Useful
    for split-mu where the asymmetric grip generates a yawing moment.
    """
    if start_idx < 0 or stop_idx < 0:
        return float("nan")
    end = stop_idx if stop_idx > 0 else len(rows)
    yaw_deg = 0.0
    for i in range(start_idx + 1, end):
        dt = rows[i]["sim_time_s"] - rows[i - 1]["sim_time_s"]
        yaw_deg += rows[i].get("yaw_rate", 0.0) * dt * 180.0 / math.pi
    return yaw_deg


def slip_stats(rows: list[dict], start_idx: int, stop_idx: int, wheel: str) -> dict:
    key = f"slip_ratio_{wheel}"
    if key not in rows[0]:
        return {"peak": float("nan"), "mean": float("nan"), "locked_pct": float("nan")}
    end = stop_idx if stop_idx > 0 else len(rows)
    slips = [abs(r[key]) for r in rows[start_idx:end]]
    if not slips:
        return {"peak": float("nan"), "mean": float("nan"), "locked_pct": float("nan")}
    locked = sum(1 for s in slips if s > 0.95)
    return {
        "peak": max(slips),
        "mean": sum(slips) / len(slips),
        "locked_pct": 100.0 * locked / len(slips),
    }


def count_abs_events(rows: list[dict], start_idx: int, stop_idx: int) -> dict:
    """Count phase transitions per front wheel during the brake event."""
    end = stop_idx if stop_idx > 0 else len(rows)
    out = {"fl": 0, "fr": 0}
    for wheel in ("fl", "fr"):
        key = f"abs_phase_{wheel}"
        if key not in rows[0]:
            continue
        prev = rows[start_idx].get(key, -1)
        for r in rows[start_idx + 1:end]:
            cur = r.get(key, -1)
            # Count transitions among real phases (not stale → fresh).
            if cur != prev and cur != -1 and prev != -1:
                out[wheel] += 1
            prev = cur
    return out


def ascii_plot_overlay(
    series: list[tuple[str, list[tuple[float, float]]]],
    width: int = 60,
    height: int = 12,
    x_label: str = "t",
    y_label: str = "v",
    chars: str = "*+x.",
    annotation_lane: list[str] | None = None,
) -> str:
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
                    grid[row][col] = "#"

    lines = []
    for r, row in enumerate(grid):
        if r == 0:
            label = f"{y_max:6.2f}"
        elif r == height - 1:
            label = f"{y_min:6.2f}"
        elif r == height // 2:
            label = f"{(y_min + y_max) / 2:6.2f}"
        else:
            label = "      "
        lines.append(f"{label} | {''.join(row)}")
    if annotation_lane is not None:
        lane = "".join(annotation_lane[:width])
        lines.append("       | " + lane)
    lines.append(" " * 7 + "+" + "-" * width)
    lines.append(
        " " * 7
        + " "
        + f"{x_min:.2f}".ljust(width // 2)
        + f"{x_max:.2f}".rjust(width - width // 2)
    )
    legend_parts = [f"{ch}={lbl}" for ch, (lbl, _) in zip(chars, series)]
    lines.append(f"        ({y_label} vs {x_label})  " + "  ".join(legend_parts) + "  #=overlap")
    return "\n".join(lines)


def abs_phase_lane(
    rows: list[dict], start_idx: int, end_idx: int, wheel: str, width: int
) -> list[str]:
    """Build a width-char wide annotation lane: H=HOLD, D=DUMP, _=APPLY, ' '=stale.

    Buckets the rows into width slots and picks the dominant phase per slot.
    """
    if not rows:
        return [" "] * width
    key = f"abs_phase_{wheel}"
    span = max(1, end_idx - start_idx)
    bucket = [[0, 0, 0, 0] for _ in range(width)]  # APPLY HOLD DUMP STALE
    for i in range(start_idx, end_idx):
        slot = int((i - start_idx) * width / span)
        slot = min(slot, width - 1)
        ph = int(rows[i].get(key, -1))
        if   ph == 0: bucket[slot][0] += 1
        elif ph == 1: bucket[slot][1] += 1
        elif ph == 2: bucket[slot][2] += 1
        else:         bucket[slot][3] += 1
    out = []
    for b in bucket:
        # If any HOLD/DUMP samples in this slot, show that (stickier than counts).
        if b[2] > 0:    out.append("D")
        elif b[1] > 0:  out.append("H")
        elif b[0] > 0:  out.append("_")
        else:           out.append(" ")
    return out


def fmt(x: float, prec: int = 2) -> str:
    if x != x:
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
    print(f"  brake-on  at t={t_brake:6.2f} s, v={v_brake:6.2f} m/s")
    if stop_idx >= 0:
        t_stop = rows[stop_idx]["sim_time_s"]
        dist = stopping_distance(rows, brake_idx, stop_idx)
        yaw = yaw_drift_deg(rows, brake_idx, stop_idx)
        print(f"  stopped   at t={t_stop:6.2f} s  Δt={t_stop - t_brake:5.2f} s  "
              f"distance={dist:6.2f} m")
        print(f"  yaw drift over brake event: {yaw:+6.2f}°  "
              f"(positive = car pulls left)")
    else:
        v_final = rows[-1]["speed_mps"]
        print(f"  did NOT stop within scenario.  Final speed: {v_final:.2f} m/s")

    end = stop_idx if stop_idx > 0 else len(rows) - 1
    print("  per-wheel slip during brake event   (peak / mean / time-locked):")
    for wheel in ("fl", "fr", "rl", "rr"):
        s = slip_stats(rows, brake_idx, end, wheel)
        flag = "  ← MOSTLY LOCKED" if s["locked_pct"] > 50 else ""
        print(f"    {wheel.upper()}: peak={fmt(s['peak'], 3)}  "
              f"mean={fmt(s['mean'], 3)}  "
              f"locked={s['locked_pct']:5.1f}%{flag}")

    abs_events = count_abs_events(rows, brake_idx, end)
    print(f"  ABS phase transitions: FL={abs_events.get('fl', 0)}  "
          f"FR={abs_events.get('fr', 0)}")
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

    print("ABS scenario comparison: BTCM-on vs BTCM-off\n")
    on_brake, on_stop = summarize("BTCM-on  (anti-lock + 4-wheel braking)", on_rows)
    off_brake, off_stop = summarize("BTCM-off (front-only, hydraulic passthrough)", off_rows)

    # Vehicle speed overlay.
    def speed_pts(rows, brake_idx, stop_idx):
        if brake_idx < 0:
            return []
        end = stop_idx + 30 if stop_idx > 0 else len(rows)
        end = min(end, len(rows))
        return [(r["sim_time_s"], r["speed_mps"]) for r in rows[brake_idx:end]]

    print("Vehicle speed during brake event:")
    print(ascii_plot_overlay(
        [("BTCM-on", speed_pts(on_rows, on_brake, on_stop)),
         ("BTCM-off", speed_pts(off_rows, off_brake, off_stop))],
        width=60, height=12, x_label="t [s]", y_label="v [m/s]",
    ))
    print()

    # Per-wheel ground-equivalent speed (omega × radius) — when this drops
    # below the chassis speed, the wheel is slipping/locked.
    def wheel_speed_pts(rows, brake_idx, stop_idx, wheel_idx_field):
        if brake_idx < 0:
            return []
        end = stop_idx + 30 if stop_idx > 0 else len(rows)
        end = min(end, len(rows))
        return [
            (r["sim_time_s"], abs(r.get(wheel_idx_field, 0)) * TIRE_RADIUS_M)
            for r in rows[brake_idx:end]
        ]

    if on_brake >= 0:
        end = (on_stop + 30) if on_stop > 0 else len(on_rows)
        end = min(end, len(on_rows))
        v_pts  = [(r["sim_time_s"], r["speed_mps"]) for r in on_rows[on_brake:end]]
        fl_pts = wheel_speed_pts(on_rows, on_brake, on_stop, "wheel_omega_fl")
        rl_pts = wheel_speed_pts(on_rows, on_brake, on_stop, "wheel_omega_rl")
        fl_lane = abs_phase_lane(on_rows, on_brake, end, "fl", width=60)
        print("BTCM-on: chassis vs wheel ground-speed (gap = slip), FL ABS phases below:")
        print(ascii_plot_overlay(
            [("chassis", v_pts), ("FL wheel", fl_pts), ("RL wheel", rl_pts)],
            width=60, height=12, x_label="t [s]", y_label="m/s",
            annotation_lane=fl_lane,
        ))
        print(" " * 8 + "(annotation: H=HOLD  D=DUMP  _=APPLY for FL wheel)")
        print()

    if off_brake >= 0:
        end = (off_stop + 30) if off_stop > 0 else len(off_rows)
        end = min(end, len(off_rows))
        v_pts  = [(r["sim_time_s"], r["speed_mps"]) for r in off_rows[off_brake:end]]
        fl_pts = wheel_speed_pts(off_rows, off_brake, off_stop, "wheel_omega_fl")
        rl_pts = wheel_speed_pts(off_rows, off_brake, off_stop, "wheel_omega_rl")
        print("BTCM-off: chassis vs wheel ground-speed (no ABS — wheels lock and stay there):")
        print(ascii_plot_overlay(
            [("chassis", v_pts), ("FL wheel", fl_pts), ("RL wheel", rl_pts)],
            width=60, height=12, x_label="t [s]", y_label="m/s",
        ))
        print()

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
