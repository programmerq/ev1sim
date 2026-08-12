#!/usr/bin/env python3
"""Re-derive the runway budget printed in the level/*.json headers.

Every distance in those headers -- how far the launch takes, how much settle
is left, where the brake lands relative to the surface boundary -- is a
measurement, and a measurement nobody can repeat is a number that quietly goes
stale.  It goes stale for a specific reason here: launch distance is a function
of the torque map, so a powertrain change moves the whole budget without
touching a single level file.  That is exactly what happened on 2026-08-11,
when the motor's corner point moved to the manual's rating and left
abs_diagonal_mu's runway 1.5 m from its boundary.

So: run this after any powertrain, tyre, mass or throttle-ramp change, and
compare its output against the level headers.

    scripts/scenario_runway_report.py                # all seven ABS scenarios
    scripts/scenario_runway_report.py mu_jump        # just one

It runs each scenario headless with the external sim OFF, so it needs only
this repo and a built ./build/ev1sim -- no electricsim, no controllers.  With
no BTCM attached, front braking is straight hydraulic pass-through, which is
the same condition the committed headers were measured under.  Numbers are
printed in metres and in mph.

Boundary positions come from the level file, not from a constant here, so a
level edit cannot silently invalidate the report.

Since 2026-08-12 this also covers the three uniform-surface stops (high_mu,
hard_brake, brake_and_steer).  They have no runway budget to re-derive -- there
is no boundary -- but they have the same SETTLE requirement, and it was
violated in all three: a set_brake scheduled behind a wait_for_speed barrier
fires on the barrier's release tick, not at its own at_time_s, so full brake
landed on the tick the throttle dropped to zero.  For those three the report
prints the settle and the stop distance instead of the crossing geometry.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

MPS_TO_MPH = 2.2369362920544
G = 9.81

ROOT = Path(__file__).resolve().parent.parent

# scenario key -> (config wrapper, how to find the surface boundary in the level)
#   "x_of_second_patch"  : boundary is the -X edge of the level's second patch
#   "x_of_runway_end"    : boundary is the +X edge of patch 0 (the runway)
#   None                 : uniform surface, no boundary to clear
#
# The uniform-surface stops have no transition to mistime, so the crossing and
# runway checks below do not apply to them — but the settle does, and it is the
# same defect: a set_brake scheduled behind the wait_for_speed barrier fires on
# the release tick, so the brake event begins with launch slip in the tyres.
# They were outside this report until 2026-08-12, which is part of why that
# went unmeasured for as long as it did.
SCENARIOS = {
    "mu_jump":     ("config/abs_mu_jump.json",     "x_of_second_patch", "ice"),
    "split_mu":    ("config/abs_split_mu.json",    "x_of_runway_end",   "the split"),
    "diagonal_mu": ("config/abs_diagonal_mu.json", "x_of_runway_end",   "the stripes"),
    "low_mu":      ("config/abs_low_mu.json",      "x_of_second_patch", "packed snow"),
    "high_mu":     ("config/abs_high_mu.json",     None,                "dry asphalt"),
    "hard_brake":  ("config/abs_hard_brake.json",  None,                "dry asphalt"),
    "brake_and_steer": ("config/abs_brake_and_steer.json", None,        "dry asphalt"),
}

# A gap shorter than this between throttle release and brake-on is the
# zero-settle defect, not a deliberately short coast.
#
# Raised from 0.5 to 2.0 on 2026-08-12, which tightens the check for the four
# transition scenarios as well as the three added that day.  0.5 s only ever
# separated "fired on the release tick" from "did not"; 2.0 s is the settle the
# [Runway] cases in tests/test_scenario.cpp actually require.  All seven clear
# it with margin — the tightest is 2.96 s — so nothing was retimed to satisfy
# the new threshold.
MIN_SETTLE_S = 2.0


def boundary_x(level: dict, how: str) -> float:
    patches = level["patches"]
    if how == "x_of_second_patch":
        p = patches[1]
        return p["center"][0] - p["size"][0] / 2.0
    p = patches[0]
    return p["center"][0] + p["size"][0] / 2.0


def run_scenario(cfg_rel: str, workdir: Path, binary: Path) -> tuple[list[dict], list[tuple[float, str, str]]]:
    """Run one scenario headless with the external sim off; return rows + events."""
    cfg = json.loads((ROOT / cfg_rel).read_text())
    scen_rel = cfg["scenario"]["path"]
    scen = json.loads((ROOT / scen_rel).read_text())

    # The committed scenarios wait for an external sim before arming their
    # clock.  Nothing here runs one, so use a throwaway copy with that flag
    # cleared -- the committed file is not touched.
    scen["requires_external_sim"] = False
    scen_path = workdir / "scenario.json"
    scen_path.write_text(json.dumps(scen, indent=2))

    cfg["simulation"]["realtime"] = False
    cfg["simulation"]["headless"] = True
    cfg.setdefault("external_sim", {})["enabled"] = False
    cfg["scenario"]["path"] = str(scen_path)
    if cfg.get("terrain", {}).get("type") == "level":
        cfg["terrain"]["level_file"] = str(ROOT / cfg["terrain"]["level_file"])
    cfg_path = workdir / "config.json"
    cfg_path.write_text(json.dumps(cfg, indent=2))

    log_path = workdir / "run.log"
    with log_path.open("w") as log:
        subprocess.run([str(binary), "--config", str(cfg_path)],
                       cwd=workdir, stdout=log, stderr=subprocess.STDOUT,
                       check=True)

    csv_path = workdir / scen["stats"]["output_csv"]
    with csv_path.open() as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)
                if all(v not in (None, "") for v in r.values())]

    # Event times come from the LOG, never from the scenario file: wait_for_speed
    # is a barrier (src/Scenario.cpp), so every later event fires when the
    # barrier releases, not at its own at_time_s.  Reading the JSON would report
    # "brake at 6.0 s" for a brake that actually fired at 8.47 s.
    events = []
    for line in log_path.read_text().splitlines():
        m = re.match(r"\[Scenario\] t=([\d.]+) (?:action=(\S+)(?: value=(\S+))?"
                     r"|(wait_for_speed released))", line)
        if m:
            events.append((float(m.group(1)), m.group(2) or "wait_released",
                           m.group(3) or ""))
    return rows, events


def sample_at_time(rows, t):
    best = None
    for r in rows:
        if r["sim_time_s"] <= t:
            best = r
        else:
            break
    return best


def sample_at_x(rows, x):
    for r in rows:
        if r["pos_x"] >= x:
            return r
    return None


def stop_distance(rows, brake_t):
    """Ground distance covered from brake-on to standstill, or None."""
    b = sample_at_time(rows, brake_t)
    if b is None:
        return None, None, False
    # pos_y is per-scenario opt-in in the stats block.  Falling back to 0.0
    # would silently turn path length into |dx| for any scenario that omits it
    # -- abs_hard_brake does -- so say which one is being reported instead.
    # (The two are within 1 cm on a straight stop, which is exactly why a
    # silent fallback would never be noticed.)
    has_y = "pos_y" in rows[0]
    prev = None
    dist = 0.0
    for r in rows:
        if r["sim_time_s"] < brake_t:
            continue
        if prev is not None:
            dx = r["pos_x"] - prev["pos_x"]
            dy = (r["pos_y"] - prev["pos_y"]) if has_y else 0.0
            dist += (dx * dx + dy * dy) ** 0.5
        if r["speed_mps"] <= 0.1:
            return dist, r["sim_time_s"] - brake_t, has_y
        prev = r
    return None, None, has_y


def report(key: str, binary: Path) -> bool:
    cfg_rel, how, surface = SCENARIOS[key]
    cfg = json.loads((ROOT / cfg_rel).read_text())
    if how is None:
        xb = None
        mu_runway = cfg["terrain"]["friction"]
    else:
        level = json.loads((ROOT / cfg["terrain"]["level_file"]).read_text())
        xb = boundary_x(level, how)
        mu_runway = level["patches"][0]["friction"]

    with tempfile.TemporaryDirectory() as td:
        rows, events = run_scenario(cfg_rel, Path(td), binary)

    entry_t = next((t for t, a, _ in events if a == "wait_released"), None)
    brake_t = next((t for t, a, v in events
                    if a == "set_brake" and v not in ("", "0")), None)
    spawn_x = rows[0]["pos_x"]

    if xb is None:
        print(f"=== {key}   (uniform {surface}, mu {mu_runway:g}, no boundary)")
        print(f"    spawn x={spawn_x:.2f} m")
    else:
        print(f"=== {key}   (boundary at x={xb:g} m, onto {surface})")
        print(f"    spawn x={spawn_x:.2f} m   spawn-to-boundary {xb - spawn_x:.2f} m")

    ok = True
    if entry_t is None:
        print("    NEVER REACHED ENTRY SPEED — the runway or max_time_s is too short")
        return False

    e = sample_at_time(rows, entry_t)
    print(f"    launch       {e['pos_x'] - spawn_x:7.2f} m   entry speed "
          f"{e['speed_mps']:.3f} m/s ({e['speed_mps'] * MPS_TO_MPH:.1f} mph) "
          f"at t={entry_t:.3f} s")

    if xb is None:
        # Uniform surface: no crossing to clear, so the settle IS the whole
        # entry-condition question.
        if brake_t is None:
            print("    no brake event fired")
            return ok
        b = sample_at_time(rows, brake_t)
        settle_s = brake_t - entry_t
        print(f"    brake on     x={b['pos_x']:7.2f} m   t={brake_t:.3f} s, "
              f"{b['speed_mps']:.3f} m/s ({b['speed_mps'] * MPS_TO_MPH:.1f} mph), "
              f"{settle_s:.2f} s after entry speed")
        if settle_s < MIN_SETTLE_S:
            print(f"    *** ZERO SETTLE: the brake fired {settle_s:.3f} s after "
                  f"the throttle released — a set_brake scheduled behind the "
                  f"wait_for_speed barrier fires on the release tick")
            ok = False
        d, dt, has_y = stop_distance(rows, brake_t)
        if d is None:
            print("    never reaches standstill inside max_time_s")
            ok = False
        else:
            ideal = b["speed_mps"] ** 2 / (2.0 * mu_runway * G)
            kind = "path" if has_y else "along-X (scenario logs no pos_y)"
            print(f"    stop         {d:7.2f} m in {dt:.2f} s   "
                  f"(ideal at mu {mu_runway:g}: {ideal:.2f} m, "
                  f"ratio {d / ideal:.2f}; {kind})")
        peak_slip = max(abs(r[k]) for r in rows if r["sim_time_s"] <= brake_t
                        for k in ("slip_ratio_fl", "slip_ratio_fr",
                                  "slip_ratio_rl", "slip_ratio_rr")
                        if k in r)
        line = f"    peak |slip| before the brake {peak_slip:.3f}"
        if "yaw_deg" in rows[0]:
            line += f"   yaw carried in {b['yaw_deg']:+.3f} deg"
        print(line)
        return ok

    c = sample_at_x(rows, xb)
    if c is None:
        print("    never reaches the boundary")
        return False

    if c["sim_time_s"] < entry_t:
        print(f"    *** CROSSES UNDER THROTTLE at t={c['sim_time_s']:.3f} s, "
              f"{entry_t - c['sim_time_s']:.2f} s before entry speed — "
              f"the launch is happening on the surface under test")
        ok = False
    else:
        print(f"    to boundary  {c['pos_x'] - e['pos_x']:7.2f} m   "
              f"{c['sim_time_s'] - entry_t:.2f} s after entry speed, crosses at "
              f"{c['speed_mps']:.3f} m/s ({c['speed_mps'] * MPS_TO_MPH:.1f} mph)")

    if brake_t is None:
        print("    no brake event fired")
        return ok

    b = sample_at_time(rows, brake_t)
    settle_s = brake_t - entry_t
    print(f"    brake on     x={b['pos_x']:7.2f} m   t={brake_t:.3f} s, "
          f"{b['speed_mps']:.3f} m/s ({b['speed_mps'] * MPS_TO_MPH:.1f} mph), "
          f"{settle_s:.2f} s after entry speed")
    if settle_s < MIN_SETTLE_S:
        print(f"    *** ZERO SETTLE: the brake fired {settle_s:.3f} s after the "
              f"throttle released — a set_brake scheduled behind the "
              f"wait_for_speed barrier fires on the release tick")
        ok = False

    d = xb - b["pos_x"]
    if d > 0:
        stop = b["speed_mps"] ** 2 / (2.0 * mu_runway * G)
        print(f"    brake-on sits {d:.2f} m BEFORE the boundary; the shortest "
              f"possible stop at mu {mu_runway:g} is {stop:.2f} m "
              f"({100.0 * d / stop:.0f} % of it), so the crossing "
              f"{'cannot' if d < stop else 'MAY NOT'} be swallowed by it")
        if d >= stop:
            print("    *** the car can stop before ever reaching the surface "
                  "under test")
            ok = False
    else:
        print(f"    brake-on sits {-d:.2f} m past the boundary — the whole brake "
              f"event is on {surface}")

    peak_slip = max(abs(r[k]) for r in rows if r["sim_time_s"] <= brake_t
                    for k in ("slip_ratio_fl", "slip_ratio_fr",
                              "slip_ratio_rl", "slip_ratio_rr")
                    if k in r)
    line = f"    peak |slip| before the brake {peak_slip:.3f}"
    if "yaw_deg" in rows[0]:
        line += f"   yaw carried in {sample_at_time(rows, brake_t)['yaw_deg']:+.3f} deg"
    print(line)
    return ok


def main(argv) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenarios", nargs="*", choices=sorted(SCENARIOS) or None,
                    default=sorted(SCENARIOS),
                    help="which to run (default: all)")
    ap.add_argument("--binary", default=str(ROOT / "build" / "ev1sim"),
                    help="path to a built ev1sim")
    args = ap.parse_args(argv)

    binary = Path(args.binary)
    if not binary.is_file():
        print(f"no ev1sim at {binary} — build it first "
              f"(cmake --build --preset default)", file=sys.stderr)
        return 2

    all_ok = True
    for key in args.scenarios:
        all_ok &= report(key, binary)
        print()
    if not all_ok:
        print("At least one scenario is not braking from a settled entry: "
              "either its launch is still finishing when the brake goes down, "
              "or (transition levels only) it has not cleared its surface "
              "boundary. See the level/*.json headers for the runway budget "
              "each transition scenario holds, and the [Runway] cases in "
              "tests/test_scenario.cpp for the settle each of the seven is "
              "pinned to.", file=sys.stderr)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
