#!/usr/bin/env python3
"""Re-derive the measured settle table, and the runway budget in the level headers.

Every distance in those headers -- how far the launch takes, how much settle
is left, where the brake lands relative to the surface boundary -- is a
measurement, and a measurement nobody can repeat is a number that quietly goes
stale.  It goes stale for a specific reason here: launch distance is a function
of the torque map, so a powertrain change moves the whole budget without
touching a single level file.  That is exactly what happened on 2026-08-11,
when the motor's corner point moved to the manual's rating and left
abs_diagonal_mu's runway 1.5 m from its boundary.

    scripts/scenario_runway_report.py                # all seven ABS scenarios
    scripts/scenario_runway_report.py mu_jump        # just one
    scripts/scenario_runway_report.py --update       # re-record what it measured
    scripts/scenario_runway_report.py --selftest     # no sim; prove the check fails

IT NOW CHECKS THE TABLE RATHER THAN JUST PRINTING NUMBERS
=========================================================
config/scenarios/measured_settle.json records, per scenario, when the
wait_for_speed barrier actually releases and when the car crosses onto the
surface under test.  tests/test_scenario.cpp's [Runway] cases check each
scenario's set_brake against those numbers -- which catches a brake moved
backwards, and cannot catch the PLANT moving forwards.  If a barrier starts
releasing at 17.5 s, an 18.0 s brake still clears a recorded 15.028 + 2.0
while the real settle has shrunk to 0.5 s, and nothing goes red.  That is the
staleness that produced the phantom 12.011 s.

So this script re-derives those fields from an actual run and exits non-zero
when a derived value and the recorded one disagree by more than the tolerance
the table itself carries.  The table is not a second copy of anything: it is
the one copy, read by the C++ cases and by this script.  --update re-records
what was measured, so a legitimate move is a command rather than a hand-edit.

It runs each scenario headless with the external sim OFF, so it needs only
this repo and a built ./build/ev1sim -- no electricsim, no controllers.  With
no BTCM attached, front braking is straight hydraulic pass-through, which is
the same condition the committed numbers were measured under.  Numbers are
printed in metres and in mph.

Boundary positions come from the level file, not from a constant here, so a
level edit cannot silently invalidate the report.
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
TABLE_PATH = ROOT / "config" / "scenarios" / "measured_settle.json"

# Fields the table records as MEASUREMENTS, and which this script re-derives.
# The other numbers in a case (min_settle_s, min_on_surface_s) are
# requirements we chose; nothing re-derives a requirement.
MEASURED_FIELDS = ("measured_release_s", "measured_crossing_s")


def load_table() -> dict:
    t = json.loads(TABLE_PATH.read_text())
    seen = set()
    for c in t["cases"]:
        if c["key"] in seen:
            raise SystemExit(f"{TABLE_PATH}: duplicate key {c['key']!r}")
        seen.add(c["key"])
    return t


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


# --- the staleness check ------------------------------------------------------

def compare_to_table(case: dict, derived: dict, tol: dict) -> list[str]:
    """Complaints where a re-derived measurement disagrees with the table.

    Pure: takes the recorded case, the derived values and the tolerances, and
    returns one message per disagreement.  It runs no simulation, which is
    what lets --selftest drive it across its own threshold.
    """
    out = []
    for field in MEASURED_FIELDS:
        recorded = case.get(field)
        got = derived.get(field)
        name = field.replace("measured_", "").replace("_s", "")
        limit = tol[f"{name}_s"]

        if recorded is None and got is None:
            continue
        if recorded is None:
            out.append(
                f"{case['key']}: {field} is null in the table but the run "
                f"produced {got:.3f} s — the table says this scenario has no "
                f"{name} to time, and it does")
            continue
        if got is None:
            out.append(
                f"{case['key']}: {field} is {recorded:.3f} s in the table but "
                f"the run produced no {name} at all")
            continue
        if abs(got - recorded) > limit:
            out.append(
                f"{case['key']}: {field} recorded {recorded:.3f} s, re-derived "
                f"{got:.3f} s — off by {got - recorded:+.3f} s, past the "
                f"{limit:.3f} s tolerance")
    return out


def update_table(text: str, key: str, field: str, value: float | None) -> str:
    """Rewrite one measured field of one case, in place, comments intact.

    Textual rather than a json round-trip: the table carries its rationale in
    "//" keys and blank-line grouping, and dumping it back would flatten both.
    """
    m = re.search(r'"key"\s*:\s*"%s"' % re.escape(key), text)
    if m is None:
        raise SystemExit(f"{TABLE_PATH}: no case with key {key!r} to update")
    nxt = re.search(r'"key"\s*:\s*"', text[m.end():])
    end = m.end() + (nxt.start() if nxt else len(text) - m.end())
    block = text[m.start():end]

    lit = "null" if value is None else f"{value:.3f}"
    new_block, n = re.subn(r'("%s"\s*:\s*)(null|-?[0-9.]+)' % re.escape(field),
                           lambda mm: mm.group(1) + lit, block, count=1)
    if n != 1:
        raise SystemExit(f"{TABLE_PATH}: case {key!r} has no {field} to update")
    return text[:m.start()] + new_block + text[end:]


# --- reporting ----------------------------------------------------------------

def report(case: dict, binary: Path, tol: dict) -> tuple[bool, dict]:
    """Print one scenario's budget; return (ok, the values it re-derived).

    Every early exit below goes through _finish(), so the table comparison
    runs even on a scenario that failed for some other reason.  Skipping it on
    the way out of a failure is how a stale number survives a red run.
    """
    key = case["key"]
    cfg = json.loads((ROOT / case["config"]).read_text())
    how = case["boundary"]
    surface = case["surface"]
    if how is None:
        xb = None
        mu_runway = cfg["terrain"]["friction"]
    else:
        level = json.loads((ROOT / cfg["terrain"]["level_file"]).read_text())
        xb = boundary_x(level, how)
        mu_runway = level["patches"][0]["friction"]

    with tempfile.TemporaryDirectory() as td:
        rows, events = run_scenario(case["config"], Path(td), binary)

    entry_t = next((t for t, a, _ in events if a == "wait_released"), None)
    brake_t = next((t for t, a, v in events
                    if a == "set_brake" and v not in ("", "0")), None)
    spawn_x = rows[0]["pos_x"]
    min_settle_s = case["min_settle_s"]

    derived: dict = {"measured_release_s": entry_t, "measured_crossing_s": None}

    if xb is None:
        print(f"=== {key}   (uniform {surface}, mu {mu_runway:g}, no boundary)")
        print(f"    spawn x={spawn_x:.2f} m")
    else:
        print(f"=== {key}   (boundary at x={xb:g} m, onto {surface})")
        print(f"    spawn x={spawn_x:.2f} m   spawn-to-boundary {xb - spawn_x:.2f} m")

    def _finish(ok: bool) -> tuple[bool, dict]:
        complaints = compare_to_table(case, derived, tol)
        if complaints:
            for msg in complaints:
                print(f"    *** STALE TABLE: {msg}")
            return False, derived
        rec = case["measured_release_s"]
        got = derived["measured_release_s"]
        line = f"    table        release {rec:.3f} s recorded"
        line += (f", {got:.3f} s re-derived ({got - rec:+.3f} s)"
                 if got is not None else ", nothing re-derived")
        if derived["measured_crossing_s"] is not None:
            rc, dc = case["measured_crossing_s"], derived["measured_crossing_s"]
            line += f"; crossing {rc:.3f} -> {dc:.3f} ({dc - rc:+.3f} s)"
        print(line + "  — agrees")
        return ok, derived

    ok = True
    if entry_t is None:
        print("    NEVER REACHED ENTRY SPEED — the runway or max_time_s is too short")
        return _finish(False)

    e = sample_at_time(rows, entry_t)
    print(f"    launch       {e['pos_x'] - spawn_x:7.2f} m   entry speed "
          f"{e['speed_mps']:.3f} m/s ({e['speed_mps'] * MPS_TO_MPH:.1f} mph) "
          f"at t={entry_t:.3f} s")

    if xb is not None:
        c = sample_at_x(rows, xb)
        if c is None:
            print("    never reaches the boundary")
            return _finish(False)
        # The crossing is a measurement only where the table asks for one.
        # mu_jump crosses under braking BY DESIGN, and a crossing inside the
        # brake event is not an entry condition, so its row records null.
        if case["min_on_surface_s"] is not None:
            derived["measured_crossing_s"] = c["sim_time_s"]

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
        # Not "ok".  A barrier-gated scenario with no brake is a scenario that
        # tests nothing, and returning True here would let it stay that way.
        print("    *** no brake event fired — this scenario brakes nothing")
        return _finish(False)

    b = sample_at_time(rows, brake_t)
    settle_s = brake_t - entry_t
    print(f"    brake on     x={b['pos_x']:7.2f} m   t={brake_t:.3f} s, "
          f"{b['speed_mps']:.3f} m/s ({b['speed_mps'] * MPS_TO_MPH:.1f} mph), "
          f"{settle_s:.2f} s after entry speed")
    if settle_s < min_settle_s:
        print(f"    *** ZERO SETTLE: the brake fired {settle_s:.3f} s after "
              f"the throttle released, under the {min_settle_s:.1f} s this "
              f"scenario is pinned to — a set_brake scheduled behind the "
              f"wait_for_speed barrier fires on the release tick")
        ok = False

    if xb is None:
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
    else:
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
        line += f"   yaw carried in {b['yaw_deg']:+.3f} deg"
    print(line)

    return _finish(ok)


# --- selftest -----------------------------------------------------------------

def selftest() -> int:
    """Prove the staleness comparison can FAIL, without running a scenario.

    A check that has never been watched go red is a check nobody knows can.
    This drives compare_to_table across its own tolerance with three values
    and prints the number each verdict turns on.
    """
    ok = True

    def case_(name: str, passed: bool, detail: str) -> None:
        nonlocal ok
        ok &= passed
        print(f"  [{'ok  ' if passed else 'FAIL'}] {name}: {detail}")

    table = load_table()
    tol = table["tolerances"]
    lim = tol["release_s"]
    row = dict(table["cases"][0])
    rec = row["measured_release_s"]

    # 1. The recorded value against itself: must be silent, or the check is
    #    red for everything and means nothing.
    d = {"measured_release_s": rec, "measured_crossing_s": row["measured_crossing_s"]}
    case_("the table agrees with itself", compare_to_table(row, d, tol) == [],
          f"{row['key']} release {rec:.3f} s vs {rec:.3f} s")

    # 2. Three drifts across the tolerance, so the verdict is seen to change.
    for drift, want_fail in ((lim * 0.5, False), (lim * 0.9, False),
                             (lim * 2.0, True)):
        d = {"measured_release_s": rec + drift,
             "measured_crossing_s": row["measured_crossing_s"]}
        msgs = compare_to_table(row, d, tol)
        case_(f"a {drift:+.3f} s drift -> {'fail' if want_fail else 'pass'}",
              bool(msgs) == want_fail,
              msgs[0] if msgs else f"accepted {rec + drift:.3f} s "
                                   f"(tolerance {lim:.3f} s)")

    # 3. The drift that matters: the plant moving forwards far enough to eat
    #    the settle while the [Runway] assertion still passes.  This is the
    #    exact hole the check exists to close, so it is worth asserting by
    #    name and not just as another number past the tolerance.
    eaten = row["min_settle_s"] * 0.45
    d = {"measured_release_s": rec + eaten,
         "measured_crossing_s": row["measured_crossing_s"]}
    case_("a barrier releasing late enough to halve the settle is caught",
          bool(compare_to_table(row, d, tol)),
          f"{eaten:+.3f} s would leave {row['min_settle_s'] - eaten:.2f} s of "
          f"real settle against a {row['min_settle_s']:.1f} s requirement")

    # 4. A table row whose crossing is null while the run produced one (and
    #    the reverse) is a disagreement about the SHAPE of the scenario, not
    #    just its timing, and has to be caught too.
    with_x = next((c for c in table["cases"]
                   if c["measured_crossing_s"] is not None), None)
    if with_x is None:
        case_("a null-vs-value crossing is caught", False,
              "no case in the table records a crossing")
    else:
        d = {"measured_release_s": with_x["measured_release_s"],
             "measured_crossing_s": None}
        msgs = compare_to_table(with_x, d, tol)
        case_("a crossing that stopped happening is caught", bool(msgs),
              msgs[0] if msgs else "accepted a missing crossing")

        d = {"measured_release_s": rec, "measured_crossing_s": 5.0}
        msgs = compare_to_table(row, d, tol)
        case_("a crossing that started happening is caught",
              bool(msgs) if row["measured_crossing_s"] is None else True,
              msgs[0] if msgs else "accepted an unexpected crossing")

    # 5. --update has to actually move the number it claims to move, and
    #    leave the rest of the file alone.
    text = TABLE_PATH.read_text()
    moved = update_table(text, row["key"], "measured_release_s", rec + 1.0)
    reparsed = json.loads(moved)
    got = next(c for c in reparsed["cases"] if c["key"] == row["key"])
    others_intact = all(
        a == b for a, b in zip(
            [c["measured_release_s"] for c in reparsed["cases"][1:]],
            [c["measured_release_s"] for c in table["cases"][1:]]))
    case_("--update rewrites one field and nothing else",
          abs(got["measured_release_s"] - (rec + 1.0)) < 1e-9
          and others_intact and moved.count("//") == text.count("//"),
          f"{row['key']} release {rec:.3f} -> {got['measured_release_s']:.3f}, "
          f"{len(table['cases']) - 1} other rows and "
          f"{text.count(chr(34) + '//')} comment keys untouched")

    print()
    print("selftest " + ("ok" if ok else "FAILED"))
    return 0 if ok else 1


def main(argv) -> int:
    table = load_table()
    keys = [c["key"] for c in table["cases"]]

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    # No `choices=` here.  With nargs="*" and a list default, argparse checks
    # the whole default list against choices and rejects it, so the documented
    # bare invocation exited 2 without running anything — which is a fair part
    # of why this table went years without being re-derived.  Validate by hand
    # instead, and name the valid keys when one is wrong.
    ap.add_argument("scenarios", nargs="*", default=None,
                    help=f"which to run (default: all; one of {', '.join(keys)})")
    ap.add_argument("--binary", default=str(ROOT / "build" / "ev1sim"),
                    help="path to a built ev1sim")
    ap.add_argument("--update", action="store_true",
                    help="re-record what this run measured into "
                         "config/scenarios/measured_settle.json")
    ap.add_argument("--selftest", action="store_true",
                    help="prove the staleness check can fail; runs no scenario")
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()

    wanted = args.scenarios or keys
    bad = [s for s in wanted if s not in keys]
    if bad:
        print(f"unknown scenario(s): {', '.join(bad)} — "
              f"the table lists {', '.join(keys)}", file=sys.stderr)
        return 2
    if args.update and args.scenarios:
        # A partial --update would leave the file half re-derived and half
        # historical, with nothing recording which row is which.
        print("--update re-records the whole table, so it cannot be combined "
              "with a scenario list", file=sys.stderr)
        return 2

    binary = Path(args.binary)
    if not binary.is_file():
        print(f"no ev1sim at {binary} — build it first "
              f"(cmake --build --preset default)", file=sys.stderr)
        return 2

    tol = table["tolerances"]
    by_key = {c["key"]: c for c in table["cases"]}
    all_ok = True
    measured: dict[str, dict] = {}
    for key in wanted:
        ok, derived = report(by_key[key], binary, tol)
        measured[key] = derived
        all_ok &= ok
        print()

    if args.update:
        text = TABLE_PATH.read_text()
        changed = []
        for key, derived in measured.items():
            for field in MEASURED_FIELDS:
                before = by_key[key][field]
                after = derived[field]
                if before == after:
                    continue
                if (before is not None and after is not None
                        and abs(before - after) < 1e-9):
                    continue
                text = update_table(text, key, field, after)
                changed.append(f"{key}.{field}: {before} -> {after}")
        TABLE_PATH.write_text(text)
        if changed:
            print(f"--update rewrote {TABLE_PATH.relative_to(ROOT)}:")
            for c in changed:
                print(f"    {c}")
            print("Commit that, and say in the message what moved the plant.")
        else:
            print(f"--update: {TABLE_PATH.relative_to(ROOT)} already matches "
                  "what this run measured; nothing rewritten.")
        return 0 if all_ok else 1

    # A COMPLETION MARKER, printed only after every requested scenario has
    # been re-derived.  This run takes ~18 minutes; if a CI timeout kills it
    # partway, the log ends after some number of green per-scenario blocks and
    # otherwise looks exactly like a clean run.  The exit code differs, but
    # logs get read on their own.  So: no marker, no verdict.
    print(f"[runway] re-derived {len(measured)} of {len(wanted)} scenarios "
          f"against {TABLE_PATH.relative_to(ROOT)} — "
          f"{'all agree' if all_ok else 'SOMETHING DISAGREES, see above'}")

    if not all_ok:
        print("Either a scenario is not braking from a settled entry (its launch "
              "is still finishing when the brake goes down, or — transition "
              "levels only — it has not cleared its surface boundary), or the "
              "measured settle table no longer matches the plant.  For the "
              "first, see the level/*.json headers and the [Runway] cases in "
              "tests/test_scenario.cpp.  For the second, re-run with --update "
              "and commit what moved.", file=sys.stderr)
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
