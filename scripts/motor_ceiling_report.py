#!/usr/bin/env python3
"""Report the motor shaft speed a wheelspin/grip probe run actually reached.

The drive's speed ceiling lives on the MOTOR SHAFT, but the scenario CSV records
WHEEL speeds, so the conversion has to be done deliberately or the answer is
wrong in two ways at once:

  * Use one wheel and you get the wrong number.  The differential is open, so
    the two front wheels sit either side of the shaft speed and can diverge a
    long way on a slippery surface.  The shaft follows their AVERAGE.  Reading
    the slower wheel understates the shaft; reading the faster one overstates
    it.  This script always reports both wheels and the average.
  * The ratio and the tyre radius are read from the shipped vehicle JSON rather
    than hard-coded, so a change to either cannot silently invalidate the
    numbers printed here.

Usage:
    scripts/motor_ceiling_report.py <csv> [<csv> ...]

Reproducing the before/after receipts in docs/ev1_chrono_audit.md:

    ./build/ev1sim --headless --config config/wheelspin_ceiling_probe_ice.json
    scripts/motor_ceiling_report.py scenario_wheelspin_ceiling_probe.csv

    ./build/ev1sim --headless --config config/wheelspin_ceiling_probe_asphalt.json
    scripts/motor_ceiling_report.py scenario_wheelspin_ceiling_probe.csv

For the "before" side, check out the powertrain JSON from a commit preceding the
ceiling change and re-run; nothing else needs rebuilding, because the maps are
read from JSON at startup.
"""

import csv
import json
import math
import os
import sys

MPS_TO_MPH = 2.2369362920544

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)


def _load(rel):
    with open(os.path.join(_ROOT, rel)) as f:
        return json.load(f)


def drivetrain():
    """Reduction ratio and unloaded tyre radius, from the shipped vehicle JSON."""
    conical = _load("data/vehicle/ev1/driveline/EV1_Driveline2WD.json")["Gear Ratio"]["Conical Gear"]
    radius = _load("data/vehicle/ev1/tire/EV1_TMeasyTire.json")["Design"]["Unloaded Radius [m]"]
    ceiling = _load("data/vehicle/ev1/powertrain/EV1_EngineSimpleMap.json")["Maximal Engine Speed RPM"]
    return 1.0 / conical, radius, ceiling


def report(path, ratio, radius, ceiling_rpm):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("%s: empty" % path)
        return

    def shaft_rpm(row):
        fl = abs(float(row["wheel_omega_fl"]))
        fr = abs(float(row["wheel_omega_fr"]))
        return (fl + fr) / 2.0 * ratio * 60.0 / (2.0 * math.pi)

    peak_i = max(range(len(rows)), key=lambda i: shaft_rpm(rows[i]))
    peak = rows[peak_i]
    last = rows[-1]

    fl = abs(float(peak["wheel_omega_fl"]))
    fr = abs(float(peak["wheel_omega_fr"]))
    faster = max(fl, fr)

    print("== %s" % path)
    print("   samples %d, %.1f s" % (len(rows), float(last["sim_time_s"])))
    print("   peak shaft speed  %8.0f rpm   at t = %.2f s" % (shaft_rpm(peak), float(peak["sim_time_s"])))
    print("     front wheels    %8.1f / %.1f rad/s (FL / FR)" % (fl, fr))
    print("     faster wheel    %8.1f mph of tread speed" % (faster * radius * MPS_TO_MPH))
    print("   final shaft speed %8.0f rpm" % shaft_rpm(last))
    print("   final road speed  %8.1f mph" % (abs(float(last["speed_mps"])) * MPS_TO_MPH))

    over = shaft_rpm(peak) - ceiling_rpm
    if over > 0.0:
        print("   OVER the %d rpm drive ceiling by %.0f rpm" % (ceiling_rpm, over))
    else:
        print("   under the %d rpm drive ceiling by %.0f rpm" % (ceiling_rpm, -over))


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    ratio, radius, ceiling = drivetrain()
    print("drivetrain: reduction %.3f:1, unloaded tyre radius %.4f m, ceiling %d rpm"
          % (ratio, radius, ceiling))
    for path in argv[1:]:
        report(path, ratio, radius, ceiling)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
