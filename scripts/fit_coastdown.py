#!/usr/bin/env python3
"""Fit F_rr and CdA from a coastdown CSV.

Reads the stats CSV produced by config/scenarios/coastdown.json, locates
the throttle-release moment, and fits

    decel(v) = F_rr/m + (0.5 * rho * CdA / m) * v^2

via ordinary least squares on the post-release samples.  Reports the
fitted parameters next to the published EV1 spec values so it's obvious
how close the simulator is to ground truth.

Usage:
    scripts/fit_coastdown.py [path/to/scenario_coastdown.csv]

Defaults to ./scenario_coastdown.csv (where ev1sim writes it).

Standard library only — no numpy / scipy dependency.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

# Plant constants (sync with data/vehicle/ev1/chassis/EV1_Chassis.json + spec).
MASS_KG = 1281.0       # Gen 2 NiMH curb weight (chassis 1199 + ~82 accessories)
RHO_AIR = 1.225        # kg/m^3 at sea level

# EV1 published spec for comparison.
SPEC_F_RR_N = 100.0    # rolling resistance at curb weight: m*g*Crr ~= 100 N
SPEC_CD_A   = 0.36     # 0.19 Cd * 1.89 m^2 frontal area

# Filtering thresholds — keep the "clean" coastdown region only.
# The Chrono sim becomes unstable as v approaches 0 (wheels lock at low
# speed → solver chatter, eventually negative speeds), so we cap the
# usable window at v >= 10 m/s and within ~30 s of throttle release.
MIN_V_MPS = 10.0
MAX_V_MPS = 35.0       # anything above 35 is a blow-up artifact
TRANSIENT_AFTER_RELEASE_S = 1.0  # ignore the first second after release
MAX_DURATION_AFTER_RELEASE_S = 30.0  # only fit the first 30 s of coastdown
MIN_VALID_DECEL = 0.05  # below this is noise / numerical drift
MAX_VALID_DECEL = 3.0   # above this is solver chatter (>3 m/s² without brakes is implausible)


def load_csv(path: Path) -> list[dict]:
    with path.open() as f:
        return list(csv.DictReader(f))


def find_release(rows: list[dict]) -> float | None:
    """Return the sim_time_s where throttle drops back below ~0.05.

    Requires throttle to first cross above 0.5 (acceleration phase) so we
    don't catch the initial t=0 idle.
    """
    seen_high = False
    prev_throttle = 0.0
    for r in rows:
        thr = float(r.get("applied_throttle", 0.0))
        if not seen_high:
            if thr > 0.5:
                seen_high = True
        elif prev_throttle > 0.5 and thr < 0.05:
            return float(r["sim_time_s"])
        prev_throttle = thr
    return None


def central_decel(rows: list[dict], idx: int, half_window: int = 4) -> float | None:
    """Centered finite-difference decel (m/s^2) at row idx.

    Wider window than 1 sample → smooths Chrono solver chatter.
    """
    if idx - half_window < 0 or idx + half_window >= len(rows):
        return None
    v_prev = float(rows[idx - half_window]["speed_mps"])
    v_next = float(rows[idx + half_window]["speed_mps"])
    t_prev = float(rows[idx - half_window]["sim_time_s"])
    t_next = float(rows[idx + half_window]["sim_time_s"])
    if t_next <= t_prev:
        return None
    return -(v_next - v_prev) / (t_next - t_prev)


def fit_quadratic(samples: list[tuple[float, float]]) -> tuple[float, float]:
    """OLS fit: decel = a + b * v^2.  Returns (a, b).

    Fits on per-velocity-bin centroids rather than raw samples — Chrono
    solver chatter on individual samples bias the OLS estimator badly,
    but the bin means (averaging tens to hundreds of samples each)
    track the underlying physics cleanly.
    """
    # 5 m/s bins for the fit (matches the print summary).  Each bin
    # contributes its centroid weighted by the number of samples in it
    # — sparse edge bins are naturally downweighted, dense bins drive
    # the fit.
    bin_w = 5
    bins: dict[int, list[tuple[float, float]]] = {}
    for v, d in samples:
        key = (int(v) // bin_w) * bin_w
        bins.setdefault(key, []).append((v, d))

    centroids: list[tuple[float, float, int]] = []  # (mean_v, mean_d, weight)
    for s in bins.values():
        if len(s) < 10:
            continue
        mean_v = sum(v for v, _ in s) / len(s)
        mean_d = sum(d for _, d in s) / len(s)
        centroids.append((mean_v, mean_d, len(s)))
    centroids.sort()

    n_bins = len(centroids)
    if n_bins < 2:
        raise ValueError(
            f"too few populated bins ({n_bins}, need >=2 with >=10 samples each) "
            "— coastdown may be too short or the analysis window too narrow"
        )

    # Weighted OLS: each centroid contributes proportional to its sample
    # count (treats it as `weight` independent measurements at the centroid).
    n_w = sum(w for _, _, w in centroids)
    sum_x = sum(w * v * v for v, _, w in centroids)
    sum_x2 = sum(w * (v * v) ** 2 for v, _, w in centroids)
    sum_y = sum(w * d for _, d, w in centroids)
    sum_xy = sum(w * (v * v) * d for v, d, w in centroids)

    denom = n_w * sum_x2 - sum_x * sum_x
    if abs(denom) < 1e-12:
        raise ValueError("ill-conditioned fit — coastdown speeds may be too narrow")
    b = (n_w * sum_xy - sum_x * sum_y) / denom
    a = (sum_y - b * sum_x) / n_w
    return a, b


def bin_summary(samples: list[tuple[float, float]], bin_width: float = 5.0) -> str:
    """Return a textual table of (v_bin, count, mean_v, mean_decel, F_avg)."""
    bins: dict[int, list[tuple[float, float]]] = {}
    for v, d in samples:
        key = int(v // bin_width) * int(bin_width)
        bins.setdefault(key, []).append((v, d))
    lines = ["  v range   |  N  |  mean v  |  mean decel  |  F_avg"]
    lines.append("  ---------+-----+----------+--------------+--------")
    for key in sorted(bins):
        s = bins[key]
        if len(s) < 3:
            continue
        mean_v = sum(v for v, _ in s) / len(s)
        mean_d = sum(d for _, d in s) / len(s)
        F = MASS_KG * mean_d
        lines.append(
            f"  {key:>4}-{key + int(bin_width):<3} |"
            f" {len(s):>3} | {mean_v:>7.2f} | {mean_d:>10.4f} | {F:>6.1f} N"
        )
    return "\n".join(lines)


def main(argv: list[str]) -> int:
    csv_path = Path(argv[1]) if len(argv) > 1 else Path("scenario_coastdown.csv")
    if not csv_path.exists():
        print(f"[fit_coastdown] CSV not found: {csv_path}", file=sys.stderr)
        return 1

    rows = load_csv(csv_path)
    if not rows:
        print("[fit_coastdown] empty CSV", file=sys.stderr)
        return 1

    release_t = find_release(rows)
    if release_t is None:
        print("[fit_coastdown] could not locate throttle-release event", file=sys.stderr)
        return 1
    print(f"[fit_coastdown] throttle release at t={release_t:.2f} s")

    samples: list[tuple[float, float]] = []
    for i, r in enumerate(rows):
        t = float(r["sim_time_s"])
        if t < release_t + TRANSIENT_AFTER_RELEASE_S:
            continue
        if t > release_t + MAX_DURATION_AFTER_RELEASE_S:
            break
        v = float(r["speed_mps"])
        if v < MIN_V_MPS or v > MAX_V_MPS:
            continue
        d = central_decel(rows, i)
        if d is None:
            continue
        if d < MIN_VALID_DECEL or d > MAX_VALID_DECEL:
            continue
        samples.append((v, d))

    print(f"[fit_coastdown] {len(samples)} usable post-release samples\n")
    print(bin_summary(samples))

    a, b = fit_quadratic(samples)
    f_rr = MASS_KG * a
    cd_a = 2.0 * MASS_KG * b / RHO_AIR

    print("\n=== Fit ===")
    print(f"  decel(v) = {a:.6f} + {b:.8f} * v^2")
    print(f"  F_rr     = {f_rr:7.1f} N    (real EV1 spec ~{SPEC_F_RR_N:.0f} N,"
          f" ratio {f_rr / SPEC_F_RR_N:.2f}x)")
    print(f"  CdA      = {cd_a:7.3f} m^2  (real EV1 spec ~{SPEC_CD_A:.2f} m^2,"
          f" ratio {cd_a / SPEC_CD_A:.2f}x)")

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
