#!/usr/bin/env python3
"""Fit F_rr and CdA from one or more coastdown CSVs.

Reads the stats CSV produced by config/coastdown.json, locates the
throttle-release moment, and fits

    decel(v) = F_rr/m + (0.5 * rho * CdA / m) * v^2

via weighted least squares on the post-release samples.  Reports the fitted
parameters next to the published EV1 spec values so it's obvious how close the
simulator is to ground truth.

    scripts/fit_coastdown.py                       # ./scenario_coastdown.csv
    scripts/fit_coastdown.py run.csv               # one run
    scripts/fit_coastdown.py before.csv after.csv  # COMPARE two runs
    scripts/fit_coastdown.py --v-max 23.28 a.csv b.csv   # explicit window
    scripts/fit_coastdown.py --selftest            # prove the guards can fail

THE FIT WINDOW IS A SPEED RANGE, NEVER A TIME SPAN.
=====================================================================
This is the whole design of the tool and it is a correction, not a
preference.  Until 2026-08-12 the window was capped at 30 s after throttle
release (MAX_DURATION_AFTER_RELEASE_S).  The model above is a function of
SPEED, so a window expressed in TIME lands on a different speed range for
every run -- and the runs you most want to compare are exactly the ones whose
coast length has changed, because changing drag or regen is what changes it.

What that cost, concretely: comparing the coast map either side of the
docs/ev1_chrono_audit.md 3.1 correction, the 30 s cap truncated the two runs
at 18.26 and 19.24 m/s (40.8 and 43.0 mph).  The low-speed bin then covered
different speeds in the two runs, the fit pivoted on it, and the tool
confidently reported CdA collapsing 0.748 -> 0.091 m^2 -- a quarter of spec,
with the residual loss "not aero-shaped".  That conclusion was relayed and
then retracted.  Fitted over the speeds both runs actually reach, the same two
CSVs give 0.914 -> 0.621 m^2: a real improvement, and nothing like the
artefact.  The tool, not the plant, produced the difference.

So:

  * There is no time cap.  A single run is fitted over its whole coast, down
    to MIN_V_MPS.  That bound is what the time cap was really standing in for
    -- Chrono goes unstable as v approaches 0 -- and it is stated in the unit
    the model uses.
  * Two or more runs are fitted over the speed range they ALL cover, that
    range is printed above the numbers, and anything trimmed off an
    individual run is named.
  * If the runs do not share enough speed range to fit, the tool REFUSES and
    prints each run's range.  It does not fall back to fitting them over
    different windows and reporting the difference: that is the defect.
  * Every F_rr and CdA carries the spread of the bins about the fitted line,
    and a comparison whose window cannot separate the two says so and exits
    non-zero.  A common window is necessary and not sufficient: feed this
    tool the same 30 s-capped pair and it reports the honest window AND
    refuses to support a CdA statement off it (0.088 +/- 0.145 m^2), which is
    what the retracted claim actually looked like.

Standard library only -- no numpy / scipy dependency.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import tempfile
from pathlib import Path

# Plant constants (sync with data/vehicle/ev1/chassis/EV1_Chassis.json + spec).
MASS_KG = 1281.0       # Gen 2 NiMH curb weight (chassis 1199 + ~82 accessories)
RHO_AIR = 1.225        # kg/m^3 at sea level

# EV1 published spec for comparison.
SPEC_F_RR_N = 100.0    # rolling resistance at curb weight: m*g*Crr ~= 100 N
SPEC_CD_A   = 0.36     # 0.19 Cd * 1.89 m^2 frontal area

MPS_TO_MPH = 2.2369362920544

# Filtering thresholds -- keep the "clean" coastdown region only.
# The Chrono sim becomes unstable as v approaches 0 (wheels lock at low
# speed -> solver chatter, eventually negative speeds), so the usable window
# is bounded in SPEED.  See the module docstring for why it is not bounded in
# time as well.
MIN_V_MPS = 10.0
MAX_V_MPS = 35.0       # anything above 35 is a blow-up artifact
TRANSIENT_AFTER_RELEASE_S = 1.0  # ignore the first second after release
MIN_VALID_DECEL = 0.05  # below this is noise / numerical drift
MAX_VALID_DECEL = 3.0   # above this is solver chatter (>3 m/s^2 without brakes is implausible)

BIN_WIDTH_MPS = 5.0     # fit and print on the same bins
MIN_SAMPLES_PER_BIN = 10

# The narrowest window that can carry a two-parameter fit at all: two full
# bins.  Below that the weighted fit is riding one bin and a fragment.  This is
# the tool's only threshold on window WIDTH -- whether a window actually
# separates F_rr from CdA is judged from the fit's own spread instead, which is
# derived from the data rather than guessed (see Fit.identified).
MIN_SPAN_MPS = 2.0 * BIN_WIDTH_MPS

# Coverage ends land on whatever sample straddles the bound; differences below
# this are arithmetic, not a window mismatch worth printing.
TRIM_REPORT_MPS = 0.01


class FitError(Exception):
    """A window that cannot honestly be fitted, with the reason in the text."""


def mph(v: float) -> float:
    return v * MPS_TO_MPH


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

    Wider window than 1 sample -> smooths Chrono solver chatter.
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


class Run:
    """One coastdown CSV, reduced to (speed, decel) samples.

    Holds every usable sample in [MIN_V_MPS, MAX_V_MPS] with no time cap.  The
    fit window is applied later, by fit(), so that a comparison can hand every
    run the SAME window.
    """

    def __init__(self, path: Path):
        self.path = path
        rows = load_csv(path)
        if not rows:
            raise FitError(f"{path}: empty CSV")
        self.release_t = find_release(rows)
        if self.release_t is None:
            raise FitError(f"{path}: could not locate throttle-release event")

        self.samples: list[tuple[float, float]] = []
        for i, r in enumerate(rows):
            t = float(r["sim_time_s"])
            if t < self.release_t + TRANSIENT_AFTER_RELEASE_S:
                continue
            v = float(r["speed_mps"])
            if v < MIN_V_MPS or v > MAX_V_MPS:
                continue
            d = central_decel(rows, i)
            if d is None:
                continue
            if d < MIN_VALID_DECEL or d > MAX_VALID_DECEL:
                continue
            self.samples.append((v, d))
        if not self.samples:
            raise FitError(
                f"{path}: no usable post-release samples in "
                f"{MIN_V_MPS:.1f}-{MAX_V_MPS:.1f} m/s")

        self.v_lo = min(v for v, _ in self.samples)
        self.v_hi = max(v for v, _ in self.samples)
        self.end_v = float(rows[-1]["speed_mps"])

    @property
    def name(self) -> str:
        return self.path.name

    def coverage_str(self) -> str:
        return (f"{self.v_lo:5.2f} - {self.v_hi:5.2f} m/s "
                f"({mph(self.v_lo):4.1f} - {mph(self.v_hi):4.1f} mph)")

    def window(self, v_lo: float, v_hi: float) -> list[tuple[float, float]]:
        return [(v, d) for v, d in self.samples if v_lo <= v <= v_hi]


def check_window(v_lo: float, v_hi: float, what: str) -> None:
    """Raise FitError unless [v_lo, v_hi] can carry a two-parameter fit."""
    span = v_hi - v_lo
    if span < MIN_SPAN_MPS:
        raise FitError(
            f"{what} is {v_lo:.2f} - {v_hi:.2f} m/s "
            f"({mph(v_lo):.1f} - {mph(v_hi):.1f} mph), only {span:.2f} m/s wide "
            f"— below the {MIN_SPAN_MPS:.1f} m/s minimum (two {BIN_WIDTH_MPS:.0f} m/s bins)")
    # There is deliberately no second, geometric guard here ("the window must
    # span at least Nx in v^2").  Whether a window separates F_rr from CdA is
    # a property of the DATA in it, not of its width, and the fit reports that
    # directly: see Fit.identified below, which is derived rather than guessed.


def bin_centroids(samples: list[tuple[float, float]]) -> list[tuple[float, float, int]]:
    """(mean_v, mean_decel, count) per populated BIN_WIDTH_MPS bin."""
    bins: dict[int, list[tuple[float, float]]] = {}
    for v, d in samples:
        key = int(v // BIN_WIDTH_MPS) * int(BIN_WIDTH_MPS)
        bins.setdefault(key, []).append((v, d))
    out = []
    for s in bins.values():
        if len(s) < MIN_SAMPLES_PER_BIN:
            continue
        out.append((sum(v for v, _ in s) / len(s),
                    sum(d for _, d in s) / len(s), len(s)))
    out.sort()
    return out


class Fit:
    """decel = a + b*v^2, with a spread on each coefficient.

    THE SPREAD IS NOT A SAMPLING ERROR.  Each bin averages tens to hundreds of
    samples, so the statistical error on a bin mean is negligible.  What the
    +/- below measures is how well a TWO-PARAMETER F_rr + CdA*v^2 law describes
    the bins in this window at all — the scatter of the bin means about the
    fitted line, with n_bins - 2 degrees of freedom.  It is model error, and
    that is the useful thing to report: it is large exactly when the window
    cannot tell a constant term from a v^2 term, which is the state the tool
    used to report as a confident number.
    """

    def __init__(self, samples: list[tuple[float, float]]):
        self.centroids = bin_centroids(samples)
        self.n_samples = len(samples)
        n = len(self.centroids)
        if n < 2:
            raise FitError(
                f"too few populated bins ({n}, need >=2 with "
                f">={MIN_SAMPLES_PER_BIN} samples each) — the coast may be too "
                "short, or the window too narrow")

        # Weighted least squares on the bin centroids.  A bin mean of w
        # samples has variance sigma^2/w, so weighting by w is inverse-variance
        # weighting, which is what makes the covariance below the standard one.
        sw = sum(w for _, _, w in self.centroids)
        sx = sum(w * v * v for v, _, w in self.centroids)
        sx2 = sum(w * (v * v) ** 2 for v, _, w in self.centroids)
        sy = sum(w * d for _, d, w in self.centroids)
        sxy = sum(w * (v * v) * d for v, d, w in self.centroids)

        denom = sw * sx2 - sx * sx
        if abs(denom) < 1e-12:
            raise FitError("ill-conditioned fit — the fitted speeds are too narrow")
        self.b = (sw * sxy - sx * sy) / denom
        self.a = (sy - self.b * sx) / sw

        self.n_bins = n
        self.dof = n - 2
        if self.dof < 1:
            # Two bins fit two parameters exactly: residuals are zero by
            # construction and say nothing.  Refusing to invent a spread is
            # the point — a fit that cannot be checked must not look checked.
            self.sigma_a = self.sigma_b = None
        else:
            s2 = sum(w * (d - self.a - self.b * v * v) ** 2
                     for v, d, w in self.centroids) / self.dof
            self.sigma_a = math.sqrt(max(s2, 0.0) * sx2 / denom)
            self.sigma_b = math.sqrt(max(s2, 0.0) * sw / denom)

    @property
    def f_rr(self) -> float:
        return MASS_KG * self.a

    @property
    def cd_a(self) -> float:
        return 2.0 * MASS_KG * self.b / RHO_AIR

    @property
    def f_rr_sigma(self) -> float | None:
        return None if self.sigma_a is None else MASS_KG * self.sigma_a

    @property
    def cd_a_sigma(self) -> float | None:
        return None if self.sigma_b is None else 2.0 * MASS_KG * self.sigma_b / RHO_AIR

    @property
    def identified(self) -> bool | None:
        """False when the window does not separate F_rr from CdA.

        The test is the fit's own spread, not a threshold on the window's
        width: CdA is identified when its magnitude exceeds its spread.  At
        |CdA| <= sigma the window is consistent with there being no aero term
        at all, so quoting a CdA off it — "a quarter of spec", say — is
        quoting noise.  None when there are no degrees of freedom to judge on.

        IT IS NOT THRESHOLD-FREE, and saying so plainly matters, because a
        verdict resting on a constant nobody declared is the defect this tool
        exists to fix.  There is no window-WIDTH term — but BIN_WIDTH_MPS
        decides how many bins a window is cut into, hence the degrees of
        freedom, and sigma moves with those.  On the 19.31-29.78 m/s window of
        docs/ev1_chrono_audit.md 11.1 the verdict flips:

            bin width  bins  dof     CdA   sigma  verdict
                  2.0     6    4   0.097   0.089  IDENTIFIED
                  2.5     5    3   0.097   0.098  NOT IDENTIFIED
                  4.0     4    2   0.086   0.099  NOT IDENTIFIED
                  5.0     3    1   0.086   0.144  NOT IDENTIFIED  <- shipped
                  6.0     2    0   0.092    None  no dof

        So the caller has to be able to see the knob: the fit line prints the
        bin width and the dof beside every number, and the NOT IDENTIFIED
        block names the constant.

        Two things about the shipped 5.0, both OBSERVED on those fits rather
        than proved in general.  It gave the widest spread of the widths
        tried, i.e. the cautious end, the one that reaches NOT IDENTIFIED
        soonest.  And the substantive reading survives the flip anyway:
        0.097 +/- 0.089 is a ratio of 1.09, no more a measurement of an aero
        term than 0.086 +/- 0.144 is.  The COEFFICIENTS are far steadier than
        the verdict — over the whole coast CdA holds 0.610-0.621 (new map) and
        0.908-0.914 (old) across those same widths, so what section 11.1
        publishes does not turn on this constant even though the verdict does.
        """
        if self.cd_a_sigma is None:
            return None
        return abs(self.cd_a) > self.cd_a_sigma


def bin_summary(samples: list[tuple[float, float]], indent: str = "  ") -> str:
    """Textual table of (v_bin, count, mean_v, mean_decel, F_avg)."""
    bins: dict[int, list[tuple[float, float]]] = {}
    for v, d in samples:
        key = int(v // BIN_WIDTH_MPS) * int(BIN_WIDTH_MPS)
        bins.setdefault(key, []).append((v, d))
    lines = [f"{indent}v range   |  N  |  mean v  |  mean decel  |  F_avg",
             f"{indent}---------+-----+----------+--------------+--------"]
    for key in sorted(bins):
        s = bins[key]
        if len(s) < 3:
            continue
        mean_v = sum(v for v, _ in s) / len(s)
        mean_d = sum(d for _, d in s) / len(s)
        lines.append(
            f"{indent}{key:>4}-{key + int(BIN_WIDTH_MPS):<3} |"
            f" {len(s):>3} | {mean_v:>7.2f} | {mean_d:>10.4f} |"
            f" {MASS_KG * mean_d:>6.1f} N")
    return "\n".join(lines)


def print_fit(fit: Fit, indent: str = "  ") -> None:
    f_sig = fit.f_rr_sigma
    c_sig = fit.cd_a_sigma
    # The bin width is printed, not just the bin count, because it is the
    # constant the spread and the identifiability verdict below actually turn
    # on — see Fit.identified.  A reader who can see the knob can judge the
    # number; one who cannot will read the verdict as absolute.
    print(f"{indent}decel(v) = {fit.a:.6f} + {fit.b:.8f} * v^2   "
          f"({fit.n_bins} bins of {BIN_WIDTH_MPS:.1f} m/s, {fit.dof} dof)")
    if f_sig is None:
        print(f"{indent}F_rr     = {fit.f_rr:7.1f} N          "
              f"(real EV1 spec ~{SPEC_F_RR_N:.0f} N, ratio {fit.f_rr / SPEC_F_RR_N:.2f}x)")
        print(f"{indent}CdA      = {fit.cd_a:7.3f} m^2        "
              f"(real EV1 spec ~{SPEC_CD_A:.2f} m^2, ratio {fit.cd_a / SPEC_CD_A:.2f}x)")
        print(f"{indent}NO SPREAD: {fit.n_bins} bins fit 2 parameters exactly, so nothing "
              f"here has been\n{indent}           checked against anything. Widen the window.")
        return
    print(f"{indent}F_rr     = {fit.f_rr:7.1f} +/- {f_sig:5.1f} N    "
          f"(real EV1 spec ~{SPEC_F_RR_N:.0f} N, ratio {fit.f_rr / SPEC_F_RR_N:.2f}x)")
    print(f"{indent}CdA      = {fit.cd_a:7.3f} +/- {c_sig:5.3f} m^2  "
          f"(real EV1 spec ~{SPEC_CD_A:.2f} m^2, ratio {fit.cd_a / SPEC_CD_A:.2f}x)")
    if not fit.identified:
        print(f"{indent}*** CdA IS NOT IDENTIFIED over this window: "
              f"|{fit.cd_a:.3f}| <= {c_sig:.3f}.")
        print(f"{indent}    The bins here are consistent with no aero term at all, so this "
              f"CdA is\n{indent}    not a measurement of one. Widen the window before quoting it.")
        print(f"{indent}    This verdict depends on BIN_WIDTH_MPS = {BIN_WIDTH_MPS:.1f}, which "
              f"sets the {fit.dof} dof\n{indent}    behind that spread; narrower bins buy dof and "
              f"can flip it. 5.0 is the\n{indent}    cautious end of the widths tried — see "
              f"Fit.identified for the sweep.")


# --- the common-window rule ---------------------------------------------------

def common_window(runs: list[Run], v_min: float | None,
                  v_max: float | None) -> tuple[float, float]:
    """The speed range every run covers, or FitError naming what each covers.

    With --v-min/--v-max the caller has stated the window; a run that does not
    cover it is an error rather than something to quietly shrink around.
    """
    lo = max(r.v_lo for r in runs)
    hi = min(r.v_hi for r in runs)

    if v_min is not None or v_max is not None:
        want_lo = v_min if v_min is not None else lo
        want_hi = v_max if v_max is not None else hi
        short = [r for r in runs if r.v_lo > want_lo + 1e-9 or r.v_hi < want_hi - 1e-9]
        if short:
            detail = "\n".join(f"    {r.name}  covers {r.coverage_str()}" for r in runs)
            raise FitError(
                f"the requested window {want_lo:.2f} - {want_hi:.2f} m/s "
                f"({mph(want_lo):.1f} - {mph(want_hi):.1f} mph) is not covered by "
                f"{'all runs' if len(runs) > 1 else 'the run'}:\n{detail}")
        check_window(want_lo, want_hi, "the requested window")
        return want_lo, want_hi

    if hi <= lo:
        detail = "\n".join(f"    {r.name}  covers {r.coverage_str()}" for r in runs)
        raise FitError(
            "the runs share NO speed range, so there is no window in which "
            f"their fits mean the same thing:\n{detail}")
    try:
        check_window(lo, hi, "the speed range every run covers")
    except FitError as e:
        detail = "\n".join(f"    {r.name}  covers {r.coverage_str()}" for r in runs)
        raise FitError(f"{e}\n{detail}") from None
    return lo, hi


# --- reporting ----------------------------------------------------------------

def report_single(run: Run, v_lo: float, v_hi: float) -> bool:
    print(f"[fit_coastdown] {run.name}")
    print(f"  throttle release at t={run.release_t:.3f} s")
    print(f"  fitted over {v_lo:.2f} - {v_hi:.2f} m/s "
          f"({mph(v_lo):.1f} - {mph(v_hi):.1f} mph)  "
          f"— the whole coast inside {MIN_V_MPS:.0f}-{MAX_V_MPS:.0f} m/s, no time cap")
    samples = run.window(v_lo, v_hi)
    print(f"  {len(samples)} usable post-release samples\n")
    print(bin_summary(samples))
    print("\n  === Fit ===")
    fit = Fit(samples)
    print_fit(fit)
    return fit.identified is not False


def report_compare(runs: list[Run], v_lo: float, v_hi: float,
                   stated_window: bool) -> bool:
    print(f"[fit_coastdown] comparing {len(runs)} runs\n")
    for r in runs:
        print(f"  {r.name}")
        print(f"      release t={r.release_t:.3f} s   covers {r.coverage_str()}")
    print()
    kind = "requested window" if stated_window else "speed range every run covers"
    print(f"  FIT WINDOW ({kind}): {v_lo:.2f} - {v_hi:.2f} m/s "
          f"({mph(v_lo):.1f} - {mph(v_hi):.1f} mph)")
    print("  Every number below is fitted over exactly that range, so the "
          "difference between\n  runs is the plant and not the window.")
    for r in runs:
        # TRIM_REPORT_MPS, not zero: the runs' coverage ends land on whatever
        # sample straddles the bound, so a hair of difference is arithmetic
        # rather than a window mismatch worth naming.
        trimmed = []
        if r.v_lo < v_lo - TRIM_REPORT_MPS:
            trimmed.append(f"{v_lo - r.v_lo:.2f} m/s off the bottom")
        if r.v_hi > v_hi + TRIM_REPORT_MPS:
            trimmed.append(f"{r.v_hi - v_hi:.2f} m/s off the top")
        if trimmed:
            print(f"      {r.name}: trimmed {' and '.join(trimmed)}")
    print()

    results: list[tuple[Run, Fit]] = []
    for r in runs:
        fit = Fit(r.window(v_lo, v_hi))
        print(f"  === {r.name} ===  ({fit.n_samples} samples)")
        print_fit(fit, indent="      ")
        print()
        results.append((r, fit))

    base_r, base = results[0]
    if len(results) > 1:
        print(f"  === Change from {base_r.name} ===")
        for r, fit in results[1:]:
            d_f, d_c = fit.f_rr - base.f_rr, fit.cd_a - base.cd_a
            line = (f"      {r.name}:  F_rr {base.f_rr:.1f} -> {fit.f_rr:.1f} N "
                    f"({d_f:+.1f} N),  CdA {base.cd_a:.3f} -> {fit.cd_a:.3f} m^2 "
                    f"({d_c:+.3f} m^2)")
            print(line)
            # A change smaller than the two fits' combined spread is not a
            # result.  Saying so here is the difference between "the plant
            # changed" and "the numbers changed".
            if base.cd_a_sigma is not None and fit.cd_a_sigma is not None:
                comb = math.hypot(base.cd_a_sigma, fit.cd_a_sigma)
                verdict = ("larger than" if abs(d_c) > comb else "WITHIN")
                print(f"          the CdA change is {verdict} the combined spread "
                      f"(+/- {comb:.3f} m^2)")

    unidentified = [r.name for r, f in results if f.identified is False]
    if unidentified:
        print()
        print("  *** This comparison does not support a statement about CdA: the "
              "window does\n      not identify it for " + ", ".join(unidentified) +
              ".  Fit over a wider speed\n      range — extend max_time_s so both "
              "coasts run further down.")
    return not unidentified


# --- selftest -----------------------------------------------------------------

def _synth_csv(path: Path, f_rr: float, cd_a: float, v0: float, t_end: float,
               knee_v: float = 0.0, knee_n_per_mps: float = 0.0,
               dt: float = 0.05, mass: float = MASS_KG) -> None:
    """Write a coastdown CSV for a car with a known drag law.

    Integrates m dv/dt = -(F_rr + 0.5 rho CdA v^2 + knee) forward from v0,
    with a 3 s full-throttle lead-in so find_release() has something to find.

    `knee` is knee_n_per_mps * (v - knee_v) newtons above knee_v and zero
    below it.  That is the shape of the real defect in miniature: the EV1
    coast map added a term that rises with speed above ~8350 RPM, so the
    plant is NOT exactly two-parameter, and a fit truncated at a different
    speed sees a different mix of the two regimes.  With this term at zero the
    car is exactly (f_rr, cd_a) and the fit must recover it.
    """
    rows = [("0.000", f"{v0:.6f}", "1.0", "0.0")]
    t = 0.0
    while t < 3.0:
        t += dt
        rows.append((f"{t:.3f}", f"{v0:.6f}", "1.0", "0.0"))
    v = v0
    while t < t_end:
        # sub-step the integration so the CSV is the analytic solution to
        # within far less than the fit's resolution
        for _ in range(50):
            extra = knee_n_per_mps * max(0.0, v - knee_v)
            v -= (f_rr + 0.5 * RHO_AIR * cd_a * v * v + extra) / mass * (dt / 50.0)
            if v <= 0.1:
                break
        t += dt
        rows.append((f"{t:.3f}", f"{max(v, 0.1):.6f}", "0.0", "0.0"))
        if v <= 0.1:
            break
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sim_time_s", "speed_mps", "applied_throttle", "applied_front_brake"])
        w.writerows(rows)


def selftest() -> int:
    """Prove the fit recovers a known car and that the window guards can FAIL.

    A guard nobody has watched fail is not a guard.  Each case below prints
    the value the verdict turns on, and the suite fails if a case that should
    refuse is accepted or vice versa.
    """
    ok = True

    def case(name: str, passed: bool, detail: str) -> None:
        nonlocal ok
        ok &= passed
        print(f"  [{'ok  ' if passed else 'FAIL'}] {name}: {detail}")

    def guarded(name: str, thunk):
        """Run thunk(); a raised FitError becomes a reported FAIL, not a crash.

        A mutation that breaks an early case must not stop the later cases
        from reporting — a selftest that dies partway tells you something is
        wrong without telling you what, which is most of the value gone.
        (Setting MIN_SPAN_MPS to 25.0 used to end this run in a traceback.)
        """
        try:
            return thunk()
        except FitError as e:
            case(name, False, f"raised instead of returning — {e.args[0].splitlines()[0]}")
            return None

    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        TRUE_F_RR, TRUE_CD_A = 200.0, 0.600

        # 1. The fit recovers a car whose drag we planted.  Without this the
        #    rest is a consistency check on an unknown.
        long_a = d / "long_a.csv"
        _synth_csv(long_a, TRUE_F_RR, TRUE_CD_A, 30.0, 200.0)
        run = Run(long_a)
        f = guarded("recovers planted drag",
                    lambda: Fit(run.window(*common_window([run], None, None))))
        if f is not None:
            case("recovers planted drag",
                 abs(f.f_rr - TRUE_F_RR) < 2.0 and abs(f.cd_a - TRUE_CD_A) < 0.01,
                 f"F_rr {f.f_rr:.1f} +/- {f.f_rr_sigma:.1f} N (planted {TRUE_F_RR:.0f}), "
                 f"CdA {f.cd_a:.3f} +/- {f.cd_a_sigma:.3f} m^2 (planted {TRUE_CD_A:.3f})")
            case("...and calls that CdA identified", f.identified is True,
                 f"|{f.cd_a:.3f}| vs spread {f.cd_a_sigma:.3f}")

        # 2. THE ARTEFACT, in miniature.  ONE car -- a car with a knee, i.e.
        #    any real plant -- sampled over two coasts of different length.
        #    Different coast length is the one thing a change to drag or regen
        #    always produces, and it is what the 30 s time cap turned into a
        #    speed difference.  Fitted over each run's own range the two
        #    disagree; fitted over the speeds they share they cannot, because
        #    over those speeds they are the same rows.
        KNEE_V, KNEE_N = 23.0, 12.0
        knee_long, knee_short = d / "knee_long.csv", d / "knee_short.csv"
        _synth_csv(knee_long, TRUE_F_RR, TRUE_CD_A, 30.0, 200.0, KNEE_V, KNEE_N)
        _synth_csv(knee_short, TRUE_F_RR, TRUE_CD_A, 30.0, 60.0, KNEE_V, KNEE_N)
        ra, rb = Run(knee_long), Run(knee_short)

        own_a = guarded("unlike windows DISagree on one car",
                        lambda: Fit(ra.window(ra.v_lo, ra.v_hi)))
        own_b = guarded("unlike windows DISagree on one car",
                        lambda: Fit(rb.window(rb.v_lo, rb.v_hi)))
        if own_a is not None and own_b is not None:
          case("unlike windows DISagree on one car (the defect, reproduced)",
             abs(own_a.f_rr - own_b.f_rr) > 5.0 or abs(own_a.cd_a - own_b.cd_a) > 0.02,
             f"own ranges {ra.v_lo:.1f}-{ra.v_hi:.1f} vs {rb.v_lo:.1f}-{rb.v_hi:.1f} m/s: "
             f"F_rr {own_a.f_rr:.1f} vs {own_b.f_rr:.1f} N, "
             f"CdA {own_a.cd_a:.3f} vs {own_b.cd_a:.3f} m^2")

        def _together():
            lo, hi = common_window([ra, rb], None, None)
            fa, fb = Fit(ra.window(lo, hi)), Fit(rb.window(lo, hi))
            return lo, hi, fa, fb
        got = guarded("...and the common window puts them back together", _together)
        if got is not None:
            lo, hi, fa, fb = got
            case("...and the common window puts them back together",
                 abs(fa.f_rr - fb.f_rr) < 1.0 and abs(fa.cd_a - fb.cd_a) < 0.005,
                 f"common {lo:.1f}-{hi:.1f} m/s: F_rr {fa.f_rr:.1f} vs {fb.f_rr:.1f} N, "
                 f"CdA {fa.cd_a:.3f} vs {fb.cd_a:.3f} m^2")

        # 3. The refusal threshold.  Each probe run starts high enough that
        #    its coverage is [10 m/s, start]; pairing it with the long run
        #    makes the overlap exactly `start - 10` wide.
        def overlap_case(width: float, want_refuse: bool, why_pinned: str) -> None:
            p = d / f"overlap_{width:.1f}.csv"
            _synth_csv(p, TRUE_F_RR, TRUE_CD_A, 10.0 + width, 400.0)
            rn = Run(p)
            try:
                lo, hi = common_window([ra, rn], None, None)
                refused, detail = False, f"accepted {lo:.2f}-{hi:.2f} m/s"
            except FitError as e:
                refused, detail = True, str(e).splitlines()[0]
            case(f"overlap {rn.v_hi - 10.0:.2f} m/s wide -> "
                 f"{'refuse' if want_refuse else 'accept'} ({why_pinned})",
                 refused == want_refuse, detail)

        # 3a. TWO PINNED PROBES, and they are the ones that matter.  The three
        #     below walk the threshold, but they are written as offsets FROM
        #     MIN_SPAN_MPS, so they move when it moves: they prove the
        #     comparison is the right way round and the arithmetic is right,
        #     and they cannot notice the constant itself being changed.
        #     Moving MIN_SPAN_MPS 10.0 -> 8.0 leaves all three green.
        #
        #     These two are literals on purpose, so the guard can be TUNED but
        #     not DEFUSED, and each literal is pinned to something real rather
        #     than to the constant under test:
        #       5.0  = one BIN_WIDTH_MPS bin.  Two parameters fitted across a
        #              single bin is not a measurement at any threshold, so
        #              this must refuse however MIN_SPAN_MPS is set.
        #       19.0 = inside the 19.77 m/s-wide window (10.01-29.78 m/s) that
        #              docs/ev1_chrono_audit.md 11.1 publishes its numbers
        #              over.  A guard grown wide enough to refuse this would
        #              refuse the fit the audit rests on, so it must accept.
        #              (Realised overlap prints as ~18.6 m/s: the synthetic
        #              run's top sample sits just under its start speed.)
        #
        #     Checked by mutation, and this is the property being bought:
        #       MIN_SPAN_MPS 10.0 -> 8.0   slips through — a mild loosening
        #                                   that still refuses one-bin windows
        #                                   and still accepts the audit's
        #       MIN_SPAN_MPS 10.0 -> 4.0   CAUGHT by the 5.0 probe (defused)
        #       MIN_SPAN_MPS 10.0 -> 25.0  CAUGHT by the 19.0 probe (would
        #                                   refuse the published fit)
        overlap_case(5.0, True, "one bin wide; a fit over it is not a measurement")
        overlap_case(19.0, False, "narrower than the window audit 11.1 publishes")

        # 3b. ...and the walk across the current threshold, for the boundary
        #     arithmetic.  Offsets from MIN_SPAN_MPS by design — see 3a.
        for delta, want_refuse in ((2.0, False), (0.5, False), (-1.0, True)):
            overlap_case(MIN_SPAN_MPS + delta, want_refuse,
                         f"MIN_SPAN_MPS{delta:+.1f}")

        # 4. Runs that overlap in speed but are far apart: no common range at
        #    all.  The tool must name both ranges, not fit them separately.
        fast = d / "fast_only.csv"
        _synth_csv(fast, TRUE_F_RR, TRUE_CD_A, 34.0, 12.0)   # stays above ~31
        try:
            common_window([Run(fast), rb], None, None)
            case("disjoint coverage refuses", False, "accepted a disjoint pair")
        except FitError as e:
            first = str(e).splitlines()[0]
            case("disjoint coverage refuses", "share NO speed range" in first, first)

        # 5. An explicitly requested window a run does not cover is an error,
        #    not something to silently shrink around.
        try:
            common_window([rb], 10.0, 34.0)
            case("uncovered --v-max refuses", False, "accepted an uncovered window")
        except FitError as e:
            case("uncovered --v-max refuses", True, str(e).splitlines()[0])

    print()
    print("selftest " + ("ok" if ok else "FAILED"))
    return 0 if ok else 1


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="*", help="coastdown CSV(s); two or more compare")
    ap.add_argument("--v-min", type=float, default=None,
                    help="state the low end of the fit window in m/s "
                         "(default: the lowest speed every run reaches)")
    ap.add_argument("--v-max", type=float, default=None,
                    help="state the high end of the fit window in m/s "
                         "(default: the highest speed every run reaches)")
    ap.add_argument("--selftest", action="store_true",
                    help="prove the fit and the window guards work, on synthetic runs")
    args = ap.parse_args(argv[1:])

    if args.selftest:
        return selftest()

    paths = [Path(p) for p in (args.csv or ["scenario_coastdown.csv"])]
    missing = [p for p in paths if not p.exists()]
    if missing:
        for p in missing:
            print(f"[fit_coastdown] CSV not found: {p}", file=sys.stderr)
        return 1

    try:
        runs = [Run(p) for p in paths]
        v_lo, v_hi = common_window(runs, args.v_min, args.v_max)
    except FitError as e:
        print(f"[fit_coastdown] REFUSING to fit — {e}", file=sys.stderr)
        if len(paths) > 1:
            print("\n  Two runs fitted over different speed ranges are not "
                  "comparable: the\n  difference you would read off them is the "
                  "window, not the plant.  Extend\n  max_time_s so both coasts "
                  "reach the same speed, or state a window both\n  cover with "
                  "--v-min/--v-max.", file=sys.stderr)
        return 1

    stated = args.v_min is not None or args.v_max is not None
    try:
        if len(runs) == 1:
            # A single run is a look at one car: report the caveat, but an
            # unidentified CdA is not by itself a failed run.
            report_single(runs[0], v_lo, v_hi)
            return 0
        # A comparison's whole output is a claim about a DIFFERENCE.  If the
        # window does not identify CdA for one of the runs, that claim is not
        # supported, and the exit code has to say so or the caveat above is
        # just text somebody scrolls past.
        return 0 if report_compare(runs, v_lo, v_hi, stated) else 1
    except FitError as e:
        print(f"[fit_coastdown] REFUSING to fit — {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
