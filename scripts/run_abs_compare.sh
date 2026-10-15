#!/usr/bin/env bash
# Run an ABS test scenario twice — once with BTCM commanding the brakes,
# once without — and produce a side-by-side comparison.
#
# Usage:
#   ELECTRICSIM_DIR=/path/to/electricsim ./scripts/run_abs_compare.sh [test]
#
# Where [test] is one of:
#   high_mu          — straight stop on dry asphalt        (default)
#   low_mu           — accelerate on asphalt, brake on packed snow
#   mu_jump          — accelerate on asphalt, brake into ice transition
#   split_mu         — left wheels on asphalt, right wheels on ice
#   brake_and_steer  — brake while steering
#   diagonal_mu      — diagonal grip pattern
#
# The two runs are the external sim's acceptance-harness case abs_<test>
# (`python3 -m vat run --case abs_<test>`, run in ELECTRICSIM_DIR with
# EV1SIM_DIR pointing here), which brings the whole controller fleet up on the
# shared wire table around this checkout's build/ev1sim and its
# config/abs_<test>.json:
#
# BTCM-on run:  every module, BTCM included.  Front wheels modulated via
#               solenoid HOLD/DUMP phases; rear wheels via the electric
#               drum brakes.
# BTCM-off run: the same fleet without the BTCM.  Front brakes pass through
#               the hydraulic line; the rear free-rolls (no hydraulic backup
#               on EV1 rear brakes).
#
# ELECTRICSIM_DIR must be a built electricsim tree (firmware on); it defaults
# to ../electricsim.  The harness's own pass/fail verdict is not this
# script's: it only needs the two runs' CSVs.
#
# Outputs in /tmp/ev1sim_abs_<test>/:
#   abs_btcm_on.csv    — full-controller run
#   abs_btcm_off.csv   — BTCM disabled
#   abs_btcm_on.btcm.csv — BTCM-side firmware view of the on run
#   summary.txt        — stop distance + per-wheel slip + ASCII plots
#   harness/           — the harness's run directory (per-module logs)

set -euo pipefail

TEST="${1:-high_mu}"
case "$TEST" in
    high_mu|low_mu|mu_jump|split_mu|brake_and_steer|diagonal_mu) ;;
    *) echo "[abs] unknown test '$TEST' — use high_mu | low_mu | mu_jump | split_mu | brake_and_steer | diagonal_mu" >&2; exit 1 ;;
esac
CASE="abs_${TEST}"

EV1SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
if [[ -n "${ELECTRICSIM_DIR:-}" ]]; then
    ELECTRICSIM_DIR="$(cd "$ELECTRICSIM_DIR" && pwd)"
else
    ELECTRICSIM_DIR="$(cd "$EV1SIM_DIR/../electricsim" 2>/dev/null && pwd || true)"
fi
[[ -z "$ELECTRICSIM_DIR" ]] && {
    echo "[abs] cannot find electricsim (set ELECTRICSIM_DIR or place at ../electricsim)" >&2
    exit 1
}
[[ -x "$EV1SIM_DIR/build/ev1sim" ]] || { echo "missing $EV1SIM_DIR/build/ev1sim" >&2; exit 1; }
[[ -d "$ELECTRICSIM_DIR/vat" ]] || { echo "missing $ELECTRICSIM_DIR/vat (acceptance harness)" >&2; exit 1; }

OUT="/tmp/ev1sim_abs_${TEST}"
rm -rf "$OUT"
mkdir -p "$OUT"

echo "[abs] === $CASE: BTCM-on and BTCM-off through the acceptance harness ==="
# Exit 1 is the harness's own criteria verdict; the CSV check below decides.
set +e
( cd "$ELECTRICSIM_DIR" &&
  EV1SIM_DIR="$EV1SIM_DIR" python3 -m vat run --case "$CASE" --out "$OUT/harness" ) \
    > "$OUT/harness.log" 2>&1
rc=$?
set -e
if [[ $rc -gt 1 ]]; then
    echo "[abs] harness run failed (exit $rc); see $OUT/harness.log" >&2
    exit 1
fi

run_dir="$OUT/harness/$CASE"
for leg in on off; do
    csv="$run_dir/$leg/scenario.csv"
    if [[ ! -f "$csv" ]]; then
        echo "[abs] expected CSV not produced: $csv (see $OUT/harness.log)" >&2
        exit 1
    fi
    cp "$csv" "$OUT/abs_btcm_${leg}.csv"
done
if [[ -f "$run_dir/on/btcm.csv" ]]; then
    cp "$run_dir/on/btcm.csv" "$OUT/abs_btcm_on.btcm.csv"
fi

echo
echo "[abs] === comparison: $TEST ==="
"$EV1SIM_DIR/scripts/compare_abs_runs.py" \
    "$OUT/abs_btcm_on.csv" \
    "$OUT/abs_btcm_off.csv" \
    | tee "$OUT/summary.txt"
