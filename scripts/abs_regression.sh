#!/usr/bin/env bash
#
# NOTE (2026-06-10): the formal, gating acceptance campaign for these
# scenarios (tolerance-banded criteria, versioned baselines, nightly CI) now
# lives with the ECU-electronics side's test infrastructure. This script
# remains as the interactive illustration of the scriptable scenario
# feature; its abs_baseline.txt is no longer the gating source of truth.
# ABS regression gate — the integration test for ev1sim plant-physics changes
# the unit suite cannot exercise (brake-plant tau constants, tire model, etc.).
# It needs ev1sim AND electricsim running together over the shared-memory bus,
# so it is NOT part of fast `ctest`; run it before merging anything that
# touches the brake plant or the ABS loop.
#
# For each scenario it runs the BTCM-on/off comparison (scripts/run_abs_compare.sh)
# and checks the BTCM-on headline metric against scripts/abs_baseline.txt
# (captured at a pinned electricsim commit — see that file's header).  Any
# regression beyond tolerance exits non-zero.
#
# Usage:
#   ELECTRICSIM_DIR=/path/to/electricsim ./scripts/abs_regression.sh
#   ELECTRICSIM_DIR=/path/to/electricsim ./scripts/abs_regression.sh mu_jump high_mu
#
# ELECTRICSIM_DIR must point at a built electricsim tree (pim/btcm/rsa
# controller binaries).  From a git worktree the default ../electricsim does
# not resolve, so set it explicitly.  Controllers must be built at (or close
# to) the commit recorded in abs_baseline.txt for the comparison to be valid.
set -euo pipefail

EV1SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BASELINE="$EV1SIM_DIR/scripts/abs_baseline.txt"
[[ -f "$BASELINE" ]] || { echo "missing baseline: $BASELINE" >&2; exit 1; }

SCENARIOS=("$@")
if [[ ${#SCENARIOS[@]} -eq 0 ]]; then
    SCENARIOS=(high_mu low_mu mu_jump split_mu brake_and_steer diagonal_mu)
fi

fail=0
for s in "${SCENARIOS[@]}"; do
    echo "=== ABS regression: $s ==="
    run_log="/tmp/abs_regression_${s}.log"
    if ! "$EV1SIM_DIR/scripts/run_abs_compare.sh" "$s" > "$run_log" 2>&1; then
        echo "  [FAIL] $s — scenario run failed (see $run_log)"
        fail=1
        continue
    fi
    on="/tmp/ev1sim_abs_${s}/abs_btcm_on.csv"
    off="/tmp/ev1sim_abs_${s}/abs_btcm_off.csv"
    if [[ ! -f "$on" || ! -f "$off" ]]; then
        echo "  [FAIL] $s — expected CSVs not produced (see $run_log)"
        fail=1
        continue
    fi
    if ! python3 "$EV1SIM_DIR/scripts/compare_abs_runs.py" "$on" "$off" \
            --check "$BASELINE" "$s"; then
        fail=1
    fi
done

echo
if [[ $fail -eq 0 ]]; then
    echo "ABS regression: ALL PASS"
else
    echo "ABS regression: FAILURES (see per-scenario [FAIL] lines above)"
fi
exit $fail
