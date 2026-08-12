#!/usr/bin/env bash
# Run an ABS test scenario twice — once with BTCM commanding the brakes,
# once without — and produce a side-by-side comparison.
#
# Usage:
#   ./scripts/run_abs_compare.sh [test]
#
# Where [test] is one of:
#   high_mu     — straight stop on dry asphalt        (default)
#   low_mu      — accelerate on asphalt, brake on packed snow (mu 0.20)
#   mu_jump     — accelerate on asphalt, brake into ice transition
#   split_mu    — left wheels on asphalt, right wheels on ice
#
# BTCM-on run:  PIM + RSA + BTCM controllers running.  Front wheels
#               modulated via solenoid HOLD/DUMP phases; rear wheels via
#               EMB self-energizing drum model.
#
# BTCM-off run: only PIM running (no BTCM, no RSA-driven run mode).
#               ev1sim's freshness path sees solenoid signals as stale →
#               front passes through hydraulic line; rear free-rolls
#               (no hydraulic backup on EV1 rear brakes).
#
# Outputs in /tmp/ev1sim_abs_<test>/:
#   abs_btcm_on.csv    — full-controller run
#   abs_btcm_off.csv   — BTCM disabled
#   summary.txt        — stop distance + per-wheel slip + ASCII plots
#   *.btcm.log         — BTCM stdout (ABS phase events live here)
#   *.ev1sim.log       — ev1sim stdout (scenario events + freshness)

set -euo pipefail

TEST="${1:-high_mu}"
case "$TEST" in
    # hard_brake added 2026-08-12 alongside config/abs_hard_brake.json.  Before
    # that this list rejected it before ever resolving a config path, so the
    # scenario could not be launched through this script at all.
    high_mu|low_mu|mu_jump|split_mu|brake_and_steer|diagonal_mu|hard_brake) ;;
    *) echo "[abs] unknown test '$TEST' — use high_mu | low_mu | mu_jump | split_mu | brake_and_steer | diagonal_mu | hard_brake" >&2; exit 1 ;;
esac

EV1SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
# Allow ELECTRICSIM_DIR override via environment so the script can run
# against a worktree build of the controllers — the default points at
# the main checkout.
if [[ -n "${ELECTRICSIM_DIR:-}" ]]; then
    ELECTRICSIM_DIR="$(cd "$ELECTRICSIM_DIR" && pwd)"
else
    ELECTRICSIM_DIR="$(cd "$EV1SIM_DIR/../electricsim" 2>/dev/null && pwd || true)"
fi
[[ -z "$ELECTRICSIM_DIR" ]] && {
    echo "[abs] cannot find electricsim (set ELECTRICSIM_DIR or place at ../electricsim)" >&2
    exit 1
}

EV1SIM_BIN="$EV1SIM_DIR/build/ev1sim"
PIM_BIN="$ELECTRICSIM_DIR/build/ev1/pim/pim_controller"
BTCM_BIN="$ELECTRICSIM_DIR/build/ev1/btcm/ex_btcm_controller"
RSA_BIN="$ELECTRICSIM_DIR/build/ev1/rsa/rsa_controller"
[[ -x "$EV1SIM_BIN" ]] || { echo "missing $EV1SIM_BIN" >&2; exit 1; }
[[ -x "$PIM_BIN"    ]] || { echo "missing $PIM_BIN"    >&2; exit 1; }
[[ -x "$BTCM_BIN"   ]] || { echo "missing $BTCM_BIN"   >&2; exit 1; }
[[ -x "$RSA_BIN"    ]] || { echo "missing $RSA_BIN"    >&2; exit 1; }

CONFIG="$EV1SIM_DIR/config/abs_${TEST}.json"
[[ -f "$CONFIG" ]] || { echo "missing $CONFIG" >&2; exit 1; }

OUT="/tmp/ev1sim_abs_${TEST}"
rm -rf "$OUT"
mkdir -p "$OUT"

cleanup_shm() {
    python3 - <<'PY' 2>/dev/null || true
import ctypes
libc = ctypes.CDLL("libc.dylib")
libc.shm_unlink.argtypes = [ctypes.c_char_p]
libc.shm_unlink.restype  = ctypes.c_int
for n in ["/electricsim_ev1_bus", "/electricsim_chassis_bus"]:
    libc.shm_unlink(n.encode())
PY
}

run_scenario() {
    local label="$1"
    local with_btcm="$2"   # "1" or "0"
    local csv_dest="$3"

    cleanup_shm
    echo "[abs] === $label ($TEST) ==="
    local pim_log="$OUT/$label.pim.log"
    local btcm_log="$OUT/$label.btcm.log"
    local rsa_log="$OUT/$label.rsa.log"
    local sim_log="$OUT/$label.ev1sim.log"

    # ev1sim creates the SHM segments; controllers attach.  Wait for the
    # "connected to main harness bus" log line before starting controllers.
    "$EV1SIM_BIN" --config "$CONFIG" > "$sim_log" 2>&1 &
    local sim_pid=$!
    local waited=0
    while ! grep -q "connected to main harness bus" "$sim_log" 2>/dev/null; do
        sleep 0.2
        waited=$((waited + 1))
        if [[ $waited -gt 60 ]]; then
            echo "[abs] ev1sim failed to bring up buses within 12 s" >&2
            cat "$sim_log" >&2
            kill "$sim_pid" 2>/dev/null || true
            exit 1
        fi
    done
    sleep 0.3

    ( cd "$ELECTRICSIM_DIR/build" && exec "$PIM_BIN" ) > "$pim_log" 2>&1 &
    local pim_pid=$!
    sleep 0.3

    local rsa_pid=""
    local btcm_pid=""
    if [[ "$with_btcm" == "1" ]]; then
        ( cd "$ELECTRICSIM_DIR/build" && exec "$RSA_BIN" ) > "$rsa_log" 2>&1 &
        rsa_pid=$!
        sleep 0.3
        # Capture BTCM-side state into a CSV alongside ev1sim's
        # chassis-side stats CSV.  Lets the report overlay firmware-
        # view data on chassis-view charts (vehicle-speed estimator,
        # firmware-perceived slip, accelerometer reading, etc.).
        local btcm_csv="$OUT/$label.btcm.csv"
        ( cd "$ELECTRICSIM_DIR/build" && BTCM_CSV_LOG="$btcm_csv" exec "$BTCM_BIN" ) > "$btcm_log" 2>&1 &
        btcm_pid=$!
        sleep 0.3
    fi

    cleanup() {
        [[ -n "${btcm_pid:-}" ]] && kill "$btcm_pid" 2>/dev/null || true
        [[ -n "${rsa_pid:-}"  ]] && kill "$rsa_pid"  2>/dev/null || true
        kill "$pim_pid" 2>/dev/null || true
        wait "$pim_pid" 2>/dev/null || true
        [[ -n "${btcm_pid:-}" ]] && wait "$btcm_pid" 2>/dev/null || true
        [[ -n "${rsa_pid:-}"  ]] && wait "$rsa_pid"  2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    wait "$sim_pid" 2>/dev/null || true

    cleanup
    trap - EXIT INT TERM

    # Scenario writes its CSV in ev1sim's cwd (here, EV1SIM_DIR).  Filename
    # convention: scenario_<name>.csv where <name> matches the scenario's
    # "name" field — for our suite, that's the basename of the scenario file.
    local sc_name=""
    case "$TEST" in
        high_mu)         sc_name="abs_high_mu_stop"     ;;
        low_mu)          sc_name="abs_low_mu_stop"      ;;
        mu_jump)         sc_name="abs_mu_jump"          ;;
        split_mu)        sc_name="abs_split_mu"         ;;
        brake_and_steer) sc_name="abs_brake_and_steer"  ;;
        diagonal_mu)     sc_name="abs_diagonal_mu"      ;;
        hard_brake)      sc_name="abs_hard_brake"       ;;
    esac
    # A test name has to appear in TWO lists: the whitelist at the top and this
    # one.  Adding it to only the whitelist left sc_name unset, and under
    # `set -u` the script then died on an unbound variable at the next line
    # without saying which list was missing it.  Fail loudly and name the fix.
    if [[ -z "$sc_name" ]]; then
        echo "[abs] '$TEST' passed the whitelist but has no scenario-name mapping" >&2
        echo "[abs] add it to the case statement in run_scenario()" >&2
        exit 1
    fi
    local default_csv="$EV1SIM_DIR/scenario_${sc_name}.csv"
    if [[ -f "$default_csv" ]]; then
        mv "$default_csv" "$csv_dest"
        echo "[abs] $label CSV → $csv_dest"
    else
        echo "[abs] WARNING: expected CSV at $default_csv not found" >&2
        echo "[abs] check $sim_log for scenario errors" >&2
    fi
}

run_scenario abs_btcm_on  1  "$OUT/abs_btcm_on.csv"
run_scenario abs_btcm_off 0  "$OUT/abs_btcm_off.csv"

echo
echo "[abs] === comparison: $TEST ==="
"$EV1SIM_DIR/scripts/compare_abs_runs.py" \
    "$OUT/abs_btcm_on.csv" \
    "$OUT/abs_btcm_off.csv" \
    | tee "$OUT/summary.txt"
