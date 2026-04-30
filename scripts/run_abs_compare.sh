#!/usr/bin/env bash
# Run the ABS hard-brake scenario twice — once with BTCM commanding the
# brakes, once without — and produce a side-by-side comparison.
#
# BTCM-on run:  PIM + BTCM controllers running.  ev1sim's
#               ApplyAbsFrontBrake modulates the front via solenoid
#               phases, ApplyRearEmbBrake drives the rear via EMB cmds.
#               This is the realistic stop with anti-lock active.
#
# BTCM-off run: only PIM running (no BTCM).  ev1sim's freshness path
#               sees solenoid signals as stale → front passes through
#               local pedal; rear-motor signals stale → rear free-rolls.
#               Models the real EV1 BTCM-failure mode.
#
# After both runs, compare_abs_runs.py crunches the CSVs and prints a
# stopping-distance + per-wheel slip summary plus an ASCII speed plot.
#
# Usage:
#   ./scripts/run_abs_compare.sh
#
# Outputs:
#   /tmp/ev1sim_abs/
#       abs_btcm_on.csv     — full-controller run
#       abs_btcm_off.csv    — BTCM disabled
#       summary.txt         — stopping-distance + slip stats + ASCII plot

set -euo pipefail

EV1SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ELECTRICSIM_DIR="$(cd "$EV1SIM_DIR/../electricsim" 2>/dev/null && pwd || true)"
[[ -z "$ELECTRICSIM_DIR" ]] && {
    echo "[abs] cannot find ../electricsim" >&2
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

OUT="/tmp/ev1sim_abs"
rm -rf "$OUT"
mkdir -p "$OUT"

# Clean any stale SHM segments left behind by a previous run that didn't
# exit cleanly.  Without this, new readers can register but writers' frames
# end up on a "ghost" segment and never reach the new readers — debugged
# 2026-04-30 with the BTCM not seeing ev1sim's q8 publishes.
python3 - <<'PY' 2>/dev/null || true
import ctypes
libc = ctypes.CDLL("libc.dylib")
libc.shm_unlink.argtypes = [ctypes.c_char_p]
libc.shm_unlink.restype  = ctypes.c_int
for n in ["/electricsim_ev1_bus", "/electricsim_chassis_bus"]:
    libc.shm_unlink(n.encode())
PY

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
    echo "[abs] === $label ==="
    local pim_log="$OUT/$label.pim.log"
    local btcm_log="$OUT/$label.btcm.log"
    local sim_log="$OUT/$label.ev1sim.log"

    local rsa_log="$OUT/$label.rsa.log"

    # Launch ev1sim FIRST so the chassis + main bus segments exist before
    # the controllers attach.  Multiple SHM clients with create=true seem
    # to reliably stream frames only when the publisher is up first.
    "$EV1SIM_BIN" \
        --headless \
        --external-sim on \
        --start-propulsion-enabled \
        --scenario "$EV1SIM_DIR/config/scenarios/abs_hard_brake.json" \
        > "$sim_log" 2>&1 &
    local sim_pid=$!

    # Wait for ev1sim to log "connected to bus" before launching controllers.
    # SHM segments need to be fully initialized by ev1sim (the only process
    # using create=true) before electricsim controllers try to attach with
    # create=false.  Without this, controllers race and miss frames.
    local waited=0
    while ! grep -q "connected to main harness bus" "$sim_log" 2>/dev/null; do
        sleep 0.2
        waited=$((waited + 1))
        if [[ $waited -gt 30 ]]; then
            echo "[abs] ev1sim failed to bring up buses within 6s" >&2
            cat "$sim_log" >&2
            exit 1
        fi
    done
    sleep 0.3  # extra slack so SHM init definitely completes

    ( cd "$ELECTRICSIM_DIR/build" && exec "$PIM_BIN" ) > "$pim_log" 2>&1 &
    local pim_pid=$!
    sleep 0.3

    # RSA is needed alongside BTCM — its run-mode broadcast and the
    # downstream run_1 signal gate the BTCM ABS algorithm.  Without RSA,
    # BTCM treats the vehicle as "not enabled" and won't engage ABS.
    local rsa_pid=""
    local btcm_pid=""
    if [[ "$with_btcm" == "1" ]]; then
        ( cd "$ELECTRICSIM_DIR/build" && exec "$RSA_BIN" ) > "$rsa_log" 2>&1 &
        rsa_pid=$!
        sleep 0.3
        ( cd "$ELECTRICSIM_DIR/build" && exec "$BTCM_BIN" ) > "$btcm_log" 2>&1 &
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

    # ev1sim runs the scenario to completion; wait for it.
    wait "$sim_pid" 2>/dev/null || true

    cleanup
    trap - EXIT INT TERM

    mv "$EV1SIM_DIR/scenario_abs_hard_brake.csv" "$csv_dest"
    echo "[abs] $label CSV → $csv_dest"
}

run_scenario abs_btcm_on  1  "$OUT/abs_btcm_on.csv"
run_scenario abs_btcm_off 0  "$OUT/abs_btcm_off.csv"

echo
echo "[abs] === comparison ==="
"$EV1SIM_DIR/scripts/compare_abs_runs.py" \
    "$OUT/abs_btcm_on.csv" \
    "$OUT/abs_btcm_off.csv" \
    | tee "$OUT/summary.txt"
