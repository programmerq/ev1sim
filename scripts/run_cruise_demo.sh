#!/usr/bin/env bash
# Orchestrate the cruise demo: spawn PIM controller in the background,
# launch ev1sim in the foreground with the electronics-driven scenario,
# then clean up PIM when ev1sim exits.
#
# Both repos must be built:
#   ev1sim:      ./build/ev1sim
#   electricsim: ./build/ev1/pim/pim_controller (and ./build/firmware/pim_firmware.elf)
#
# Logs land in /tmp/ev1sim_demo/ (wiped on each run).
#
# Usage:
#   ./scripts/run_cruise_demo.sh [--headless]
#
# Options:
#   --headless    Run ev1sim without the visual window (CI-friendly).
#                 Default is interactive — a window pops up so you can watch.
#
# Exit code: forwarded from ev1sim (0=success, 3=scenario assertion failed,
# 2=max_time hit early, 130=Ctrl-C).
set -euo pipefail

HEADLESS=""
if [[ "${1:-}" == "--headless" ]]; then
    HEADLESS="--headless"
fi

EV1SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ELECTRICSIM_DIR="$(cd "$EV1SIM_DIR/../electricsim" 2>/dev/null && pwd || true)"

if [[ -z "$ELECTRICSIM_DIR" ]]; then
    echo "[demo] cannot find ../electricsim relative to ev1sim — set ELECTRICSIM_DIR env var" >&2
    exit 1
fi

EV1SIM_BIN="$EV1SIM_DIR/build/ev1sim"
PIM_BIN="$ELECTRICSIM_DIR/build/ev1/pim/pim_controller"

if [[ ! -x "$EV1SIM_BIN" ]]; then
    echo "[demo] missing $EV1SIM_BIN — did you build ev1sim?" >&2
    exit 1
fi
if [[ ! -x "$PIM_BIN" ]]; then
    echo "[demo] missing $PIM_BIN — did you build electricsim?" >&2
    exit 1
fi

LOG_DIR="/tmp/ev1sim_demo"
rm -rf "$LOG_DIR"
mkdir -p "$LOG_DIR"
PIM_LOG="$LOG_DIR/pim.log"
EV1SIM_LOG="$LOG_DIR/ev1sim.log"

cleanup() {
    if [[ -n "${PIM_PID:-}" ]] && kill -0 "$PIM_PID" 2>/dev/null; then
        kill "$PIM_PID" 2>/dev/null || true
        wait "$PIM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[demo] launching PIM controller (logs: $PIM_LOG)"
( cd "$ELECTRICSIM_DIR/build" && exec "$PIM_BIN" ) > "$PIM_LOG" 2>&1 &
PIM_PID=$!

# Give PIM a beat to load firmware + open the bus.
sleep 0.3

echo "[demo] launching ev1sim (logs: $EV1SIM_LOG)"
echo "[demo] watching: visual window pops up; demo ~60 sim seconds"

set +e
"$EV1SIM_BIN" \
    --external-sim on \
    --start-propulsion-enabled \
    $HEADLESS \
    --scenario "$EV1SIM_DIR/config/scenarios/cruise_demo_electronics.json" \
    > "$EV1SIM_LOG" 2>&1
EXIT_CODE=$?
set -e

echo "[demo] ev1sim exit=$EXIT_CODE"
echo "[demo] PIM cruise transitions:"
grep -E "cruise_engaged|cruise_setpoint_changed|cruise_disengaged" "$PIM_LOG" || \
    echo "  (no cruise events — PIM may not have engaged)"
echo "[demo] scenario assertions:"
grep -E "ASSERT (PASS|FAIL)|asserts:" "$EV1SIM_LOG" || true

exit "$EXIT_CODE"
