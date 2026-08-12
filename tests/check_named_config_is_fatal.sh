#!/usr/bin/env bash
# A --config path the user NAMED must be fatal when it cannot be opened.
#
# Config::LoadFromFile warns and returns built-in defaults for any path it
# cannot read.  That is right for the implicit config/default.json and wrong
# for a named one: it silently substitutes a different experiment, and a run
# that still succeeds is exactly the kind of substitution nobody notices.
#
# The case that motivated it (2026-08-12): the coastdown was documented with
# no --config at all, so it loaded milford where config/default.json resolved
# — car off the level, solver to 370 m/s — and a rigid_plane everywhere else,
# producing a perfectly ordinary coastdown.  Same command line, either
# outcome, decided by the working directory.
#
# NO SEPARATE --selftest HERE, deliberately.  The two assertions are each
# other's control: a binary that always exited 2 would fail the second, and
# one that never did would fail the first.  Neither can pass vacuously, so
# there is nothing a mock could prove that the pair does not.
#
# THE RULE ITSELF IS NOT PINNED HERE.  Config::NamedConfigFault is unit-tested
# in tests/test_config.cpp, which every build lane compiles.  This script only
# adds that the app exits 2 and prints the reason, which needs the built
# binary — so it is opt-in (-DEV1SIM_APP_TESTS=ON, or `make test-config-guard`).
# It used to be registered whenever Chrono was found, and went red in CI's
# chrono-smoke job: that job's second build tree builds only ev1sim_tests, so
# the app binary was absent and the run exited 127.  Hence both the opt-in and
# the explicit check below — "not built" must never look like "wrong exit code".
set -euo pipefail

bin="${1:?usage: check_named_config_is_fatal.sh <path to ev1sim>}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -x "$bin" ]]; then
    echo "FAIL: no ev1sim binary at $bin" >&2
    echo "      This check execs the app; build it first (cmake --build <dir>" >&2
    echo "      --target ev1sim).  Refusing to report anything about a binary" >&2
    echo "      that is not there." >&2
    exit 1
fi

# 1. Named-but-missing must be fatal, and must not be a generic crash.
set +e
out="$("$bin" --config "$root/config/definitely_not_a_config.json" --headless 2>&1)"
rc=$?
set -e
if [[ $rc -ne 2 ]]; then
    echo "FAIL: --config with a missing path exited $rc, expected 2" >&2
    echo "$out" >&2
    exit 1
fi
if ! grep -q "Refusing to fall back to built-in defaults" <<<"$out"; then
    echo "FAIL: exited 2 but without saying why:" >&2
    echo "$out" >&2
    exit 1
fi
echo "ok: a named --config that cannot be opened exits 2 and says why"

# 2. ...and a config that DOES open is still loaded, or the check above is
#    just "the binary always fails".  headless_smoke self-terminates in ~3 s.
set +e
out="$("$bin" --config "$root/config/headless_smoke.json" 2>&1)"
rc=$?
set -e
if [[ $rc -eq 2 ]]; then
    echo "FAIL: a config that exists was also rejected — the guard is firing" >&2
    echo "      on everything, which would make the assertion above vacuous" >&2
    echo "$out" >&2
    exit 1
fi
echo "ok: an existing named --config still loads (rc=$rc)"
