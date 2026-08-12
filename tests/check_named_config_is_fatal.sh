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
set -euo pipefail

bin="${1:?usage: check_named_config_is_fatal.sh <path to ev1sim>}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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
