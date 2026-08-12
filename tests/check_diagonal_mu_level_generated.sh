#!/usr/bin/env bash
# level/flat_diagonal_mu.json is a SECOND COPY: 202 patches emitted by
# scripts/gen_diagonal_mu_level.py.  Second copies drift — somebody nudges the
# JSON because that is the file the sim reads, the generator keeps saying
# something else, and the next regeneration silently reverts the nudge.
#
# This check regenerates the level into a temp file and requires byte equality
# with the committed one.  It is cheap enough to run with the ordinary suite,
# so the copy cannot rot between the occasions somebody remembers to look.
#
# Self-test: --selftest proves the check can FAIL.  Without it a green run
# means nothing — the trap of a guard that compares a thing to itself.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"
gen="$root/scripts/gen_diagonal_mu_level.py"
committed="$root/level/flat_diagonal_mu.json"

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

python3 "$gen" "$tmp/regenerated.json" >/dev/null

if [[ "${1:-}" == "--selftest" ]]; then
    # Perturb the regenerated copy the way a hand-edit would, and require the
    # comparison below to notice.  Runs the real comparison, not a mock of it.
    sed -i 's/"spawn"/"spawn_MUTATED"/' "$tmp/regenerated.json"
    if diff -q "$committed" "$tmp/regenerated.json" >/dev/null 2>&1; then
        echo "SELFTEST FAILED: a mutated level compared equal to the committed one" >&2
        exit 1
    fi
    echo "selftest ok: the comparison rejects a mutated level"
    exit 0
fi

if ! diff -u "$committed" "$tmp/regenerated.json"; then
    cat >&2 <<'MSG'

level/flat_diagonal_mu.json does not match what
scripts/gen_diagonal_mu_level.py produces.

That file is generated.  Change the constants in the generator and re-run it
  python3 scripts/gen_diagonal_mu_level.py
rather than editing the JSON, or the next regeneration reverts the edit.
MSG
    exit 1
fi

echo "level/flat_diagonal_mu.json matches its generator"
