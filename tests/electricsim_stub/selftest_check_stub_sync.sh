#!/usr/bin/env bash
#
# selftest_check_stub_sync.sh — proves check_stub_sync.sh can FAIL.
#
# A guard nobody has watched fail is a guard nobody knows works. The stub's
# previous drift guards were static_asserts comparing vendored constants to
# vendored constants: green forever, by construction, while 5 of 15 files rotted.
# The lesson is not "write a better guard", it is "make the guard demonstrate that
# its verdict responds to its input".
#
# So this runs check_stub_sync.sh against deliberately corrupted copies and
# requires the verdict to CHANGE. Each leg names the exact byte it broke.
#
# The legs, and what each would catch in the wild:
#   1. clean copy                     -> IN_SYNC   (0)   baseline: no false alarm
#   2. one byte flipped in a file     -> DRIFTED   (1)   a hand-edited stub
#   3. a file deleted                 -> DRIFTED   (1)   an incomplete copy
#   4. an unlisted file added         -> DRIFTED   (1)   a copy nobody recorded
#   5. manifest truncated             -> DRIFTED   (1)   a check narrowed to nothing
#   6. upstream byte differs          -> DRIFTED   (1)   the real drift case
#   7. upstream absent + --require    -> UNAVAIL   (2)   a skip that must not pass
#   8. --update on a drifted copy     -> REFUSED   (1)   a provenance never verified
#
# Legs 1-5 need no external-sim tree, so they run in CI. Legs 6-8 synthesise a
# fake upstream out of the stub itself, so they need no external-sim tree either.
#
# Exit 0 iff every leg produced the demanded verdict.

set -uo pipefail

STUB_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/stub_sync_selftest.XXXXXX")"
trap 'rm -rf "${WORK}"' EXIT

fresh_copy() {
    local dest="$1"
    rm -rf "${dest}"
    mkdir -p "${dest}"
    cp -r "${STUB_SRC}/src" "${dest}/src"
    cp "${STUB_SRC}/sync_manifest.txt" "${dest}/sync_manifest.txt"
    cp "${STUB_SRC}/check_stub_sync.sh" "${dest}/check_stub_sync.sh"
    chmod +x "${dest}/check_stub_sync.sh"
}

fails=0
leg() {
    local name="$1" want_rc="$2"; shift 2
    local out rc
    out="$("$@" 2>&1)"; rc=$?
    if [ "${rc}" = "${want_rc}" ]; then
        printf '  ok    %-46s rc=%s\n' "${name}" "${rc}"
    else
        printf '  FAIL  %-46s rc=%s (wanted %s)\n' "${name}" "${rc}" "${want_rc}"
        printf '%s\n' "${out}" | sed 's/^/          | /'
        fails=$((fails + 1))
    fi
}

echo "check_stub_sync.sh selftest — the verdict must respond to the input"

# ── Leg 1: a clean copy passes ──────────────────────────────────────────────
# Without this leg the others are satisfiable by a script that always fails.
fresh_copy "${WORK}/clean"
leg "1 clean copy -> IN_SYNC" 0 "${WORK}/clean/check_stub_sync.sh"

# ── Leg 2: one flipped byte is caught ───────────────────────────────────────
# The single most likely real corruption: someone "just tweaks" a vendored header
# instead of fixing it upstream. One byte, in a comment, is enough.
fresh_copy "${WORK}/onebyte"
target="${WORK}/onebyte/src/io/topology/topology_generated.h"
printf '\n// selftest: one deliberate byte of drift\n' >> "${target}"
leg "2 one-byte edit -> DRIFTED" 1 "${WORK}/onebyte/check_stub_sync.sh"

# ── Leg 3: a deleted file is caught ─────────────────────────────────────────
fresh_copy "${WORK}/deleted"
rm -f "${WORK}/deleted/src/io/wire_table.hpp"
leg "3 deleted file -> DRIFTED" 1 "${WORK}/deleted/check_stub_sync.sh"

# ── Leg 4: an unlisted extra file is caught ─────────────────────────────────
# A vendored copy that grew a file nobody recorded is drift in the other
# direction: the manifest no longer describes the tree.
fresh_copy "${WORK}/extra"
printf '// not in the manifest\n' > "${WORK}/extra/src/io/unlisted_addition.hpp"
leg "4 unlisted extra file -> DRIFTED" 1 "${WORK}/extra/check_stub_sync.sh"

# ── Leg 5: a truncated manifest fails instead of narrowing ──────────────────
# The subtle one. A manifest cut down to two entries would let a naive checker
# verify two files and report success. The declared count is what stops that.
#
# The cut point matters and was previously wrong. The header is 12 comment lines,
# then `provenance` (13) and `count` (14), then the entries. Cutting at 12 leaves a
# manifest with no entries and no count, which exits through the "empty or
# unparseable" branch — a real failure, but a DIFFERENT one, so the leg passed
# without ever reaching the count-vs-entries cross-check it names. Cut at 16:
# provenance + `count 17` + exactly 2 entries, so the declared count and the listed
# entries disagree, which is the branch under test.
fresh_copy "${WORK}/truncated"
head -n 16 "${STUB_SRC}/sync_manifest.txt" > "${WORK}/truncated/sync_manifest.txt"
leg "5 truncated manifest -> DRIFTED" 1 "${WORK}/truncated/check_stub_sync.sh"

# ── Leg 6: upstream drift is caught ─────────────────────────────────────────
# Synthesise an upstream that differs from the copy by one byte. This is the case
# the whole file exists for: the copy is internally consistent (leg 1 would pass)
# and still wrong, because upstream moved.
fresh_copy "${WORK}/upstream_drift"
mkdir -p "${WORK}/fake_upstream"
cp -r "${STUB_SRC}/src" "${WORK}/fake_upstream/src"
printf '\n// selftest: upstream moved and the copy did not\n' \
    >> "${WORK}/fake_upstream/src/io/wire_table.hpp"
leg "6 upstream byte differs -> DRIFTED" 1 \
    "${WORK}/upstream_drift/check_stub_sync.sh" --upstream "${WORK}/fake_upstream"

# ── Leg 7: a missing upstream must not read as success ──────────────────────
# The check-that-cannot-fail trap in its purest form: if "no external-sim tree"
# quietly returned 0, this guard would report success on every CI run forever.
fresh_copy "${WORK}/no_upstream"
leg "7 upstream absent + --require -> UNAVAILABLE" 2 \
    "${WORK}/no_upstream/check_stub_sync.sh" \
    --upstream "${WORK}/definitely_not_here" --require-upstream

# Same situation WITHOUT --require-upstream is a legitimate pass: CI has no
# external-sim tree and the integrity leg is still meaningful there.
leg "7b upstream absent, not required -> IN_SYNC" 0 \
    "${WORK}/no_upstream/check_stub_sync.sh" \
    --upstream "${WORK}/definitely_not_here"

# ── Leg 8: --update refuses to record a copy upstream does not have ─────────
# --update writes the provenance line every later reader trusts. It used to hash
# whatever was on disk and stamp it with the upstream's HEAD sha without comparing
# the two, so a hand-edited stub could be minted as "provenance <sha>" for content
# that never existed at that sha — and the integrity leg would then certify that
# forever, because the manifest and the tree agreed with each other. The refusal
# is the fix; this leg is the proof the refusal fires.
fresh_copy "${WORK}/update_drift"
mkdir -p "${WORK}/fake_upstream_upd"
cp -r "${STUB_SRC}/src" "${WORK}/fake_upstream_upd/src"
printf '\n// selftest: upstream does not have this edit\n' \
    >> "${WORK}/update_drift/src/io/wire_table.hpp"
leg "8 --update on a drifted copy -> REFUSED" 1 \
    "${WORK}/update_drift/check_stub_sync.sh" \
    --update --upstream "${WORK}/fake_upstream_upd"

# The same call on a copy that DOES match upstream must still succeed, or the
# refusal above would just be "--update is broken" rather than "--update checks".
fresh_copy "${WORK}/update_clean"
leg "8b --update on a matching copy -> RECORDED" 0 \
    "${WORK}/update_clean/check_stub_sync.sh" \
    --update --upstream "${WORK}/fake_upstream_upd"

echo ""
if [ "${fails}" != "0" ]; then
    echo "selftest FAILED: ${fails} leg(s) did not produce the demanded verdict."
    echo "check_stub_sync.sh is not proven to work — treat its green as meaningless."
    exit 1
fi
echo "selftest passed: all 10 legs produced the demanded verdict."
echo "The guard fails on drift and passes in sync — its verdict tracks its input."
exit 0
