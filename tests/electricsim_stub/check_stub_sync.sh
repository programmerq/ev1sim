#!/usr/bin/env bash
#
# check_stub_sync.sh — the honesty check for the vendored external-sim stub.
#
# WHAT THIS EXISTS TO CATCH
#
# tests/electricsim_stub/ is a second copy of the external sim's wire-truth
# substrate sources, vendored so CI can compile ev1sim's guarded connector path
# without a real external-sim checkout. Second copies drift. This one did: at the
# time this script was written, 5 of the 15 vendored files had silently diverged
# from their originals, including the substrate itself (wire_table.{cpp,hpp}), and
# nothing in the tree could tell.
#
# The stub's own drift guards could not help. They are static_asserts comparing
# vendored constants against vendored constants, so they compare the copy to
# itself and pass by construction. A check that cannot fail is not a check.
#
# WHAT IT CHECKS — two independent legs, because they catch different things
#
#   Leg 1, INTEGRITY (always runs, needs no external-sim tree):
#     Every vendored file's sha256 matches the manifest, the manifest and the tree
#     list exactly the same files, and the manifest is non-empty and complete.
#     This catches a hand-edited stub — the thing the README forbids but nothing
#     enforced — and it runs in CI, where no external-sim checkout exists.
#
#   Leg 2, UPSTREAM SYNC (runs when an external-sim tree is supplied):
#     Every vendored file is byte-identical to its original. This catches real
#     drift: upstream moved and the copy did not.
#
# WHY BOTH. Leg 1 alone would happily certify a tree that is internally consistent
# and years behind. Leg 2 alone cannot run in CI. Neither is sufficient; together
# they mean drift has nowhere to hide.
#
# THREE-VALUED ON PURPOSE. The outcome is IN_SYNC, DRIFTED, or UPSTREAM_UNAVAILABLE.
# The third is a distinct state, not a quiet pass: with --require-upstream it is a
# FAILURE. That flag is what a caller uses when it believes it has an external-sim
# tree — so that "the tree wasn't found" can never masquerade as "the check passed".
#
# EXIT CODES
#   0  IN_SYNC
#   1  DRIFTED               (a real divergence — this is the finding)
#   2  UPSTREAM_UNAVAILABLE  (only when --require-upstream was given)
#   3  usage / internal error (bad manifest, missing tools)
#
# USAGE
#   check_stub_sync.sh                        # leg 1 only
#   check_stub_sync.sh --upstream <dir>       # legs 1+2; leg 2 skipped if absent
#   check_stub_sync.sh --upstream <dir> --require-upstream
#   check_stub_sync.sh --update               # re-record the manifest after a sync
#
# The selftest that proves this script can FAIL is selftest_check_stub_sync.sh.
# It is registered as a CTest so the proof runs with the suite, not by hand.

set -uo pipefail

STUB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="${STUB_DIR}/sync_manifest.txt"

UPSTREAM=""
REQUIRE_UPSTREAM=0
UPDATE=0

while [ $# -gt 0 ]; do
    case "$1" in
        --upstream)         UPSTREAM="${2:-}"; shift 2 ;;
        --require-upstream) REQUIRE_UPSTREAM=1; shift ;;
        --update)           UPDATE=1; shift ;;
        -h|--help)          sed -n '2,50p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "check_stub_sync: unknown argument '$1'" >&2; exit 3 ;;
    esac
done

if ! command -v sha256sum >/dev/null 2>&1; then
    echo "check_stub_sync: sha256sum not found" >&2
    exit 3
fi

# The vendored set, as paths relative to the stub root. This list is the contract:
# a file here that is not in the tree, or a file in the tree that is not here, is a
# failure either way (see the completeness check below).
vendored_files() {
    (cd "${STUB_DIR}" && find src -type f \
        \( -name '*.h' -o -name '*.hpp' -o -name '*.c' -o -name '*.cpp' \) \
        | LC_ALL=C sort)
}

# ── --update: re-record the manifest ────────────────────────────────────────
# Run this ONLY as part of a sync commit, never to make a red check green. The
# selftest exists precisely because "just re-run --update" is the tempting wrong
# move when this script fails.
if [ "${UPDATE}" = "1" ]; then
    if [ -z "${UPSTREAM}" ]; then
        echo "check_stub_sync --update: --upstream <dir> is required" >&2
        echo "  the manifest records WHAT the copy is in sync WITH; recording it" >&2
        echo "  without checking upstream would bless whatever is on disk." >&2
        exit 3
    fi
    if [ ! -d "${UPSTREAM}/src/io" ]; then
        echo "check_stub_sync --update: '${UPSTREAM}' is not an external-sim tree" >&2
        exit 3
    fi
    provenance="$(cd "${UPSTREAM}" && git rev-parse HEAD 2>/dev/null || echo "unknown")"
    {
        echo "# electricsim_stub vendored-copy manifest — DO NOT HAND-EDIT."
        echo "#"
        echo "# Regenerate with:"
        echo "#   tests/electricsim_stub/check_stub_sync.sh --update --upstream <external-sim tree>"
        echo "#"
        echo "# Verified by check_stub_sync.sh; that script's ability to FAIL is itself"
        echo "# verified by selftest_check_stub_sync.sh, registered as a CTest."
        echo "#"
        echo "# 'provenance' is the external-sim commit these copies were taken from. It is"
        echo "# an opaque identifier, recorded so a reader can tell WHICH upstream state"
        echo "# this copy reflects — without it the manifest says the copy is consistent"
        echo "# but not what it is consistent with."
        echo "provenance $(printf '%s' "${provenance}")"
        echo "count $(vendored_files | wc -l | tr -d ' ')"
        vendored_files | while IFS= read -r rel; do
            printf '%s  %s\n' "$(sha256sum "${STUB_DIR}/${rel}" | awk '{print $1}')" "${rel}"
        done
    } > "${MANIFEST}"
    echo "check_stub_sync: manifest updated ($(vendored_files | wc -l | tr -d ' ') files, provenance ${provenance})"
    exit 0
fi

# ── Manifest must parse, and must not be vacuous ────────────────────────────
# A truncated or empty manifest would otherwise "pass" every file it does not
# mention. The declared count is cross-checked against the entries actually read
# so a half-written manifest fails rather than silently narrowing the check.
if [ ! -f "${MANIFEST}" ]; then
    echo "FAIL: no manifest at ${MANIFEST}" >&2
    echo "  The vendored stub has no recorded identity, so nothing can be verified." >&2
    exit 1
fi

declared_count="$(awk '$1=="count"{print $2; exit}' "${MANIFEST}")"
entry_count="$(grep -cE '^[0-9a-f]{64}  ' "${MANIFEST}" || true)"
provenance="$(awk '$1=="provenance"{print $2; exit}' "${MANIFEST}")"

if [ -z "${declared_count}" ] || [ -z "${entry_count}" ] || [ "${entry_count}" = "0" ]; then
    echo "FAIL: manifest at ${MANIFEST} is empty or unparseable" >&2
    exit 1
fi
if [ "${declared_count}" != "${entry_count}" ]; then
    echo "FAIL: manifest is truncated — declares ${declared_count} files, contains ${entry_count}" >&2
    exit 1
fi

status=0
drift_report=""

note_drift() { drift_report="${drift_report}$1"$'\n'; status=1; }

# ── Leg 1: integrity + completeness ─────────────────────────────────────────
tree_list="$(vendored_files)"
manifest_list="$(grep -E '^[0-9a-f]{64}  ' "${MANIFEST}" | sed 's/^[0-9a-f]\{64\}  //' | LC_ALL=C sort)"

if [ "${tree_list}" != "${manifest_list}" ]; then
    while IFS= read -r f; do
        [ -n "$f" ] && note_drift "  UNLISTED   ${f} (in the stub tree, absent from the manifest)"
    done < <(comm -23 <(printf '%s\n' "${tree_list}") <(printf '%s\n' "${manifest_list}"))
    while IFS= read -r f; do
        [ -n "$f" ] && note_drift "  MISSING    ${f} (in the manifest, absent from the stub tree)"
    done < <(comm -13 <(printf '%s\n' "${tree_list}") <(printf '%s\n' "${manifest_list}"))
fi

checked=0
while read -r want rel; do
    [ -z "${rel:-}" ] && continue
    if [ ! -f "${STUB_DIR}/${rel}" ]; then
        continue  # already reported as MISSING above
    fi
    got="$(sha256sum "${STUB_DIR}/${rel}" | awk '{print $1}')"
    checked=$((checked + 1))
    if [ "${got}" != "${want}" ]; then
        note_drift "  EDITED     ${rel} (content differs from the manifest — the stub was hand-edited)"
    fi
done < <(grep -E '^[0-9a-f]{64}  ' "${MANIFEST}")

# The check must have actually looked at something. If the loop above ran zero
# times the result is meaningless, and reporting IN_SYNC would be the exact defect
# this file exists to prevent.
if [ "${checked}" = "0" ]; then
    echo "FAIL: verified 0 files — the check did not see its input" >&2
    exit 1
fi

# ── Leg 2: upstream sync ────────────────────────────────────────────────────
upstream_state="UPSTREAM_UNAVAILABLE"
if [ -n "${UPSTREAM}" ] && [ -d "${UPSTREAM}/src/io" ]; then
    upstream_state="CHECKED"
    while IFS= read -r rel; do
        [ -z "${rel}" ] && continue
        if [ ! -f "${UPSTREAM}/${rel}" ]; then
            note_drift "  GONE       ${rel} (vendored here, no longer present upstream)"
            continue
        fi
        if ! cmp -s "${STUB_DIR}/${rel}" "${UPSTREAM}/${rel}"; then
            note_drift "  DRIFTED    ${rel} (differs from upstream — the copy is stale)"
        fi
    done < <(printf '%s\n' "${manifest_list}")
fi

# ── Verdict ─────────────────────────────────────────────────────────────────
echo "electricsim stub sync check"
echo "  stub          : ${STUB_DIR}"
echo "  files verified: ${checked}"
echo "  provenance    : ${provenance:-unknown}"
echo "  upstream      : ${upstream_state}${UPSTREAM:+ (${UPSTREAM})}"

if [ "${status}" != "0" ]; then
    echo "  verdict       : DRIFTED"
    echo ""
    echo "The vendored stub does not match what it claims to be:"
    printf '%s' "${drift_report}"
    echo ""
    echo "This is a finding, not a nuisance. The stub is what CI compiles the"
    echo "connector against; a stale copy means CI is proving something about code"
    echo "that no longer exists. Re-copy the listed files from the external-sim tree"
    echo "and re-record the manifest in the SAME commit:"
    echo "  tests/electricsim_stub/check_stub_sync.sh --update --upstream <tree>"
    echo ""
    echo "Do not run --update on its own to clear this. That records the drift as"
    echo "intended rather than fixing it, and the next reader has no way to tell."
    exit 1
fi

if [ "${upstream_state}" = "UPSTREAM_UNAVAILABLE" ]; then
    if [ "${REQUIRE_UPSTREAM}" = "1" ]; then
        echo "  verdict       : UPSTREAM_UNAVAILABLE"
        echo ""
        echo "--require-upstream was given but no external-sim tree was found at"
        echo "'${UPSTREAM}'. The integrity leg passed, but that only proves the copy"
        echo "matches its own manifest — it says nothing about whether the copy is"
        echo "current. Refusing to report success." >&2
        exit 2
    fi
    echo "  verdict       : IN_SYNC (integrity only — upstream not checked)"
    exit 0
fi

echo "  verdict       : IN_SYNC"
exit 0
