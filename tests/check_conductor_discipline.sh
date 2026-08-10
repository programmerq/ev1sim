#!/usr/bin/env bash
#
# check_conductor_discipline.sh — ev1sim never writes a conductor cell.
#
# WHAT A CONDUCTOR CELL IS, AND WHY THIS MATTERS
#
# The external sim's topology splits its cells by class. A CONDUCTOR is a physical
# conductor whose energisation is DERIVED — the solver works out whether the wire is
# hot from the circuit, and publishes the answer. No peer asserts it. The substrate
# enforces that with the type system: conductor cells are handed out as a distinct
# type with no write overload, so "my feed is hot" has no spelling that compiles.
#
# ev1sim sits downstream of that solver. It CONSUMES conductor cells — lamp feeds,
# telltales, horn drive — to decide what to render. It produces 85 cells of its own,
# and none of them is a conductor: they are physical inputs ev1sim owns (switches,
# pedals, vehicle dynamics) and are classed element-state or semantic.
#
# WHY A SCRIPT AND NOT JUST THE TYPE
#
# The type protects code that names a cell as a CONSTANT. ev1sim's accessors take a
# runtime id, because ids arrive from the consumer registry rather than as literals.
# So one conversion from the conductor type to a plain id has to exist. It does, it
# is named ReadOnlyWireId, and it is in src/WireTruthChassis.cpp.
#
# One is fine. Two is how the discipline dies: the second one is always added for a
# good local reason, and the type-split quietly stops meaning anything. So this is a
# ratchet on the count, and it fails on != rather than > — a <= check would let a
# future change bank headroom by deleting a legitimate use and adding an illegitimate
# one, netting zero.
#
# Exit 0 iff the counts are exactly as declared.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="${ROOT}/src"

# The declared state of the world. Change these ONLY with the reason written down.
#
# WANT_TYPED_SITES=1 is the ReadOnlyWireId signature in src/WireTruthChassis.cpp:
#   static constexpr WireId ReadOnlyWireId(electricsim::io::ConductorId id)
# That is the single place in ev1sim's production code where the conductor TYPE is
# named. Every conductor cell ev1sim reads goes through it. A second site means a
# new conductor-typed code path exists that nobody has reviewed.
readonly WANT_TYPED_SITES=1
readonly WANT_PUBLISHES=0     # ev1sim never publishes a conductor in production

fails=0

if [ ! -d "${SRC}" ]; then
    echo "FAIL: no src/ at ${SRC} — the check cannot see its input" >&2
    exit 1
fi

# Drop comment lines so prose about conductors is not mistaken for code that
# handles them. An earlier version of this check counted its own explanatory
# comment and reported the right number for the wrong reason — which is the same
# class of defect it exists to prevent, so the filter is deliberate and tested
# (leg "comment-only mention" in the selftest below).
strip_comments() { grep -vE '^[^:]+:[0-9]+:[[:space:]]*(//|\*|/\*)'; }

# Leg 1: where is the conductor TYPE named in production code?
typed_sites="$(grep -rnE '\bConductorId\b' \
    "${SRC}" --include='*.cpp' --include='*.h' --include='*.hpp' 2>/dev/null \
    | strip_comments || true)"
n_typed_sites="$(printf '%s' "${typed_sites}" | grep -c . || true)"

# Leg 2: does any production code reach the conductor publish edge?
#
# The alternation must name EVERY publish verb, not just the type. A call through
# an already-built handle (`pub_.publish_mv(id, mv)`) never spells
# ConductorPublisher, so a verb missing from this list is a publish this leg
# reports as zero. That is not hypothetical: the battery pass added publish_mv() /
# publish_conductor_mv() beside the original bit form, and until they were added
# here a millivolt publish through a handle scanned clean. When upstream grows
# another publish verb, it belongs in this alternation in the same commit that
# vendors it.
publishes="$(grep -rnE 'ConductorPublisher|publish_conductor|publish_mv' \
    "${SRC}" --include='*.cpp' --include='*.h' --include='*.hpp' 2>/dev/null \
    | strip_comments || true)"
n_publishes="$(printf '%s' "${publishes}" | grep -c . || true)"

# The check must have actually scanned something. A src/ tree that greps to zero
# because the path is wrong would otherwise pass both legs.
scanned="$(find "${SRC}" -name '*.cpp' -o -name '*.hpp' -o -name '*.h' | wc -l | tr -d ' ')"
if [ "${scanned}" = "0" ]; then
    echo "FAIL: scanned 0 source files — the check did not see its input" >&2
    exit 1
fi

echo "ev1sim conductor discipline"
echo "  source files scanned : ${scanned}"
echo "  conductor-typed sites: ${n_typed_sites} (want exactly ${WANT_TYPED_SITES})"
echo "  conductor publishes  : ${n_publishes} (want exactly ${WANT_PUBLISHES})"

if [ "${n_typed_sites}" != "${WANT_TYPED_SITES}" ]; then
    echo ""
    echo "FAIL: expected exactly ${WANT_TYPED_SITES} conductor-typed site in src/, found ${n_typed_sites}:"
    printf '%s\n' "${typed_sites}" | sed 's/^/    /'
    echo ""
    echo "  ev1sim names the conductor type in exactly one place: ReadOnlyWireId in"
    echo "  src/WireTruthChassis.cpp, which converts a conductor id to a plain id for"
    echo "  the READ path. A second site is how the type-split stops meaning anything:"
    echo "  it is always added for a good local reason, and after it the compiler no"
    echo "  longer tells anyone that ev1sim is touching a cell it does not own."
    echo "  If a new read path genuinely needs an id, route it through ReadOnlyWireId."
    fails=$((fails + 1))
fi

if [ "${n_publishes}" != "${WANT_PUBLISHES}" ]; then
    echo ""
    echo "FAIL: expected ${WANT_PUBLISHES} conductor publishes in src/, found ${n_publishes}:"
    printf '%s\n' "${publishes}" | sed 's/^/    /'
    echo ""
    echo "  ev1sim consumes conductor cells; it does not produce them. A conductor's"
    echo "  energisation is solved from the circuit. If ev1sim needs to drive this"
    echo "  signal, it is not a conductor — fix the cell's class in the topology"
    echo "  rather than reaching for the publish edge here."
    echo "  (ev1sim's TESTS legitimately publish conductors: they stand in for the"
    echo "   external sim's solver. This check covers src/ only, for that reason.)"
    fails=$((fails + 1))
fi

if [ "${fails}" != "0" ]; then
    exit 1
fi

echo "  verdict              : OK"
exit 0
