# Coordinator decision queue — ev1sim

The queue the AGENTS.md "owner decision asks are self-contained" rule checks
before any owner decision ask goes out (owner directive 2026-07-22: each repo
in the program carries its own queue). One entry per open fork raised by this
repo's work.

**Rules:** every entry is self-contained — answerable from this file on one
screen: what's being decided in plain language and why now, a link to the
source that raised it, options with one-line implications, a recommendation,
and a default-if-silent. When the owner answers, record the ruling here and in
the thread where it was asked, immediately, and move the entry to *Recently
resolved*. Never re-ask a fork cold — cite the prior ask and what changed.
Entry IDs are date-slugs `DQ-YYYY-MM-DD-<kebab-slug>` (date raised), never
incrementing numbers (owner directive 2026-07-22 — parallel sessions minting
numbers collide).

This repo is public: queue entries follow the same sharing hygiene as the rest
of the tree (no private-infrastructure identifiers).

---

## Open owner decisions

### DQ-2026-08-11-residual-cosim-speed-spread-from-a-wall-paced-producer

**What's being decided.** After the sim-clock fix on this branch, armed
17-module co-sim runs of `body_rhjb_door_lock` still land on one of exactly
**two** vehicle-speed traces, differing by at most **7.6e-05 mph** on a car
that never leaves 0.117 mph. ev1sim cannot close that on its own. The program
has to say what a VAT capture run is allowed to be: byte-identical, or equal
within a stated tolerance.

**Why now.** This is one of two named items the VAT baseline recapture is
waiting on, so "capture runs must be byte-identical" is either a requirement
the recapture can meet or one it cannot.

**Source.** PR #55 (run-determinism investigation), which fixed the *other*
cause — freshness windows that aged on the host wall clock. 18 armed runs on
an idle host: 16 produced one trace, 2 the other, and which one is decided
entirely by a single tick.

**The mechanism, stated exactly.** ev1sim's BTCM-liveness proxy is "the
BTCM's GM-8192 TX bit count advanced" (`ExternalSimConnector.cpp`, the
`btcm_tx_total_bits()` block). The external sim's BTCM paces bit production on
its own **host wall clock**, not on sim time, even under an armed barrier —
tracked there as external sim BL-0082, and gated on this same recapture. So
how many bits exist after a given barrier tick is a wall-clock race, and the
ev1sim tick that first sees a non-zero count landed on sim 1.258 s in 16 runs
and 1.241 s in 2 — one tick apart. The rear brakes come alive one tick earlier
in those two runs, and the speed trace differs from there. The correlation is
exact: every run at 1.258 has one speed checksum, every run at 1.241 the other,
no exceptions.

**Options.**

- **A — Compare with a tolerance, capture now.** State a speed tolerance in
  the capture criteria (1e-3 mph would sit ~13x above the observed spread and
  still far below anything the criteria decide on). Recapture is unblocked
  today; the residual spread stays visible rather than asserted away.
- **B — Wait for the producer to be sim-paced.** Land external sim BL-0082
  (feed sim time to the UART framers/serialisers) and recapture after. Gets
  byte-identity, at the cost of blocking the recapture on a behaviour-changing
  change in the other repo — which is itself recapture-gated, so this is a
  cycle unless the owner breaks it deliberately.
- **C — Make ev1sim's go-live depend on a sim-paced BTCM signal instead.**
  Technically possible (the BTCM's chassis cells are written inside its barrier
  tick), but it hides a producer-side wall-clock dependency behind a consumer-
  side workaround, and the heartbeat proxy exists precisely because the
  on-change chassis cells do not refresh in steady state. Not recommended.

**Recommendation: A**, and file B as the real fix rather than a blocker. The
symptom is 7.6e-05 mph on a parked car; the thing byte-identity would be
buying here is not fidelity, it is a stronger claim than the co-sim contract
currently supports.

**Default if silent:** A — capture proceeds with a stated tolerance, and this
entry stays open until B lands.

Standing work items live in `docs/TODO.md`; they only enter this queue when
they fork into a question the owner must rule on.

---

## Standing defaults (applied, owner may override)

*(none as of 2026-07-22)*

---

## Recently resolved (anti-re-ask ledger)

- **2026-07-22 — sibling-project naming in load-bearing identifiers: accepted
  as a known tradeoff.** The long-standing references to a sibling project in
  this repo's namespaces and stub directories stay as-is (a rename would break
  the build for no content-security gain). This repo is treated as public
  under the program's sharing hygiene **regardless of its hosting visibility**,
  now and in the future. Owner ruling 2026-07-22; do not re-raise a rename.
