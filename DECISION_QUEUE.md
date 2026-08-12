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

### DQ-2026-08-11-what-a-repeatable-capture-run-is-allowed-to-be

**What's being decided.** What a VAT capture run has to satisfy to count as
repeatable: byte-identical output, or output equal within a stated tolerance —
and if a tolerance, calibrated on which case. The answer cannot be "byte
identical" today, and a tolerance read off the quiet cases would be off by four
orders of magnitude for the loud one.

**Why now.** This is one of two named items the VAT baseline recapture is
waiting on. Recapture needs a pass/fail rule, and the measurements below say
which rules are available.

**Source.** PR #55 (<https://github.com/programmerq/ev1sim/pull/55>), which
fixed the ev1sim-side cause: freshness windows that aged on the host wall
clock, so a host stall zeroed the rear brakes for a tick. What is measured
below is what remains after that, none of it ev1sim's.

**What two consecutive armed runs of the same case actually differ by.** All
runs below have the tick barrier armed, 17 of 17 consumers acking, and **zero**
BTCM-stale dropouts — so none of this is the cause PR #55 fixed.

| case | what it does | runs | spread between runs |
| --- | --- | --- | --- |
| `body_rhjb_door_lock` | parked, brakes held | 18 | 2 distinct traces; **7.6e-05 mph** peak speed difference, against a 0.117 mph peak |
| `abs_high_mu` | 67 mph hard stop, ABS cycling | 2 | **0.51 mph** speed, **1.2 deg** heading, 2.3 rad/s wheel speed — against a 1.9 deg total heading excursion |

The two are different mechanisms and only the first is understood in detail:

- **Parked case — the go-live tick.** ev1sim's BTCM-liveness proxy is "the
  BTCM's GM-8192 TX bit count advanced". The external sim's BTCM paces bit
  production on its own host clock even under an armed barrier, so the ev1sim
  tick that first sees a non-zero count is a race: sim 1.258 s in 16 runs,
  1.241 s in 2. The rear brakes come alive one tick earlier in those two and
  the trace differs from there. The correlation is exact — the speed checksum
  is a function of that tick, no exceptions in 18 runs.
- **Braking case — the controller's own output.** Both runs saw the first
  heartbeat on the same tick, so the above does not explain it. They diverge
  from t = 17.2 s, which is ABS onset, and the `abs_phase_*` and `emb_cmd_*`
  columns differ: the BTCM is issuing different commands, and the closed loop
  amplifies the difference for the remaining 13 s of the stop. That is
  upstream, and the external sim already tracks a sibling finding of the same
  shape (a run-to-run yaw swing on `abs_split_mu` at a fixed commit).

**Options.**

- **A — Tolerance per case, calibrated on the case.** Recapture now; each case
  states its own band. Honest about the two regimes, but the ABS band would
  have to be ~1.2 deg of heading, which is most of the manoeuvre — a band that
  wide asserts almost nothing about the ABS cases it covers.
- **B — Make the producer sim-paced first, then recapture.** The external sim
  feeds its UART serialisers and framers the host clock; feeding them sim time
  is the real fix and is tracked there. It is behaviour-changing and itself
  recapture-gated, so choosing this means deliberately breaking that cycle.
- **C — Recapture only the quiet cases now, hold the ABS cases for B.** Splits
  the difference: the parked/body/lighting cases get a tight tolerance (1e-3
  mph sits ~13x above their measured spread) and recapture immediately; the
  cases whose closed loop amplifies upstream timing wait for B. Costs a
  partial baseline and a list of which cases are in which state.

**Recommendation: C.** A is available but the ABS band it would need is too
wide to be worth asserting, and the cases where a tight band *is* meaningful
are exactly the ones that can move today.

**Default if silent:** C — the quiet cases recapture with a stated tolerance,
the ABS cases wait, and this entry stays open until B lands.

**Not recommended, recorded so it is not re-proposed:** closing the parked-case
gap inside ev1sim. It could be done — take first-liveness from a BTCM chassis
cell, which is written inside the producer's barrier tick and so is sim-paced,
and keep the heartbeat for steady-state liveness where those on-change cells
do not refresh; or debounce the go-live edge by a tick. Both work, and both
would put a consumer-side patch over a producer-side dependence on the host
clock, leaving the braking-case divergence untouched and harder to see.

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
