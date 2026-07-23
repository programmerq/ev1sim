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

*(none as of 2026-07-22)*

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
