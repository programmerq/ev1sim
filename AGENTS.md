# AGENTS.md — the rules for agents working in ev1sim

What the project is and how to build and run it: [`README.md`](README.md).
Its process boundaries: [`ARCHITECTURE.md`](ARCHITECTURE.md). This file holds
rules only.

## Run and verify

Build and test as [`README.md`](README.md) says; CI's legs are in
[`.github/workflows/ci.yml`](.github/workflows/ci.yml). Run
`tests/electricsim_stub/check_stub_sync.sh` after touching the vendored
electricsim stub.

## Commits

Focused, complete and logical, saying why. Every code change adds a test in the same
commit, or the message says why none applies.

## Shared rules

The blocks below are shared by all four EV1 repos; edit them only in
`ev1/CONVENTIONS.md`. ev1sim has no `scripts/backlog.py`: its notes channel
and owner decision queue is [`DECISION_QUEUE.md`](DECISION_QUEUE.md). Never
merge to `main`.

<!-- BEGIN ev1-canon:pr-lifecycle v9 -->
**Branches, pushes and PRs (canonical across all four EV1 repos; this block is
the only copy of these rules).** A new ruling replaces the wording here
instead of being added beside it. Edit it in `ev1/CONVENTIONS.md`, bump the
version, and paste it byte-identical into the other carriers;
`ev1/tools/canon_sync` checks the copies.

*What CI costs:* pushing a branch that has no PR runs no CI in any of the four
repos. Opening a PR, and every push to an open PR, runs the full PR checks. The
Claude PR review re-runs on every push in ev1-manual-redux and ev1sim; in
electricsim it runs only when the PR opens or leaves draft or a review is
requested, so after pushing fixes there, re-request the review. So pushes are
free and PRs are not.

*Commit and push:*
- **Commit often and push every commit** to your `claude/<topic>` branch as
  you go. Work that is uncommitted, or committed only to a local branch, is
  stranded: a cloud session's container can disappear at any time. Never end
  a turn with uncommitted changes or unpushed commits.
- **A pushed branch with no PR is the normal home for work in progress.**
  Long-running branches are fine; keep them current with `origin/main`
  (rebase and push with `--force-with-lease`, never plain `--force`, or merge
  `main` in).

*When to open a PR:*
- **No tiny PRs.** Every PR costs a full CI run and an owner review, so open
  one only when the branch holds a chunk worth that: a coherent, finished
  piece of work he can act on, never a single page, small fix or doc tweak.
  **Few large PRs beat many small ones**; never split coherent work to keep
  diffs small. When the owner asks for a PR, open one.
- **A PR carries its knock-ons.** Before opening, sweep the change into every
  layer it touches: each other file, doc, comment, test, fixture and data
  record that states the same fact or follows from it. A fix for one symptom
  of a wider error fixes the whole class.
- **Small follow-ups ride an in-flight branch** that already touches the same
  files, even an imperfect topical fit, never a PR of their own; so does a
  doc, note or queue edit whose only purpose is carrying a record. With no
  such branch, the follow-up waits on a pushed branch until it grows into a
  chunk.
- **Stack only to avoid a merge conflict.** New work bases on `main`. A
  change lives in exactly one PR; stack a new PR only when its diff must
  change the same area, file or item as an open PR, and then base it on the
  top of that PR's stack, never on a mid-stack PR (GitHub stacks are
  linear). An open PR in your lane is not a reason to stack, and a conflict
  is never a reason to split work into tiny PRs. A stacked PR's first body
  line names its base PR and why.
- **Stacking is not a crutch to work around testing.** A PR that discovers
  failures (new scenarios exposing bugs, say) fixes them on its own branch
  and is not ready while red. A fix to an open PR is pushed to that PR,
  never opened as a separate stacked PR.

*Draft and ready:* GitHub's convention. Draft means you or another agent is
still working on the PR right now; ready means the work is done and it is
ready for review. The owner reviews only ready PRs, so a draft that nobody is
working on is abandoned work.
- **Open a finished PR ready, and keep it mergeable.** The moment main moves,
  merge it into every open PR you own, fix any conflict or red CI in place,
  and update any line of the PR body that main has made stale.
  A PR is draft only while you are actively making that fix, and goes back to
  ready as soon as it is green and merges clean. Never close, replace or
  retarget a PR to clear a conflict.
- **Never park a PR in draft**: not for a cross-repo dependency (make the PR
  work on its own, or say in the body what it needs and keep it ready), not
  for a question with a stated default (apply the default, note it), not for
  "awaiting owner sign-off" (sign-off is the review), and not because a peer
  or coordinator said so. A PR you cannot move forward is a finding to report,
  not a draft to leave.
- **The owner flipping a PR to draft is his request-changes** (GitHub won't
  let him request changes on his own PR). Fix it, push, and flip it back to
  ready yourself with a "rework landed" comment.
- Flip with `draft:false` on `mcp__github__update_pull_request` (the
  mark-ready path can be permission-blocked).

*What a PR contains:*
- **Complete.** Every item a PR mints is implemented in it, or names a
  justification class (owner-gated decision, cross-repo, genuinely
  unimplementable) in both the item and the PR body. "It belongs to another
  open PR" is not a justification: consolidate into one PR. **Identify and
  fix, don't log.**
- Non-implementable findings (negative result, owner-blocked fork, errata)
  go to the repo's backlog or notes channel, named right after this block,
  and ride the next branch with code.
- **Titles state the goal, not the process**: no "needs sign-off", no
  "proposal:" on finished work.
- **Merges are the owner's click, and each merge is an invitation to the
  next work item**: when your PR merges, start the next item instead of
  stopping.
<!-- END ev1-canon:pr-lifecycle v9 -->

<!-- BEGIN ev1-canon:pr-images v3 -->
**PR-body images (canonical across all four EV1 repos).**
Use `<img>` tags (never bare `![…]()` markdown — no size control), `src`
pinned to a **commit SHA** (never a branch name — branch URLs re-render as the
branch moves and can 404 after a rebase/merge), in the `blob` form — **never**
`raw.githubusercontent.com`:

`<img src="https://github.com/<owner>/<repo>/blob/<COMMIT_SHA>/<path>?raw=1" width="..." alt="..." />`

- **`blob/...?raw=1` in every repo**, because `raw.githubusercontent.com` does
  not authenticate against the viewer's session and fails to render in the
  private repos; one form everywhere means no per-repo exception.
- **Always set an explicit `width`** — a percentage (`width="40%"`) for
  small images, a pixel width no larger than natural size otherwise. GitHub
  stretches unsized images to the full body column and upscales small ones
  blurrily.
<!-- END ev1-canon:pr-images v3 -->

### PR text

- Use no jargon or label that is not clear on its own. Say what a check
  catches ("the check that the vendored stub matches its record") instead of
  calling it a "gate".
- Lead the body with the strongest finding, stated positively.

<!-- BEGIN ev1-canon:say-it-once v1 -->
**Say it once, say what is true now (canonical across all four EV1 repos).**

- **Code, comments, docstrings, data fields and docs describe the current
  state.** They never say what was removed, what something used to be, when
  it changed, or who asked. Git and the PR body hold that.
- **One home per reason.** A rule or decision is written once: a canon block,
  a decision record, a notes entry or a doc. Every site that follows it names
  that home in a few words (`ev1-canon:ci-checks`, a decision file path) and
  does not restate, date or attribute it.
- **A change across many sites explains itself once, in the PR body.** If two
  or more sites would carry the same sentence, the sentence belongs in the
  home and the sites get the pointer, or nothing.
- **A test or check says what it checks**, in a line or two. What it no longer
  checks, and why, belongs in the commit message.
- **A provenance tag points; it does not argue.** `@design <ref>` names the
  record it follows; reasoning longer than one line goes in that record.
- **Short structured fields over prose.** A field holds a name, value or
  pointer. When a schema cannot hold a fact, extend the schema.
- **Agent docs follow the same rule.** They state current rules, with no
  dated anecdotes, quotes or "added on" notes; the ruling lives in its
  decision record.
<!-- END ev1-canon:say-it-once v1 -->

<!-- BEGIN ev1-canon:ci-checks v2 -->
**What a blocking CI check may test (canonical across all four EV1 repos).**
This program swaps its guesses out as board traces, ROM analysis and better
manual readings arrive, so the PR gate must never freeze a guess in place.

- **A blocking check tests syntax or structure only:** the file parses, a
  schema holds, references resolve, a generated file matches its generator, a
  copy matches its source, or a code-shape rule holds.
- **Nothing blocks on a value.** A count, a designator, a part number, a page
  number, a measured or printed number, or any reading of a source goes on
  demand, in a non-blocking report, or in the nightly change detector, where a
  person re-locks it after a correct change.
- **A unit test tests code, on fixtures it builds.** A test that loads live
  data may check only that the data loads and its references resolve.
- **A check that blocks a correct change is a bug in the check.** Fix or demote
  the check on the same branch. Never bend the data, rewrite a source, or add a
  baseline or allowlist entry just to get past it.
- **Don't add a check to prove a fix.** A test in the existing suite is the
  proof. A new blocking check needs the owner's approval and a reason it must
  run on nearly every change; a new check starts on demand.
- **No check requires prose.** A check may require a field to exist and
  resolve, never a rationale, essay or docstring of some length.
<!-- END ev1-canon:ci-checks v2 -->

<!-- BEGIN ev1-canon:decision-ask v3 -->
**Owner decision asks are self-contained (canonical across all four EV1
repos).**

*First, is it a decision at all?* When a manual, patent, datasheet or other
primary source answers the question clearly, it is not a fork: implement
it with the citation, and file no ask, card or decision item. When better
reference material arrives, adopt it over the earlier guess without asking;
an earlier guess is not special, and a finding that shows it was wrong is not
a reason to ask. Ask only when a real fork remains.

Every decision ask — chat, PR comment, or queue entry — is answerable from
that one message, on one screen, and carries all seven elements:

1. **Title/moniker**, glossed — never a bare ID, letter, or hex suffix
   (`BL-0180 (SDM driver seat-belt switch polarity)`).
2. **Context** — why it exists, what's blocked, cited facts.
3. **Options in prose**, each with its implication beside it — never bare
   "A/B/C".
4. **Recommendation**, with a one-line reason.
5. **Default-if-silent**, explicitly named.
6. **Provenance** — date first asked + where it lives; check the
   coordinator's decision queue for an existing ask before raising a new
   one, and cite a prior ask instead of re-asking cold.
7. **Why not self-derived** — name the recorded-objective category
   (convention, manual/provenance, prior ruling, standing preference) that
   leaves it genuinely open, or self-decide instead of asking.

Record the ruling immediately where it was asked, in one place; everything
else points to it.
<!-- END ev1-canon:decision-ask v3 -->
