# AGENTS.md — guidance for AI agents contributing to ev1sim

Entry point for any AI agent (Claude Code, etc.) picking up work in the ev1sim
repo. This file captures the *operating conventions*; the build/run/architecture
detail is **not** duplicated here — it lives in [`README.md`](README.md)
(prerequisites, CMake presets, running, tests) and
[`ARCHITECTURE.md`](ARCHITECTURE.md) (the boundaries and process layout). Read
those for how the code works; read this for how we work on it.

## The project in one paragraph

ev1sim is the host-side **physics + visualization plant** for a future EV1
electronics toolchain: a standalone C++ vehicle simulator built on Project
Chrono that renders a drivable vehicle, accepts driver commands, and exposes
clean command/telemetry boundaries so an external electrical simulator
(`electricsim`) can drive the loads and read the sensors. See `ARCHITECTURE.md`
for the seams and `README.md` for the build.

## Git / PR workflow

Develop on a feature branch (`claude/<id>`); never push to `main`.

- **Merge stays the owner's call** — do not merge to `main`.

<!-- BEGIN ev1-canon:pr-lifecycle v4 -->
**Branches, pushes and PRs (owner directives 2026-07-24 through 2026-10-08,
canonical across all four EV1 repos; this block is the only copy of these
rules).** The owner tunes this as the program runs, so a new ruling replaces
the wording here instead of being added beside it. Edit it in
`ev1/CONVENTIONS.md`, bump the version, and paste it byte-identical into the
other carriers; `ev1/tools/canon_sync` checks the copies.

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
- **Open a PR when the branch holds a chunk worth the owner's review**: a
  coherent, finished piece of work he can act on, never a single page, small
  fix or doc tweak. **Few large PRs beat many small ones**; never split
  coherent work to keep diffs small. When the owner asks for a PR, open one.
- **No PR whose only purpose is carrying a record.** A small doc, note or
  queue edit rides an in-flight branch, even an imperfect topical fit.

*Draft and ready:* leaving draft means "ready for review", which is what
GitHub's UI calls it, and the owner reviews only ready PRs.
- **Open PRs ready.** A draft PR is not a place to hold work in progress;
  the pushed branch is.
- **The owner flipping a PR to draft is his request-changes** (GitHub won't
  let him request changes on his own PR). Fix it, push, and flip it back to
  ready yourself with a "rework landed" comment.
- The only other draft is a short cross-repo wait: a "waiting for <link> to
  merge" note at the top of the body, flipped ready as soon as that merges.
  A same-repo dependency is the base branch, not a draft.
- Nothing else holds a PR in draft: not CI status, not a question with a
  stated default (apply the default, note it, flip), not "awaiting owner
  sign-off" (sign-off is the review), and not a peer or coordinator saying
  so. Flip with `draft:false` on `mcp__github__update_pull_request` (the
  mark-ready path can be permission-blocked).

*What a PR contains:*
- **Complete.** Every item a PR mints is implemented in it, or names a
  justification class (owner-gated decision, cross-repo, genuinely
  unimplementable) in both the item and the PR body. "It belongs to another
  open PR" is not a justification: consolidate into one PR. **Identify and
  fix, don't log.** A stacked PR is a last resort (e.g. a CI-gated generated
  file forcing sequencing), and its title or first body line names its base
  PR and why.
- Non-implementable findings (negative result, owner-blocked fork, errata)
  go to the repo's backlog or notes channel, named right after this block,
  and ride the next branch with code.
- **Titles state the goal, not the process**: no "needs sign-off", no
  "proposal:" on finished work.
- **Merges are the owner's click.**
<!-- END ev1-canon:pr-lifecycle v4 -->
(ev1sim has no `scripts/backlog.py` — its notes channel is `DECISION_QUEUE.md`.)

<!-- BEGIN ev1-canon:pr-images v2 -->
**PR-body images (owner directive 2026-07-22, canonical across all four EV1 repos).**
Use `<img>` tags (never bare `![…]()` markdown — no size control), `src`
pinned to a **commit SHA** (never a branch name — branch URLs re-render as the
branch moves and can 404 after a rebase/merge), in the `blob` form — **never**
`raw.githubusercontent.com`:

`<img src="https://github.com/<owner>/<repo>/blob/<COMMIT_SHA>/<path>?raw=1" width="..." alt="..." />`

- **Why `blob/...?raw=1` and not `raw.githubusercontent.com`, everywhere:**
  one rule, no need to remember which repo you're in. In the three
  **private** repos (ev1, ev1-manual-redux, electricsim) it's more than a
  preference — `raw.githubusercontent.com` doesn't authenticate against the
  viewer's GitHub session, so images silently fail to render (or 404) for
  the owner there. **ev1sim is public**, so that specific failure doesn't
  apply to it — `raw.githubusercontent.com` would technically render — but
  `blob/...?raw=1` works identically there too, so the one rule holds
  without exception.
- **Always set an explicit `width`** — a percentage (`width="40%"`) for
  small images, a pixel width no larger than natural size otherwise. GitHub
  stretches unsized images to the full body column and upscales small ones
  blurrily.
<!-- END ev1-canon:pr-images v2 -->

- If the saved body shows `&lt;img&gt;` (a proxied environment entity-escaped
  it), redo the edit from an unproxied session via
  `gh api -X PATCH repos/<owner>/<repo>/pulls/<n> -F body=@file`.
- **Conflict resolution:** rebase onto fresh `origin/main` *or* merge
  `origin/main` into the branch — either is fine. Sync before new work; after a
  rebase that rewrites already-pushed history, push with `--force-with-lease`
  (never plain `--force`).

### PR writing conduct (owner rulings 2026-07-25, chat-only)

Not yet folded into a versioned block above — recorded in `ev1` PR #56
(<https://github.com/programmerq/ev1/pull/56>),
`decisions/2026-07-25-pr-body-conduct-chat-rulings.md` and
`decisions/2026-07-25-conflict-reduction-generated-files.md`; cite those
files directly until they are:

- **Never use the word "gate" in a PR title or body.** Say what the check
  catches, not the name of the mechanism catching it.
- **Lead the PR body with its strongest finding, stated positively** — say
  what a thing *is*, not what it is not. Prose volume isn't evidence of work.
- **Illustrate a change repeated many times with 2-3 representative images,
  not all of them** — extends the `pr-images` rule above.
- **A scheduled job owns generated/nightly-committed files; a PR diff never
  hand-carries them.** Not applicable to ev1sim today — no such file exists
  here yet; apply this if one appears.

### Owner decision asks are self-contained

<!-- BEGIN ev1-canon:decision-ask v2 -->
**Owner decision asks are self-contained (owner directive 2026-07-22,
canonical across all four EV1 repos — supersedes prior per-repo text).**

*First, is it a decision at all?* When a manual, patent, datasheet or other
primary source answers the question clearly, it is not a fork: implement
it with the citation, and file no ask, card or decision item (owner
2026-10-08). When better reference material arrives, adopt it over the
earlier guess without asking; an earlier guess is not special, and a
finding that shows it was wrong is not a reason to ask (owner 2026-10-06,
2026-10-07). Ask only when a real fork remains.

An ask whose context lives elsewhere invites an answer to the wrong
question — an already-engaged fork was once re-asked as bare option
letters and the earlier answer was never durably recorded. Every decision
ask — chat, PR comment, or queue entry — is answerable from that one
message, on one screen, and carries all seven elements:

1. **Title/moniker**, glossed — never a bare ID, letter, or hex suffix
   (`BL-0180 (SDM driver seat-belt switch polarity)`).
2. **Context** — why it exists, what's blocked, cited facts.
3. **Options in prose**, each with its implication beside it — never bare
   "A/B/C" (the owner once read option letters as motor phases).
4. **Recommendation**, with a one-line reason.
5. **Default-if-silent**, explicitly named.
6. **Provenance** — date first asked + where it lives; check the
   coordinator's decision queue for an existing ask before raising a new
   one, and cite a prior ask instead of re-asking cold.
7. **Why not self-derived** — name the recorded-objective category
   (convention, manual/provenance, prior ruling, standing preference) that
   leaves it genuinely open, or self-decide instead of asking.

Record the ruling immediately where it was asked.
<!-- END ev1-canon:decision-ask v2 -->

## Commits

Small, focused, well-described — explain *why* the change matters, not just
*what* changed. Don't batch unrelated changes. Every code change either adds a
test in the same commit or the message says why a test isn't appropriate; the
CI suite (see [`README.md`](README.md) "Running Tests" and
[`.github/workflows/ci.yml`](.github/workflows/ci.yml)) should be green on the
branch you're committing to.
