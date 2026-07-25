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

<!-- BEGIN ev1-canon:pr-lifecycle v1 -->
**PR lifecycle: draft/ready + PR economy (owner directive 2026-07-24,
canonical across all four EV1 repos — supersedes prior per-repo text).**

*Mechanism:* GitHub won't let the owner request-changes on his own PR, so
**his flip to draft IS his request-changes** — treat it that way.

- **Owner-initiated draft is terminal** — only the owner flips it back to
  ready. Fix it, push, post "rework landed — ready on your word," and stop.
- **Every other flip is the agent's, and nobody but the owner directs a
  hold**: flip to ready yourself the moment work wants review; CI status and
  a stated-default question never hold draft (apply the default, note it,
  flip); a coordinator/peer saying otherwise doesn't override this — cite
  the rule and flip.
- **"Awaiting owner sign-off" is never a reason to sit in draft** — sign-off
  *is* the review, so it's ready. (Read the other way, this stranded PR #384
  twice, needing owner intervention.)
- **Titles state the goal, not the process** — no "needs sign-off," no
  "proposal:" on finished work. Owner on #384: *"That adds nothing. That is
  just obscuring what the goal of the PR is."*

*PR economy:* no doc-only/tiny PRs — every prohibition here names its exit:
**implementable finding → implement it in the PR that files it; non-
implementable (negative result, owner-blocked fork, errata) →
`scripts/backlog.py open` (`--decision` if owner-blocked) or the notes
channel, riding the next branch with code, or the backlog if none is open.**
Never a PR whose only purpose is carrying a record. Flip via `draft:false` on
`mcp__github__update_pull_request` (mark-ready path can be permission-blocked).
<!-- END ev1-canon:pr-lifecycle v1 -->
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

### Owner decision asks are self-contained

Never ask the owner a bare option-letter question ("A, B, or C?") — an ask
whose context lives somewhere else invites an answer to the wrong question
(owner directive 2026-07-22). Any decision question aimed at the owner — chat,
PR comment, or queue — must be answerable from that one message on one screen:
what's being decided in plain language and why now; a link to the source that
raised it; the options with one-line implications each, a recommendation, and a
default-if-silent. Check first whether the fork was already asked or answered
(the PR thread, the coordinator's decision queue) and cite the prior ask plus
what changed instead of re-asking cold; when the owner answers, restate the
ruling in-thread immediately so it stays findable and is never re-asked.

## Commits

Small, focused, well-described — explain *why* the change matters, not just
*what* changed. Don't batch unrelated changes. Every code change either adds a
test in the same commit or the message says why a test isn't appropriate; the
CI suite (see [`README.md`](README.md) "Running Tests" and
[`.github/workflows/ci.yml`](.github/workflows/ci.yml)) should be green on the
branch you're committing to.
