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

- **Open a PR for your own work as a draft — don't wait to be asked.**
- **Flip it to ready-for-review yourself** once (a) an adversarial self-review
  passes and (b) there are zero open owner-questions. Marking ready *is*
  requesting the owner's review; while any owner-question is open, keep it a
  draft. **Don't gate ready on CI status** — CI is an independent signal the
  owner watches himself, never a reason to hold the PR in draft.
- **Merge stays the owner's call** — do not merge to `main`.

<!-- BEGIN ev1-canon:pr-images v1 -->
**PR-body images (owner directive 2026-07-22, canonical across all four EV1 repos).**
Use `<img>` tags (never bare `![…]()` markdown — no size control), `src`
pinned to a **commit SHA** (never a branch name — branch URLs re-render as the
branch moves and can 404 after a rebase/merge), in the `blob` form — **never**
`raw.githubusercontent.com`:

`<img src="https://github.com/<owner>/<repo>/blob/<COMMIT_SHA>/<path>?raw=1" width="..." alt="..." />`

- **Why `blob/...?raw=1` and not `raw.githubusercontent.com`:** these repos
  are private. `raw.githubusercontent.com` URLs don't authenticate against
  the viewer's GitHub session, so images silently fail to render (or 404)
  for the owner. The `github.com/.../blob/...?raw=1` form resolves through
  the normal authenticated session and renders.
- **Always set an explicit `width`** — a percentage (`width="40%"`) for
  small images, a pixel width no larger than natural size otherwise. GitHub
  stretches unsized images to the full body column and upscales small ones
  blurrily.
<!-- END ev1-canon:pr-images v1 -->

- If the saved body shows `&lt;img&gt;` (a proxied environment entity-escaped
  it), redo the edit from an unproxied session via
  `gh api -X PATCH repos/<owner>/<repo>/pulls/<n> -F body=@file`.
- **Batch small changes into an adjacent open PR** (owner directive
  2026-07-23). Docs, queue, and other small housekeeping edits ride an
  in-flight branch — even an imperfect topical fit — never a
  single-small-file PR of their own. Prefer larger-than-comfortable PRs:
  every extra PR spends Actions minutes and spins up review-bot + CI
  machinery for no review value.
- **Conflict resolution:** rebase onto fresh `origin/main` *or* merge
  `origin/main` into the branch — either is fine. Sync before new work; after a
  rebase that rewrites already-pushed history, push with `--force-with-lease`
  (never plain `--force`).

### Don't strand a PR in draft

An owner comment plus a flip of the PR back to draft means **"rework wanted,"
NOT "park it forever."** The session addresses the feedback and then flips the
PR back to ready-for-review **itself** the moment the rework is done — ready
means the work is done and the PR wants the owner's eyes again; never wait for
the owner to ask you to flip it. **Don't gate this on CI** — CI is an
independent signal the owner watches himself, not a reason to hold the PR in
draft. A PR may stay draft **only** while rework is actively in progress, or
while a blocking question is **both** posted on the PR **and** genuinely
unanswered. If you stated defaults for your open questions, apply those defaults
and flip ready rather than stranding the PR in draft.

Mechanism: flip via the update-pull-request API with `draft=false`
(`mcp__github__update_pull_request`, `draft: false`) — the dedicated
mark-ready path can be permission-blocked.

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
