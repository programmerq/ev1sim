#!/usr/bin/env python3
"""Any script that runs a subprocess in a DIFFERENT directory must prove its
paths are absolute first.

THE PATTERN, twice in one PR (2026-08-12).  A path is validated in one working
directory and used in another, so the check passes and the use fails — or
worse, silently succeeds against something else:

  * ev1sim resolved `config/default.json` against whatever cwd it happened to
    have.  From the repo root the coastdown loaded the milford level and the
    solver diverged to 370 m/s; from anywhere else it printed one stderr line,
    fell back to a rigid_plane, and produced a perfectly ordinary-looking
    coastdown.  One command line, either experiment.  (Fixed by
    Config::NamedConfigFault + the named_config_is_fatal check.)

  * scripts/scenario_runway_report.py accepted `--binary ./build/ev1sim`,
    confirmed it with is_file() in the caller's directory, then exec'd it with
    cwd set to a throwaway temp dir.  CI died on
    `FileNotFoundError: 'build/ev1sim'` AFTER the check had passed.

A third instance in this PR looked like the same thing and was not — a ctest
registered on whether the app COULD be built rather than whether the lane had
built it — so this checks the narrow, decidable version and nothing wider.

THE RULE.  In any scripts/*.py, if a subprocess call passes `cwd=`, the
enclosing function must also mention `is_absolute` — i.e. it has to say
somewhere that it thought about which directory its paths mean.  That is a
heuristic, deliberately: whether an arbitrary path expression is absolute is
not decidable here, but "did the author of a cwd-moving function check" is.

Self-test: --selftest proves the check can FAIL, on a sample built to fail.
"""

from __future__ import annotations

import ast
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SUBPROCESS_CALLS = {"run", "Popen", "call", "check_call", "check_output"}


def offenders(source: str, label: str) -> list[str]:
    """Functions that move the cwd for a subprocess without an absolute check."""
    tree = ast.parse(source)
    out: list[str] = []

    for fn in ast.walk(tree):
        if not isinstance(fn, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        moves_cwd = False
        for node in ast.walk(fn):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", "")
            if name not in SUBPROCESS_CALLS:
                continue
            if any(kw.arg == "cwd" for kw in node.keywords):
                moves_cwd = True
        if not moves_cwd:
            continue
        # Does the function anywhere establish that its paths are absolute?
        guarded = any(
            isinstance(n, ast.Attribute) and n.attr in ("is_absolute", "resolve")
            for n in ast.walk(fn)
        )
        if not guarded:
            out.append(
                f"{label}: {fn.name}() runs a subprocess with cwd= but never "
                f"checks is_absolute() / resolve() — a relative path handed to "
                f"it is valid where the caller stands and meaningless here")
    return out


def selftest() -> int:
    bad = """
import subprocess
def run_it(binary, workdir):
    subprocess.run([str(binary)], cwd=workdir, check=True)
"""
    good = """
import subprocess
def run_it(binary, workdir):
    if not binary.is_absolute():
        raise SystemExit("needs an absolute path")
    subprocess.run([str(binary)], cwd=workdir, check=True)
"""
    unrelated = """
import subprocess
def run_here(binary):
    subprocess.run([str(binary)], check=True)
"""
    ok = True
    for label, src, want in (("cwd= without a check", bad, True),
                             ("cwd= with is_absolute()", good, False),
                             ("no cwd= at all", unrelated, False)):
        found = bool(offenders(src, "sample"))
        good_verdict = found == want
        ok &= good_verdict
        print(f"  [{'ok  ' if good_verdict else 'FAIL'}] {label}: "
              f"{'flagged' if found else 'accepted'}")
    print("selftest " + ("ok" if ok else "FAILED"))
    return 0 if ok else 1


def main(argv: list[str]) -> int:
    if argv and argv[0] == "--selftest":
        return selftest()

    files = sorted((ROOT / "scripts").glob("*.py"))
    if not files:
        print("no scripts/*.py found — the scan is looking in the wrong place",
              file=sys.stderr)
        return 2

    found: list[str] = []
    for f in files:
        found += offenders(f.read_text(), str(f.relative_to(ROOT)))
    for msg in found:
        print(msg, file=sys.stderr)
    if found:
        print("\nResolve the path before the cwd moves (see "
              "scenario_runway_report.py's resolve_binary), or assert it is "
              "absolute at the top of the function.", file=sys.stderr)
        return 1
    print(f"{len(files)} script(s): no cwd-moving subprocess call without an "
          f"absolute-path check")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
