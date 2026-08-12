#!/usr/bin/env python3
"""Report any JSON object that repeats a key.  Exit 1 if any file does.

Used by tests/check_json_no_duplicate_keys.sh; see that script for why.  Kept
as its own file rather than a heredoc so the selftest runs the same code the
real check does.
"""

from __future__ import annotations

import collections
import json
import sys
from pathlib import Path


def find_dups(path: Path) -> list[str]:
    found: list[str] = []

    def hook(pairs):
        counts = collections.Counter(k for k, _ in pairs)
        for key, n in counts.items():
            if n > 1:
                found.append(f"{key!r} appears {n} times in one object")
        return dict(pairs)

    json.loads(path.read_text(), object_pairs_hook=hook)
    return found


def main(argv: list[str]) -> int:
    if not argv:
        print("usage: json_dup_keys.py <file.json> ...", file=sys.stderr)
        return 2
    bad = 0
    for arg in argv:
        p = Path(arg)
        try:
            dups = find_dups(p)
        except json.JSONDecodeError as e:
            # Not this check's job — validate-configs covers parse errors —
            # but staying silent about an unparseable file would let one hide
            # from the duplicate scan.
            print(f"{p}: does not parse ({e})", file=sys.stderr)
            bad += 1
            continue
        for d in dups:
            print(f"{p}: {d}", file=sys.stderr)
        bad += bool(dups)
    if bad:
        print(f"\n{bad} JSON file(s) repeat a key. Every parser here keeps the "
              f"LAST one, so the\nearlier lines are committed text that nothing "
              f"reads.", file=sys.stderr)
        return 1
    print(f"{len(argv)} JSON file(s): no repeated keys")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
