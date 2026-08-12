#!/usr/bin/env bash
# No committed JSON may repeat a key inside one object.
#
# RFC 8259 allows duplicates and every parser here silently takes the last
# one, so a repeated key is a line of text that looks committed and is not
# read.  That is not hypothetical in this repo: the config and data files
# carry their rationale in numbered "//" keys, and while writing
# config/coastdown.json on 2026-08-12 I twice reused a number already used
# further down, which silently dropped four lines of the explanation.  CI's
# validate-configs job parses every JSON file, and parsing is exactly the step
# that cannot see this.
#
# Self-test: --selftest proves the check can FAIL, on a file built to fail.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"

if [[ "${1:-}" == "--selftest" ]]; then
    tmp="$(mktemp -d)"
    trap 'rm -rf "$tmp"' EXIT
    printf '{"a": 1, "b": 2, "a": 3}\n' > "$tmp/dup.json"
    if python3 "$here/json_dup_keys.py" "$tmp/dup.json" >/dev/null 2>&1; then
        echo "SELFTEST FAILED: a file with a repeated key was accepted" >&2
        exit 1
    fi
    printf '{"a": 1, "b": {"a": 2}}\n' > "$tmp/ok.json"
    if ! python3 "$here/json_dup_keys.py" "$tmp/ok.json" >/dev/null 2>&1; then
        echo "SELFTEST FAILED: the same key in two different objects was rejected" >&2
        exit 1
    fi
    echo "selftest ok: repeated keys rejected, same key in sibling objects allowed"
    exit 0
fi

mapfile -t files < <(cd "$root" && git ls-files '*.json')
python3 "$here/json_dup_keys.py" "${files[@]/#/$root/}"
