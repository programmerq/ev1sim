#!/usr/bin/env bash
# Build + install Project Chrono (Vehicle + Irrlicht modules) — the single
# source of truth for the recipe shared by CI (.github/workflows/ci.yml's
# chrono-smoke job) and local developers.  Mirrors the README "Installing
# Project Chrono from Source" steps, Linux-correct.
#
# Usage:
#   scripts/build_chrono.sh <install-prefix>
#   # e.g.  scripts/build_chrono.sh ~/chrono-install
#   # then  cmake .. -DChrono_DIR=<install-prefix>/lib/cmake/Chrono
#
# The pinned ref below is part of this file, so the CI cache key
# (hashFiles('scripts/build_chrono.sh')) rotates automatically whenever the
# recipe — ref or flags — changes.  Bumping to a 10.x tag also requires
# renaming the module flags to CH_ENABLE_MODULE_* (Chrono 10 renamed them).
set -euo pipefail

PREFIX="${1:?usage: build_chrono.sh <install-prefix>}"
CHRONO_REF="${CHRONO_REF:-release/9.0}"
CHRONO_SRC="${CHRONO_SRC:-/tmp/chrono-src}"

# Idempotent: skip if this prefix already holds a Chrono install (fast local
# re-runs; in CI this script only runs on a cache miss, where PREFIX is empty).
if [ -d "${PREFIX}/lib/cmake/Chrono" ]; then
    echo "Chrono already installed at ${PREFIX} — skipping build."
    exit 0
fi

# Use ccache when available (CI puts it on PATH); harmless when absent locally.
extra_flags=()
if command -v ccache >/dev/null 2>&1; then
    extra_flags+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache
                  -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi

rm -rf "${CHRONO_SRC}"
git clone --depth 1 --branch "${CHRONO_REF}" \
    https://github.com/projectchrono/chrono.git "${CHRONO_SRC}"

cmake -S "${CHRONO_SRC}" -B "${CHRONO_SRC}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_MODULE_VEHICLE=ON \
    -DENABLE_MODULE_IRRLICHT=ON \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    "${extra_flags[@]}"

cmake --build "${CHRONO_SRC}/build" -j"$(nproc)"
cmake --install "${CHRONO_SRC}/build"

# Drop the multi-GB build tree; only the (small) install prefix is kept/cached.
rm -rf "${CHRONO_SRC}"

echo "Chrono ${CHRONO_REF} installed to ${PREFIX}"
echo "Use: -DChrono_DIR=${PREFIX}/lib/cmake/Chrono"
