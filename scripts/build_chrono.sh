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

# Portable-by-construction ISA baseline.
#
# Chrono's cmake/FindSIMD.cmake (invoked from src/CMakeLists.txt via the USE_SIMD
# option) probes the BUILD host and, on GCC/x86, unconditionally bakes
# `-march=native` into SIMD_CXX_FLAGS.  Those flags flow into CH_CXX_FLAGS and are
# re-exported as CHRONO_CXX_FLAGS (see cmake/chrono-config.cmake.in).  Downstream
# consumers (ev1sim's CMakeLists.txt applies CHRONO_CXX_FLAGS globally) then inherit
# the build host's widest ISA (AVX/AVX2/...).  When this install is cached and
# restored onto a narrower CI runner, the first wide SIMD op traps with SIGILL.
#
# Fix, two parts that must go together:
#   * -DUSE_SIMD=OFF disables the host-SIMD probe entirely, so `-march=native`
#     never enters CH_CXX_FLAGS / CHRONO_CXX_FLAGS.  A baseline `-march` alone is
#     NOT enough: with USE_SIMD=ON, FindSIMD appends `-march=native` AFTER our
#     flags and GCC's last-`-march`-wins rule re-widens the binary.
#   * -march=x86-64-v2 pins a portable floor (SSE4.2/POPCNT) that every
#     ubuntu-latest runner supports, so the actual code Chrono compiles runs
#     anywhere in the heterogeneous runner pool.
# CMAKE_CXX_FLAGS is applied to Chrono's own build but is NOT re-exported, so
# ev1sim mirrors the same `-march=x86-64-v2` on its side (CMakeLists.txt).
BASELINE_ISA="-march=x86-64-v2"

cmake -S "${CHRONO_SRC}" -B "${CHRONO_SRC}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_MODULE_VEHICLE=ON \
    -DENABLE_MODULE_IRRLICHT=ON \
    -DBUILD_DEMOS=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_BENCHMARKING=OFF \
    -DUSE_SIMD=OFF \
    -DCMAKE_C_FLAGS="${BASELINE_ISA}" \
    -DCMAKE_CXX_FLAGS="${BASELINE_ISA}" \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    "${extra_flags[@]}"

# Cap build parallelism (default -j4): the cold Chrono build fans out to host
# core count under -j"$(nproc)" and OOMs on constrained-RAM runners even at
# 7Gi. Tune via CHRONO_BUILD_JOBS.
cmake --build "${CHRONO_SRC}/build" -j"${CHRONO_BUILD_JOBS:-4}"
cmake --install "${CHRONO_SRC}/build"

# Drop the multi-GB build tree; only the (small) install prefix is kept/cached.
rm -rf "${CHRONO_SRC}"

echo "Chrono ${CHRONO_REF} installed to ${PREFIX}"
echo "Use: -DChrono_DIR=${PREFIX}/lib/cmake/Chrono"
