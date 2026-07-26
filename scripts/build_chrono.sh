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
#   scripts/build_chrono.sh --print-deps
#   # the Debian/Ubuntu package list, for `sudo apt install $(... --print-deps)`
#
# The pinned ref below is part of this file, so the CI cache key
# (hashFiles('scripts/build_chrono.sh')) rotates automatically whenever the
# recipe — ref or flags — changes.  Bumping to a 10.x tag also requires
# renaming the module flags to CH_ENABLE_MODULE_* (Chrono 10 renamed them).
set -euo pipefail

# Debian/Ubuntu packages Chrono needs to configure and link.  This is the one
# copy: CI installs exactly `$(scripts/build_chrono.sh --print-deps)` rather
# than repeating the list in ci.yml, so the two cannot drift apart.
#
# libirrlicht-dev  Irrlicht headers + libIrrlicht.so
# libeigen3-dev    Chrono's src/CMakeLists.txt does find_package(Eigen3 3.3.0)
#                  and FATAL_ERRORs without it
# the rest         the .so symlinks for the libraries Chrono hardcodes into the
#                  Irrlicht link line (see the preflight below)
#
# freeglut3-dev is a transitional stub for libglut-dev on Debian 12 / Ubuntu
# 23.04+, and the real package on older releases — so it is the name that
# resolves on all of them.
CHRONO_APT_DEPS="libirrlicht-dev libeigen3-dev \
libgl1-mesa-dev libglu1-mesa-dev \
libxxf86vm-dev libxext-dev libx11-dev freeglut3-dev"

if [ "${1:-}" = "--print-deps" ]; then
    echo "${CHRONO_APT_DEPS}"
    exit 0
fi

PREFIX="${1:?usage: build_chrono.sh <install-prefix> | --print-deps}"
CHRONO_REF="${CHRONO_REF:-release/9.0}"
CHRONO_SRC="${CHRONO_SRC:-/tmp/chrono-src}"

# Idempotent: skip if this prefix already holds a Chrono install (fast local
# re-runs; in CI this script only runs on a cache miss, where PREFIX is empty).
if [ -d "${PREFIX}/lib/cmake/Chrono" ]; then
    echo "Chrono already installed at ${PREFIX} — skipping build."
    exit 0
fi

# Fail now, not 45% into a 20-40 minute build, when an X/GL dev package is absent.
#
# Chrono's src/chrono_irrlicht/CMakeLists.txt:69 (release/9.0) hardcodes its
# Linux link line:
#     SET(IRRLICHT_LIBRARY "${IRRLICHT_LIBRARY}" -lXxf86vm -lglut -lX11 -lGL)
# Debian's libirrlicht-dev declares `Depends: libirrlicht1.8` and nothing more,
# so none of those four arrive with it.  A missing one surfaces only when
# libChronoEngine_irrlicht.so links, as
#     /usr/bin/ld: cannot find -lXxf86vm: No such file or directory
# Note it is the -dev symlink that is absent, not the library: libXxf86vm.so.1
# is typically already installed, and `-l` does not look at versioned sonames.
#
# Ask the linker instead of dpkg.  It tests the thing that actually breaks, and
# it stays correct on distros that name these packages differently.
if [ "$(uname -s)" = "Linux" ]; then
    probe_cc="${CC:-cc}"
    probe_src="$(mktemp -t chrono-linkprobe-XXXXXX.c)"
    printf 'int main(void){return 0;}\n' > "${probe_src}"

    # Only trust the probe if a bare link works; otherwise the toolchain itself
    # is the problem and every -l would report a false missing.
    if command -v "${probe_cc}" >/dev/null 2>&1 \
       && "${probe_cc}" "${probe_src}" -o /dev/null >/dev/null 2>&1; then
        missing_libs=()
        for probe in Xxf86vm:libxxf86vm-dev glut:freeglut3-dev \
                     X11:libx11-dev GL:libgl1-mesa-dev; do
            lib="${probe%%:*}"
            if ! "${probe_cc}" "${probe_src}" -o /dev/null "-l${lib}" >/dev/null 2>&1; then
                missing_libs+=("  -l${lib}  needs  ${probe##*:}")
            fi
        done

        if [ ${#missing_libs[@]} -gt 0 ]; then
            rm -f "${probe_src}"
            echo "ERROR: Chrono's Irrlicht module links these, and ld cannot find them:" >&2
            printf '%s\n' "${missing_libs[@]}" >&2
            echo >&2
            echo "Install the full dependency set and re-run:" >&2
            echo "  sudo apt install ${CHRONO_APT_DEPS}" >&2
            exit 1
        fi
    fi
    rm -f "${probe_src}"
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
