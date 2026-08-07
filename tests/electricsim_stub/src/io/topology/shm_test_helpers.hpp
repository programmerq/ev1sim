// shm_test_helpers.hpp — portable test-side housekeeping for POSIX
// shared-memory segments.
//
// Many of the wire-truth tests pre-clean a possibly-stale segment with
// `::shm_unlink(("/" + name).c_str())` before standing up a fresh
// WireTable, to keep a previous leaked run from poisoning the new one.
// On POSIX that's a deliberate insurance call; on Windows it can't
// compile (MinGW64 has no <sys/mman.h>) and isn't needed (Win32 named
// mappings are reclaimed automatically when the last handle closes —
// there is no explicit unlink API).
//
// This header hides that platform split behind a single inline helper
// so each test can call `shm_unlink_quiet(name)` and stay portable.
// The implementation mirrors the `posix_name` / `win32_name` split in
// src/io/wire_table.cpp and the `segment_exists()` probe in
// src/io/topology/env_open.cpp.
//
// @design 2026-06-17 claude — Win32 backend follow-up after
//   epic/wire-truth-substrate Phase 1 (substrate-side backend at
//   commit 28b54db; test-side cleanup helper here).

#ifndef ELECTRICSIM_SRC_IO_TOPOLOGY_SHM_TEST_HELPERS_HPP_
#define ELECTRICSIM_SRC_IO_TOPOLOGY_SHM_TEST_HELPERS_HPP_

#include <string>

#if !defined(_WIN32)
#include <sys/mman.h>
#endif

namespace electricsim::io {

// Best-effort cleanup of a possibly-stale POSIX shm segment with the
// given name. No-op on Windows. Failures are silently ignored —
// the segment may legitimately not exist yet (first run after
// `make clean-vehicle`), and a stale-segment race after a peer crash
// is exactly what this call is meant to recover from.
inline void shm_unlink_quiet(const std::string& name) {
#if defined(_WIN32)
  (void)name;
#else
  const std::string posix = (name.empty() || name[0] == '/')
                            ? name
                            : "/" + name;
  ::shm_unlink(posix.c_str());
#endif
}

}  // namespace electricsim::io

#endif  // ELECTRICSIM_SRC_IO_TOPOLOGY_SHM_TEST_HELPERS_HPP_
