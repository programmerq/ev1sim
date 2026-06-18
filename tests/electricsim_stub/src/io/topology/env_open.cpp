/*
 * env_open implementation. See env_open.hpp for the contract.
 *
 * @design 2026-06-08 claude — epic/wire-truth-substrate, step 1c.
 */

#include "topology/env_open.hpp"
#include "topology/topology_generated.h"
#include "wire_table.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#if !defined(_WIN32)
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace electricsim::topology {

namespace {

// Default startup-attach retry window. The BPM controller's call to
// try_open_from_env() may race against the test_plant's create() —
// if the controller wins, the segment doesn't exist yet and a single
// shm_open() returns nullptr. Poll briefly (quietly, via raw
// shm_open) before falling back to wire-truth disabled. Configurable
// via ELECTRICSIM_WIRES_ATTACH_TIMEOUT_MS.
constexpr auto kAttachPollInterval = std::chrono::milliseconds(50);
constexpr auto kAttachTimeoutDefault = std::chrono::seconds(2);

std::chrono::milliseconds attach_timeout_from_env() {
  const char* s = std::getenv("ELECTRICSIM_WIRES_ATTACH_TIMEOUT_MS");
  if (s == nullptr || s[0] == '\0') return kAttachTimeoutDefault;
  char* end = nullptr;
  const long ms = std::strtol(s, &end, 10);
  if (end == s || *end != '\0' || ms < 0) {
    std::fprintf(stderr,
                 "topology::try_open_from_env: ignoring unparseable "
                 "ELECTRICSIM_WIRES_ATTACH_TIMEOUT_MS=%s\n",
                 s);
    return kAttachTimeoutDefault;
  }
  return std::chrono::milliseconds(ms);
}

#if !defined(_WIN32)
// Quietly probe whether the named segment exists. Returns true if a
// non-creating shm_open succeeds (and closes the fd); false on ENOENT
// or any other open failure. Used by the attach-poll loop to suppress
// the per-iteration stderr noise WireTable::attach() would emit.
bool segment_exists(const std::string& name) {
  const std::string posix = (name.empty() || name[0] == '/')
                            ? name
                            : std::string("/") + name;
  int fd = ::shm_open(posix.c_str(), O_RDONLY, 0);
  if (fd < 0) return false;
  ::close(fd);
  return true;
}
#endif

}  // namespace

std::unique_ptr<::electricsim::io::WireTable> try_open_from_env(
    const char* default_role) {
  const char* name = std::getenv("ELECTRICSIM_WIRES_NAME");
  if (name == nullptr || name[0] == '\0') {
    return nullptr;  // wire-truth substrate disabled
  }

  ::electricsim::io::WireTableOptions opts;
  opts.name = name;
  opts.topology_hash = kTopologyHash;

  const char* role = std::getenv("ELECTRICSIM_WIRES_ROLE");
  if (role == nullptr) role = default_role;
  const bool is_creator = (role != nullptr && std::strcmp(role, "creator") == 0);

  // Pre-built declare callback for the creator path; identical
  // first-try and orphan-retry paths share it.
  auto declare_cb = [name](::electricsim::io::WireTable& table) -> bool {
    if (!declare_all(table)) {
      std::fprintf(stderr,
                   "topology::try_open_from_env: declare_all failed on %s\n",
                   name);
      return false;
    }
    return true;
  };

  if (is_creator) {
    // First attempt — normal O_EXCL create.
    auto table = ::electricsim::io::WireTable::create(opts, declare_cb);
    if (table != nullptr) return table;

#if !defined(_WIN32)
    // create() failed — most likely the segment already exists (a
    // previous driver process died without unlinking; shm segments
    // outlive their creator). Recover.
    //
    // Step 1: ADOPT the orphan if it is structurally valid. adopt()
    // attaches to the SAME inode and takes lifecycle ownership, so any
    // readers still mapped to that orphan keep working and we write
    // exactly where they read. This is the critical fix for the
    // "consumer stuck on unlinked segment" hazard (Bugbot review of
    // epic PR #85): the previous unlink+recreate minted a fresh inode
    // and silently orphaned every live reader. adopt() validates magic
    // + version + topology_hash, so it only succeeds when the orphan
    // is the segment we'd have created anyway.
    auto adopted = ::electricsim::io::WireTable::adopt(opts);
    if (adopted != nullptr) {
      std::fprintf(stderr,
                   "topology::try_open_from_env: adopted existing valid "
                   "segment %s (reusing inode; readers unaffected)\n",
                   name);
      return adopted;
    }

    // Step 2: the orphan is UNUSABLE (corrupt, wrong topology hash,
    // wrong version, or a creator that died mid-init). Only now is
    // unlink + recreate safe: a reader could not have validly attached
    // to such a segment (attach() rejects the same conditions adopt()
    // just did), so re-minting the inode orphans nobody that was
    // working. Retry the create exactly once.
    //
    // Caveat: single-creator model. Concurrent legitimate creators
    // would step on each other; for the wire-truth design that's a
    // usage error, not a supported scenario.
    if (segment_exists(name)) {
      const std::string posix = (name[0] == '/') ? std::string(name)
                                                 : std::string("/") + name;
      if (::shm_unlink(posix.c_str()) == 0) {
        std::fprintf(stderr,
                     "topology::try_open_from_env: unlinked UNUSABLE orphan "
                     "segment %s (failed adopt); retrying create\n",
                     name);
        table = ::electricsim::io::WireTable::create(opts, declare_cb);
        if (table != nullptr) return table;
      }
    }
#endif
    return nullptr;
  }

  // Attacher path. Wait briefly for the segment to exist — handles
  // the "controller booted before the test_plant created the segment"
  // race. Probing via raw shm_open keeps the polling quiet; only
  // when the segment is observed do we hand off to WireTable::attach
  // (which logs further failures normally). (Bugbot review of epic
  // PR #85: attach was never retried, leaving the controller
  // permanently wire-truth-disabled on a slightly slow producer.)
#if defined(_WIN32)
  return ::electricsim::io::WireTable::attach(opts);
#else
  const auto deadline =
      std::chrono::steady_clock::now() + attach_timeout_from_env();
  while (true) {
    if (segment_exists(name)) {
      return ::electricsim::io::WireTable::attach(opts);
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      std::fprintf(stderr,
                   "topology::try_open_from_env: attach timed out for segment %s "
                   "(producer never came up?); wire-truth disabled for this "
                   "process\n",
                   name);
      return nullptr;
    }
    std::this_thread::sleep_for(kAttachPollInterval);
  }
#endif
}

}  // namespace electricsim::topology
