/*
 * env_open — opens the process's handle to the wire-truth substrate (a
 * WireTable) from environment variables. Since the ring-bus transport was
 * removed (commit 19f17aa0, 2026-06-17) this is THE way a binary joins the
 * bus; there is no other transport to fall back to.
 *
 *   1. Each producer/consumer binary calls topology::try_open_from_env()
 *      once at startup.
 *   2. If ELECTRICSIM_WIRES_NAME is unset/empty (the "wire-truth disabled"
 *      state) or the open fails, the helper returns nullptr. With no ring
 *      fallback, a substrate-dependent binary treats nullptr as a hard
 *      error and refuses to boot (these null-checks are what
 *      scripts/audit_bus_attach_check.py enforces).
 *   3. Otherwise the helper opens (create or attach) the WireTable
 *      configured by the env vars; the binary then reads/writes typed
 *      cells by WireId.
 *
 * Environment variables:
 *
 *   ELECTRICSIM_WIRES_NAME   — POSIX shm segment name (without "/").
 *                              If unset or empty, the helper returns
 *                              nullptr.
 *   ELECTRICSIM_WIRES_ROLE   — "creator" → opens with create() and
 *                              calls topo::declare_all on the result.
 *                              Anything else (or unset) → attach().
 *                              On attach, topology_hash is checked
 *                              against the generated kTopologyHash so
 *                              ABI drift refuses to start.
 *
 * Why env vars and not a config file? "Is this binary on the wire-truth
 * substrate, and as creator or attacher?" is a cross-cutting startup flag
 * with no per-binary knobs to tune — exactly what env vars handle cleanly.
 *
 * @design 2026-06-08 claude — epic/wire-truth-substrate, step 1c.
 * @design 2026-06-24 claude — refreshed after the ring-bus removal
 *   (19f17aa0): the dual-substrate / ELECTRICSIM_BUS_NAME framing is gone;
 *   the WireTable substrate is now the sole IPC fabric.
 */

#ifndef ELECTRICSIM_SRC_IO_TOPOLOGY_ENV_OPEN_HPP_
#define ELECTRICSIM_SRC_IO_TOPOLOGY_ENV_OPEN_HPP_

#include "wire_table.hpp"

#include <memory>

namespace electricsim::topology {

// Open a WireTable from environment variables. Returns nullptr if
// ELECTRICSIM_WIRES_NAME is unset/empty (the documented "wire-truth
// disabled" state) or if the underlying create()/attach() fails (a
// stderr diagnostic is emitted in that case).
//
// On the creator path, also calls declare_all() before returning, so
// the caller receives a ready-to-use table with all topology wires
// declared. declare_all is run BEFORE init_complete is published, so
// attachers either see the empty pre-init segment (and poll) or a
// fully-declared segment — never the in-between state where
// init_complete=1 but cell_count=0.
//
// `default_role` controls the role when ELECTRICSIM_WIRES_ROLE is
// unset: pass "creator" to make the calling binary the canonical
// segment owner when only ELECTRICSIM_WIRES_NAME is set (the natural
// case for test_plant / vehicle_sim — the bus orchestrator is also
// the wire-truth creator). An explicit ELECTRICSIM_WIRES_ROLE env
// var always wins; the default is only used when the env var is
// missing. Pass nullptr (the default) to fall back to attach mode
// when the env var is unset.
//
// Robustness behaviors (Bugbot review of epic PR #85):
//
//   - Creator path: if WireTable::create() fails because an orphan
//     segment exists (a previous creator that died without unlinking),
//     env_open detects the stale segment, shm_unlinks it, and retries
//     create() exactly once. Single-creator model — concurrent
//     legitimate creators are a usage error.
//
//   - Attacher path: env_open polls quietly (via raw shm_open) for
//     the segment to appear, up to a configurable timeout (default
//     2 s; override via ELECTRICSIM_WIRES_ATTACH_TIMEOUT_MS, in
//     milliseconds). This handles the "controller booted slightly
//     before the test_plant created the segment" race. When the
//     segment is observed, WireTable::attach() is called normally
//     (with its own init_complete polling for the in-between
//     window).
std::unique_ptr<::electricsim::io::WireTable> try_open_from_env(
    const char* default_role = nullptr);

}  // namespace electricsim::topology

#endif  // ELECTRICSIM_SRC_IO_TOPOLOGY_ENV_OPEN_HPP_
