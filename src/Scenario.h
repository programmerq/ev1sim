#pragma once

#include <chrono>
#include <fstream>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "DriverCommand.h"
#include "VehicleState.h"

class ExternalSimConnector;

namespace ev1sim {

class PhysicalWorld;

// One scheduled scenario action. Fires when sim_time crosses at_time_s.
//
// Action semantics:
//   set_throttle / set_brake / set_steering:  override the corresponding
//       DriverCommand field for every subsequent tick until another set_*
//       overrides it.  Use value = 0 to release.  Steering value is in [-1, 1].
//   key_on_cycle:  cycle the RSA key one detent (OFF → ACC → RUN → OFF, …).
//       After two cycles from cold the vehicle is in RUN; a cycle never
//       downgrades RUN→ACC.  The first cycle enters the keypad code (opening
//       the RSA auth window) then presses ACC; the second presses RUN, queued
//       after the code so RUN is authenticated even when the two cycles are
//       only ~300 ms apart.  See RsaKeypadDriver (src/PhysicalWorld.h).
//   headlight_cycle:  cycle the combination switch (OFF → PARK → ON → HI).
//   prnd_up / prnd_down:  shift the PRND selector one position toward D / P.
//   turn_signal_left / turn_signal_right:  toggle that turn-signal direction.
//   hazard:  toggle the hazard switch.
//   ipc_trip_reset:  press the IPC trip-reset button (momentary).
//   cruise_set / cruise_resume / cruise_cancel / cruise_speed_up / cruise_speed_down:
//       press the corresponding cruise stalk button (momentary).
//   exterior_keypad_code:  enter the factory 6-digit code "111111" on the RSA
//       EXTERIOR keypad as a timed button sequence (~100 ms per digit, so the
//       whole entry takes ~0.6 s — schedule any follow-on door_handle_driver
//       at least 1 s later).
//   door_handle_driver:  pull the driver door handle (momentary).
//   door_lock_switch:  press a door-lock rocker once.  value != 0 is a LOCK
//       press, value == 0 an UNLOCK press (the same 1=locked/0=unlocked
//       encoding as the door_lock_cmd_* columns).  value2 picks the door:
//       0 (default) = LH/driver, non-zero = RH/passenger.  The contact closes
//       for a few hundred ms and opens again, because the junction block's
//       lock module one-shots on the RISING edge and a held rocker never
//       re-pulses.  Nothing about the lock state is asserted here: what the
//       motors then do is the electronics' answer, read back on the
//       door_lock_motor_* columns.
//       Press ONE side per event.  The lock module ORs the two doors' inputs,
//       so a single press is what commands both motors — pressing both at once
//       would hide whether that OR works.
//   set_horn:      value != 0 closes the driver horn contact (circuit 28) and
//       holds it until a set_horn 0; the LHJB decides which tones sound.
//   flash_to_pass: value != 0 holds the combination-switch flash-to-pass lever
//       until a flash_to_pass 0.
//
// Conditional events (gate event-list progression):
//   wait_for_speed:  block subsequent events until speed_mps >= value.
//       Once the threshold is reached the gate releases and the next
//       event fires on the same tick.  Acts as a barrier — useful when
//       a follow-on action (cruise_set) requires the car to first reach
//       a minimum speed.  No timeout — pair with the scenario's
//       max_time_s if a runaway is possible.
//
// Assertions:
//   assert_speed_within:  at this scheduled time, check |speed_mps - value|
//       <= value2.  Result is logged; if it fails, IsScenarioFailed()
//       returns true and the run reports a non-zero exit code.  Useful
//       for verifying cruise-control hold, brake-stopping distance, etc.
//
// Unknown actions log a warning and are skipped — keeps a typo from
// killing the run.
struct ScenarioEvent {
    double      at_time_s = 0.0;
    std::string action;
    double      value = 0.0;     // primary value (set_throttle target, wait_for_speed threshold, etc.)
    double      value2 = 0.0;    // secondary value (assert_speed_within tolerance, etc.)
};

// What to record at the configured sample period.  Fields recognised by
// Scenario::WriteStatsRow:
//   sim_time_s, speed_mps, accel_long, accel_lat, yaw_rate,
//   applied_throttle, applied_front_brake, applied_rear_brake,
//   throttle_cmd_q8, throttle_cmd_fresh,
//   cruise_active (bus, when published), motor_rpm, motor_torque_nm,
//   front_brake_pressure, rear_brake_position,
//   ipc_brake_telltale, ipc_antilock_telltale, ipc_air_bag_telltale,
//   ipc_service_now_telltale, ipc_reduced_perf_telltale,
//   ipc_check_messages_telltale
struct ScenarioStats {
    std::string              output_csv;
    std::vector<std::string> fields;
    double                   sample_period_s = 0.05;  // 20 Hz default
};

// Scenario hook interface — SimApp implements these so the Scenario can
// dispatch physical-world actions without depending on Irrlicht / Chrono.
class ScenarioHooks {
public:
    virtual ~ScenarioHooks() = default;
    virtual void KeyOnCycle()         = 0;
    virtual void HeadlightCycle()     = 0;
    virtual void PrndUp()             = 0;
    virtual void PrndDown()           = 0;
    virtual void TurnSignalLeft()     = 0;
    virtual void TurnSignalRight()    = 0;
    virtual void HazardToggle()       = 0;
    virtual void IpcTripResetPress()  = 0;
    virtual void CruiseSet()          = 0;
    virtual void CruiseResume()       = 0;
    virtual void CruiseCancel()       = 0;
    virtual void CruiseSpeedUp()      = 0;
    virtual void CruiseSpeedDown()    = 0;
    // Door locks: headless equivalents of the Lock All / Unlock All UI
    // buttons — drive every door-lock to LOCKED / unlocked at once, so
    // acceptance scenarios can exercise the lock/unlock actuation timeline
    // without the interactive UI.
    virtual void DoorLockAll()        = 0;
    virtual void DoorUnlockAll()      = 0;
    // One momentary press of a door-lock rocker: lock=true closes that door's
    // LOCK contact, lock=false its UNLOCK contact; driver_door=true is the
    // LH/driver rocker, false the RH/passenger one.  Unlike DoorLockAll this
    // does NOT set a lock state — it closes a switch contact and leaves the
    // outcome to the junction block, so a scenario using it exercises the
    // electronics instead of asserting the answer it wanted.
    virtual void DoorLockSwitchPress(bool lock, bool driver_door) = 0;
    // RSA EXTERIOR keypad (the five buttons on the driver's door pillar) —
    // queues the factory 6-digit code "111111" as a timed button sequence,
    // exactly as the interactive `K` binding does. This is the physical way
    // a locked EV1 is opened from outside; door_unlock_all is the UI
    // shortcut that bypasses the RSA's code validation entirely.
    virtual void ExteriorKeypadCode() = 0;
    // Driver door handle pull. With the RSA's code-OK window open the RSA
    // publishes door_lock_cmd = unlocked; with it closed the pull is refused.
    virtual void DoorHandleDriver()   = 0;
    // Combination-switch flash-to-pass lever, held until released.
    virtual void FlashToPass(bool held) = 0;
    // Fault injection (used by external acceptance scenarios): fail=true
    // suppresses the driver THROTTLE publish (6903) on the main harness —
    // models the accelerator-pedal acquisition feed going dead at the
    // consuming controller. fail=false restores it (next heartbeat
    // republishes).
    virtual void FailThrottleInput(bool fail) = 0;
};

class Scenario {
public:
    // Load a scenario from the given JSON file.  Returns std::nullopt on
    // parse / IO failure (with a [Scenario] error logged to stderr).
    static std::optional<Scenario> LoadFromFile(const std::string& path);

    // Construct from raw fields (tests + LoadFromFile).
    Scenario() = default;

    // Apply pending events for the current sim time.  Holds for any
    // set_throttle / set_brake / set_steering values.  Calls hooks for
    // physical-world events.  state.speed_mps is read by wait_for_speed
    // / assert_speed_within actions; pass the physics state from the
    // previous substep.
    void Tick(double sim_time, const VehicleState& state,
              ScenarioHooks& hooks, DriverCommand& cmd);

    // True if any assert_* action has failed during the run.  Surfaces
    // the failure to the SimApp run loop so it can return a non-zero
    // exit code.
    bool IsScenarioFailed() const { return m_failed_assertions > 0; }
    int  FailedAssertions() const { return m_failed_assertions; }
    int  PassedAssertions() const { return m_passed_assertions; }

    // Write the CSV header (idempotent — only the first call does work).
    void OpenStats();

    // Sample stats if the configured sample period has elapsed.
    //
    // `physical` is the body-peripheral plant: some columns report what an
    // ev1sim actuator DID (a door-lock motor's winding state and stroke), which
    // is not derivable from `state` or from `bus` alone.  It is a required
    // argument rather than an optional one so a caller that forgot it is a
    // compile error — the alternative silently writes a column of zeros, which
    // is indistinguishable from a plant that never moved.
    void MaybeSampleStats(double sim_time, const VehicleState& state,
                          const PhysicalWorld& physical,
                          const ExternalSimConnector& bus,
                          const DriverCommand& applied_cmd);

    // Flush + close the stats file.
    void Close();

    // Names this writer did not recognise, in the order they were first seen.
    // Empty on a scenario whose every stats field is implemented.
    const std::set<std::string>& unknown_fields() const { return m_warned_fields; }

    bool IsDone(double sim_time) const {
        return m_max_time_s > 0.0 && sim_time >= m_max_time_s;
    }

    double               max_time_s() const { return m_max_time_s; }
    const std::string&   driver_mode() const { return m_driver_mode; }
    const std::string&   name()        const { return m_name; }
    // True if this scenario drives/asserts on signals only the external
    // electronics sim produces (BTCM ABS, PIM cruise, ...); SimApp skips the
    // run when that sim can never be present (stub build / --external-sim off).
    bool                 requires_external_sim() const { return m_requires_external_sim; }
    bool                 has_stats()   const { return !m_stats.output_csv.empty(); }
    const ScenarioStats& stats() const { return m_stats; }
    std::size_t          event_count() const { return m_events.size(); }

    // Test-only setters.
    void set_events(std::vector<ScenarioEvent> e) { m_events = std::move(e); }
    void set_stats(ScenarioStats s)               { m_stats  = std::move(s); }
    void set_max_time_s(double v)                 { m_max_time_s = v; }
    void set_driver_mode(const std::string& v)    { m_driver_mode = v; }
    void set_requires_external_sim(bool v)        { m_requires_external_sim = v; }

private:
    std::string                m_name;
    std::string                m_driver_mode = "local";
    double                     m_max_time_s  = 0.0;
    bool                       m_requires_external_sim = false;

    std::vector<ScenarioEvent> m_events;
    std::size_t                m_next_event = 0;

    // Held DriverCommand overrides (set_throttle / set_brake / set_steering).
    // optional: when engaged, the scenario forces the field to this value
    // every tick until released by another set_* event.
    std::optional<double>      m_held_throttle;
    std::optional<double>      m_held_brake;
    std::optional<double>      m_held_steering;
    // Horn contact, held like the pedals: set_horn value != 0 closes the
    // single driver horn contact (circuit 28) until the next set_horn 0.
    std::optional<bool>        m_held_horn;

    ScenarioStats              m_stats;
    std::ofstream              m_csv;
    bool                       m_csv_opened = false;
    double                     m_last_stats_sim_time = -1.0;

    int                        m_passed_assertions = 0;
    int                        m_failed_assertions = 0;

    // Stats field names this writer does not implement, so the warning fires
    // once per name rather than once per sample row.
    std::set<std::string>      m_warned_fields;

    // Report an unrecognised stats field once. The cell stays blank; see the
    // definition for why that is deliberate and what the warning adds.
    void WarnUnknownField(const std::string& f);
};

}  // namespace ev1sim
