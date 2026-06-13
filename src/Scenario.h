#pragma once

#include <chrono>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "DriverCommand.h"
#include "VehicleState.h"

class ExternalSimConnector;

namespace ev1sim {

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
//   ipc_brake_telltale, ipc_antilock_telltale, ipc_service_now_telltale,
//   ipc_reduced_perf_telltale, ipc_check_messages_telltale
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
    void MaybeSampleStats(double sim_time, const VehicleState& state,
                          const ExternalSimConnector& bus,
                          const DriverCommand& applied_cmd);

    // Flush + close the stats file.
    void Close();

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

    ScenarioStats              m_stats;
    std::ofstream              m_csv;
    bool                       m_csv_opened = false;
    double                     m_last_stats_sim_time = -1.0;

    int                        m_passed_assertions = 0;
    int                        m_failed_assertions = 0;
};

}  // namespace ev1sim
