#include "Scenario.h"

#include <cctype>
#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "ExternalSimConnector.h"
#include "PhysicalWorld.h"

namespace ev1sim {

using json = nlohmann::json;

namespace {

bool ApplyDriverFieldEvent(const ScenarioEvent& e, std::optional<double>& held) {
    if (e.action == "set_throttle" || e.action == "set_brake" ||
        e.action == "set_steering") {
        held = e.value;
        return true;
    }
    return false;
}

}  // namespace

std::optional<Scenario> Scenario::LoadFromFile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[Scenario] cannot open '" << path << "'\n";
        return std::nullopt;
    }
    json j;
    try {
        j = json::parse(f);
    } catch (const std::exception& ex) {
        std::cerr << "[Scenario] parse error in '" << path << "': "
                  << ex.what() << "\n";
        return std::nullopt;
    }

    Scenario s;
    if (j.contains("name"))        s.m_name        = j["name"].get<std::string>();
    if (j.contains("driver_mode")) s.m_driver_mode = j["driver_mode"].get<std::string>();
    if (j.contains("max_time_s"))  s.m_max_time_s  = j["max_time_s"].get<double>();
    if (j.contains("requires_external_sim"))
        s.m_requires_external_sim = j["requires_external_sim"].get<bool>();

    if (j.contains("events") && j["events"].is_array()) {
        for (const auto& ev : j["events"]) {
            ScenarioEvent e;
            if (ev.contains("at_time_s")) e.at_time_s = ev["at_time_s"].get<double>();
            if (ev.contains("action"))    e.action    = ev["action"].get<std::string>();
            if (ev.contains("value"))     e.value     = ev["value"].get<double>();
            if (ev.contains("value2"))    e.value2    = ev["value2"].get<double>();
            // Comment-only entries (e.g. {"//": "explanation"}) carry no
            // action — skip them.  This lets scenario authors interleave
            // free-form notes with real events.
            if (e.action.empty()) continue;
            s.m_events.push_back(std::move(e));
        }
    }
    // Stable ordering by time so events fire in the right order regardless of
    // file authoring order.
    std::sort(s.m_events.begin(), s.m_events.end(),
              [](const ScenarioEvent& a, const ScenarioEvent& b) {
                  return a.at_time_s < b.at_time_s;
              });

    if (j.contains("stats") && j["stats"].is_object()) {
        const auto& st = j["stats"];
        if (st.contains("output_csv"))      s.m_stats.output_csv      = st["output_csv"].get<std::string>();
        if (st.contains("sample_period_s")) s.m_stats.sample_period_s = st["sample_period_s"].get<double>();
        if (st.contains("fields") && st["fields"].is_array()) {
            for (const auto& f : st["fields"]) {
                s.m_stats.fields.push_back(f.get<std::string>());
            }
        }
    }

    if (s.m_stats.fields.empty() && !s.m_stats.output_csv.empty()) {
        // Sensible default: speed + applied actuators + bus throttle freshness.
        s.m_stats.fields = {"sim_time_s", "speed_mps", "applied_throttle",
                            "applied_front_brake", "throttle_cmd_q8",
                            "throttle_cmd_fresh"};
    }

    std::cout << "[Scenario] loaded '" << s.m_name
              << "' from " << path
              << " (" << s.m_events.size() << " events, "
              << "driver_mode=" << s.m_driver_mode << ", "
              << "max_time=" << s.m_max_time_s << " s"
              << (s.m_requires_external_sim ? ", requires_external_sim" : "")
              << ")\n";
    return s;
}

void Scenario::Tick(double sim_time, const VehicleState& state,
                    ScenarioHooks& hooks, DriverCommand& cmd) {
    while (m_next_event < m_events.size() &&
           m_events[m_next_event].at_time_s <= sim_time) {
        const auto& e = m_events[m_next_event];

        // Conditional barrier: wait_for_speed blocks subsequent events
        // until vehicle speed reaches the threshold.  Don't advance the
        // index; just bail out of the dispatch loop.
        if (e.action == "wait_for_speed") {
            if (state.speed_mps >= e.value) {
                std::cout << "[Scenario] t=" << sim_time
                          << " wait_for_speed released "
                          << "(speed=" << state.speed_mps
                          << " >= " << e.value << " m/s)\n";
                ++m_next_event;
                continue;  // try next event on the same tick
            }
            // Still waiting — leave the index where it is.
            break;
        }

        ++m_next_event;
        std::cout << "[Scenario] t=" << sim_time
                  << " action=" << e.action;
        if (e.action.rfind("set_", 0) == 0)        std::cout << " value=" << e.value;
        if (e.action == "assert_speed_within")     std::cout << " target=" << e.value
                                                              << " tol=" << e.value2;
        std::cout << "\n";

        if (e.action == "set_throttle") {
            m_held_throttle = e.value;
        } else if (e.action == "set_brake") {
            m_held_brake = e.value;
        } else if (e.action == "set_steering") {
            m_held_steering = e.value;
        } else if (e.action == "assert_speed_within") {
            const double err = state.speed_mps - e.value;
            const double abs_err = err < 0.0 ? -err : err;
            if (abs_err <= e.value2) {
                ++m_passed_assertions;
                std::cout << "[Scenario] ASSERT PASS: speed "
                          << state.speed_mps << " ≈ " << e.value
                          << " (|err|=" << abs_err
                          << " <= " << e.value2 << ")\n";
            } else {
                ++m_failed_assertions;
                std::cerr << "[Scenario] ASSERT FAIL: speed "
                          << state.speed_mps << " not within "
                          << e.value2 << " of " << e.value
                          << " (|err|=" << abs_err << ")\n";
            }
        } else if (e.action == "key_on_cycle")        { hooks.KeyOnCycle();
        } else if (e.action == "headlight_cycle")     { hooks.HeadlightCycle();
        } else if (e.action == "prnd_up")             { hooks.PrndUp();
        } else if (e.action == "prnd_down")           { hooks.PrndDown();
        } else if (e.action == "turn_signal_left")    { hooks.TurnSignalLeft();
        } else if (e.action == "turn_signal_right")   { hooks.TurnSignalRight();
        } else if (e.action == "hazard")              { hooks.HazardToggle();
        } else if (e.action == "ipc_trip_reset")      { hooks.IpcTripResetPress();
        } else if (e.action == "cruise_set")          { hooks.CruiseSet();
        } else if (e.action == "cruise_resume")       { hooks.CruiseResume();
        } else if (e.action == "cruise_cancel")       { hooks.CruiseCancel();
        } else if (e.action == "cruise_speed_up")     { hooks.CruiseSpeedUp();
        } else if (e.action == "cruise_speed_down")   { hooks.CruiseSpeedDown();
        } else if (e.action == "door_lock_all")       { hooks.DoorLockAll();
        } else if (e.action == "door_unlock_all")     { hooks.DoorUnlockAll();
        } else if (e.action == "door_lock_switch")    {
            // value != 0 → LOCK press, value == 0 → UNLOCK press: the same
            // 1=locked / 0=unlocked encoding the door_lock_cmd_* columns use.
            hooks.DoorLockSwitchPress(e.value != 0.0);
        } else if (e.action == "exterior_keypad_code"){ hooks.ExteriorKeypadCode();
        } else if (e.action == "door_handle_driver")  { hooks.DoorHandleDriver();
        } else if (e.action == "set_horn")            { m_held_horn = (e.value != 0.0);
        } else if (e.action == "flash_to_pass")       { hooks.FlashToPass(e.value != 0.0);
        } else if (e.action == "fail_throttle_input") {
            // value != 0 → fail, 0 → restore.
            hooks.FailThrottleInput(e.value != 0.0);
        } else {
            std::cerr << "[Scenario] unknown action '" << e.action
                      << "' — skipping\n";
        }
    }

    if (m_held_throttle) cmd.throttle    = *m_held_throttle;
    if (m_held_brake)    cmd.front_brake = cmd.rear_brake = *m_held_brake;
    if (m_held_steering) cmd.steering    = *m_held_steering;
    // One physical horn contact (circuit 28); SimApp ORs low||high into
    // HornButton::set_held, so driving both mirrors a closed contact.
    if (m_held_horn)     cmd.horn_low    = cmd.horn_high = *m_held_horn;
}

void Scenario::OpenStats() {
    if (m_csv_opened || m_stats.output_csv.empty()) return;
    m_csv.open(m_stats.output_csv, std::ios::out | std::ios::trunc);
    if (!m_csv.is_open()) {
        std::cerr << "[Scenario] cannot open stats CSV '"
                  << m_stats.output_csv << "'\n";
        return;
    }
    bool first = true;
    for (const auto& f : m_stats.fields) {
        if (!first) m_csv << ",";
        m_csv << f;
        first = false;
    }
    m_csv << "\n";
    m_csv_opened = true;
    std::cout << "[Scenario] stats → " << m_stats.output_csv
              << " (fields: " << m_stats.fields.size()
              << ", period " << m_stats.sample_period_s << " s)\n";
}

void Scenario::MaybeSampleStats(double sim_time, const VehicleState& state,
                                const PhysicalWorld& physical,
                                const ExternalSimConnector& bus,
                                const DriverCommand& applied_cmd) {
    if (!m_csv_opened) return;
    if (m_last_stats_sim_time >= 0.0 &&
        sim_time - m_last_stats_sim_time < m_stats.sample_period_s) {
        return;
    }
    m_last_stats_sim_time = sim_time;

    const auto bus_throttle =
        bus.GetThrottleCmd(std::chrono::milliseconds(200));

    // Freshness window for the BTCM-sourced ABS-phase and rear-EMB reads.
    // This MUST match the control path's window (SimApp::kAbsFreshnessWindow
    // = 3000 ms, the authoritative source of truth).  The two diverged once:
    // this stats-CSV observer read the same signals with a 200 ms window while
    // the control path used 3000 ms.  Liveness comes from the BTCM's ~5 Hz
    // canonical-frame heartbeat, which is paced in sim time; under wall-clock
    // pacing on long, low-friction stops the inter-heartbeat gap can stretch
    // past 200 ms, so the observer wrongly declared the BTCM dead and emitted
    // the -1 no-data sentinel for a phase that was actually live (and that the
    // control path, on 3000 ms, still saw). Keep these two windows in lockstep.
    constexpr auto kAbsPhaseFreshness = std::chrono::milliseconds(3000);
    const auto bus_abs    = bus.GetAbsPhaseFront(kAbsPhaseFreshness);
    const auto bus_rear   = bus.GetRearEmbCmd(kAbsPhaseFreshness);

    // Encode ABS phase as int for CSV-friendly plotting.
    auto phase_to_int = [](ExternalSimConnector::AbsPhaseFront::Phase p) -> int {
        switch (p) {
            case ExternalSimConnector::AbsPhaseFront::Phase::APPLY: return 0;
            case ExternalSimConnector::AbsPhaseFront::Phase::HOLD:  return 1;
            case ExternalSimConnector::AbsPhaseFront::Phase::DUMP:  return 2;
        }
        return 0;
    };

    auto write_field = [&](const std::string& f) -> void {
        if      (f == "sim_time_s")           m_csv << sim_time;
        else if (f == "speed_mps")            m_csv << state.speed_mps;
        else if (f == "accel_long")           m_csv << state.accel_long;
        else if (f == "accel_lat")            m_csv << state.accel_lat;
        else if (f == "yaw_rate")             m_csv << state.yaw_rate;
        else if (f == "yaw_deg")              m_csv << state.yaw_deg;
        else if (f == "pitch_deg")            m_csv << state.pitch_deg;
        else if (f == "road_grade_pct")       m_csv << state.road_grade_pct;
        else if (f == "applied_throttle")     m_csv << applied_cmd.throttle;
        else if (f == "applied_front_brake")  m_csv << applied_cmd.front_brake;
        else if (f == "applied_rear_brake")   m_csv << applied_cmd.rear_brake;
        else if (f == "applied_steering")     m_csv << applied_cmd.steering;
        else if (f == "front_brake_pressure") m_csv << state.front_brake_pressure;
        else if (f == "rear_brake_position")  m_csv << state.rear_brake_position;
        else if (f == "throttle_cmd_q8")      m_csv << static_cast<int>(bus_throttle.q8);
        else if (f == "throttle_cmd_fresh")   m_csv << (bus_throttle.fresh ? 1 : 0);
        // Vehicle pose — useful for stopping-distance computations.
        else if (f == "pos_x")                m_csv << state.pos_x;
        else if (f == "pos_y")                m_csv << state.pos_y;
        // Per-wheel angular speed (rad/s).  VehicleState array order is
        // FL, FR, RL, RR.  For ABS validation, watching these chatter
        // during a hard brake is the key diagnostic.
        else if (f == "wheel_omega_fl")       m_csv << state.wheel_omega[0];
        else if (f == "wheel_omega_fr")       m_csv << state.wheel_omega[1];
        else if (f == "wheel_omega_rl")       m_csv << state.wheel_omega[2];
        else if (f == "wheel_omega_rr")       m_csv << state.wheel_omega[3];
        // Per-wheel longitudinal slip ratio: 0 = free rolling, +1 = locked,
        // -1 = spinning.  Same FL/FR/RL/RR order as wheel_omega.
        else if (f == "slip_ratio_fl")        m_csv << state.slip_ratio[0];
        else if (f == "slip_ratio_fr")        m_csv << state.slip_ratio[1];
        else if (f == "slip_ratio_rl")        m_csv << state.slip_ratio[2];
        else if (f == "slip_ratio_rr")        m_csv << state.slip_ratio[3];
        // BTCM-side bus state — captures what the controller is commanding,
        // independent of what the physics is doing.  Useful for annotating
        // ABS events on a wheel-speed graph.
        //   abs_phase_fl/fr:  0=APPLY (full pressure), 1=HOLD (frozen),
        //                     2=DUMP (release).  Only meaningful when the
        //                     bus value is fresh; -1 when BTCM is silent.
        //   abs_fresh_fl/fr:  1 when the BTCM is alive per the freshness
        //                     window (3000 ms, matching the control path).
        //   emb_cmd_lr/rr:    rear motor command in [-1, +1].  +1=full apply.
        //   emb_fresh_lr/rr:  1 when the BTCM is alive per the freshness
        //                     window (3000 ms, matching the control path).
        else if (f == "abs_phase_fl")
            m_csv << (bus_abs.fl_fresh ? phase_to_int(bus_abs.fl) : -1);
        else if (f == "abs_phase_fr")
            m_csv << (bus_abs.fr_fresh ? phase_to_int(bus_abs.fr) : -1);
        else if (f == "abs_fresh_fl")         m_csv << (bus_abs.fl_fresh ? 1 : 0);
        else if (f == "abs_fresh_fr")         m_csv << (bus_abs.fr_fresh ? 1 : 0);
        else if (f == "emb_cmd_lr")           m_csv << bus_rear.lr;
        else if (f == "emb_cmd_rr")           m_csv << bus_rear.rr;
        else if (f == "emb_fresh_lr")         m_csv << (bus_rear.lr_fresh ? 1 : 0);
        else if (f == "emb_fresh_rr")         m_csv << (bus_rear.rr_fresh ? 1 : 0);
        // IPC telltale states mirrored from the chassis bus (IDs 4134
        // brake / 4136 antilock / 4138 air-bag / 4140 service-now /
        // 4141 check-messages / 4144 reduced-perf) —
        // lets acceptance scenarios assert the driver-facing indication
        // timeline (e.g. brake telltale lighting after a BTCM death, the
        // reduced-perf latch after a pedal-feed loss). Latched IPC state:
        // false until first received.
        else if (f == "ipc_brake_telltale")
            m_csv << (bus.GetIpcBrakeTelltale() ? 1 : 0);
        else if (f == "ipc_antilock_telltale")
            m_csv << (bus.GetIpcAntilockTelltale() ? 1 : 0);
        else if (f == "ipc_service_now_telltale")
            m_csv << (bus.GetIpcServiceNowTelltale() ? 1 : 0);
        else if (f == "ipc_reduced_perf_telltale")
            m_csv << (bus.GetIpcReducedPerfTelltale() ? 1 : 0);
        // AIR BAG (4138) is the IPC's SIR/SDM occupant-restraint telltale —
        // lit on airbag-system faults (IPC DTC 40) — the driver-facing
        // indication the SIR acceptance scenarios assert on.
        else if (f == "ipc_air_bag_telltale")
            m_csv << (bus.GetIpcAirBagTelltale() ? 1 : 0);
        // CHECK MESSAGES (4141) is the IPC's aggregate comm-loss telltale —
        // the one that lights when a peer's UART stream dies (e.g. IPC
        // DTC 15 on BTCM eavesdrop loss), so it's the loss-of-module
        // indicator the safety scenarios assert on.
        else if (f == "ipc_check_messages_telltale")
            m_csv << (bus.GetIpcCheckMessagesTelltale() ? 1 : 0);
        // Auto Disconnect HV status mirrored from the main harness (5224
        // main contactor / 5225 precharge relay / 5230 state enum) — lets
        // acceptance scenarios assert the HV power-up sequence (precharge
        // then main) and its failure modes (precharge-timeout latch with
        // the main contactor never closing). 0 until first received.
        else if (f == "ad_main_contactor_closed")
            m_csv << (bus.GetAdMainContactorClosed() ? 1 : 0);
        else if (f == "ad_precharge_relay_closed")
            m_csv << (bus.GetAdPrechargeRelayClosed() ? 1 : 0);
        else if (f == "ad_state_enum")
            m_csv << bus.GetAdStateEnum();
        // Latched precharge participation (derived from 5225) — sticky-true
        // once the precharge relay has ever been observed closed. Alias-proofs
        // a brief relay-closed transient the periodic sampler could step over,
        // so the HV power-up assertion is witnessed regardless of the CSV
        // sample-grid alignment. 0 until the relay is first seen closed.
        else if (f == "ad_precharge_participated")
            m_csv << (bus.GetAdPrechargeParticipated() ? 1 : 0);
        // HVAC blower level mirrored from the chassis bus (4082) — the
        // commanded fan speed enum (0=OFF, 1=LOW, 2=MED, 3=HIGH). Cast to int
        // so it prints as a number, not the char of a small uint8_t.
        //
        // BEFORE THE FIRST 4082 ARRIVES this reports OFF (0), not the raw
        // cache. ExternalSimConnector seeds hvac_blower_level to 0xFF as its
        // "nothing received yet" marker, and 255 is OUTSIDE this field's own
        // declared 0..3 enum — so emitting the raw cache published a value the
        // column's own contract says cannot occur. Measured on the 2026-07-31
        // electricsim nightly: 255 from t=0.017 s to t=1.241 s, while the
        // RSA->RHJB-PMM->HTCM key-on chain settled, which turned that run's
        // actuator-range rule (max(hvac_blower_level) <= 3) red on a number no
        // module ever commanded. The HTCM cannot be the source — it clamps its
        // own setpoint to 3 (ev1/htcm tests: "level 255 clamped to 3").
        //
        // Reporting OFF matches the documented convention of every neighbouring
        // mirrored field above ("false until first received", "0 until first
        // received") and is physically honest for a COMMAND: no command
        // received yet is not commanding the fan. Deliberately NOT emitted as
        // an empty cell — an empty cell is already this writer's "unknown field
        // name" output (see the final else), and electricsim's VAT keys its
        // never-produced-column diagnostic on all-blank columns, so a blank
        // here would make "the signal never arrived" indistinguishable from
        // "this scenario named a field that does not exist".
        else if (f == "hvac_blower_level")
            m_csv << (bus.HasReceivedHvacBlowerLevel()
                          ? static_cast<int>(bus.GetHvacBlowerLevel())
                          : 0);
        // ── Door locks: the relay legs, the motors, and the latch ────────────
        //
        // Three layers, deliberately distinct columns, because collapsing them
        // is how a lock scenario ends up unable to fail:
        //
        //   *_drive  — the RELAY LEG as the junction block published it
        //              (4182 LH lock / 4183 LH unlock / 4184 RH lock /
        //              4185 RH unlock).  1 = that leg is energised.  Raw bus
        //              mirror; says nothing about whether a motor turned.
        //   (no suffix) — the MOTOR WINDING, from ev1sim's door_lock_motor
        //              plant.  The two relay commons sit across one reversible
        //              motor, so a winding is live only when its leg is driven
        //              AND the opposite leg is not; both legs driven puts both
        //              brushes on the same rail and the motor sits still.  This
        //              is what the body acceptance criteria score.
        //   *_stroke / door_lock_state_* — the MECHANICS: normalized pawl
        //              travel (0 = unlocked stop, 1 = locked stop) and the
        //              latched per-door result.  A drive that never moves the
        //              pawl shows up here as a flat stroke.
        //
        // A scenario's own lock/unlock press appears in NONE of these — it goes
        // out on the switch contacts (4170-4173) and everything below is what
        // came back.
        else if (f == "door_lock_motor_lh_lock_drive")
            m_csv << (bus.GetDoorLockMotorDrive(0) ? 1 : 0);
        else if (f == "door_lock_motor_lh_unlock_drive")
            m_csv << (bus.GetDoorLockMotorDrive(1) ? 1 : 0);
        else if (f == "door_lock_motor_rh_lock_drive")
            m_csv << (bus.GetDoorLockMotorDrive(2) ? 1 : 0);
        else if (f == "door_lock_motor_rh_unlock_drive")
            m_csv << (bus.GetDoorLockMotorDrive(3) ? 1 : 0);
        else if (f == "door_lock_motor_lh_lock")
            m_csv << (physical.door_lock_motor_lh().lock_energised() ? 1 : 0);
        else if (f == "door_lock_motor_lh_unlock")
            m_csv << (physical.door_lock_motor_lh().unlock_energised() ? 1 : 0);
        else if (f == "door_lock_motor_rh_lock")
            m_csv << (physical.door_lock_motor_rh().lock_energised() ? 1 : 0);
        else if (f == "door_lock_motor_rh_unlock")
            m_csv << (physical.door_lock_motor_rh().unlock_energised() ? 1 : 0);
        // Pawl travel, 0..1.  The one column that distinguishes "the motor was
        // energised" from "the lock actually moved".
        else if (f == "door_lock_stroke_lh")
            m_csv << physical.door_lock_motor_lh().position();
        else if (f == "door_lock_stroke_rh")
            m_csv << physical.door_lock_motor_rh().position();
        // Winding live but the pawl is already against its stop — the real
        // motor stalls there for the rest of the pulse, so a drive that is
        // stalled for seconds is the held-drive failure this case hunts.
        else if (f == "door_lock_motor_lh_stalled")
            m_csv << (physical.door_lock_motor_lh().stalled() ? 1 : 0);
        else if (f == "door_lock_motor_rh_stalled")
            m_csv << (physical.door_lock_motor_rh().stalled() ? 1 : 0);
        // Latched per-door state after the stroke reached an end of travel —
        // the same value ev1sim feeds back to the bus on 4165/4166.
        else if (f == "door_lock_state_driver")
            m_csv << (physical.door_locks().driver() ==
                          DoorLocks::State::LOCKED ? 1 : 0);
        else if (f == "door_lock_state_passenger")
            m_csv << (physical.door_locks().passenger() ==
                          DoorLocks::State::LOCKED ? 1 : 0);
        // The driver stimulus itself (4170-4173), so a run that produced no
        // motor drive can be read as "the press never went out" vs "the
        // junction block ignored it".
        else if (f == "door_lock_switch_lh_lock")
            m_csv << (bus.GetDoorLockSwitchContact(0) ? 1 : 0);
        else if (f == "door_lock_switch_lh_unlock")
            m_csv << (bus.GetDoorLockSwitchContact(1) ? 1 : 0);
        // Two-tone horn commands (1 = commanded sounding, 0 otherwise).
        else if (f == "horn_low_cmd")
            m_csv << (bus.GetHornLowCmd() ? 1 : 0);
        else if (f == "horn_high_cmd")
            m_csv << (bus.GetHornHighCmd() ? 1 : 0);
        // RSA power mode as ev1sim resolves it from RSA_RUN1_OUT: 0 = OFF,
        // 2 = RUN.  ev1sim cannot see the ACC detent (the RSA's ACC output has
        // no wire cell), so this column is a two-state "is the RSA commanding
        // run mode" readout, NOT the full OFF/ACC/RUN/START enum.
        else if (f == "rsa_run_mode")
            m_csv << static_cast<int>(bus.GetRsaRunMode());
        // RSA shift-blocked cue (4088): 1 while a P→non-P selector request is
        // refused because the brake switch is open — the BTSI (brake/
        // transmission shift interlock) saying no.
        else if (f == "rsa_shift_blocked")
            m_csv << (bus.GetRsaShiftBlocked() ? 1 : 0);
        // PARKING BRAKE telltale (4135, IPC ← BTCM park_brake_ind).
        // @source:manual brakes-314: "The PARKING BRAKE light will illuminate
        // when the park brake is applied."  This is the driver-facing witness
        // that the rear electric park brake latched.
        else if (f == "ipc_park_brake_telltale")
            m_csv << (bus.GetIpcParkBrakeTelltale() ? 1 : 0);
        // RSA door-lock commands (4084/4085): 0 = unlocked, 1 = locked,
        // 0xFF = never received.  Cast so a uint8 prints as a number.
        else if (f == "door_lock_cmd_driver")
            m_csv << static_cast<int>(bus.GetDoorLockCmd(0));
        else if (f == "door_lock_cmd_passenger")
            m_csv << static_cast<int>(bus.GetDoorLockCmd(1));
        // BTCM retard-request PWM duty (4191), Q8 percent 0..25600 — regen
        // torque the BTCM is asking the propulsion side to produce.
        else if (f == "btcm_retard_request_duty_q8")
            m_csv << static_cast<int>(bus.GetBtcmRetardRequestDutyQ8());
        // TJB rear-lamp branches, downstream of the trunk junction block's
        // RUN1 gate (4198/4199 tail, 4203/4204 stop).
        else if (f == "tjb_lr_tail_lamp")  m_csv << (bus.GetTjbLampBranch(0) ? 1 : 0);
        else if (f == "tjb_rr_tail_lamp")  m_csv << (bus.GetTjbLampBranch(1) ? 1 : 0);
        else if (f == "tjb_lr_stop_lamp")  m_csv << (bus.GetTjbLampBranch(2) ? 1 : 0);
        else if (f == "tjb_rr_stop_lamp")  m_csv << (bus.GetTjbLampBranch(3) ? 1 : 0);
        // Sounder / piezo drive (LHJB flasher module).
        else if (f == "sounder_piezo_drive")
            m_csv << (bus.GetSounderPiezoDrive() ? 1 : 0);
        // Bulb feed lines by EV1-manual short name — "<abbrev>_bulb_feed_line"
        // (e.g. lhbh_bulb_feed_line, lrsl_bulb_feed_line): 1 = the junction
        // block is commanding the bulb lit. Covers all NUM_LIGHTS elements so
        // lighting/HMI scenarios can assert indication timelines (turn
        // signal flash cadence, stop lamps on brake) without bespoke
        // per-bulb plumbing. 0 until a bulb command is first received.
        else if (f.size() > 15 &&
                 f.compare(f.size() - 15, 15, "_bulb_feed_line") == 0) {
            bool matched = false;
            const std::string abbrev = f.substr(0, f.size() - 15);
            for (int i = 0; i < NUM_LIGHTS; ++i) {
                std::string name = LightIDName(static_cast<LightID>(i));
                for (auto& ch : name) ch = static_cast<char>(std::tolower(
                    static_cast<unsigned char>(ch)));
                if (name == abbrev) {
                    m_csv << (bus.GetBulbCmd(static_cast<LightID>(i)) ? 1 : 0);
                    matched = true;
                    break;
                }
            }
            if (!matched) m_csv << "";  // unknown bulb abbrev
        }
        else                                  m_csv << "";  // unknown field
    };

    bool first = true;
    for (const auto& f : m_stats.fields) {
        if (!first) m_csv << ",";
        write_field(f);
        first = false;
    }
    m_csv << "\n";
}

void Scenario::Close() {
    if (m_csv_opened) {
        m_csv.flush();
        m_csv.close();
        m_csv_opened = false;
        std::cout << "[Scenario] stats CSV closed\n";
    }
}

}  // namespace ev1sim
