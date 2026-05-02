#include "Scenario.h"

#include <chrono>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "ExternalSimConnector.h"

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
              << "max_time=" << s.m_max_time_s << " s)\n";
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
        } else {
            std::cerr << "[Scenario] unknown action '" << e.action
                      << "' — skipping\n";
        }
    }

    if (m_held_throttle) cmd.throttle    = *m_held_throttle;
    if (m_held_brake)    cmd.front_brake = cmd.rear_brake = *m_held_brake;
    if (m_held_steering) cmd.steering    = *m_held_steering;
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
    const auto bus_abs    = bus.GetAbsPhaseFront(std::chrono::milliseconds(200));
    const auto bus_rear   = bus.GetRearEmbCmd(std::chrono::milliseconds(200));

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
        else if (f == "applied_throttle")     m_csv << applied_cmd.throttle;
        else if (f == "applied_front_brake")  m_csv << applied_cmd.front_brake;
        else if (f == "applied_rear_brake")   m_csv << applied_cmd.rear_brake;
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
        //   abs_fresh_fl/fr:  1 when BTCM data arrived within the last 200 ms.
        //   emb_cmd_lr/rr:    rear motor command in [-1, +1].  +1=full apply.
        //   emb_fresh_lr/rr:  1 when BTCM data arrived within the last 200 ms.
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
