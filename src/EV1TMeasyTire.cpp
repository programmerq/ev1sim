#include "EV1TMeasyTire.h"

#include "chrono/functions/ChFunctionSineStep.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace ev1sim {

EV1TMeasyTire::EV1TMeasyTire(const std::string& filename)
    : chrono::vehicle::TMeasyTire(filename) {
    std::ifstream f(filename);
    if (!f.is_open())
        throw std::runtime_error("EV1TMeasyTire: cannot open " + filename);
    const nlohmann::json j = nlohmann::json::parse(f);
    if (j.contains("Longitudinal Tire Dynamics")) {
        const auto& d = j["Longitudinal Tire Dynamics"];
        m_cx = d.at("Stiffness [N/m]").get<double>();
        m_dx = d.at("Damping [Ns/m]").get<double>();
        // dx > 0, not >= 0: at zero slip fos = 0, and dx is then the whole
        // denominator of xe_dot.
        if (!(m_cx > 0.0) || !(m_dx > 0.0))
            throw std::runtime_error(
                "EV1TMeasyTire: Longitudinal Tire Dynamics needs Stiffness > 0 "
                "and Damping > 0 in " + filename);
    }
    if (j.contains("Longitudinal Grip Peak")) {
        const auto& g = j["Longitudinal Grip Peak"];
        m_grip.friction = g.at("Friction").get<std::vector<double>>();
        m_grip.peak_slip = g.at("Peak Slip").get<std::vector<double>>();
        m_grip.validate();
    }
}

void EV1TMeasyTire::Advance(double step) {
    // Steady-state TMeasy force, Dahl bristle update, moments: all Chrono's.
    ChTMeasyTire::Advance(step);

    const bool dynamics = m_cx > 0.0;
    if (!m_data.in_contact || (!dynamics && m_grip.empty())) {
        m_xe = 0.0;  // tread relaxes while unloaded
        return;
    }

    // Re-derive the TMeasy (slip-curve) share of Fx and its secant slope
    // fos = f/s, exactly as ChTMeasyTire::Advance does (Chrono 9.0.1,
    // ChTMeasyTire.cpp "TMeasy horizontal forces").  Pure reads of the
    // states Synchronize computed; nothing here mutates base state.
    const double sx = m_states.sx;
    const double sy = m_states.sy;
    const double sc = std::hypot(sx, sy);
    double calpha, salpha;
    if (sc > 0.0) {
        calpha = sx / sc;
        salpha = sy / sc;
    } else {
        calpha = std::sqrt(2.0) / 2.0;
        salpha = std::sqrt(2.0) / 2.0;
    }
    const double mu = m_states.muscale;
    const double df0 = std::hypot(m_states.dfx0 * calpha, m_states.dfy0 * salpha);
    const double fm = mu * std::hypot(m_states.fxm * calpha, m_states.fym * salpha);
    const double sm = mu * std::hypot(m_states.sxm * calpha, m_states.sym * salpha);
    const double fs = mu * std::hypot(m_states.fxs * calpha, m_states.fys * salpha);
    const double ss = mu * std::hypot(m_states.sxs * calpha, m_states.sys * salpha);
    double f = 0.0, fos = 0.0;
    tmxy_combined(f, fos, sc, df0, sm, fm, ss, fs);
    const double Fx_ss = (sc > 0.0) ? f * calpha : 0.0;  // = fos*sx = -ks*vsx

    // The grip peak where published curves put it (TireGripPeak.h): the
    // longitudinal peak and sliding-onset slips move by the table's factor
    // instead of by road friction; forces, the lateral slips and the initial
    // slope stay as TMeasy scaled them.
    double Fx_peak = Fx_ss, fos_peak = fos;
    if (!m_grip.empty()) {
        // muscale is road friction over m_par.mu_0, the friction the slip
        // parameters were written at ("Coefficient of Friction").
        const double r = m_grip.slip_scale(mu * m_par.mu_0, m_par.mu_0);
        const double smp = std::hypot(r * m_states.sxm * calpha, mu * m_states.sym * salpha);
        const double ssp = std::hypot(r * m_states.sxs * calpha, mu * m_states.sys * salpha);
        double fp = 0.0;
        tmxy_combined(fp, fos_peak, sc, df0, smp, fm, ssp, fs);
        Fx_peak = (sc > 0.0) ? fp * calpha : 0.0;
    }

    // Series spring-damper (Rill's first-order tire dynamics), semi-implicit
    // in xe as Chrono 8's integration_method 2 ("absolutely stable").
    double Fx_dyn = Fx_peak;
    if (dynamics) {
        const double ks = fos_peak / m_states.vta;  // N per (m/s) of sliding speed
        const double denom = m_dx + ks;
        const double xe_dot =
            (Fx_peak - m_cx * m_xe) / denom / (1.0 + step * m_cx / denom);
        m_xe += step * xe_dot;
        Fx_dyn = m_cx * m_xe + m_dx * xe_dot;
    }

    // Swap the TMeasy share of the base's blended force for this one, with the same friction-blend and startup weights the base
    // applied: force.x = startup * ((1-fb)*Fx_dahl + fb*Fx_ss).
    const double fb = chrono::ChFunctionSineStep::Eval(
        m_data.vel.x(), m_frblend_begin, 0.0, m_frblend_end, 1.0);
    double startup = 1.0;
    if (m_use_startup_transition)
        startup = chrono::ChFunctionSineStep::Eval(
            m_time, m_begin_start_transition, 0.0, m_end_start_transition, 1.0);
    m_tireforce.force.x() += startup * fb * (Fx_dyn - Fx_ss);
}

std::shared_ptr<EV1TMeasyTire> ReadEV1TireJSON(const std::string& filename) {
    return std::make_shared<EV1TMeasyTire>(filename);
}

}  // namespace ev1sim
