#include "Telemetry.h"

#include <cstdio>
#include <iomanip>
#include <iostream>
#include <sstream>

Telemetry::Telemetry(double log_rate_hz, bool log_to_file,
                     const std::string& log_file, bool show_hud)
    : m_log_interval(log_rate_hz > 0 ? 1.0 / log_rate_hz : 0.1),
      m_show_hud(show_hud) {
    if (log_to_file) {
        m_file.open(log_file);
        if (!m_file.is_open())
            std::cerr << "[Telemetry] Cannot open " << log_file << "\n";
    }
}

Telemetry::~Telemetry() = default;

// ---------------------------------------------------------------------------
void Telemetry::Record(const VehicleState& s, double dt) {
    m_accum += dt;
    if (m_accum < m_log_interval) return;
    m_accum = 0.0;

    // CSV header (once).
    if (m_file.is_open() && !m_file_header_written) {
        m_file << "sim_time,speed_mps,throttle,front_brake,rear_brake,"
                  "steering,pos_x,pos_y,pos_z,yaw_deg,"
                  "wheel_fl_omega,wheel_fr_omega,wheel_rl_omega,wheel_rr_omega\n";
        m_file_header_written = true;
    }

    // Console log.
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "[%.2fs] spd=%.1f m/s  thr=%.2f  brk=%.2f/%.2f  str=%+.2f  "
        "pos=(%.1f,%.1f,%.1f)  yaw=%.1f",
        s.sim_time, s.speed_mps,
        s.applied_throttle,
        s.applied_front_brake, s.applied_rear_brake,
        s.applied_steering,
        s.pos_x, s.pos_y, s.pos_z, s.yaw_deg);
    std::cout << buf << "\n";

    // CSV row.
    if (m_file.is_open()) {
        m_file << s.sim_time << ","
               << s.speed_mps << ","
               << s.applied_throttle << ","
               << s.applied_front_brake << ","
               << s.applied_rear_brake << ","
               << s.applied_steering << ","
               << s.pos_x << "," << s.pos_y << "," << s.pos_z << ","
               << s.yaw_deg << ","
               << s.wheel_omega[0] << "," << s.wheel_omega[1] << ","
               << s.wheel_omega[2] << "," << s.wheel_omega[3] << "\n";
    }
}

// ---------------------------------------------------------------------------
void Telemetry::DrawHUD(irr::IrrlichtDevice* device,
                        const VehicleState& state,
                        const std::string& camera_mode,
                        const std::string& surface,
                        const double* wheel_mu) {
    if (!m_show_hud || !device) return;

    auto* gui  = device->getGUIEnvironment();
    auto* font = gui->getBuiltInFont();
    if (!font) return;

    irr::video::SColor white(255, 255, 255, 255);
    irr::video::SColor yellow(255, 255, 255, 100);

    int x = 10, y = 10, h = 18;

    auto draw = [&](const std::string& text, irr::video::SColor col = {255, 255, 255, 255}) {
        irr::core::stringw ws(text.c_str());
        font->draw(ws, irr::core::rect<irr::s32>(x, y, x + 400, y + h), col);
        y += h;
    };

    char buf[128];
    double kph = state.speed_mps * 3.6;
    std::snprintf(buf, sizeof(buf), "Speed: %.1f km/h  (%.1f m/s)", kph, state.speed_mps);
    draw(buf, white);

    std::snprintf(buf, sizeof(buf), "Throttle: %.0f%%   Brake: %.0f%%   Steer: %+.0f%%",
                  state.applied_throttle * 100, state.applied_front_brake * 100,
                  state.applied_steering * 100);
    draw(buf, white);

    draw("Camera: " + camera_mode, yellow);
    draw("Surface: " + surface, yellow);

    // Per-wheel ground friction — makes split-mu and transition levels
    // legible at a glance (e.g. asphalt wheels at 0.90 vs ice at 0.08).
    if (wheel_mu) {
        char mu_buf[160];
        auto fmt = [](double v, char* out, size_t n) {
            if (v < 0) std::snprintf(out, n, "  -  ");
            else       std::snprintf(out, n, "%.2f", v);
        };
        char fl[8], fr[8], rl[8], rr[8];
        fmt(wheel_mu[0], fl, sizeof(fl));
        fmt(wheel_mu[1], fr, sizeof(fr));
        fmt(wheel_mu[2], rl, sizeof(rl));
        fmt(wheel_mu[3], rr, sizeof(rr));
        std::snprintf(mu_buf, sizeof(mu_buf),
                      "mu:  FL=%s  FR=%s  RL=%s  RR=%s",
                      fl, fr, rl, rr);
        draw(mu_buf, yellow);
    }

    std::snprintf(buf, sizeof(buf), "Sim time: %.1f s", state.sim_time);
    draw(buf, yellow);
}
