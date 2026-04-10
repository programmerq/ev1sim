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
                  "steering,pos_x,pos_y,pos_z,yaw_deg\n";
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
               << s.yaw_deg << "\n";
    }
}

// ---------------------------------------------------------------------------
void Telemetry::DrawHUD(irr::IrrlichtDevice* device,
                        const VehicleState& state,
                        const std::string& camera_mode,
                        const std::string& surface) {
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

    std::snprintf(buf, sizeof(buf), "Sim time: %.1f s", state.sim_time);
    draw(buf, yellow);
}
