#include "CameraManager.h"

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static constexpr double kDeg2Rad = M_PI / 180.0;

// Helper: convert VehiclePose components to Irrlicht vector.
// Chrono Irrlicht scenes use the same axis convention as Chrono (Z-up).
static irr::core::vector3df v3(double x, double y, double z) {
    return irr::core::vector3df(
        static_cast<irr::f32>(x),
        static_cast<irr::f32>(y),
        static_cast<irr::f32>(z));
}

// ---------------------------------------------------------------------------
CameraManager::CameraManager(irr::scene::ICameraSceneNode* camera,
                             double chase_distance, double chase_height)
    : m_camera(camera),
      m_chase_dist(chase_distance),
      m_chase_height(chase_height) {}

// ---------------------------------------------------------------------------
void CameraManager::CycleMode() {
    int next = (static_cast<int>(m_mode) + 1) % static_cast<int>(CameraMode::COUNT);
    m_mode = static_cast<CameraMode>(next);
}

std::string CameraManager::GetModeName() const {
    switch (m_mode) {
        case CameraMode::Chase:    return "Chase";
        case CameraMode::Hood:     return "Hood";
        case CameraMode::Rear:     return "Rear";
        case CameraMode::TopDown:  return "Top-Down";
        case CameraMode::FreeLook: return "Free-Look";
        default:                   return "Unknown";
    }
}

void CameraManager::Zoom(double delta) {
    m_zoom = std::clamp(m_zoom + delta, 0.2, 20.0);
}

void CameraManager::SetModeFromString(const std::string& name) {
    if (name == "chase")     m_mode = CameraMode::Chase;
    else if (name == "hood")     m_mode = CameraMode::Hood;
    else if (name == "rear")     m_mode = CameraMode::Rear;
    else if (name == "topdown")  m_mode = CameraMode::TopDown;
    else if (name == "freelook") m_mode = CameraMode::FreeLook;
    // else keep default
}

// ---------------------------------------------------------------------------
void CameraManager::Update(const VehiclePose& pose) {
    switch (m_mode) {
        case CameraMode::Chase:    ApplyChase(pose);    break;
        case CameraMode::Hood:     ApplyHood(pose);     break;
        case CameraMode::Rear:     ApplyRear(pose);     break;
        case CameraMode::TopDown:  ApplyTopDown(pose);  break;
        case CameraMode::FreeLook: ApplyFreeLook(pose); break;
        default: break;
    }
}

// ---------------------------------------------------------------------------
// Camera modes
// ---------------------------------------------------------------------------

void CameraManager::ApplyChase(const VehiclePose& p) {
    // Behind and above the vehicle, looking at it.
    double dist   = m_chase_dist   * m_zoom;
    double height = m_chase_height * m_zoom;
    auto pos = v3(p.px - p.fx * dist + p.ux * height,
                  p.py - p.fy * dist + p.uy * height,
                  p.pz - p.fz * dist + p.uz * height);
    auto tgt = v3(p.px, p.py, p.pz);
    auto up  = v3(p.ux, p.uy, p.uz);

    m_camera->setPosition(pos);
    m_camera->setTarget(tgt);
    m_camera->setUpVector(up);
}

void CameraManager::ApplyHood(const VehiclePose& p) {
    // On the hood, looking forward.  Slight height offset above chassis.
    double hood_fwd = 1.2;
    double hood_up  = 0.8;
    auto pos = v3(p.px + p.fx * hood_fwd + p.ux * hood_up,
                  p.py + p.fy * hood_fwd + p.uy * hood_up,
                  p.pz + p.fz * hood_fwd + p.uz * hood_up);
    auto tgt = v3(p.px + p.fx * 20.0,
                  p.py + p.fy * 20.0,
                  p.pz + p.fz * 20.0);
    auto up  = v3(p.ux, p.uy, p.uz);

    m_camera->setPosition(pos);
    m_camera->setTarget(tgt);
    m_camera->setUpVector(up);
}

void CameraManager::ApplyRear(const VehiclePose& p) {
    // Behind the vehicle, looking backward.
    double rear_back = 1.5;
    double rear_up   = 1.0;
    auto pos = v3(p.px - p.fx * rear_back + p.ux * rear_up,
                  p.py - p.fy * rear_back + p.uy * rear_up,
                  p.pz - p.fz * rear_back + p.uz * rear_up);
    auto tgt = v3(p.px - p.fx * 20.0,
                  p.py - p.fy * 20.0,
                  p.pz - p.fz * 20.0);
    auto up  = v3(p.ux, p.uy, p.uz);

    m_camera->setPosition(pos);
    m_camera->setTarget(tgt);
    m_camera->setUpVector(up);
}

void CameraManager::ApplyTopDown(const VehiclePose& p) {
    double height = 30.0 * m_zoom;
    auto pos = v3(p.px + p.ux * height,
                  p.py + p.uy * height,
                  p.pz + p.uz * height);
    auto tgt = v3(p.px, p.py, p.pz);
    // Use forward as "up" so the view is oriented with the vehicle.
    auto up  = v3(p.fx, p.fy, p.fz);

    m_camera->setPosition(pos);
    m_camera->setTarget(tgt);
    m_camera->setUpVector(up);
}

void CameraManager::ApplyFreeLook(const VehiclePose& p) {
    // Orbit camera around the vehicle controlled by mouse.
    double yaw_rad   = m_orbit_yaw * kDeg2Rad;
    double pitch_rad = m_orbit_pitch * kDeg2Rad;

    double cp = std::cos(pitch_rad);
    double sp = std::sin(pitch_rad);
    double cy = std::cos(yaw_rad);
    double sy = std::sin(yaw_rad);

    // Offset in world frame (Z-up convention).
    double dist = m_orbit_dist * m_zoom;
    double ox = dist * cp * cy;
    double oy = dist * cp * sy;
    double oz = dist * sp;

    auto pos = v3(p.px + ox, p.py + oy, p.pz + oz);
    auto tgt = v3(p.px, p.py, p.pz);
    auto up  = v3(0, 0, 1);  // world Z-up

    m_camera->setPosition(pos);
    m_camera->setTarget(tgt);
    m_camera->setUpVector(up);
}

// ---------------------------------------------------------------------------
// Mouse event handling (free-look only)
// ---------------------------------------------------------------------------
bool CameraManager::OnEvent(const irr::SEvent& event) {
    if (event.EventType != irr::EET_MOUSE_INPUT_EVENT)
        return false;

    // Only consume mouse events in free-look mode.
    if (m_mode != CameraMode::FreeLook)
        return false;

    const auto& me = event.MouseInput;

    switch (me.Event) {
        case irr::EMIE_LMOUSE_PRESSED_DOWN:
            m_dragging = true;
            m_last_mx = me.X;
            m_last_my = me.Y;
            return true;

        case irr::EMIE_LMOUSE_LEFT_UP:
            m_dragging = false;
            return true;

        case irr::EMIE_MOUSE_MOVED:
            if (m_dragging) {
                int dx = me.X - m_last_mx;
                int dy = me.Y - m_last_my;
                m_orbit_yaw   += dx * 0.3;
                m_orbit_pitch  = std::clamp(m_orbit_pitch - dy * 0.3, -89.0, 89.0);
            }
            m_last_mx = me.X;
            m_last_my = me.Y;
            return m_dragging;

        case irr::EMIE_MOUSE_WHEEL:
            m_orbit_dist = std::clamp(m_orbit_dist - me.Wheel * 2.0, 2.0, 100.0);
            return true;

        default:
            return false;
    }
}
