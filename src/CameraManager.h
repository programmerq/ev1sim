#pragma once

#include "VehiclePose.h"

#include <irrlicht.h>

#include <string>

enum class CameraMode { Chase, Hood, Rear, TopDown, FreeLook, COUNT };

class CameraManager : public irr::IEventReceiver {
public:
    CameraManager(irr::scene::ICameraSceneNode* camera,
                  double chase_distance, double chase_height);

    void CycleMode();
    CameraMode GetMode() const { return m_mode; }
    std::string GetModeName() const;

    /// Adjust camera distance.  Positive = zoom out, negative = zoom in.
    void Zoom(double delta);

    // Call after vis->Advance() and before vis->BeginScene() each frame.
    void Update(const VehiclePose& pose);

    // IEventReceiver — handles mouse input for free-look orbit.
    bool OnEvent(const irr::SEvent& event) override;

    // Set the initial mode from a config string.
    void SetModeFromString(const std::string& name);

private:
    irr::scene::ICameraSceneNode* m_camera;
    CameraMode m_mode = CameraMode::Chase;

    // Chase / follow parameters
    double m_chase_dist   = 6.0;
    double m_chase_height = 2.0;

    // Zoom multiplier (applied to chase/topdown/freelook distances).
    double m_zoom = 1.0;

    // Free-look orbit state
    double m_orbit_yaw   = 0.0;     // degrees
    double m_orbit_pitch = 25.0;    // degrees
    double m_orbit_dist  = 15.0;

    // Mouse tracking
    int  m_last_mx = 0, m_last_my = 0;
    bool m_dragging = false;

    void ApplyChase(const VehiclePose& p);
    void ApplyHood(const VehiclePose& p);
    void ApplyRear(const VehiclePose& p);
    void ApplyTopDown(const VehiclePose& p);
    void ApplyFreeLook(const VehiclePose& p);
};
