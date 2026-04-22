#pragma once

#include <array>

// Snapshot of vehicle state for telemetry and future external consumers.
// Uses plain types (no Chrono/Irrlicht dependencies) so this struct can
// cross process boundaries or be serialized without pulling in the engine.
struct VehicleState {
    double sim_time = 0.0;          // seconds

    // Chassis motion
    double speed_mps     = 0.0;     // forward speed (m/s)
    double steering_angle = 0.0;    // rad
    double yaw_rate      = 0.0;     // rad/s
    double roll_rate     = 0.0;     // rad/s

    // Chassis pose (world frame)
    double pos_x   = 0.0;
    double pos_y   = 0.0;
    double pos_z   = 0.0;
    double yaw_deg = 0.0;

    // Chassis acceleration (chassis frame)
    double accel_long = 0.0;        // longitudinal (m/s^2)
    double accel_lat  = 0.0;        // lateral (m/s^2)
    double accel_vert = 0.0;        // vertical (m/s^2)

    // Wheel angular speeds (rad/s): FL, FR, RL, RR
    std::array<double, 4> wheel_omega = {0, 0, 0, 0};

    // Per-wheel brake command (0..1), FL, FR, RL, RR.  Currently mirrors
    // front/rear axle values; future ABS/ESC logic will produce distinct
    // per-wheel values that feed per-wheel brake torque directly.
    std::array<double, 4> brake_cmd = {0, 0, 0, 0};

    // Echo of applied commands (for logging/diagnostics)
    double applied_throttle     = 0.0;
    double applied_front_brake  = 0.0;
    double applied_rear_brake   = 0.0;
    double applied_steering     = 0.0;
};
