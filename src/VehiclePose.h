#pragma once

// Lightweight chassis pose for camera positioning.
// Framework-agnostic (no Chrono or Irrlicht types).
struct VehiclePose {
    double px = 0, py = 0, pz = 0;   // position (world)
    double fx = 1, fy = 0, fz = 0;   // forward direction (unit)
    double ux = 0, uy = 0, uz = 1;   // up direction (unit)
};
