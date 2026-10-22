#pragma once

#include "DashScreens3D.h"  // DashVec3, ChassisToWorld
#include "VehiclePose.h"

#include <string>

// ---------------------------------------------------------------------------
// TrunkLid3D — the trunk lid as its own mesh, swung open about its hinge when
// the trunk is open (VehiclePanels' TRUNK state, the T key / panel button).
//
// The lid (ev1_trunk_lid.obj) is split out of the chassis visual mesh by
// ev1model/split_trunk_lid.py, in the same chassis-frame coordinates, so at
// open fraction 0 it sits exactly where it was in the body.
//
// Split like WiperBlades3D: the hinge and travel are pure chassis-frame math
// (no Irrlicht), and TrunkLid3D owns the scene node.  Chassis frame as
// VehiclePose: x forward, y left, z up.
// ---------------------------------------------------------------------------

namespace irr {
class IrrlichtDevice;
namespace scene { class IMeshSceneNode; }
}  // namespace irr

namespace ev1sim {

struct TrunkLidGeometry {
    double hinge_x, hinge_z;  // hinge line, parallel to the chassis y axis
    double open_deg;          // lid angle fully open, rear edge up
    double travel_s;          // closed to fully open (and back)
};

// @inferred 2026-10-22 claude: hinge at the lid's front edge as the visual
// mesh draws it (front edge x -1.300, z 0.942..0.956).
// @design 2026-10-22 claude: open angle and travel time. The manuals print
// neither, so this is a usual gas-strut decklid: 55 degrees in one second.
inline constexpr TrunkLidGeometry kTrunkLid{-1.300, 0.950, 55.0, 1.0};

/// Lid open fraction after `dt_s` moving toward open (1) or closed (0) at
/// 1 / travel_s per second. Pure.
double StepTrunkLidOpen(double open01, bool open, double dt_s);

/// A chassis-frame point on the closed lid, carried to open fraction
/// `open01` about the hinge. Pure.
DashVec3 TrunkLidPoint(const DashVec3& closed, double open01);

class TrunkLid3D {
public:
    /// Loads the lid mesh from `mesh_path` into `device`'s scene. With no
    /// scene or no mesh there is no node and Update/Place do nothing.
    TrunkLid3D(irr::IrrlichtDevice* device, const std::string& mesh_path);
    ~TrunkLid3D();

    TrunkLid3D(const TrunkLid3D&) = delete;
    TrunkLid3D& operator=(const TrunkLid3D&) = delete;

    /// Advance the lid toward `open` by `dt_s` and place it on a chassis at
    /// `pose`.
    void Update(const VehiclePose& pose, bool open, double dt_s);

    /// Place the lid at open fraction `open01` on a chassis at `pose`.
    void Place(const VehiclePose& pose, double open01);

    double open_fraction() const { return open01_; }
    irr::scene::IMeshSceneNode* node() const { return node_; }

private:
    irr::scene::IMeshSceneNode* node_ = nullptr;
    double open01_ = 0.0;
};

}  // namespace ev1sim
