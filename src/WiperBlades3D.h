#pragma once

#include "DashScreens3D.h"  // DashVec3, ChassisToWorld
#include "VehiclePose.h"

// ---------------------------------------------------------------------------
// WiperBlades3D — the two windshield wipers as moving bars in the 3D scene.
//
// The chassis visual mesh has no wiper parts, so each wiper is drawn as two
// thin boxes (arm and blade) lying on the windshield and swung about a pivot
// below the glass, to WiperRenderer::GetSweepFraction(): the blade position
// the RHJB's wiper-motor plant publishes (CHASSIS_WIPER_SWEEP_POSITION), or
// without it WiperRenderer's own phase.
//
// Split like DashScreens3D: the windshield and arm geometry are pure
// chassis-frame math (no Irrlicht), and WiperBlades3D owns the scene nodes.
// Chassis frame as VehiclePose: x forward, y left, z up.
// ---------------------------------------------------------------------------

namespace irr {
class IrrlichtDevice;
namespace scene { class IMeshSceneNode; }
}  // namespace irr

namespace ev1sim {

// The windshield as a surface over (y, s): y across the car (left +), s the
// distance up the glass from its bottom edge. @inferred 2026-10-21 claude:
// fitted to the visual mesh's "windowglass" part (ev1_chassis_vis.obj): the
// bottom edge runs x 1.087 / z 0.883 at the centreline back to x 0.942 /
// z 0.865 at y +/-0.6, and the glass rises at about 26 degrees from
// horizontal to x 0.35 / z 1.24 at the roof.
struct WindshieldFit {
    double base_x0 = 1.087, base_x_curve = 0.40;   // base x = x0 - curve * y^2
    double base_z0 = 0.8826, base_z_curve = 0.05;  // base z = z0 - curve * y^2
    double up_x = -0.898, up_z = 0.440;            // unit vector up the glass
};
inline constexpr WindshieldFit kWindshield{};

/// Chassis-frame point on the windshield at (y, s), lifted `lift_m` off the
/// glass along its outward normal. Pure.
DashVec3 WindshieldPoint(double y, double s, double lift_m = 0.0);

// One wiper in windshield coordinates. The arm turns about (pivot_y, pivot_s)
// from park_deg to park_deg + sweep_deg; 0 degrees points across the car to
// the passenger side (-y), 90 degrees straight up the glass. The blade is
// centred on the arm tip and lies along the arm.
struct WiperArmGeometry {
    double pivot_y, pivot_s;
    double arm_m, blade_m;
    double park_deg, sweep_deg;
};

// @source:manual body-126 ("WIPER ARM", installation step 3): parked, the
// blade tip sits 83 mm (driver) and 67 mm (passenger) above the bottom edge
// of the windshield, and the two blades differ (bowed on the passenger
// side). The park angles below put the blade tips at those heights.
// @design 2026-10-21 claude: everything else here. The manual prints no pivot
// positions, arm or blade lengths, or sweep angle, so this is a usual
// left-hand-drive tandem layout sized to the fitted glass: both blades park
// along the bottom edge pointing to the passenger side and sweep up toward the
// driver's pillar, pivots 30 mm below the glass edge.
inline constexpr WiperArmGeometry kDriverWiper{0.60, -0.03, 0.38, 0.50, 10.3, 88.0};
inline constexpr WiperArmGeometry kPassengerWiper{0.05, -0.03, 0.35, 0.45, 9.7, 84.0};

/// Blade tip height above the glass's bottom edge (s of the blade's outer
/// end) at sweep fraction 0..1. Pure.
double WiperBladeTipS(const WiperArmGeometry& g, double sweep01);

/// Chassis-frame ends of one wiper at sweep fraction 0 (park) .. 1 (far end
/// of the sweep). Pure.
struct WiperPose {
    DashVec3 pivot, arm_tip;      // the arm
    DashVec3 blade_in, blade_out; // the blade
};
WiperPose PlaceWiper(const WiperArmGeometry& g, double sweep01);

class WiperBlades3D {
public:
    /// Creates the four bars (two arms, two blades) in `device`'s scene.
    explicit WiperBlades3D(irr::IrrlichtDevice* device);
    ~WiperBlades3D();

    WiperBlades3D(const WiperBlades3D&) = delete;
    WiperBlades3D& operator=(const WiperBlades3D&) = delete;

    /// Move the wipers to `sweep01` on a chassis at `pose`.
    void Place(const VehiclePose& pose, double sweep01);

    irr::scene::IMeshSceneNode* driver_blade() const { return nodes_[1]; }

private:
    irr::IrrlichtDevice* device_;
    // driver arm, driver blade, passenger arm, passenger blade
    irr::scene::IMeshSceneNode* nodes_[4] = {nullptr, nullptr, nullptr, nullptr};
};

}  // namespace ev1sim
