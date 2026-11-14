#pragma once

#include "DashScreens3D.h"  // DashVec3, ChassisToWorld
#include "VehiclePose.h"

#include <cstdint>
#include <optional>

// ---------------------------------------------------------------------------
// InteriorLamps3D — the cabin's two interior lamps as lit lenses and lights in
// the 3D scene.
//
// The RHJB's delayed interior lamp module drives two lamps: the four-bar lamp
// (circuit 1732G / 156B) and the inside rear-view mirror's map lamp (1732F /
// 156). electricsim publishes whether each is lit as a conductor cell and the
// module's fade level as CHASSIS_RHJB_DILM_LEVEL; InteriorLampBrightness()
// combines the two. Each lamp is a small unlit box (its lens) plus a point
// light that lights the cabin around it.
//
// Split like DashScreens3D: brightness and mounts are pure (no Irrlicht), and
// InteriorLamps3D owns the scene nodes. Chassis frame as VehiclePose: x
// forward, y left, z up.
// ---------------------------------------------------------------------------

namespace irr {
class IrrlichtDevice;
namespace scene { class IMeshSceneNode; class ILightSceneNode; }
}  // namespace irr

namespace ev1sim {

enum class InteriorLamp : int { kFourBar = 0, kMirrorMap = 1 };
inline constexpr int kNumInteriorLamps = 2;

/// How bright an interior lamp shows, 0 (dark) .. 1 (full). A lamp whose
/// circuit is dead is dark whatever the level. A lit lamp follows the
/// module's level (level / 255); with no level yet, or a level of 0 while
/// the circuit is still live, it shows full, so a lamp with current through
/// it never draws dark. Pure.
double InteriorLampBrightness(bool lit, std::optional<std::uint8_t> level);

// Where a lamp's lens sits on the chassis, and its size.
struct InteriorLampMount {
    double x, y, z;
    double length_m, width_m, thickness_m;  // along x, along y, along z
};

// @inferred 2026-11-14 claude: no EV1 interior dimensions are in hand, so these
// are read off the visual mesh (data/vehicle/ev1/ev1_chassis_vis.obj, its
// "interior" part). The inside rear-view mirror is the block at x 0.29-0.33,
// y +/-0.12, z 1.15-1.22 below the windshield header; its map lamp sits on the
// mirror's underside. The four-bar lamp is the centre rear lamp bar (its
// connector's catalog note, ev1-connections/sourcing/12047662.yaml in
// electricsim), placed on the headliner centreline over the rear of the cabin,
// where the headliner's underside is at z ~1.26. Lens sizes are estimates.
inline constexpr InteriorLampMount kFourBarLampMount{-0.55, 0.0, 1.25, 0.08, 0.22, 0.012};
inline constexpr InteriorLampMount kMirrorMapLampMount{0.31, 0.0, 1.145, 0.04, 0.10, 0.010};

const InteriorLampMount& MountFor(InteriorLamp lamp);

// The point light's reach (m), and its colour at full brightness: a warm
// incandescent white. Estimates for a small cabin bulb.
inline constexpr double kInteriorLampRadiusM = 1.4;
inline constexpr double kInteriorLampColour[3] = {1.0, 0.86, 0.62};

class InteriorLamps3D {
public:
    /// Creates each lamp's lens and light in `device`'s scene, dark.
    explicit InteriorLamps3D(irr::IrrlichtDevice* device);
    ~InteriorLamps3D();

    InteriorLamps3D(const InteriorLamps3D&) = delete;
    InteriorLamps3D& operator=(const InteriorLamps3D&) = delete;

    /// Move both lamps to follow the chassis at `pose` and show each at its
    /// brightness (0..1, indexed by InteriorLamp).
    void Update(const VehiclePose& pose, const double brightness[kNumInteriorLamps]);

    irr::scene::IMeshSceneNode* lens(InteriorLamp lamp) const {
        return lens_[static_cast<int>(lamp)];
    }
    irr::scene::ILightSceneNode* light(InteriorLamp lamp) const {
        return light_[static_cast<int>(lamp)];
    }

private:
    irr::IrrlichtDevice* device_;
    irr::scene::IMeshSceneNode* lens_[kNumInteriorLamps] = {nullptr, nullptr};
    irr::scene::ILightSceneNode* light_[kNumInteriorLamps] = {nullptr, nullptr};
};

}  // namespace ev1sim
