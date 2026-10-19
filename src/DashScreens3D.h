#pragma once

#include "DashSnapshot.h"
#include "InstrumentClusterPanel.h"
#include "RsaConsolePanel.h"
#include "VehiclePose.h"

// ---------------------------------------------------------------------------
// DashScreens3D — the IPC cluster and RSA console as screens in the 3D scene.
//
// Phase 3 of docs/ipc_rsa_display_plan.md. ev1sim has no cabin or dash model,
// so each display is a flat quad fixed to the chassis frame, textured with a
// render target that the existing 2D panel (InstrumentClusterPanel,
// RsaConsolePanel) is drawn into. Same DashSnapshot feed as the HUD panels;
// only the render backend changes.
//
// Split like the panels: PlaceDashScreen() / CabinEye() are pure chassis-frame
// math (no Irrlicht), and DashScreens3D owns the scene nodes and textures.
//
// Chassis frame (Chrono vehicle convention, as VehiclePose): x forward,
// y left, z up, origin at the chassis reference point.
// ---------------------------------------------------------------------------

namespace irr {
class IrrlichtDevice;
namespace scene { class IMeshSceneNode; }
namespace video { class ITexture; class IVideoDriver; }
namespace gui { class IGUIFont; }
}  // namespace irr

namespace ev1sim {

struct DashVec3 {
    double x = 0, y = 0, z = 0;
};

// Where a screen sits on the chassis. pitch_deg tilts the screen's top edge
// forward: 0 = upright facing the rear, 90 = flat facing up.
struct DashScreenMount {
    double x, y, z;
    double pitch_deg;
    double width_m, height_m;
};

// A placed screen in world coordinates. right/up/normal are unit vectors as
// seen by someone facing the screen; normal points out of the screen face.
struct DashScreenFrame {
    DashVec3 center;
    DashVec3 right;
    DashVec3 up;
    DashVec3 normal;
};

// Mount points and eye. @inferred 2026-10-18 claude: no EV1 interior
// dimensions are in hand, so these are read off the visual mesh
// (data/vehicle/ev1/ev1_chassis_vis.obj, its "interior" and "windowglass"
// parts): windshield base at x ~1.09, dash top z ~0.85-0.88 at x 0.7-1.1,
// steering wheel rim at x ~0.5 on the driver's side (y +0.35, left-hand
// drive), driver's seat back topping out at z ~1.09 around x -0.45. The
// cluster is the EV1's centre-mounted display on the dash top below the
// windshield; the console sits low between the seats ahead of the shifter
// area. The chassis JSON's "Driver Position" ([0.8, 0.35, 1.0]) is not used:
// on this mesh it lands at the windshield glass.
inline constexpr DashScreenMount kClusterMount{0.85, 0.0, 0.94, 35.0, 0.24, 0.24};
inline constexpr DashScreenMount kConsoleMount{0.45, 0.0, 0.58, 45.0, 0.18, 0.18};

// Driver's eye point in the chassis frame: seated head height in front of
// the seat back, on the steering-wheel line.
inline constexpr DashVec3 kDriverEye{-0.30, 0.35, 1.12};

/// World-space placement of `mount` on a chassis at `pose`. Pure.
DashScreenFrame PlaceDashScreen(const VehiclePose& pose, const DashScreenMount& mount);

/// World-space point for chassis-frame point `p` on a chassis at `pose`. Pure.
DashVec3 ChassisToWorld(const VehiclePose& pose, const DashVec3& p);

/// Where the cabin camera looks: midway between the two screens, so both are
/// in view from kDriverEye. Chassis frame. Pure.
DashVec3 CabinLookPoint();

// Texture size for each screen (pixels, square, power of two). The panels
// are laid out at this width, so the cluster's 2D layout fits unchanged.
inline constexpr int kDashScreenTexturePx = 256;

class DashScreens3D {
public:
    // Creates both quads in `device`'s scene. Textures are render targets;
    // on a driver without render-target support (the null driver) the quads
    // exist untextured and Refresh() is a no-op.
    explicit DashScreens3D(irr::IrrlichtDevice* device,
                           ClusterUnits units = ClusterUnits::kMetric);
    ~DashScreens3D();

    DashScreens3D(const DashScreens3D&) = delete;
    DashScreens3D& operator=(const DashScreens3D&) = delete;

    /// Rebuild both panel views from this tick's snapshot.
    void Update(const DashSnapshot& snap);

    /// Redraw both panels into their textures, then restore the frame buffer
    /// without clearing it. Call between the driver's beginScene/endScene.
    void Refresh(irr::gui::IGUIFont* font);

    /// Move both quads to follow the chassis.
    void Place(const VehiclePose& pose);

    bool HasTextures() const { return cluster_tex_ != nullptr && console_tex_ != nullptr; }

    irr::scene::IMeshSceneNode* cluster_node() const { return cluster_node_; }
    irr::scene::IMeshSceneNode* console_node() const { return console_node_; }
    const InstrumentClusterPanel& cluster_panel() const { return cluster_; }
    const RsaConsolePanel& console_panel() const { return console_; }

private:
    irr::IrrlichtDevice* device_;
    InstrumentClusterPanel cluster_;
    RsaConsolePanel console_;
    irr::video::ITexture* cluster_tex_ = nullptr;
    irr::video::ITexture* console_tex_ = nullptr;
    irr::scene::IMeshSceneNode* cluster_node_ = nullptr;
    irr::scene::IMeshSceneNode* console_node_ = nullptr;
};

}  // namespace ev1sim
