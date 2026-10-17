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

// Mount points. @inferred 2026-10-17 claude: no EV1 interior dimensions are
// in hand, so these are placed relative to the chassis "Driver Position"
// (data/vehicle/ev1/chassis/EV1_Chassis.json, [0.8, 0.35, 1.0]): the cluster
// is the EV1's centre-mounted display at the base of the windshield, the
// console sits low between the seats, ahead of the driver's hip.
inline constexpr DashScreenMount kClusterMount{1.45, 0.0, 0.95, 20.0, 0.24, 0.24};
inline constexpr DashScreenMount kConsoleMount{1.20, 0.0, 0.62, 50.0, 0.18, 0.18};

// Driver eye point in the chassis frame: the chassis JSON's driver position.
inline constexpr DashVec3 kDriverEye{0.8, 0.35, 1.0};

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

    void SetVisible(bool visible);
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
