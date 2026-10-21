#include "DashScreens3D.h"

#include <irrlicht.h>

#include <algorithm>
#include <cmath>

namespace ev1sim {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

DashVec3 Add(const DashVec3& a, const DashVec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
DashVec3 Scale(const DashVec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
DashVec3 Sub(const DashVec3& a, const DashVec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
double Dot(const DashVec3& a, const DashVec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
DashVec3 Cross(const DashVec3& a, const DashVec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct ChassisAxes {
    DashVec3 fwd, left, up;
};

ChassisAxes Axes(const VehiclePose& p) {
    const DashVec3 fwd{p.fx, p.fy, p.fz};
    const DashVec3 up{p.ux, p.uy, p.uz};
    return {fwd, Cross(up, fwd), up};
}

irr::core::vector3df ToIrr(const DashVec3& v) {
    return {static_cast<irr::f32>(v.x), static_cast<irr::f32>(v.y), static_cast<irr::f32>(v.z)};
}

// A textured quad in the node's local frame: x = screen right, y = screen up,
// z = out of the face. UV (0,0) is the texture's top-left, at the screen's
// top-left corner.
irr::scene::IMesh* MakeQuad(double width_m, double height_m) {
    using irr::video::S3DVertex;
    using irr::video::SColor;
    const irr::f32 hw = static_cast<irr::f32>(width_m / 2.0);
    const irr::f32 hh = static_cast<irr::f32>(height_m / 2.0);
    const SColor white(255, 255, 255, 255);

    auto* buf = new irr::scene::SMeshBuffer();
    buf->Vertices.push_back(S3DVertex(-hw, hh, 0, 0, 0, 1, white, 0, 0));
    buf->Vertices.push_back(S3DVertex(hw, hh, 0, 0, 0, 1, white, 1, 0));
    buf->Vertices.push_back(S3DVertex(-hw, -hh, 0, 0, 0, 1, white, 0, 1));
    buf->Vertices.push_back(S3DVertex(hw, -hh, 0, 0, 0, 1, white, 1, 1));
    const irr::u16 idx[] = {0, 1, 2, 2, 1, 3};
    for (irr::u16 i : idx) buf->Indices.push_back(i);
    buf->recalculateBoundingBox();

    auto* mesh = new irr::scene::SMesh();
    mesh->addMeshBuffer(buf);
    buf->drop();
    mesh->recalculateBoundingBox();
    return mesh;
}

irr::scene::IMeshSceneNode* AddScreen(irr::scene::ISceneManager* smgr,
                                      const DashScreenMount& mount,
                                      irr::video::ITexture* tex) {
    irr::scene::IMesh* mesh = MakeQuad(mount.width_m, mount.height_m);
    irr::scene::IMeshSceneNode* node = smgr->addMeshSceneNode(mesh);
    mesh->drop();
    if (node == nullptr) return nullptr;
    // The panel is its own light source; both faces drawn since the scene's
    // projection handedness decides which winding faces the camera.
    node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
    node->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
    if (tex != nullptr) node->setMaterialTexture(0, tex);
    return node;
}

void PlaceNode(irr::scene::IMeshSceneNode* node, const DashScreenFrame& f) {
    if (node == nullptr) return;
    // Columns of the rotation: local x/y/z map to screen right/up/normal.
    irr::core::matrix4 m;
    m[0] = static_cast<irr::f32>(f.right.x);
    m[1] = static_cast<irr::f32>(f.right.y);
    m[2] = static_cast<irr::f32>(f.right.z);
    m[4] = static_cast<irr::f32>(f.up.x);
    m[5] = static_cast<irr::f32>(f.up.y);
    m[6] = static_cast<irr::f32>(f.up.z);
    m[8] = static_cast<irr::f32>(f.normal.x);
    m[9] = static_cast<irr::f32>(f.normal.y);
    m[10] = static_cast<irr::f32>(f.normal.z);
    node->setRotation(m.getRotationDegrees());
    node->setPosition(ToIrr(f.center));
}

// Each panel sits at the texture's left edge, centred vertically.
template <typename Panel>
void AnchorInTexture(Panel& panel) {
    panel.SetAnchor(0, std::max(0, (kDashScreenTexturePx - panel.Height()) / 2));
}

// Opaque version of the panels' glass colour, so the unused texture area
// matches the panel background.
const irr::video::SColor kScreenClear(255, 12, 20, 24);

}  // namespace

DashVec3 ChassisToWorld(const VehiclePose& pose, const DashVec3& p) {
    const ChassisAxes a = Axes(pose);
    DashVec3 w{pose.px, pose.py, pose.pz};
    w = Add(w, Scale(a.fwd, p.x));
    w = Add(w, Scale(a.left, p.y));
    w = Add(w, Scale(a.up, p.z));
    return w;
}

DashVec3 CabinLookPoint() {
    return {(kClusterMount.x + kConsoleMount.x) / 2.0, (kClusterMount.y + kConsoleMount.y) / 2.0,
            (kClusterMount.z + kConsoleMount.z) / 2.0};
}

std::optional<DashScreenUv> DashScreenUvAt(const DashScreenFrame& f, const DashScreenMount& mount,
                                           const DashVec3& origin, const DashVec3& dir) {
    const double denom = Dot(dir, f.normal);
    if (std::abs(denom) < 1e-12) return std::nullopt;  // parallel to the face
    const double t = Dot(Sub(f.center, origin), f.normal) / denom;
    if (t <= 0.0) return std::nullopt;                 // behind the origin
    const DashVec3 d = Sub(Add(origin, Scale(dir, t)), f.center);
    const double u = Dot(d, f.right) / mount.width_m + 0.5;
    const double v = 0.5 - Dot(d, f.up) / mount.height_m;
    if (u < 0.0 || u >= 1.0 || v < 0.0 || v >= 1.0) return std::nullopt;
    return DashScreenUv{u, v};
}

DashScreenFrame PlaceDashScreen(const VehiclePose& pose, const DashScreenMount& mount) {
    const ChassisAxes a = Axes(pose);
    const double pitch = mount.pitch_deg * kDeg2Rad;
    DashScreenFrame f;
    f.center = ChassisToWorld(pose, {mount.x, mount.y, mount.z});
    // Facing the screen from the driver's side, its right is the car's right.
    f.right = Scale(a.left, -1.0);
    f.up = Add(Scale(a.up, std::cos(pitch)), Scale(a.fwd, std::sin(pitch)));
    f.normal = Cross(f.right, f.up);
    return f;
}

DashScreens3D::DashScreens3D(irr::IrrlichtDevice* device, ClusterUnits units)
    : device_(device),
      cluster_(units, 0, 0, kDashScreenTexturePx),
      console_(0, 0, kDashScreenTexturePx) {
    AnchorInTexture(cluster_);
    AnchorInTexture(console_);
    if (device_ == nullptr) return;
    irr::video::IVideoDriver* driver = device_->getVideoDriver();
    irr::scene::ISceneManager* smgr = device_->getSceneManager();
    const irr::core::dimension2d<irr::u32> px(kDashScreenTexturePx, kDashScreenTexturePx);
    if (driver != nullptr && driver->queryFeature(irr::video::EVDF_RENDER_TO_TARGET)) {
        cluster_tex_ = driver->addRenderTargetTexture(px, "ev1sim_dash_cluster");
        console_tex_ = driver->addRenderTargetTexture(px, "ev1sim_dash_console");
    }
    if (smgr != nullptr) {
        cluster_node_ = AddScreen(smgr, kClusterMount, cluster_tex_);
        console_node_ = AddScreen(smgr, kConsoleMount, console_tex_);
    }
}

DashScreens3D::~DashScreens3D() {
    if (cluster_node_ != nullptr) cluster_node_->remove();
    if (console_node_ != nullptr) console_node_->remove();
    irr::video::IVideoDriver* driver = device_ ? device_->getVideoDriver() : nullptr;
    if (driver != nullptr) {
        if (cluster_tex_ != nullptr) driver->removeTexture(cluster_tex_);
        if (console_tex_ != nullptr) driver->removeTexture(console_tex_);
    }
}

void DashScreens3D::Update(const DashSnapshot& snap) {
    cluster_.Update(snap);
    console_.Update(snap);
}

void DashScreens3D::Refresh(irr::gui::IGUIFont* font) {
    if (!HasTextures()) return;
    irr::video::IVideoDriver* driver = device_->getVideoDriver();
    auto draw_into = [&](irr::video::ITexture* tex, auto& panel) {
        driver->setRenderTarget(tex, true, true, kScreenClear);
        AnchorInTexture(panel);
        panel.Draw(driver, font);
    };
    draw_into(cluster_tex_, cluster_);
    draw_into(console_tex_, console_);
    driver->setRenderTarget(static_cast<irr::video::ITexture*>(nullptr), false, false);
}

void DashScreens3D::Place(const VehiclePose& pose) {
    pose_ = pose;
    PlaceNode(cluster_node_, PlaceDashScreen(pose, kClusterMount));
    PlaceNode(console_node_, PlaceDashScreen(pose, kConsoleMount));
}

RsaConsoleControl DashScreens3D::ConsoleControlAt(const DashVec3& origin,
                                                  const DashVec3& dir) const {
    const auto uv = DashScreenUvAt(PlaceDashScreen(pose_, kConsoleMount), kConsoleMount,
                                   origin, dir);
    if (!uv) return RsaConsoleControl::kNone;
    return console_.HitTest(static_cast<int>(uv->u * kDashScreenTexturePx),
                            static_cast<int>(uv->v * kDashScreenTexturePx));
}

}  // namespace ev1sim
