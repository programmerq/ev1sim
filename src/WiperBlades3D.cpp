#include "WiperBlades3D.h"

#include <irrlicht.h>

#include <cmath>

namespace ev1sim {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// Bars float this far off the fitted surface: the fit is a smooth bowl and
// the mesh's glass bulges a little between its edges, so closer than this
// the glass cuts through the blades.
constexpr double kLiftM = 0.025;
// Bar cross-sections (width on the glass, height off it).
constexpr double kArmWidthM = 0.018, kArmHeightM = 0.012;
constexpr double kBladeWidthM = 0.012, kBladeHeightM = 0.020;

DashVec3 Sub(const DashVec3& a, const DashVec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
DashVec3 Scale(const DashVec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double Dot(const DashVec3& a, const DashVec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
DashVec3 Cross(const DashVec3& a, const DashVec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
DashVec3 Unit(const DashVec3& a) {
    const double n = std::sqrt(Dot(a, a));
    return n > 0.0 ? Scale(a, 1.0 / n) : a;
}

// Point at distance r from the pivot along the arm at angle `deg`, in
// windshield coordinates.
DashVec3 AlongArm(const WiperArmGeometry& g, double deg, double r) {
    const double a = deg * kDeg2Rad;
    return WindshieldPoint(g.pivot_y - r * std::cos(a), g.pivot_s + r * std::sin(a), kLiftM);
}

// Stretch a unit cube between world points a and b, `up` roughly off the
// glass.
void PlaceBar(irr::scene::IMeshSceneNode* node, const DashVec3& a, const DashVec3& b,
              const DashVec3& up, double width_m, double height_m) {
    if (node == nullptr) return;
    const DashVec3 d = Sub(b, a);
    const double len = std::sqrt(Dot(d, d));
    if (len <= 0.0) return;
    const DashVec3 ex = Scale(d, 1.0 / len);
    const DashVec3 ey = Unit(Cross(up, ex));
    const DashVec3 ez = Cross(ex, ey);
    irr::core::matrix4 m;
    m[0] = static_cast<irr::f32>(ex.x);
    m[1] = static_cast<irr::f32>(ex.y);
    m[2] = static_cast<irr::f32>(ex.z);
    m[4] = static_cast<irr::f32>(ey.x);
    m[5] = static_cast<irr::f32>(ey.y);
    m[6] = static_cast<irr::f32>(ey.z);
    m[8] = static_cast<irr::f32>(ez.x);
    m[9] = static_cast<irr::f32>(ez.y);
    m[10] = static_cast<irr::f32>(ez.z);
    node->setRotation(m.getRotationDegrees());
    node->setScale({static_cast<irr::f32>(len), static_cast<irr::f32>(width_m),
                    static_cast<irr::f32>(height_m)});
    node->setPosition({static_cast<irr::f32>((a.x + b.x) / 2.0),
                       static_cast<irr::f32>((a.y + b.y) / 2.0),
                       static_cast<irr::f32>((a.z + b.z) / 2.0)});
}

DashVec3 ToWorldDir(const VehiclePose& p, const DashVec3& v) {
    const DashVec3 o = ChassisToWorld(p, {0.0, 0.0, 0.0});
    return Sub(ChassisToWorld(p, v), o);
}

}  // namespace

DashVec3 WindshieldPoint(double y, double s, double lift_m) {
    const WindshieldFit& w = kWindshield;
    const DashVec3 normal{w.up_z, 0.0, -w.up_x};  // up rotated 90 deg, out of the glass
    return {w.base_x0 - w.base_x_curve * y * y + s * w.up_x + lift_m * normal.x, y,
            w.base_z0 - w.base_z_curve * y * y + s * w.up_z + lift_m * normal.z};
}

double WiperBladeTipS(const WiperArmGeometry& g, double sweep01) {
    const double a = (g.park_deg + sweep01 * g.sweep_deg) * kDeg2Rad;
    return g.pivot_s + (g.arm_m + g.blade_m / 2.0) * std::sin(a);
}

WiperPose PlaceWiper(const WiperArmGeometry& g, double sweep01) {
    if (sweep01 < 0.0) sweep01 = 0.0;
    if (sweep01 > 1.0) sweep01 = 1.0;
    const double deg = g.park_deg + sweep01 * g.sweep_deg;
    WiperPose p;
    p.pivot = AlongArm(g, deg, 0.0);
    p.arm_tip = AlongArm(g, deg, g.arm_m);
    p.blade_in = AlongArm(g, deg, g.arm_m - g.blade_m / 2.0);
    p.blade_out = AlongArm(g, deg, g.arm_m + g.blade_m / 2.0);
    return p;
}

WiperBlades3D::WiperBlades3D(irr::IrrlichtDevice* device) : device_(device) {
    irr::scene::ISceneManager* smgr = device_ ? device_->getSceneManager() : nullptr;
    if (smgr == nullptr) return;
    for (auto*& node : nodes_) {
        node = smgr->addCubeSceneNode(1.0f);
        if (node == nullptr) continue;
        node->setMaterialFlag(irr::video::EMF_LIGHTING, false);
        // Unlit, so the cube's vertex colours are what shows: blade black.
        smgr->getMeshManipulator()->setVertexColors(node->getMesh(),
                                                    irr::video::SColor(255, 18, 18, 20));
    }
}

WiperBlades3D::~WiperBlades3D() {
    for (auto* node : nodes_) {
        if (node != nullptr) node->remove();
    }
}

void WiperBlades3D::Place(const VehiclePose& pose, double sweep01) {
    const DashVec3 off_glass = ToWorldDir(pose, {kWindshield.up_z, 0.0, -kWindshield.up_x});
    const WiperArmGeometry* arms[2] = {&kDriverWiper, &kPassengerWiper};
    for (int i = 0; i < 2; ++i) {
        const WiperPose w = PlaceWiper(*arms[i], sweep01);
        PlaceBar(nodes_[2 * i], ChassisToWorld(pose, w.pivot), ChassisToWorld(pose, w.arm_tip),
                 off_glass, kArmWidthM, kArmHeightM);
        PlaceBar(nodes_[2 * i + 1], ChassisToWorld(pose, w.blade_in),
                 ChassisToWorld(pose, w.blade_out), off_glass, kBladeWidthM, kBladeHeightM);
    }
}

}  // namespace ev1sim
