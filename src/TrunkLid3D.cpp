#include "TrunkLid3D.h"

#include <irrlicht.h>

#include <algorithm>
#include <cmath>

namespace ev1sim {

namespace {

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

DashVec3 Sub(const DashVec3& a, const DashVec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

}  // namespace

double StepTrunkLidOpen(double open01, bool open, double dt_s) {
    const double step = (kTrunkLid.travel_s > 0.0) ? dt_s / kTrunkLid.travel_s : 1.0;
    const double next = open ? open01 + step : open01 - step;
    return std::clamp(next, 0.0, 1.0);
}

DashVec3 TrunkLidPoint(const DashVec3& closed, double open01) {
    // Rotation about the hinge line (+y): positive angles lift points behind
    // the hinge (x below hinge_x) upward.
    const double a = std::clamp(open01, 0.0, 1.0) * kTrunkLid.open_deg * kDeg2Rad;
    const double dx = closed.x - kTrunkLid.hinge_x;
    const double dz = closed.z - kTrunkLid.hinge_z;
    return {kTrunkLid.hinge_x + dx * std::cos(a) + dz * std::sin(a), closed.y,
            kTrunkLid.hinge_z - dx * std::sin(a) + dz * std::cos(a)};
}

TrunkLid3D::TrunkLid3D(irr::IrrlichtDevice* device, const std::string& mesh_path) {
    irr::scene::ISceneManager* smgr = device ? device->getSceneManager() : nullptr;
    if (smgr == nullptr) return;
    irr::scene::IAnimatedMesh* mesh = smgr->getMesh(mesh_path.c_str());
    if (mesh == nullptr) return;
    // Irrlicht's OBJ loader mirrors x (into its left-handed frame) and
    // reverses the winding to match; Chrono loads the chassis OBJ as written.
    // Mirror back so the lid lines up with the body, normals included.
    irr::core::matrix4 mirror_x;
    mirror_x.setScale(irr::core::vector3df(-1.0f, 1.0f, 1.0f));
    smgr->getMeshManipulator()->transform(mesh->getMesh(0), mirror_x);
    smgr->getMeshManipulator()->flipSurfaces(mesh->getMesh(0));
    node_ = smgr->addMeshSceneNode(mesh->getMesh(0));
    if (node_ == nullptr) return;
    // Open, the lid's underside faces the camera; the mesh is single-sided.
    node_->setMaterialFlag(irr::video::EMF_BACK_FACE_CULLING, false);
    node_->setMaterialFlag(irr::video::EMF_NORMALIZE_NORMALS, true);
}

TrunkLid3D::~TrunkLid3D() {
    if (node_ != nullptr) node_->remove();
}

void TrunkLid3D::Update(const VehiclePose& pose, bool open, double dt_s) {
    open01_ = StepTrunkLidOpen(open01_, open, dt_s);
    Place(pose, open01_);
}

void TrunkLid3D::Place(const VehiclePose& pose, double open01) {
    if (node_ == nullptr) return;
    // The node maps mesh (chassis-frame) points to world: build it from where
    // the origin and the three unit axes land.
    auto to_world = [&](const DashVec3& p) { return ChassisToWorld(pose, TrunkLidPoint(p, open01)); };
    const DashVec3 o = to_world({0.0, 0.0, 0.0});
    const DashVec3 ex = Sub(to_world({1.0, 0.0, 0.0}), o);
    const DashVec3 ey = Sub(to_world({0.0, 1.0, 0.0}), o);
    const DashVec3 ez = Sub(to_world({0.0, 0.0, 1.0}), o);
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
    node_->setRotation(m.getRotationDegrees());
    node_->setPosition({static_cast<irr::f32>(o.x), static_cast<irr::f32>(o.y),
                        static_cast<irr::f32>(o.z)});
}

}  // namespace ev1sim
