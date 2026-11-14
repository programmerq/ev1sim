#include "InteriorLamps3D.h"

#include <irrlicht.h>

#include <algorithm>

namespace ev1sim {

namespace {

// Lens colour when dark: the off-white of an unlit diffuser.
constexpr double kLensDark[3] = {0.55, 0.55, 0.52};

irr::video::SColor LensColour(double b) {
    auto mix = [b](double dark, double lit) {
        return static_cast<irr::u32>(std::clamp(dark + (lit - dark) * b, 0.0, 1.0) * 255.0 + 0.5);
    };
    return irr::video::SColor(255, mix(kLensDark[0], kInteriorLampColour[0]),
                              mix(kLensDark[1], kInteriorLampColour[1]),
                              mix(kLensDark[2], kInteriorLampColour[2]));
}

irr::core::vector3df ToIrr(const DashVec3& v) {
    return {static_cast<irr::f32>(v.x), static_cast<irr::f32>(v.y), static_cast<irr::f32>(v.z)};
}

// Rotation that takes the box's local x/y/z to the chassis's forward/left/up.
irr::core::vector3df ChassisRotationDegrees(const VehiclePose& p) {
    const DashVec3 f{p.fx, p.fy, p.fz};
    const DashVec3 u{p.ux, p.uy, p.uz};
    const DashVec3 l{u.y * f.z - u.z * f.y, u.z * f.x - u.x * f.z, u.x * f.y - u.y * f.x};
    irr::core::matrix4 m;
    m[0] = static_cast<irr::f32>(f.x);
    m[1] = static_cast<irr::f32>(f.y);
    m[2] = static_cast<irr::f32>(f.z);
    m[4] = static_cast<irr::f32>(l.x);
    m[5] = static_cast<irr::f32>(l.y);
    m[6] = static_cast<irr::f32>(l.z);
    m[8] = static_cast<irr::f32>(u.x);
    m[9] = static_cast<irr::f32>(u.y);
    m[10] = static_cast<irr::f32>(u.z);
    return m.getRotationDegrees();
}

}  // namespace

double InteriorLampBrightness(bool lit, std::optional<std::uint8_t> level) {
    if (!lit) return 0.0;
    if (!level || *level == 0u) return 1.0;
    return static_cast<double>(*level) / 255.0;
}

const InteriorLampMount& MountFor(InteriorLamp lamp) {
    return lamp == InteriorLamp::kFourBar ? kFourBarLampMount : kMirrorMapLampMount;
}

InteriorLamps3D::InteriorLamps3D(irr::IrrlichtDevice* device) : device_(device) {
    irr::scene::ISceneManager* smgr = device_ ? device_->getSceneManager() : nullptr;
    if (smgr == nullptr) return;
    for (int i = 0; i < kNumInteriorLamps; ++i) {
        const InteriorLampMount& m = MountFor(static_cast<InteriorLamp>(i));
        lens_[i] = smgr->addCubeSceneNode(1.0f);
        if (lens_[i] != nullptr) {
            // Unlit, so the vertex colours are what shows.
            lens_[i]->setMaterialFlag(irr::video::EMF_LIGHTING, false);
            lens_[i]->setScale({static_cast<irr::f32>(m.length_m), static_cast<irr::f32>(m.width_m),
                                static_cast<irr::f32>(m.thickness_m)});
            smgr->getMeshManipulator()->setVertexColors(lens_[i]->getMesh(), LensColour(0.0));
        }
        light_[i] = smgr->addLightSceneNode(nullptr, irr::core::vector3df(0, 0, 0),
                                            irr::video::SColorf(0.0f, 0.0f, 0.0f),
                                            static_cast<irr::f32>(kInteriorLampRadiusM));
        if (light_[i] != nullptr) light_[i]->setVisible(false);
    }
}

InteriorLamps3D::~InteriorLamps3D() {
    for (int i = 0; i < kNumInteriorLamps; ++i) {
        if (lens_[i] != nullptr) lens_[i]->remove();
        if (light_[i] != nullptr) light_[i]->remove();
    }
}

void InteriorLamps3D::Update(const VehiclePose& pose,
                             const double brightness[kNumInteriorLamps]) {
    irr::scene::ISceneManager* smgr = device_ ? device_->getSceneManager() : nullptr;
    const irr::core::vector3df rot = ChassisRotationDegrees(pose);
    for (int i = 0; i < kNumInteriorLamps; ++i) {
        const InteriorLampMount& m = MountFor(static_cast<InteriorLamp>(i));
        const double b = std::clamp(brightness[i], 0.0, 1.0);
        const irr::core::vector3df at = ToIrr(ChassisToWorld(pose, {m.x, m.y, m.z}));
        if (lens_[i] != nullptr) {
            lens_[i]->setPosition(at);
            lens_[i]->setRotation(rot);
            if (smgr != nullptr)
                smgr->getMeshManipulator()->setVertexColors(lens_[i]->getMesh(), LensColour(b));
        }
        if (light_[i] != nullptr) {
            // The light hangs just below the lens so the lens itself does not
            // shadow it.
            light_[i]->setPosition(ToIrr(ChassisToWorld(pose, {m.x, m.y, m.z - 0.03})));
            irr::video::SLight& data = light_[i]->getLightData();
            const auto c = [b](double k) { return static_cast<irr::f32>(k * b); };
            data.DiffuseColor = irr::video::SColorf(c(kInteriorLampColour[0]),
                                                    c(kInteriorLampColour[1]),
                                                    c(kInteriorLampColour[2]));
            light_[i]->setVisible(b > 0.0);
        }
    }
}

}  // namespace ev1sim
