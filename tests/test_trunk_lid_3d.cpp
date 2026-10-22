// TrunkLid3D — the trunk lid split out of the chassis mesh: the hinge swing,
// the open/close travel, and the scene node on the real lid mesh.

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "TrunkLid3D.h"

#include <irrlicht.h>

#include <cmath>
#include <string>

using namespace ev1sim;
using Catch::Matchers::WithinAbs;

namespace {
double Dist(const DashVec3& a, const DashVec3& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}
const std::string kLidMesh = std::string(EV1SIM_SOURCE_DIR) + "/data/vehicle/ev1/ev1_trunk_lid.obj";
}  // namespace

TEST_CASE("Trunk lid swings its rear edge up about the hinge", "[trunklid3d]") {
    const DashVec3 rear_edge{-2.0, 0.3, 0.80};
    const DashVec3 hinge{kTrunkLid.hinge_x, 0.3, kTrunkLid.hinge_z};
    // Closed: unchanged. The hinge line never moves.
    const DashVec3 closed = TrunkLidPoint(rear_edge, 0.0);
    CHECK_THAT(Dist(closed, rear_edge), WithinAbs(0.0, 1e-12));
    CHECK_THAT(Dist(TrunkLidPoint(hinge, 1.0), hinge), WithinAbs(0.0, 1e-12));
    // Open: same distance from the hinge, raised and pulled forward, y kept.
    const DashVec3 open = TrunkLidPoint(rear_edge, 1.0);
    CHECK_THAT(Dist(open, hinge), WithinAbs(Dist(rear_edge, hinge), 1e-12));
    CHECK(open.z > rear_edge.z + 0.4);
    CHECK(open.x > rear_edge.x);
    CHECK(open.y == rear_edge.y);
    // The lid's chord from the hinge turns by the open angle.
    const double a0 = std::atan2(rear_edge.z - hinge.z, hinge.x - rear_edge.x);
    const double a1 = std::atan2(open.z - hinge.z, hinge.x - open.x);
    CHECK_THAT((a1 - a0) * 180.0 / 3.14159265358979323846, WithinAbs(kTrunkLid.open_deg, 1e-9));
}

TEST_CASE("Trunk lid travels open and closed over the travel time", "[trunklid3d]") {
    double f = 0.0;
    f = StepTrunkLidOpen(f, true, kTrunkLid.travel_s / 2.0);
    CHECK_THAT(f, WithinAbs(0.5, 1e-12));
    f = StepTrunkLidOpen(f, true, kTrunkLid.travel_s);
    CHECK(f == 1.0);  // clamps fully open
    f = StepTrunkLidOpen(f, false, kTrunkLid.travel_s / 4.0);
    CHECK_THAT(f, WithinAbs(0.75, 1e-12));
    f = StepTrunkLidOpen(f, false, 10.0);
    CHECK(f == 0.0);  // clamps closed
}

TEST_CASE("Trunk lid node loads the split mesh and follows the chassis", "[trunklid3d]") {
    irr::IrrlichtDevice* dev = irr::createDevice(irr::video::EDT_NULL,
                                                 irr::core::dimension2d<irr::u32>(320, 240));
    REQUIRE(dev != nullptr);
    {
        TrunkLid3D lid(dev, kLidMesh);
        REQUIRE(lid.node() != nullptr);
        // The mesh is the lid at the back of the car, behind the hinge.
        const irr::core::aabbox3df box = lid.node()->getMesh()->getBoundingBox();
        CHECK(box.MaxEdge.X <= kTrunkLid.hinge_x + 0.01);
        CHECK(box.MinEdge.X < -1.9);
        CHECK(box.MaxEdge.Z < kTrunkLid.hinge_z + 0.05);

        VehiclePose pose{};
        pose.px = 10.0;
        pose.py = -4.0;
        pose.pz = 0.5;
        pose.fx = 1.0;
        pose.uz = 1.0;

        // Closed: mesh coordinates land where ChassisToWorld puts them.
        lid.Update(pose, false, 0.1);
        CHECK(lid.open_fraction() == 0.0);
        lid.node()->updateAbsolutePosition();
        irr::core::vector3df p(-2.0f, 0.3f, 0.8f);
        lid.node()->getAbsoluteTransformation().transformVect(p);
        const DashVec3 want0 = ChassisToWorld(pose, {-2.0, 0.3, 0.8});
        CHECK_THAT(p.X, WithinAbs(want0.x, 1e-4));
        CHECK_THAT(p.Y, WithinAbs(want0.y, 1e-4));
        CHECK_THAT(p.Z, WithinAbs(want0.z, 1e-4));

        // Open for the full travel: the same point swung about the hinge.
        lid.Update(pose, true, kTrunkLid.travel_s);
        CHECK(lid.open_fraction() == 1.0);
        lid.node()->updateAbsolutePosition();
        irr::core::vector3df q(-2.0f, 0.3f, 0.8f);
        lid.node()->getAbsoluteTransformation().transformVect(q);
        const DashVec3 want1 = ChassisToWorld(pose, TrunkLidPoint({-2.0, 0.3, 0.8}, 1.0));
        CHECK_THAT(q.X, WithinAbs(want1.x, 1e-4));
        CHECK_THAT(q.Y, WithinAbs(want1.y, 1e-4));
        CHECK_THAT(q.Z, WithinAbs(want1.z, 1e-4));
    }
    {
        // No mesh: no node, and placing it is a no-op.
        TrunkLid3D none(dev, std::string(EV1SIM_SOURCE_DIR) + "/no/such/lid.obj");
        CHECK(none.node() == nullptr);
        none.Update(VehiclePose{}, true, 1.0);
        CHECK(none.open_fraction() == 1.0);
    }
    dev->drop();
}
