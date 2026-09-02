//
// OctreeChargeTests.cpp
//
// Covers the charge-side Octree aggregates (GetTotalCharge,
// GetCenterOfCharge, GetDipoleMoment) that OctreeTests.cpp doesn't touch
// -- that file is entirely mass/center-of-mass focused. These matter
// separately because charge, unlike mass, can cancel to (near) zero,
// which triggers the geometric-center fallback and makes the dipole
// moment the only signal a distant EM force evaluator has left to work
// with.
//
// Follows the same Insert-then-UpdateProperties() convention as
// OctreeTests.cpp, since that's the bottom-up recompute path
// PhysicsWorld::RebuildOctree() actually calls every step.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;

namespace {
    void RequireWithinAbs(const Vector3& actual, const Vector3& expected, double tolerance) {
        REQUIRE_THAT(actual.x, WithinAbs(expected.x, tolerance));
        REQUIRE_THAT(actual.y, WithinAbs(expected.y, tolerance));
        REQUIRE_THAT(actual.z, WithinAbs(expected.z, tolerance));
    }
}


TEST_CASE("Octree - An empty tree has zero total charge, zero center of charge, and zero dipole moment", "[octree][charge]")
{
    Bounds bounds{Vector3::Zero(), Vector3(10.0, 10.0, 10.0)};
    Octree tree(bounds);
    tree.UpdateProperties();

    REQUIRE(tree.GetRoot().GetTotalCharge() == 0.0);
    RequireWithinAbs(tree.GetRoot().GetCenterOfCharge(), Vector3::Zero(), 1e-9);
    RequireWithinAbs(tree.GetRoot().GetDipoleMoment(), Vector3::Zero(), 1e-9);
}


TEST_CASE("Octree - A single charged body sets total charge to its own charge and center of charge to its position", "[octree][charge]")
{
    Bounds bounds{Vector3::Zero(), Vector3(10.0, 10.0, 10.0)};
    Octree tree(bounds);

    const Vector3 position(2.0, -1.0, 3.0);
    tree.Insert(Handle{0, 0}, position, 1.0, 5.0);
    tree.UpdateProperties();

    REQUIRE(tree.GetRoot().GetTotalCharge() == 5.0);
    RequireWithinAbs(tree.GetRoot().GetCenterOfCharge(), position, 1e-9);
    // A single point mass/charge has no internal separation to produce a
    // dipole moment.
    RequireWithinAbs(tree.GetRoot().GetDipoleMoment(), Vector3::Zero(), 1e-9);
}


TEST_CASE("Octree - A neutral (zero-charge) body contributes nothing to the charge aggregates", "[octree][charge]")
{
    Bounds bounds{Vector3::Zero(), Vector3(10.0, 10.0, 10.0)};
    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(1.0, 0.0, 0.0), 1.0, 0.0);
    tree.UpdateProperties();

    REQUIRE(tree.GetRoot().GetTotalCharge() == 0.0);
    // Mass aggregates are unaffected by charge being zero.
    REQUIRE(tree.GetRoot().GetTotalMass() == 1.0);
}


TEST_CASE("Octree - Two same-sign charges combine into a charge-weighted center of charge", "[octree][charge]")
{
    Bounds bounds{Vector3::Zero(), Vector3(10.0, 10.0, 10.0)};
    Octree tree(bounds);

    // q=1 at x=0, q=3 at x=4 -> weighted center at x = (1*0 + 3*4)/4 = 3.
    tree.Insert(Handle{0, 0}, Vector3(0.0, 0.0, 0.0), 1.0, 1.0);
    tree.Insert(Handle{1, 0}, Vector3(4.0, 0.0, 0.0), 1.0, 3.0);
    tree.UpdateProperties();

    REQUIRE(tree.GetRoot().GetTotalCharge() == 4.0);
    RequireWithinAbs(tree.GetRoot().GetCenterOfCharge(), Vector3(3.0, 0.0, 0.0), 1e-9);
}


TEST_CASE("Octree - Equal and opposite charges cancel to (near) zero total charge", "[octree][charge]")
{
    Bounds bounds{Vector3::Zero(), Vector3(10.0, 10.0, 10.0)};
    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(-1.0, 0.0, 0.0), 1.0, 2.0);
    tree.Insert(Handle{1, 0}, Vector3(1.0, 0.0, 0.0), 1.0, -2.0);
    tree.UpdateProperties();

    REQUIRE_THAT(tree.GetRoot().GetTotalCharge(), WithinAbs(0.0, 1e-9));
}


TEST_CASE("Octree - When total charge cancels to (near) zero, center of charge falls back to the geometric center", "[octree][charge]")
{
    Bounds bounds{Vector3::Zero(), Vector3(10.0, 10.0, 10.0)};
    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(-1.0, 0.0, 0.0), 1.0, 2.0);
    tree.Insert(Handle{1, 0}, Vector3(3.0, 0.0, 0.0), 1.0, -2.0);
    tree.UpdateProperties();

    // Geometric (unweighted) average of the two positions, since the
    // charge-weighted average is undefined at zero net charge.
    RequireWithinAbs(tree.GetRoot().GetCenterOfCharge(), Vector3(1.0, 0.0, 0.0), 1e-9);
}


TEST_CASE("Octree - A charge-neutral pair still has a nonzero dipole moment reflecting their separation", "[octree][charge][dipole]")
{
    Bounds bounds{Vector3::Zero(), Vector3(10.0, 10.0, 10.0)};
    Octree tree(bounds);

    // +q at posA, -q at posB: dipole moment = q * (posA - posB).
    const double q = 2.0;
    const Vector3 posA(-1.0, 0.0, 0.0);
    const Vector3 posB(1.0, 0.0, 0.0);

    tree.Insert(Handle{0, 0}, posA, 1.0, q);
    tree.Insert(Handle{1, 0}, posB, 1.0, -q);
    tree.UpdateProperties();

    const Vector3 expectedDipole = (posA - posB) * q;
    RequireWithinAbs(tree.GetRoot().GetDipoleMoment(), expectedDipole, 1e-6);
}


TEST_CASE("Octree - Removing one charge of a canceling pair leaves the survivor's own charge and position", "[octree][charge]")
{
    Bounds bounds{Vector3::Zero(), Vector3(10.0, 10.0, 10.0)};
    Octree tree(bounds);

    const Handle handleA{0, 0};
    const Handle handleB{1, 0};
    const Vector3 posA(-1.0, 0.0, 0.0);
    const Vector3 posB(3.0, 0.0, 0.0);

    tree.Insert(handleA, posA, 1.0, 2.0);
    tree.Insert(handleB, posB, 1.0, -2.0);
    tree.Remove(handleB);
    tree.UpdateProperties();

    REQUIRE(tree.GetRoot().GetTotalCharge() == 2.0);
    RequireWithinAbs(tree.GetRoot().GetCenterOfCharge(), posA, 1e-9);
    RequireWithinAbs(tree.GetRoot().GetDipoleMoment(), Vector3::Zero(), 1e-9);
}


TEST_CASE("Octree - A three-body cluster's dipole moment is invariant to the choice of reference (matches direct definition)", "[octree][charge][dipole]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);

    struct Body { Vector3 position; double charge; };
    const std::vector<Body> chargedBodies = {
        {Vector3(1.0, 0.0, 0.0), 3.0},
        {Vector3(-2.0, 1.0, 0.0), -1.0},
        {Vector3(0.0, -3.0, 2.0), -2.0},
    };

    uint32_t index = 0;
    for (const Body& b : chargedBodies) {
        tree.Insert(Handle{index++, 0}, b.position, 1.0, b.charge);
    }
    tree.UpdateProperties();

    // Total charge here is 3 - 1 - 2 = 0, so center of charge falls back
    // to the geometric center, and the dipole moment about that
    // reference should equal the textbook definition sum(q_i * r_i)
    // (since sum(q_i) == 0, the dipole moment is independent of the
    // reference point entirely).
    Vector3 expectedDipole = Vector3::Zero();
    for (const Body& b : chargedBodies) {
        expectedDipole += b.position * b.charge;
    }

    RequireWithinAbs(tree.GetRoot().GetDipoleMoment(), expectedDipole, 1e-6);
}


TEST_CASE("Octree - Charge aggregates after subdivision match direct computation across all inserted bodies", "[octree][charge][subdivide]")
{
    Bounds bounds{Vector3::Zero(), Vector3(50.0, 50.0, 50.0)};
    Octree tree(bounds);

    struct Body { Vector3 position; double charge; };
    const std::vector<Body> chargedBodies = {
        {Vector3(10.0, 10.0, 10.0), 1.0},
        {Vector3(-10.0, 10.0, 10.0), 2.0},
        {Vector3(10.0, -10.0, 10.0), -1.5},
        {Vector3(10.0, 10.0, -10.0), 0.5},
        {Vector3(-10.0, -10.0, -10.0), -1.0},
    };

    uint32_t index = 0;
    for (const Body& b : chargedBodies) {
        tree.Insert(Handle{index++, 0}, b.position, 1.0, b.charge);
    }
    tree.UpdateProperties();

    REQUIRE_FALSE(tree.GetRoot().IsLeaf()); // sanity: spread out enough to subdivide

    double expectedTotalCharge = 0.0;
    Vector3 weightedPosition = Vector3::Zero();
    for (const Body& b : chargedBodies) {
        expectedTotalCharge += b.charge;
        weightedPosition += b.position * b.charge;
    }
    const Vector3 expectedCenterOfCharge = weightedPosition / expectedTotalCharge;

    CAPTURE(expectedTotalCharge);
    CAPTURE(tree.GetRoot().GetTotalCharge());

    REQUIRE_THAT(tree.GetRoot().GetTotalCharge(), WithinAbs(expectedTotalCharge, 1e-6));
    RequireWithinAbs(tree.GetRoot().GetCenterOfCharge(), expectedCenterOfCharge, 1e-6);
}
