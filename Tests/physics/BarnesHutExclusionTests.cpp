#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;

namespace {

    // Mirrors UniversalGravity::PointForce / UniversalElectroMag::PointForce
    // exactly, so exact-descent (theta = 0, no Barnes-Hut approximation)
    // traversal results can be pinned against a hand-derived analytic
    // answer rather than re-deriving the octree's own internal math.
    Vector3 ExpectedGravityPointForce(const Vector3& sourcePosition, double sourceMass,
                                       const Vector3& queryPosition, double queryMass) {
        const Vector3 offset = sourcePosition - queryPosition;
        const double distSq = offset.LengthSquared();
        const double dist = std::sqrt(distSq);
        const double forceMag = (OsseusConstants::GravitationalConstant * queryMass * sourceMass) / distSq;
        return offset * (forceMag / dist);
    }

}

// ==========================================================================
// Bounds::Contains
// ==========================================================================

TEST_CASE("Bounds - Contains reports true for a point inside the bounds", "[bounds]")
{
    const Bounds bounds{ Vector3::Zero(), Vector3(10.0, 10.0, 10.0) };

    REQUIRE(bounds.Contains(Vector3(1.0, -2.0, 3.0)));
}

TEST_CASE("Bounds - Contains reports false for a point outside the bounds", "[bounds]")
{
    const Bounds bounds{ Vector3::Zero(), Vector3(10.0, 10.0, 10.0) };

    REQUIRE_FALSE(bounds.Contains(Vector3(11.0, 0.0, 0.0)));
    REQUIRE_FALSE(bounds.Contains(Vector3(0.0, -20.0, 0.0)));
}

TEST_CASE("Bounds - Contains treats points exactly on the boundary as inside", "[bounds][boundary]")
{
    // Inclusive on both ends by design: GetOctantIndex resolves boundary
    // ties with >=, so a strict Contains could wrongly reject the one
    // node that actually owns a boundary position. Being inclusive on
    // both sides can occasionally admit a neighboring node too, but that
    // only costs a redundant (and still correct) ContainsBody descent.
    const Bounds bounds{ Vector3::Zero(), Vector3(10.0, 10.0, 10.0) };

    REQUIRE(bounds.Contains(Vector3(10.0, 10.0, 10.0)));
    REQUIRE(bounds.Contains(Vector3(-10.0, -10.0, -10.0)));
}

TEST_CASE("Bounds - Contains accounts for a non-origin center", "[bounds]")
{
    const Bounds bounds{ Vector3(5.0, 5.0, 5.0), Vector3(2.0, 2.0, 2.0) };

    REQUIRE(bounds.Contains(Vector3(5.0, 5.0, 5.0)));
    REQUIRE(bounds.Contains(Vector3(7.0, 3.0, 6.0)));
    REQUIRE_FALSE(bounds.Contains(Vector3(8.0, 5.0, 5.0)));
}

// ==========================================================================
// UniversalGravity - self-exclusion (leaf case)
// ==========================================================================

TEST_CASE("UniversalGravity - CalculateForce excludes self-force for the only body in the tree", "[barneshut][gravity][self-exclusion]")
{
    const Bounds bounds{ Vector3::Zero(), Vector3(10.0, 10.0, 10.0) };
    Octree tree(bounds);

    const Handle handle{0, 0};
    const Vector3 position(1.0, 1.0, 1.0);
    tree.Insert(handle, position, 5.0, 0.0);

    BodyData body;
    body.position = position;
    body.mass = 5.0;

    UniversalGravity gravity;

    REQUIRE(gravity.CalculateForce(tree.GetRoot(), handle, body) == Vector3::Zero());
}

// ==========================================================================
// UniversalGravity - self-exclusion after subdivision
// ==========================================================================

TEST_CASE("UniversalGravity - CalculateForce excludes self-force after the tree subdivides several levels deep", "[barneshut][gravity][self-exclusion][subdivide]")
{
    // Same closely-spaced positions used in OctreeTests' parent-chain
    // probe: these two bodies travel together through several octant
    // levels before finally separating. That deep, mostly-shared descent
    // is exactly the scenario the bounds pre-check needs to stay correct
    // for -- most nodes on the path do spatially contain both positions.
    const Bounds bounds{ Vector3::Zero(), Vector3(100.0, 100.0, 100.0) };
    Octree tree(bounds);

    const Handle handleA{0, 0};
    const Handle handleB{1, 0};
    const Vector3 positionA(1.0, 1.0, 1.0);
    const Vector3 positionB(1.5, 1.5, 1.5);

    tree.Insert(handleA, positionA, 2.0, 0.0);
    tree.Insert(handleB, positionB, 3.0, 0.0);

    REQUIRE_FALSE(tree.GetRoot().IsLeaf()); // sanity: this did subdivide

    BodyData bodyA;
    bodyA.position = positionA;
    bodyA.mass = 2.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0); // force exact full descent, no Barnes-Hut approximation

    const Vector3 result = gravity.CalculateForce(tree.GetRoot(), handleA, bodyA);
    const Vector3 expected = ExpectedGravityPointForce(positionB, 3.0, positionA, 2.0);

    REQUIRE(result == expected);
}

TEST_CASE("UniversalGravity - CalculateForce still applies force from a spatially close but distinct body", "[barneshut][gravity][regression]")
{
    // Regression guard for the bounds pre-check specifically: a
    // neighboring body whose position is close to (but not equal to)
    // the query position must still exert force. The bounds check is
    // only a cheap reject for nodes that can't possibly matter -- it
    // must never substitute for the handle-based identity check.
    const Bounds bounds{ Vector3::Zero(), Vector3(100.0, 100.0, 100.0) };
    Octree tree(bounds);

    const Handle handleA{0, 0};
    const Handle handleB{1, 0};
    const Vector3 positionA(1.0, 1.0, 1.0);
    const Vector3 positionB(1.5, 1.5, 1.5);

    tree.Insert(handleA, positionA, 2.0, 0.0);
    tree.Insert(handleB, positionB, 3.0, 0.0);

    BodyData bodyB;
    bodyB.position = positionB;
    bodyB.mass = 3.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);

    const Vector3 result = gravity.CalculateForce(tree.GetRoot(), handleB, bodyB);
    const Vector3 expected = ExpectedGravityPointForce(positionA, 2.0, positionB, 3.0);

    REQUIRE(result == expected);
    REQUIRE_FALSE(result == Vector3::Zero());
}

TEST_CASE("UniversalGravity - CalculateForce sums exact contributions from multiple other bodies", "[barneshut][gravity]")
{
    const Bounds bounds{ Vector3::Zero(), Vector3(100.0, 100.0, 100.0) };
    Octree tree(bounds);

    const Handle handleA{0, 0};
    const Vector3 positionA(10.0, 0.0, 0.0);
    tree.Insert(handleA, positionA, 2.0, 0.0);

    const Vector3 positionB(-10.0, 0.0, 0.0);
    tree.Insert(Handle{1, 0}, positionB, 3.0, 0.0);

    const Vector3 positionC(0.0, 10.0, 0.0);
    tree.Insert(Handle{2, 0}, positionC, 5.0, 0.0);

    BodyData bodyA;
    bodyA.position = positionA;
    bodyA.mass = 2.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0); // exact descent -- no approximation to account for

    const Vector3 result = gravity.CalculateForce(tree.GetRoot(), handleA, bodyA);
    const Vector3 expected = ExpectedGravityPointForce(positionB, 3.0, positionA, 2.0)
                            + ExpectedGravityPointForce(positionC, 5.0, positionA, 2.0);

    REQUIRE(result == expected);
}

TEST_CASE("UniversalGravity - CalculateForce still approximates a distant node under default theta", "[barneshut][gravity]")
{
    // Sanity check that the bounds pre-check doesn't interfere with the
    // ordinary Barnes-Hut approximation path (farEnough == true, a whole
    // node treated as one point mass) at default theta -- only the
    // self-exclusion checks changed, not the opening-angle criterion.
    const Bounds bounds{ Vector3::Zero(), Vector3(1000.0, 1000.0, 1000.0) };
    Octree tree(bounds);

    const Handle handleA{0, 0};
    const Vector3 positionA(500.0, 0.0, 0.0);
    tree.Insert(handleA, positionA, 1.0, 0.0);

    tree.Insert(Handle{1, 0}, Vector3(-500.0, 1.0, 0.0), 10.0, 0.0);
    tree.Insert(Handle{2, 0}, Vector3(-500.0, -1.0, 0.0), 10.0, 0.0);

    BodyData bodyA;
    bodyA.position = positionA;
    bodyA.mass = 1.0;

    UniversalGravity gravity; // default theta

    const Vector3 result = gravity.CalculateForce(tree.GetRoot(), handleA, bodyA);

    // The far-side pair pulls A in the -X direction; A contributes
    // nothing to its own force.
    REQUIRE(result.x < 0.0);
    REQUIRE_FALSE(result == Vector3::Zero());
}

// ==========================================================================
// UniversalElectroMag - self-exclusion (mirrors the gravity evaluator;
// same redundant-descent pattern, same bounds-check fix)
// ==========================================================================

TEST_CASE("UniversalElectroMag - CalculateForce excludes self-force for the only charged body in the tree", "[barneshut][em][self-exclusion]")
{
    const Bounds bounds{ Vector3::Zero(), Vector3(10.0, 10.0, 10.0) };
    Octree tree(bounds);

    const Handle handle{0, 0};
    const Vector3 position(1.0, 1.0, 1.0);
    tree.Insert(handle, position, 1.0, 4.0); // mass just gates tree insertion; charge is what matters here

    BodyData body;
    body.position = position;
    body.mass = 1.0;
    body.charge = 4.0;

    UniversalElectroMag electroMag;

    REQUIRE(electroMag.CalculateForce(tree.GetRoot(), handle, body) == Vector3::Zero());
}

TEST_CASE("UniversalElectroMag - CalculateForce excludes self-force after the tree subdivides several levels deep", "[barneshut][em][self-exclusion][subdivide]")
{
    const Bounds bounds{ Vector3::Zero(), Vector3(100.0, 100.0, 100.0) };
    Octree tree(bounds);

    const Handle handleA{0, 0};
    const Handle handleB{1, 0};
    const Vector3 positionA(1.0, 1.0, 1.0);
    const Vector3 positionB(1.5, 1.5, 1.5);

    tree.Insert(handleA, positionA, 1.0, 2.0);
    tree.Insert(handleB, positionB, 1.0, -3.0);

    REQUIRE_FALSE(tree.GetRoot().IsLeaf()); // sanity: this did subdivide

    BodyData bodyA;
    bodyA.position = positionA;
    bodyA.charge = 2.0;

    UniversalElectroMag electroMag;
    electroMag.SetTheta(0.0); // exact descent -- stays on the PointForce path, never opens the dipole term

    const Vector3 offset = positionB - positionA;
    const double distSq = offset.LengthSquared();
    const double dist = std::sqrt(distSq);
    const double forceMag = -(OsseusConstants::CoulombConstant * 2.0 * -3.0) / distSq;
    const Vector3 expected = offset * (forceMag / dist);

    const Vector3 result = electroMag.CalculateForce(tree.GetRoot(), handleA, bodyA);

    REQUIRE(result == expected);
}