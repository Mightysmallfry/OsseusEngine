//
// NumericalEdgeCaseTests.cpp
//
// Section 29 of the test spec. Most per-subsystem edge cases already
// have direct coverage elsewhere (zero-vector normalize in
// Vector3Tests.cpp, coincident-shape EPA in CollisionTests.cpp,
// coincident-position Octree subdivision in OctreeTests.cpp, dt=0 and
// large-dt in PhysicsWorldEdgeCaseTests.cpp). This file covers the
// cross-subsystem extreme-magnitude cases that don't have a natural
// home elsewhere: near-singularity distances, very large masses, and
// very large positions.
//
// Every threshold below was empirically probed first rather than
// assumed -- in particular, the overflow test documents an actual
// floating-point limit of the engine (masses around 1e200 overflow
// double-precision Coulomb/gravity products), not a defect. It's
// included so that limit is a known, visible, tested boundary rather
// than a surprise.
//

#include <cmath>
#include <stdexcept>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;


TEST_CASE("Numerical edge cases - Gravity between near-singularity-close bodies stays finite, if very large", "[numerical-edge-cases][gravity]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 100.0, 0.0);
    tree.UpdateProperties();

    BodyData query;
    query.position = Vector3(1e-6, 0.0, 0.0); // 1 micrometer from the source
    query.mass = 1.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);
    const Vector3 force = gravity.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);

    REQUIRE(std::isfinite(force.x));
    REQUIRE(force.x < 0.0); // still pulls toward the source
    REQUIRE(std::abs(force.x) > 1e6); // and it's a huge force at this distance
}

TEST_CASE("Numerical edge cases - Gravity between very large (but not overflow-inducing) masses stays finite", "[numerical-edge-cases][gravity]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 1e150, 0.0);
    tree.UpdateProperties();

    BodyData query;
    query.position = Vector3(5.0, 0.0, 0.0);
    query.mass = 1e150;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);
    const Vector3 force = gravity.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);

    REQUIRE(std::isfinite(force.x));
}

TEST_CASE("Numerical edge cases - CURRENT BEHAVIOR: sufficiently extreme masses overflow double precision to infinity", "[numerical-edge-cases][gravity][regression]")
{
    // Documents where the floating-point ceiling actually sits rather
    // than leaving it as an unknown/assumed boundary. m1*m2 here is on
    // the order of 1e400, which exceeds a double's ~1.8e308 max -- this
    // is an inherent double-precision limit, not something the engine
    // can reasonably guard against without switching numeric types.
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 1e200, 0.0);
    tree.UpdateProperties();

    BodyData query;
    query.position = Vector3(5.0, 0.0, 0.0);
    query.mass = 1e200;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);
    const Vector3 force = gravity.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);

    REQUIRE(std::isinf(force.x));
}

TEST_CASE("Numerical edge cases - Electromagnetism between near-singularity-close opposite charges stays finite", "[numerical-edge-cases][em]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 1.0, 100.0);
    tree.UpdateProperties();

    BodyData query;
    query.position = Vector3(1e-6, 0.0, 0.0);
    query.charge = -1.0;

    UniversalElectroMag em;
    em.SetTheta(0.0);
    const Vector3 force = em.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);

    REQUIRE(std::isfinite(force.x));
    REQUIRE(std::abs(force.x) > 1e6);
}

TEST_CASE("Numerical edge cases - A body at a very large but finite position integrates one step without overflow", "[numerical-edge-cases][physicsworld]")
{
    PhysicsWorld world;
    Handle body = world.CreateBody(BodyData{Vector3(1e10, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), 1.0, 1.0},
                                   std::make_unique<ShapePoint>());

    world.Step(0.01);

    const BodyData* result = world.GetBody(body);
    REQUIRE(std::isfinite(result->position.x));
    REQUIRE(std::isfinite(result->velocity.x));
}

TEST_CASE("Numerical edge cases - ComputeWorldBounds (via RebuildOctree) produces a finite bounding box for a body at a very large position", "[numerical-edge-cases][physicsworld][octree]")
{
    PhysicsWorld world;
    world.CreateBody(BodyData{Vector3(1e8, -1e8, 1e8), Vector3::Zero(), 1.0, 1.0}, std::make_unique<ShapePoint>());

    world.RebuildOctree();

    const Vector3 com = world.GetOctree().GetRoot().GetCenterOfMass();
    REQUIRE(std::isfinite(com.x));
    REQUIRE(std::isfinite(com.y));
    REQUIRE(std::isfinite(com.z));
}

TEST_CASE("Numerical edge cases - Baumgarte positional correction stays finite for an extremely large penetration depth", "[numerical-edge-cases][Baumgarte]")
{
    BodyManager bodyManager;
    Handle a{0, 0};
    Handle b{1, 0};
    bodyManager.AddBody(a, BodyData{Vector3::Zero(), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(1.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 1e12; // absurdly large, e.g. from a degenerate shape configuration

    Baumgarte solver;
    solver.ResolveContacts({contact}, bodyManager);

    REQUIRE(std::isfinite(bodyManager.GetBody(a)->position.x));
    REQUIRE(std::isfinite(bodyManager.GetBody(b)->position.x));
}

TEST_CASE("Numerical edge cases - CURRENT BEHAVIOR: an extremely small mass makes Octree::Insert throw rather than produce a tiny finite force", "[numerical-edge-cases][gravity][regression]")
{
    // REAL BUG, not just an edge case: Vector3::operator/ throws
    // std::runtime_error for any |scalar| < 1e-12 -- but its own
    // comment directly above it states the opposite policy ("division
    // by zero produces inf/nan (IEEE 754 behavior)"), and the sibling
    // operator/= has no such check at all (silently produces inf/nan,
    // matching the documented policy). Octree::AccumulateEntry divides
    // by totalMass_ with no epsilon guard of its own, relying on
    // operator/ -- so ANY body whose mass (or a node's accumulated
    // mass) drops under 1e-12 throws an uncaught exception straight out
    // of Octree::Insert, which PhysicsWorld::Step() calls every single
    // step with no try/catch anywhere in the pipeline. In practice this
    // isn't just an astronomical-scale extreme: any simulation using
    // small-magnitude mass units (grams instead of kilograms, for
    // instance) could hit this on an ordinary object.
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);

    REQUIRE_THROWS_AS(tree.Insert(Handle{0, 0}, Vector3::Zero(), 1e-300, 0.0), std::runtime_error);
}

TEST_CASE("Numerical edge cases - CURRENT BEHAVIOR: the same mass throws even just under the 1e-12 threshold, and does not throw just above it", "[numerical-edge-cases][gravity][regression]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree treeBelow(bounds);
    REQUIRE_THROWS_AS(treeBelow.Insert(Handle{0, 0}, Vector3::Zero(), 9e-13, 0.0), std::runtime_error);

    Bounds bounds2{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree treeAbove(bounds2);
    REQUIRE_NOTHROW(treeAbove.Insert(Handle{0, 0}, Vector3::Zero(), 2e-12, 0.0));
}
