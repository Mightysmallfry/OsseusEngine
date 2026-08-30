#include <algorithm>
#include <cmath>
#include <memory>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using Catch::Matchers::WithinRel;

// ==========================================================================
// Background
//
// Octree's default constructor (and Clear()) resets the root to a fixed
// Bounds{center = (0,0,0), halfSize = (1,1,1)} -- a 2-unit box around the
// origin. UniversalGravity::CalculateForce relies on that root's GetBounds()
// for two separate things:
//
//   1. inBounds() self-exclusion: a query body outside the node's bounds
//      is treated as "definitely not part of this node's contents", so
//      the self-exclusion check never fires for it.
//   2. The Barnes-Hut opening-angle test (width / dist < theta): "width"
//      comes from the node's bounds extent, not the actual spread of the
//      bodies it contains.
//
// If the root bounds don't reflect where the bodies actually are (as with
// the untouched 2-unit default box), both of those break at once for any
// body more than ~1 unit from the origin: self-exclusion never fires, and
// the traversal almost always treats the whole node as one aggregate point
// -- including the querying body's own mass -- instead of correctly
// recursing to the individual bodies.
//
// This is easy to miss in testing because the error is proportional to the
// query body's own mass share of the aggregate: a light test particle
// orbiting a much heavier fixed body (see OrbitTests.cpp) picks up only a
// tiny, tolerance-absorbed contamination from its own negligible mass. Two
// comparable-mass bodies (e.g. a binary system) make the same bug obvious:
// each body ends up attracted to an aggregate that's ~50% itself.
// ==========================================================================

namespace {

    double NewtonianForceMagnitude(double mass1, double mass2, double separation) {
        return OsseusConstants::GravitationalConstant * mass1 * mass2 / (separation * separation);
    }

} // namespace

TEST_CASE("Octree - SetRootBounds fixes gravity between two equal-mass bodies far from the origin",
          "[octree][barneshut][gravity][regression]")
{
    // Starts from the Octree() default constructor -- the fixed 2-unit box
    // around the origin that PhysicsWorld's spatialTree_ member used to be
    // stuck with permanently, since Clear() (called every RebuildOctree())
    // just recreates the root from that same stale rootBounds_. This is
    // exactly what PhysicsWorld::RebuildOctree() now does instead of
    // relying on the untouched default: derive real bounds from the actual
    // body positions and push them in via SetRootBounds() before inserting.
    //
    // Every other Barnes-Hut test in this suite constructs an Octree with
    // an explicit, correctly-sized Bounds up front, which is exactly why
    // this default-then-resize path wasn't already covered.
    osseus::Octree tree;

    constexpr double mass = 125000.0;
    constexpr double orbitalRadius = 200.0;

    const osseus::Handle handleA{0, 0};
    const osseus::Handle handleB{1, 0};
    const osseus::Vector3 positionA(-orbitalRadius, 0.0, 0.0);
    const osseus::Vector3 positionB(orbitalRadius, 0.0, 0.0);

    // The fix under test: resize the root to actually contain the bodies
    // before inserting, mirroring PhysicsWorld::RebuildOctree()'s
    // ComputeWorldBounds() + SetRootBounds() call pair.
    tree.SetRootBounds(osseus::Bounds{osseus::Vector3::Zero(), osseus::Vector3(1000.0, 1000.0, 1000.0)});

    tree.Insert(handleA, positionA, mass, 0.0);
    tree.Insert(handleB, positionB, mass, 0.0);
    tree.UpdateProperties();

    osseus::BodyData bodyA;
    bodyA.position = positionA;
    bodyA.mass = mass;

    osseus::UniversalGravity gravity; // default theta -- this is the path BinaryScenario actually exercises

    const osseus::Vector3 result = gravity.CalculateForce(tree.GetRoot(), handleA, bodyA);
    const double resultMagnitude = std::sqrt(result.x * result.x + result.y * result.y + result.z * result.z);

    const double separation = 2.0 * orbitalRadius;
    const double expectedMagnitude = NewtonianForceMagnitude(mass, mass, separation);

    // Before the fix this was 781250 (8x expectedMagnitude of 97656.25),
    // because the aggregate treated as the "other" body was actually both
    // bodies combined (including A's own mass) sitting at their barycenter,
    // 200 units away, rather than just B's mass at the true 400-unit
    // separation.
    REQUIRE_THAT(resultMagnitude, WithinRel(expectedMagnitude, 1e-9));

    // Force must pull A toward B (+X), not toward the barycenter by some
    // other magnitude/direction artifact.
    REQUIRE(result.x > 0.0);
}

TEST_CASE("PhysicsWorld - RebuildOctree derives root bounds from body positions so gravity stays undistorted",
          "[physicsworld][octree][gravity][regression]")
{
    osseus::PhysicsWorld world;
    world.SetIntegrator(std::make_unique<osseus::IntegratorEulerCromer>());

    osseus::UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    constexpr double mass = 125000.0;
    constexpr double orbitalRadius = 200.0;
    constexpr double delta = 1.0 / 60.0;

    const osseus::Handle handleA = world.CreateBody(
        osseus::BodyData{osseus::Vector3(-orbitalRadius, 0.0, 0.0), osseus::Vector3::Zero(), mass, 1.0 / mass, 0.0},
        std::make_unique<osseus::ShapeSphere>(20.0));

    world.CreateBody(
        osseus::BodyData{osseus::Vector3(orbitalRadius, 0.0, 0.0), osseus::Vector3::Zero(), mass, 1.0 / mass, 0.0},
        std::make_unique<osseus::ShapeSphere>(20.0));

    world.Step(delta);

    // IntegratorEulerCromer applies velocity += (force / mass) * delta as
    // its only update to velocity before this first Step() call returns,
    // so the resulting velocity change exactly encodes the force that was
    // computed -- letting this test check PhysicsWorld's real RebuildOctree
    // + BarnesHut pipeline without needing to peek at ForceManager
    // mid-Step (it's cleared again before Step() returns).
    const osseus::BodyData* bodyA = world.GetBody(handleA);
    REQUIRE(bodyA != nullptr);

    const double impliedAccel = bodyA->velocity.Length() / delta;
    const double impliedForce = impliedAccel * mass;

    const double separation = 2.0 * orbitalRadius;
    const double expectedForce = NewtonianForceMagnitude(mass, mass, separation);

    REQUIRE_THAT(impliedForce, WithinRel(expectedForce, 1e-6));
}

TEST_CASE("PhysicsWorld - a symmetric equal-mass binary holds its separation over one full orbit",
          "[physicsworld][octree][gravity][orbit][regression]")
{
    // End-to-end regression pinned to the exact BinaryScenario configuration
    // that surfaced this bug: two 125000-mass bodies at orbitalRadius=200,
    // given the analytic circular-orbit speed sqrt(G*M/(4r)) = 12.5. Before
    // the fix, this configuration collapsed from separation 400 to a
    // near-collision (~40) within about 7 simulated seconds; the orbital
    // period here is roughly 100s, so surviving one full period at a near
    // -constant separation is a meaningful stability check.
    osseus::PhysicsWorld world;
    world.SetIntegrator(std::make_unique<osseus::IntegratorEulerCromer>());

    osseus::UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    constexpr double mass = 125000.0;
    constexpr double orbitalRadius = 200.0;
    constexpr double orbitalSpeed = 12.5; // sqrt(mass / (4 * orbitalRadius)), G = 1
    constexpr double delta = 1.0 / 60.0;
    constexpr int steps = 6000; // 100 simulated seconds ~= one full orbital period

    const osseus::Handle handleA = world.CreateBody(
        osseus::BodyData{osseus::Vector3(-orbitalRadius, 0.0, 0.0), osseus::Vector3(0.0, -orbitalSpeed, 0.0), mass,
                         1.0 / mass, 0.0},
        std::make_unique<osseus::ShapeSphere>(20.0));

    const osseus::Handle handleB = world.CreateBody(
        osseus::BodyData{osseus::Vector3(orbitalRadius, 0.0, 0.0), osseus::Vector3(0.0, orbitalSpeed, 0.0), mass,
                         1.0 / mass, 0.0},
        std::make_unique<osseus::ShapeSphere>(20.0));

    const double initialSeparation = 2.0 * orbitalRadius;
    double maxSeparationError = 0.0;

    for (int step = 0; step < steps; ++step) {
        world.Step(delta);

        const osseus::BodyData* bodyA = world.GetBody(handleA);
        const osseus::BodyData* bodyB = world.GetBody(handleB);
        REQUIRE(bodyA != nullptr);
        REQUIRE(bodyB != nullptr);

        const double separation = (bodyB->position - bodyA->position).Length();
        REQUIRE(std::isfinite(separation));

        maxSeparationError = std::max(maxSeparationError, std::abs(separation - initialSeparation));
    }

    // Euler-Cromer drift is small over one period at this dt/T ratio; the
    // pre-fix bug wasn't subtle drift, it was outright collapse (separation
    // dropping by hundreds of units within seconds), so a generous but
    // still meaningfully tight bound is enough to catch a regression.
    REQUIRE(maxSeparationError < 1.0);
}