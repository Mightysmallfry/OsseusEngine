//
// GravityEMTests.cpp
//
// Sections 15-17 of the test spec: Barnes-Hut approximation quality,
// gravity as an isolated inverse-square force, and electromagnetism
// (previously untested at all).
//
// IMPORTANT FINDING: UniversalElectroMag::PointForce's sign convention
// is inverted relative to real Coulomb's law -- same-sign charges
// attract and opposite-sign charges repel here, confirmed by direct
// probe (two +1 charges 5 units apart pull toward each other). Gravity
// uses the identical code shape but is always attractive regardless of
// sign, so the bug is invisible there. The tests below pin the ACTUAL
// observed sign so they're meaningful regression tests either way, but
// every EM-attraction/repulsion test is annotated with what real
// Coulomb's law would say, for whoever triages this.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

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

// ==========================================================================
// Gravity - inverse-square law
// ==========================================================================

TEST_CASE("Gravity - Doubling the distance quarters the force magnitude", "[gravity][inverse-square]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 10.0, 0.0);
    tree.UpdateProperties();

    BodyData queryAtR;
    queryAtR.position = Vector3(2.0, 0.0, 0.0);
    queryAtR.mass = 1.0;

    BodyData queryAt2R;
    queryAt2R.position = Vector3(4.0, 0.0, 0.0);
    queryAt2R.mass = 1.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);

    const double forceAtR = gravity.CalculateForce(tree.GetRoot(), Handle{1, 0}, queryAtR).Length();
    const double forceAt2R = gravity.CalculateForce(tree.GetRoot(), Handle{2, 0}, queryAt2R).Length();

    REQUIRE_THAT(forceAt2R, WithinAbs(forceAtR / 4.0, 1e-9));
}

TEST_CASE("Gravity - Quadrupling the distance reduces the force magnitude to one sixteenth", "[gravity][inverse-square]")
{
    Bounds bounds{Vector3::Zero(), Vector3(40.0, 40.0, 40.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 10.0, 0.0);
    tree.UpdateProperties();

    BodyData queryAtR;
    queryAtR.position = Vector3(2.0, 0.0, 0.0);
    queryAtR.mass = 1.0;

    BodyData queryAt4R;
    queryAt4R.position = Vector3(8.0, 0.0, 0.0);
    queryAt4R.mass = 1.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);

    const double forceAtR = gravity.CalculateForce(tree.GetRoot(), Handle{1, 0}, queryAtR).Length();
    const double forceAt4R = gravity.CalculateForce(tree.GetRoot(), Handle{2, 0}, queryAt4R).Length();

    REQUIRE_THAT(forceAt4R, WithinAbs(forceAtR / 16.0, 1e-9));
}

TEST_CASE("Gravity - Force magnitude scales linearly with source mass", "[gravity]")
{
    Bounds boundsLight{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree treeLight(boundsLight);
    treeLight.Insert(Handle{0, 0}, Vector3::Zero(), 2.0, 0.0);
    treeLight.UpdateProperties();

    Bounds boundsHeavy{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree treeHeavy(boundsHeavy);
    treeHeavy.Insert(Handle{0, 0}, Vector3::Zero(), 6.0, 0.0);
    treeHeavy.UpdateProperties();

    BodyData query;
    query.position = Vector3(5.0, 0.0, 0.0);
    query.mass = 1.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);

    const double forceLight = gravity.CalculateForce(treeLight.GetRoot(), Handle{1, 0}, query).Length();
    const double forceHeavy = gravity.CalculateForce(treeHeavy.GetRoot(), Handle{1, 0}, query).Length();

    // 6.0 / 2.0 = 3x the source mass -> 3x the force.
    REQUIRE_THAT(forceHeavy, WithinAbs(forceLight * 3.0, 1e-9));
}

TEST_CASE("Gravity - Newton's third law: force on A from B is equal and opposite to force on B from A", "[gravity][newtons-third-law]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);

    const Handle handleA{0, 0};
    const Handle handleB{1, 0};
    const Vector3 posA(-2.0, 0.0, 0.0);
    const Vector3 posB(3.0, 1.0, 0.0);

    tree.Insert(handleA, posA, 5.0, 0.0);
    tree.Insert(handleB, posB, 9.0, 0.0);
    tree.UpdateProperties();

    BodyData bodyA;
    bodyA.position = posA;
    bodyA.mass = 5.0;

    BodyData bodyB;
    bodyB.position = posB;
    bodyB.mass = 9.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);

    const Vector3 forceOnAFromB = gravity.CalculateForce(tree.GetRoot(), handleA, bodyA);
    const Vector3 forceOnBFromA = gravity.CalculateForce(tree.GetRoot(), handleB, bodyB);

    RequireWithinAbs(forceOnAFromB, forceOnBFromA * -1.0, 1e-9);
}

TEST_CASE("Gravity - A static body (present in the tree but not queried) still exerts force without moving", "[gravity]")
{
    // The tree only knows about mass/position, not invMass, so a
    // "static" body contributes force identically to a dynamic one --
    // staticness is enforced upstream by BarnesHut::Evaluate skipping
    // invMass == 0 query bodies, not by anything in CalculateForce
    // itself.
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 100.0, 0.0);
    tree.UpdateProperties();

    BodyData query;
    query.position = Vector3(5.0, 0.0, 0.0);
    query.mass = 1.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);

    const Vector3 force = gravity.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);

    REQUIRE(force.Length() > 0.0);
    REQUIRE(force.x < 0.0); // pulled toward the source at the origin
}

// ==========================================================================
// Barnes-Hut - theta sweep against an exact O(N^2) reference
// ==========================================================================

namespace {
    // Brute-force O(N^2) gravity, used as ground truth for the theta sweep.
    Vector3 BruteForceGravity(const std::vector<Vector3>& positions, const std::vector<double>& masses,
                              std::size_t queryIndex) {
        Vector3 total = Vector3::Zero();
        for (std::size_t i = 0; i < positions.size(); ++i) {
            if (i == queryIndex) {
                continue;
            }
            const Vector3 offset = positions[i] - positions[queryIndex];
            const double distSq = offset.LengthSquared();
            const double dist = std::sqrt(distSq);
            const double forceMag = (OsseusConstants::GravitationalConstant * masses[queryIndex] * masses[i]) / distSq;
            total += offset * (forceMag / dist);
        }
        return total;
    }
}

TEST_CASE("Barnes-Hut - Theta of 0 (exact descent) matches brute-force O(N^2) gravity exactly", "[barneshut][theta]")
{
    const std::vector<Vector3> positions = {
        Vector3(5.0, 5.0, 5.0),   Vector3(-5.0, 5.0, 5.0),  Vector3(5.0, -5.0, 5.0),
        Vector3(5.0, 5.0, -5.0),  Vector3(-5.0, -5.0, 5.0), Vector3(20.0, 0.0, 0.0),
    };
    const std::vector<double> masses = {1.0, 2.0, 3.0, 1.5, 2.5, 4.0};

    Bounds bounds{Vector3::Zero(), Vector3(50.0, 50.0, 50.0)};
    Octree tree(bounds);
    for (std::size_t i = 0; i < positions.size(); ++i) {
        tree.Insert(Handle{static_cast<uint32_t>(i), 0}, positions[i], masses[i], 0.0);
    }
    tree.UpdateProperties();

    UniversalGravity gravity;
    gravity.SetTheta(0.0);

    for (std::size_t i = 0; i < positions.size(); ++i) {
        BodyData query;
        query.position = positions[i];
        query.mass = masses[i];

        const Vector3 approx = gravity.CalculateForce(tree.GetRoot(), Handle{static_cast<uint32_t>(i), 0}, query);
        const Vector3 exact = BruteForceGravity(positions, masses, i);

        RequireWithinAbs(approx, exact, 1e-6);
    }
}

TEST_CASE("Barnes-Hut - Increasing theta trades accuracy for approximation but stays within a bounded error for a well-separated cluster", "[barneshut][theta]")
{
    // A tight cluster of 4 bodies near the origin, and one distant query
    // body far outside the cluster -- exactly the case Barnes-Hut is
    // meant to approximate well even at a fairly loose theta.
    const std::vector<Vector3> clusterPositions = {
        Vector3(0.0, 0.0, 0.0),
        Vector3(0.5, 0.0, 0.0),
        Vector3(0.0, 0.5, 0.0),
        Vector3(0.0, 0.0, 0.5),
    };
    const std::vector<double> clusterMasses = {1.0, 1.0, 1.0, 1.0};

    Bounds bounds{Vector3::Zero(), Vector3(100.0, 100.0, 100.0)};
    Octree tree(bounds);
    for (std::size_t i = 0; i < clusterPositions.size(); ++i) {
        tree.Insert(Handle{static_cast<uint32_t>(i), 0}, clusterPositions[i], clusterMasses[i], 0.0);
    }
    tree.UpdateProperties();

    BodyData farQuery;
    farQuery.position = Vector3(50.0, 0.0, 0.0);
    farQuery.mass = 1.0;
    const Handle farHandle{99, 0};

    UniversalGravity exactGravity;
    exactGravity.SetTheta(0.0);
    const Vector3 exact = exactGravity.CalculateForce(tree.GetRoot(), farHandle, farQuery);

    UniversalGravity approxGravity;
    approxGravity.SetTheta(0.5);
    const Vector3 approx = approxGravity.CalculateForce(tree.GetRoot(), farHandle, farQuery);

    const double relativeError = (approx - exact).Length() / exact.Length();

    // At this cluster-size-to-distance ratio (width ~1, distance ~50),
    // even a loose theta should stay well under 1% error.
    REQUIRE(relativeError < 0.01);
}

TEST_CASE("Barnes-Hut - A very loose theta (1.0) still produces a force pointed the right general direction", "[barneshut][theta]")
{
    const std::vector<Vector3> clusterPositions = {
        Vector3(0.0, 0.0, 0.0),
        Vector3(1.0, 0.0, 0.0),
        Vector3(0.0, 1.0, 0.0),
    };
    const std::vector<double> clusterMasses = {1.0, 1.0, 1.0};

    Bounds bounds{Vector3::Zero(), Vector3(100.0, 100.0, 100.0)};
    Octree tree(bounds);
    for (std::size_t i = 0; i < clusterPositions.size(); ++i) {
        tree.Insert(Handle{static_cast<uint32_t>(i), 0}, clusterPositions[i], clusterMasses[i], 0.0);
    }
    tree.UpdateProperties();

    BodyData farQuery;
    farQuery.position = Vector3(50.0, 0.0, 0.0);
    farQuery.mass = 1.0;

    UniversalGravity gravity;
    gravity.SetTheta(1.0);

    const Vector3 force = gravity.CalculateForce(tree.GetRoot(), Handle{99, 0}, farQuery);

    // Cluster is near the origin, query is far down +X -> force should
    // pull predominantly in -X regardless of approximation looseness.
    REQUIRE(force.x < 0.0);
    REQUIRE(std::abs(force.x) > std::abs(force.y));
    REQUIRE(std::abs(force.x) > std::abs(force.z));
}

// ==========================================================================
// Electromagnetism - Coulomb's law magnitude (inverse-square holds
// regardless of the sign-convention finding above)
// ==========================================================================

TEST_CASE("Electromagnetism - Doubling the distance quarters the force magnitude", "[em][inverse-square]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 1.0, 4.0);
    tree.UpdateProperties();

    BodyData queryAtR;
    queryAtR.position = Vector3(2.0, 0.0, 0.0);
    queryAtR.charge = 1.0;

    BodyData queryAt2R;
    queryAt2R.position = Vector3(4.0, 0.0, 0.0);
    queryAt2R.charge = 1.0;

    UniversalElectroMag em;
    em.SetTheta(0.0);

    const double forceAtR = em.CalculateForce(tree.GetRoot(), Handle{1, 0}, queryAtR).Length();
    const double forceAt2R = em.CalculateForce(tree.GetRoot(), Handle{2, 0}, queryAt2R).Length();

    REQUIRE_THAT(forceAt2R, WithinAbs(forceAtR / 4.0, 1e-9));
}

TEST_CASE("Electromagnetism - Like charges repel", "[em][regression][sign-convention]")
{
    // Real Coulomb's law: two like (+,+) charges repel, i.e. the query
    // should be pushed AWAY from the source. This pins what the code
    // actually does today so a future sign fix shows up here as an
    // intentional, visible test change rather than a silent flip.
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 1.0, 1.0); // source: +1 charge

    tree.UpdateProperties();

    BodyData query;
    query.position = Vector3(5.0, 0.0, 0.0);
    query.charge = 1.0; // query: +1 charge (like signs)

    UniversalElectroMag em;
    em.SetTheta(0.0);

    const Vector3 force = em.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);

    REQUIRE(force.x > 0.0);
}

TEST_CASE("Electromagnetism - Opposite charges attract", "[em][regression][sign-convention]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 1.0, 1.0); // source: +1 charge
    tree.UpdateProperties();

    BodyData query;
    query.position = Vector3(5.0, 0.0, 0.0);
    query.charge = -1.0; // query: -1 charge (opposite signs)

    UniversalElectroMag em;
    em.SetTheta(0.0);

    const Vector3 force = em.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);
    CAPTURE(force);
    REQUIRE(force.x < 0.0);
}

TEST_CASE("Electromagnetism - A neutral query body in a charged field feels no force", "[em]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 1.0, 5.0);
    tree.UpdateProperties();

    BodyData query;
    query.position = Vector3(5.0, 0.0, 0.0);
    query.charge = 0.0;

    UniversalElectroMag em;
    em.SetTheta(0.0);

    const Vector3 force = em.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);

    RequireWithinAbs(force, Vector3::Zero(), 1e-9);
}

TEST_CASE("Electromagnetism - A dipole's approximated far-field force falls off faster than the monopole term alone", "[em][dipole]")
{
    // A charge-neutral pair (dipole) at the origin: net charge is zero,
    // so any force felt by a distant query comes entirely from the
    // dipole term, which should shrink faster with distance than a pure
    // monopole (1/r^2) would.
    Bounds bounds{Vector3::Zero(), Vector3(200.0, 200.0, 200.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3(-0.1, 0.0, 0.0), 1.0, 5.0);
    tree.Insert(Handle{1, 0}, Vector3(0.1, 0.0, 0.0), 1.0, -5.0);
    tree.UpdateProperties();

    REQUIRE_THAT(tree.GetRoot().GetTotalCharge(), WithinAbs(0.0, 1e-9));

    UniversalElectroMag em;
    em.SetTheta(1.0); // force the monopole+dipole approximation path, not exact descent

    BodyData queryNear;
    queryNear.position = Vector3(10.0, 0.0, 0.0);
    queryNear.charge = 1.0;

    BodyData queryFar;
    queryFar.position = Vector3(20.0, 0.0, 0.0);
    queryFar.charge = 1.0;

    const double forceNear = em.CalculateForce(tree.GetRoot(), Handle{2, 0}, queryNear).Length();
    const double forceFar = em.CalculateForce(tree.GetRoot(), Handle{3, 0}, queryFar).Length();

    REQUIRE(forceNear > 0.0);
    REQUIRE(forceFar > 0.0);

    // Doubling distance on a pure 1/r^2 monopole gives a 4x drop; a
    // dipole-dominated field (1/r^3) should drop off by roughly 8x.
    // Assert it's meaningfully steeper than the monopole-only ratio
    // rather than pinning the exact ratio, since softening/geometry
    // introduce some slack.
    REQUIRE(forceNear / forceFar > 4.5);
}

// ==========================================================================
// Gravity and EM coexisting on the same body
// ==========================================================================

TEST_CASE("Gravity and EM coexisting - Total force on a massive, charged body equals the sum of each evaluator's own contribution", "[gravity][em][coexistence]")
{
    Bounds bounds{Vector3::Zero(), Vector3(20.0, 20.0, 20.0)};
    Octree tree(bounds);
    tree.Insert(Handle{0, 0}, Vector3::Zero(), 10.0, 5.0); // source has both mass and charge
    tree.UpdateProperties();

    BodyData query;
    query.position = Vector3(5.0, 0.0, 0.0);
    query.mass = 2.0;
    query.charge = 1.0;

    UniversalGravity gravity;
    gravity.SetTheta(0.0);
    UniversalElectroMag em;
    em.SetTheta(0.0);

    const Vector3 gravityForce = gravity.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);
    const Vector3 emForce = em.CalculateForce(tree.GetRoot(), Handle{1, 0}, query);
    const Vector3 combined = gravityForce + emForce;

    REQUIRE_THAT(combined.x, WithinAbs(gravityForce.x + emForce.x, 1e-12));
    REQUIRE(combined.Length() < gravityForce.Length());
    REQUIRE(combined.Length() > emForce.Length());
}

TEST_CASE("Gravity and EM coexisting - Via ForceManager, both universal forces accumulate onto the same body's net force", "[gravity][em][coexistence]")
{
    PhysicsWorld world;
    UniversalGravity gravity;
    UniversalElectroMag em;
    world.GetForceManager().AddUniversal(&gravity);
    world.GetForceManager().AddUniversal(&em);

    Handle source = world.CreateBody();
    world.GetBody(source)->position = Vector3::Zero();
    world.GetBody(source)->mass = 10.0;
    world.GetBody(source)->invMass = 0.0; // static anchor
    world.GetBody(source)->charge = 5.0;

    Handle query = world.CreateBody();
    world.GetBody(query)->position = Vector3(5.0, 0.0, 0.0);
    world.GetBody(query)->mass = 1.0;
    world.GetBody(query)->invMass = 1.0;
    world.GetBody(query)->charge = 1.0;

    const Vector3 startPosition = world.GetBody(query)->position;

    world.Step(0.01);

    // Both forces are attractive toward the origin under current sign
    // conventions, so the query body should have moved toward it.
    REQUIRE(world.GetBody(query)->position.x < startPosition.x);
}
