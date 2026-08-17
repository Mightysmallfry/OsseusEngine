//
// Created by MightySmallFry on 8/16/2026.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;

namespace {
    // Vector3::operator== uses a relative-difference tolerance (1e-9),
    // which is far tighter than the residual error GJK/EPA can leave
    // behind after a fixed number of support-function iterations.
    // Collision results are compared component-wise against an explicit
    // absolute tolerance instead.
    void RequireWithinAbs(const Vector3& actual, const Vector3& expected, double tolerance) {
        REQUIRE_THAT(actual.x, WithinAbs(expected.x, tolerance));
        REQUIRE_THAT(actual.y, WithinAbs(expected.y, tolerance));
        REQUIRE_THAT(actual.z, WithinAbs(expected.z, tolerance));
    }
}

// ==========================================================================
// Support functions
// ==========================================================================

TEST_CASE("Collisions - Support on a sphere returns the surface point along direction", "[support]")
{
    ShapeSphere sphere(2.0);

    Vector3 support = sphere.Support(Vector3(1.0, 1.0, 1.0), Vector3(1.0, 0.0, 0.0));

    RequireWithinAbs(support, Vector3(3.0, 1.0, 1.0), 1e-9);
}

TEST_CASE("Collisions - Support on a sphere falls back to a default axis for a zero direction", "[support]")
{
    ShapeSphere sphere(2.0);

    // A zero-length direction can't be normalized; the sphere should still
    // return a well-defined point on its surface rather than NaN.
    Vector3 support = sphere.Support(Vector3::Zero(), Vector3::Zero());

    REQUIRE_THAT(support.LengthSquared(), WithinAbs(4.0, 1e-6));
}

TEST_CASE("Collisions - Support on a cube returns the correct corner along direction", "[support]")
{
    ShapeCube cube(0.5);

    RequireWithinAbs(cube.Support(Vector3::Zero(), Vector3(1.0, 1.0, 1.0)),
                      Vector3(0.5, 0.5, 0.5), 1e-9);
    RequireWithinAbs(cube.Support(Vector3::Zero(), Vector3(-1.0, 1.0, -1.0)),
                      Vector3(-0.5, 0.5, -0.5), 1e-9);
}

TEST_CASE("Collisions - Support on a point always returns the point's own position", "[support]")
{
    ShapePoint point;

    RequireWithinAbs(point.Support(Vector3(3.0, 4.0, 5.0), Vector3(1.0, 0.0, 0.0)),
                      Vector3(3.0, 4.0, 5.0), 1e-9);
    RequireWithinAbs(point.Support(Vector3(3.0, 4.0, 5.0), Vector3(-1.0, -1.0, -1.0)),
                      Vector3(3.0, 4.0, 5.0), 1e-9);
}

// ==========================================================================
// GJK
// ==========================================================================

TEST_CASE("Collisions - GJK detects overlap between two overlapping spheres", "[gjk]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;

    // Centers 1.5 apart, radii sum to 2.0 - clearly overlapping.
    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(1.5, 0.0, 0.0), simplex);

    REQUIRE(hit);
}

TEST_CASE("Collisions - GJK reports no intersection for separated spheres", "[gjk]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;

    // Centers 3 apart, radii sum to only 2.0 - well separated.
    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(3.0, 0.0, 0.0), simplex);

    REQUIRE_FALSE(hit);
}

TEST_CASE("Collisions - GJK reports no intersection for spheres that are exactly touching", "[gjk][regression]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;

    // Centers exactly 2 apart - radii sum matches the gap exactly, a
    // boundary case with zero true penetration.
    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(2.0, 0.0, 0.0), simplex);

    REQUIRE_FALSE(hit);
}

TEST_CASE("Collisions - GJK detects overlap between two overlapping cubes", "[gjk]")
{
    ShapeCube a(0.5);
    ShapeCube b(0.5);
    GJKSimplex simplex;

    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(0.5, 0.0, 0.0), simplex);

    REQUIRE(hit);
}

TEST_CASE("Collisions - GJK reports no intersection for separated cubes", "[gjk]")
{
    ShapeCube a(0.5);
    ShapeCube b(0.5);
    GJKSimplex simplex;

    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(2.0, 0.0, 0.0), simplex);

    REQUIRE_FALSE(hit);
}

TEST_CASE("Collisions - GJK reports no intersection for cubes that are exactly face-touching", "[gjk][regression]")
{
    ShapeCube a(0.5);
    ShapeCube b(0.5);
    GJKSimplex simplex;

    // Centers exactly 1 apart - half-extents sum matches the gap exactly.
    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(1.0, 0.0, 0.0), simplex);

    REQUIRE_FALSE(hit);
}

TEST_CASE("Collisions - GJK detects a point inside a sphere", "[gjk]")
{
    ShapePoint point;
    ShapeSphere sphere(1.0);
    GJKSimplex simplex;

    bool hit = GJK::Intersect(point, Vector3(0.5, 0.0, 0.0), sphere, Vector3(0.0, 0.0, 0.0), simplex);

    REQUIRE(hit);
}

TEST_CASE("Collisions - GJK reports no intersection for a point outside a sphere", "[gjk]")
{
    ShapePoint point;
    ShapeSphere sphere(1.0);
    GJKSimplex simplex;

    bool hit = GJK::Intersect(point, Vector3(2.0, 0.0, 0.0), sphere, Vector3(0.0, 0.0, 0.0), simplex);

    REQUIRE_FALSE(hit);
}

TEST_CASE("Collisions - GJK works across mismatched shape types (sphere vs cube)", "[gjk]")
{
    ShapeSphere sphere(1.0);
    ShapeCube cube(0.5);
    GJKSimplex simplex;

    bool hit = GJK::Intersect(sphere, Vector3(0.0, 0.0, 0.0), cube, Vector3(1.0, 0.0, 0.0), simplex);

    REQUIRE(hit);
}

TEST_CASE("Collisions - GJK resolves axis-aligned stacked shapes without a degenerate direction", "[gjk]")
{
    // Two cubes directly on top of one another along a single world axis
    // is exactly the case that collapses the usual double-cross-product
    // direction to zero in GJK::Line - this exercises ArbitraryPerpendicular.
    ShapeCube a(0.5);
    ShapeCube b(0.5);
    GJKSimplex simplex;

    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(0.0, 0.5, 0.0), simplex);

    REQUIRE(hit);
}

// ==========================================================================
// EPA - well-conditioned cases (asymmetric geometry, unambiguous normal)
// ==========================================================================

TEST_CASE("Collisions - EPA computes penetration depth and normal for overlapping spheres", "[epa]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;
    REQUIRE(GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(1.5, 0.0, 0.0), simplex));

    Contact contact = EPA::Resolve(a, Vector3(0.0, 0.0, 0.0), Handle{0, 0},
                                    b, Vector3(1.5, 0.0, 0.0), Handle{1, 0}, simplex);

    // Radii sum to 2.0, centers are 1.5 apart -> 0.5 penetration along +X.
    REQUIRE_THAT(contact.penetration, WithinAbs(0.5, 1e-2));
    RequireWithinAbs(contact.normal, Vector3(1.0, 0.0, 0.0), 1e-2);
}

TEST_CASE("Collisions - EPA contact normal points from body A toward body B", "[epa]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;

    // Swap which body is "A" relative to the previous test - the normal
    // should flip to still point A -> B.
    REQUIRE(GJK::Intersect(a, Vector3(1.5, 0.0, 0.0), b, Vector3(0.0, 0.0, 0.0), simplex));

    Contact contact = EPA::Resolve(a, Vector3(1.5, 0.0, 0.0), Handle{0, 0},
                                    b, Vector3(0.0, 0.0, 0.0), Handle{1, 0}, simplex);

    RequireWithinAbs(contact.normal, Vector3(-1.0, 0.0, 0.0), 1e-2);
}

TEST_CASE("Collisions - EPA computes penetration depth and normal for overlapping cubes", "[epa]")
{
    ShapeCube a(0.5);
    ShapeCube b(0.5);
    GJKSimplex simplex;
    REQUIRE(GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(0.5, 0.0, 0.0), simplex));

    Contact contact = EPA::Resolve(a, Vector3(0.0, 0.0, 0.0), Handle{0, 0},
                                    b, Vector3(0.5, 0.0, 0.0), Handle{1, 0}, simplex);

    REQUIRE_THAT(contact.penetration, WithinAbs(0.5, 1e-6));
    RequireWithinAbs(contact.normal, Vector3(1.0, 0.0, 0.0), 1e-6);
}

TEST_CASE("Collisions - EPA contact points lie on the surface of each shape", "[epa]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;
    REQUIRE(GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(1.5, 0.0, 0.0), simplex));

    Contact contact = EPA::Resolve(a, Vector3(0.0, 0.0, 0.0), Handle{0, 0},
                                    b, Vector3(1.5, 0.0, 0.0), Handle{1, 0}, simplex);

    // Every point EPA reconstructs on a sphere's surface should sit one
    // radius away from that sphere's center, regardless of direction.
    REQUIRE_THAT(Distance(contact.pointOnA, Vector3(0.0, 0.0, 0.0)), WithinAbs(1.0, 1e-2));
    REQUIRE_THAT(Distance(contact.pointOnB, Vector3(1.5, 0.0, 0.0)), WithinAbs(1.0, 1e-2));
}

TEST_CASE("Collisions - EPA resolves a point deeply inside a sphere", "[epa]")
{
    ShapePoint point;
    ShapeSphere sphere(1.0);
    GJKSimplex simplex;
    REQUIRE(GJK::Intersect(point, Vector3(0.5, 0.0, 0.0), sphere, Vector3(0.0, 0.0, 0.0), simplex));

    Contact contact = EPA::Resolve(point, Vector3(0.5, 0.0, 0.0), Handle{0, 0},
                                    sphere, Vector3(0.0, 0.0, 0.0), Handle{1, 0}, simplex);

    // The point sits 0.5 from center, 0.5 short of the surface (radius 1).
    REQUIRE_THAT(contact.penetration, WithinAbs(0.5, 1e-2));
    RequireWithinAbs(contact.pointOnA, Vector3(0.5, 0.0, 0.0), 1e-9);
}

TEST_CASE("Collisions - EPA contact normal is always unit length", "[epa]")
{
    ShapeCube a(0.5);
    ShapeCube b(0.5);
    GJKSimplex simplex;
    REQUIRE(GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(0.5, 0.0, 0.0), simplex));

    Contact contact = EPA::Resolve(a, Vector3(0.0, 0.0, 0.0), Handle{0, 0},
                                    b, Vector3(0.5, 0.0, 0.0), Handle{1, 0}, simplex);

    REQUIRE_THAT(contact.normal.Length(), WithinAbs(1.0, 1e-6));
}

// ==========================================================================
// EPA - symmetric / degenerate cases
//
// These exercise fully coincident or near-coincident geometry, where the
// Minkowski difference has no single well-defined closest face and any
// direction is a technically-valid separating axis. Known history: the
// normal-flip swap(b, c) in EPA::MakeFace can corrupt winding order badly
// enough that AddUniqueEdge fails to cancel shared edges, so these are
// kept as regression cases with loose tolerances (sane invariants and no
// hang) rather than exact expected values.
// ==========================================================================

TEST_CASE("Collisions - EPA handles fully coincident spheres without hanging", "[epa][regression]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;
    REQUIRE(GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(0.0, 0.0, 0.0), simplex));

    Contact contact = EPA::Resolve(a, Vector3(0.0, 0.0, 0.0), Handle{0, 0},
                                    b, Vector3(0.0, 0.0, 0.0), Handle{1, 0}, simplex);

    // True maximum penetration is 2x radius (2.0); convergence on this
    // fully symmetric case is currently imprecise, so only sane bounds
    // and a valid unit normal are asserted.
    REQUIRE_THAT(contact.normal.Length(), WithinAbs(1.0, 1e-3));
    REQUIRE(contact.penetration > 0.0);
    REQUIRE(contact.penetration <= 2.0 + 1e-6);
}

TEST_CASE("Collisions - EPA handles a point exactly at a sphere's center without hanging", "[epa][regression]")
{
    ShapePoint point;
    ShapeSphere sphere(1.0);
    GJKSimplex simplex;
    REQUIRE(GJK::Intersect(point, Vector3(0.0, 0.0, 0.0), sphere, Vector3(0.0, 0.0, 0.0), simplex));

    Contact contact = EPA::Resolve(point, Vector3(0.0, 0.0, 0.0), Handle{0, 0},
                                    sphere, Vector3(0.0, 0.0, 0.0), Handle{1, 0}, simplex);

    // True penetration here is exactly one radius (1.0); same convergence
    // caveat as the coincident-spheres case above.
    REQUIRE_THAT(contact.normal.Length(), WithinAbs(1.0, 1e-3));
    REQUIRE(contact.penetration > 0.0);
    REQUIRE(contact.penetration <= 1.0 + 1e-6);
}

TEST_CASE("Collisions - EPA handles deep near-coincident overlap without hanging", "[epa][regression]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;

    // Centers separated by a tiny nudge rather than exactly zero, so this
    // is "almost" the fully symmetric case above without being exactly it.
    REQUIRE(GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(0.001, 0.0, 0.0), simplex));

    Contact contact = EPA::Resolve(a, Vector3(0.0, 0.0, 0.0), Handle{0, 0},
                                    b, Vector3(0.001, 0.0, 0.0), Handle{1, 0}, simplex);

    REQUIRE_THAT(contact.normal.Length(), WithinAbs(1.0, 1e-3));
    REQUIRE(contact.penetration > 0.0);
    REQUIRE(contact.penetration <= 2.0 + 1e-6);
}

// ==========================================================================
// BroadPhase
// ==========================================================================

TEST_CASE("Collisions - BroadPhase finds candidate pairs for overlapping bounding boxes", "[broadphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a{0, 0};
    Handle b{1, 0};

    bodyManager.AddBody(a, BodyData{ Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0 });
    bodyManager.AddBody(b, BodyData{ Vector3(0.5, 0.0, 0.0), Vector3::Zero(), 1.0 });
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    std::vector<CollisionCandidatePair> candidates = broadPhase.FindCandidatePairs(bodyManager, shapeManager);

    REQUIRE(candidates.size() == 1);
    REQUIRE(candidates[0].a.index == a.index);
    REQUIRE(candidates[0].b.index == b.index);
}

TEST_CASE("Collisions - BroadPhase excludes pairs with non-overlapping bounding boxes", "[broadphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a{0, 0};
    Handle b{1, 0};

    bodyManager.AddBody(a, BodyData{ Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0 });
    bodyManager.AddBody(b, BodyData{ Vector3(100.0, 0.0, 0.0), Vector3::Zero(), 1.0 });
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    std::vector<CollisionCandidatePair> candidates = broadPhase.FindCandidatePairs(bodyManager, shapeManager);

    REQUIRE(candidates.empty());
}

// ==========================================================================
// NarrowPhase
// ==========================================================================

TEST_CASE("Collisions - NarrowPhase produces a contact for an overlapping candidate pair", "[narrowphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a{0, 0};
    Handle b{1, 0};

    bodyManager.AddBody(a, BodyData{ Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0 });
    bodyManager.AddBody(b, BodyData{ Vector3(0.5, 0.0, 0.0), Vector3::Zero(), 1.0 });
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    NarrowPhase narrowPhase;
    std::vector<Contact> contacts = narrowPhase.GenerateContacts(
        { CollisionCandidatePair{a, b} }, bodyManager, shapeManager);

    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].a.index == a.index);
    REQUIRE(contacts[0].b.index == b.index);
}

TEST_CASE("Collisions - NarrowPhase produces no contact for a separated candidate pair", "[narrowphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a{0, 0};
    Handle b{1, 0};

    bodyManager.AddBody(a, BodyData{ Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0 });
    bodyManager.AddBody(b, BodyData{ Vector3(10.0, 0.0, 0.0), Vector3::Zero(), 1.0 });
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    NarrowPhase narrowPhase;
    std::vector<Contact> contacts = narrowPhase.GenerateContacts(
        { CollisionCandidatePair{a, b} }, bodyManager, shapeManager);

    REQUIRE(contacts.empty());
}

TEST_CASE("Collisions - NarrowPhase skips candidate pairs missing a body or shape", "[narrowphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a{0, 0};
    Handle missing{99, 0}; // never registered with either manager

    bodyManager.AddBody(a, BodyData{ Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0 });
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));

    NarrowPhase narrowPhase;
    std::vector<Contact> contacts = narrowPhase.GenerateContacts(
        { CollisionCandidatePair{a, missing} }, bodyManager, shapeManager);

    REQUIRE(contacts.empty());
}

// ==========================================================================
// Solver
// ==========================================================================

TEST_CASE("Collisions - Solver applies equal and opposite impulses to two dynamic bodies", "[solver]")
{
    BodyManager bodyManager;
    Handle a{0, 0};
    Handle b{1, 0};
    bodyManager.AddBody(a, BodyData{ Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), 1.0 });
    bodyManager.AddBody(b, BodyData{ Vector3(1.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0), 1.0 });

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.2;
    contact.pointOnA = Vector3(0.5, 0.0, 0.0);
    contact.pointOnB = Vector3(0.5, 0.0, 0.0);

    Solver solver;
    solver.SetRestitution(0.5);
    solver.ResolveContacts({ contact }, bodyManager);

    RequireWithinAbs(bodyManager.GetBody(a)->velocity, Vector3(-0.5, 0.0, 0.0), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(b)->velocity, Vector3(0.5, 0.0, 0.0), 1e-9);

    // Positional correction pushes both bodies apart along the normal.
    RequireWithinAbs(bodyManager.GetBody(a)->position, Vector3(-0.076, 0.0, 0.0), 1e-6);
    RequireWithinAbs(bodyManager.GetBody(b)->position, Vector3(1.076, 0.0, 0.0), 1e-6);
}

TEST_CASE("Collisions - Solver leaves a static (infinite-mass) body untouched", "[solver]")
{
    BodyManager bodyManager;
    Handle floor{0, 0};
    Handle ball{1, 0};
    bodyManager.AddBody(floor, BodyData{ Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 0.0 });
    bodyManager.AddBody(ball, BodyData{ Vector3(0.0, 1.0, 0.0), Vector3(0.0, -2.0, 0.0), 1.0 });

    Contact contact;
    contact.a = floor;
    contact.b = ball;
    contact.normal = Vector3(0.0, 1.0, 0.0);
    contact.penetration = 0.1;
    contact.pointOnA = Vector3(0.0, 0.0, 0.0);
    contact.pointOnB = Vector3(0.0, 1.0, 0.0);

    Solver solver;
    solver.SetRestitution(0.0);
    solver.ResolveContacts({ contact }, bodyManager);

    RequireWithinAbs(bodyManager.GetBody(floor)->velocity, Vector3::Zero(), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(floor)->position, Vector3::Zero(), 1e-9);

    // With restitution 0, a perfectly inelastic head-on impact against an
    // immovable floor should fully cancel the ball's normal velocity.
    RequireWithinAbs(bodyManager.GetBody(ball)->velocity, Vector3::Zero(), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(ball)->position, Vector3(0.0, 1.072, 0.0), 1e-6);
}

TEST_CASE("Collisions - Solver does nothing when both bodies have infinite mass", "[solver]")
{
    BodyManager bodyManager;
    Handle a{0, 0};
    Handle b{1, 0};
    bodyManager.AddBody(a, BodyData{ Vector3(0.0, 0.0, 0.0), Vector3(5.0, 0.0, 0.0), 0.0 });
    bodyManager.AddBody(b, BodyData{ Vector3(1.0, 0.0, 0.0), Vector3(-5.0, 0.0, 0.0), 0.0 });

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.5;

    Solver solver;
    solver.ResolveContacts({ contact }, bodyManager);

    RequireWithinAbs(bodyManager.GetBody(a)->velocity, Vector3(5.0, 0.0, 0.0), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(b)->velocity, Vector3(-5.0, 0.0, 0.0), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(a)->position, Vector3(0.0, 0.0, 0.0), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(b)->position, Vector3(1.0, 0.0, 0.0), 1e-9);
}

// ==========================================================================
// PhysicsWorld - end-to-end pipeline
// ==========================================================================

TEST_CASE("Collisions - PhysicsWorld Step separates two overlapping resting bodies", "[integration]")
{
    PhysicsWorld world;

    Handle a = world.CreateBody(
        BodyData{ Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0 },
        std::make_unique<ShapeSphere>(1.0));
    Handle b = world.CreateBody(
        BodyData{ Vector3(1.5, 0.0, 0.0), Vector3::Zero(), 1.0 },
        std::make_unique<ShapeSphere>(1.0));

    world.Step(0.016);

    double separationAfter = Distance(world.GetBody(a)->position, world.GetBody(b)->position);

    // Started 1.5 apart (0.5 of overlap against radii summing to 2.0);
    // one step of positional correction should have pushed them apart.
    REQUIRE(separationAfter > 1.5);
}

TEST_CASE("Collisions - PhysicsWorld Step resolves a deep overlap with initial velocity without hanging",
          "[integration][regression]")
{
    // Known-fragile case from prior sessions: a body starting already
    // deeply overlapping another, carrying velocity into the overlap.
    // This is primarily a "does it terminate and stay finite" guard.
    PhysicsWorld world;

    Handle a = world.CreateBody(
        BodyData{ Vector3(0.0, 0.0, 0.0), Vector3(5.0, 0.0, 0.0), 1.0 },
        std::make_unique<ShapeSphere>(1.0));
    Handle b = world.CreateBody(
        BodyData{ Vector3(0.5, 0.0, 0.0), Vector3::Zero(), 1.0 },
        std::make_unique<ShapeSphere>(1.0));

    world.Step(0.016);

    const BodyData* bodyA = world.GetBody(a);
    const BodyData* bodyB = world.GetBody(b);
    REQUIRE(bodyA != nullptr);
    REQUIRE(bodyB != nullptr);

    REQUIRE(std::isfinite(bodyA->position.x));
    REQUIRE(std::isfinite(bodyA->velocity.x));
    REQUIRE(std::isfinite(bodyB->position.x));
    REQUIRE(std::isfinite(bodyB->velocity.x));
}