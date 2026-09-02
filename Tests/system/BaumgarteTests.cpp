//
// BaumgarteTests.cpp
//
// Expands the three smoke tests in math/CollisionTests.cpp with the
// restitution sweep, positional-slop/correction behavior, and the
// floor-drop stability scenario the test spec calls out for the
// Baumgarte solver specifically (Section 13), separate from the fuller
// end-to-end floor scenario that belongs to the PhysicsWorld level
// (Section 25, Scenario H).
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <memory>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;

namespace {
    Handle MakeHandle(uint32_t index) {
        return Handle{index, 0};
    }

    void RequireWithinAbs(const Vector3& actual, const Vector3& expected, double tolerance) {
        REQUIRE_THAT(actual.x, WithinAbs(expected.x, tolerance));
        REQUIRE_THAT(actual.y, WithinAbs(expected.y, tolerance));
        REQUIRE_THAT(actual.z, WithinAbs(expected.z, tolerance));
    }
}

// ==========================================================================
// Restitution
// ==========================================================================

TEST_CASE("Baumgarte - Restitution 0 brings two equal-mass head-on bodies to a common rest velocity", "[Baumgarte][restitution]")
{
    BodyManager bodyManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);
    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(2.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0), 1.0, 1.0});

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.0;

    Baumgarte solver;
    solver.SetRestitution(0.0);
    solver.ResolveContacts({contact}, bodyManager);

    // Perfectly inelastic, equal mass, opposite equal speeds -> both stop.
    RequireWithinAbs(bodyManager.GetBody(a)->velocity, Vector3::Zero(), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(b)->velocity, Vector3::Zero(), 1e-9);
}

TEST_CASE("Baumgarte - Restitution 0.5 produces a partial bounce", "[Baumgarte][restitution]")
{
    BodyManager bodyManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);
    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(2.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0), 1.0, 1.0});

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.0;

    Baumgarte solver;
    solver.SetRestitution(0.5);
    solver.ResolveContacts({contact}, bodyManager);

    RequireWithinAbs(bodyManager.GetBody(a)->velocity, Vector3(-0.5, 0.0, 0.0), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(b)->velocity, Vector3(0.5, 0.0, 0.0), 1e-9);
}

TEST_CASE("Baumgarte - Restitution 1 fully swaps velocities for equal-mass bodies (perfectly elastic)", "[Baumgarte][restitution]")
{
    BodyManager bodyManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);
    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(2.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0), 1.0, 1.0});

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.0;

    Baumgarte solver;
    solver.SetRestitution(1.0);
    solver.ResolveContacts({contact}, bodyManager);

    RequireWithinAbs(bodyManager.GetBody(a)->velocity, Vector3(-1.0, 0.0, 0.0), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(b)->velocity, Vector3(1.0, 0.0, 0.0), 1e-9);
}

// ==========================================================================
// Positional correction
// ==========================================================================

TEST_CASE("Baumgarte - Zero penetration produces zero positional correction", "[Baumgarte][slop]")
{
    BodyManager bodyManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);
    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(2.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.0;

    Baumgarte solver;
    solver.ResolveContacts({contact}, bodyManager);

    RequireWithinAbs(bodyManager.GetBody(a)->position, Vector3(0.0, 0.0, 0.0), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(b)->position, Vector3(2.0, 0.0, 0.0), 1e-9);
}

TEST_CASE("Baumgarte - Penetration below the positional slop produces zero correction", "[Baumgarte][slop]")
{
    BodyManager bodyManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);
    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(2.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.005; // below the 0.01 slop, per Baumgarte.cpp

    Baumgarte solver;
    solver.ResolveContacts({contact}, bodyManager);

    RequireWithinAbs(bodyManager.GetBody(a)->position, Vector3(0.0, 0.0, 0.0), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(b)->position, Vector3(2.0, 0.0, 0.0), 1e-9);
}

TEST_CASE("Baumgarte - Penetration above the positional slop produces a nonzero correction", "[Baumgarte][slop]")
{
    BodyManager bodyManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);
    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(2.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.05;

    Baumgarte solver;
    solver.ResolveContacts({contact}, bodyManager);

    REQUIRE(bodyManager.GetBody(a)->position.x < 0.0);
    REQUIRE(bodyManager.GetBody(b)->position.x > 2.0);
}

TEST_CASE("Baumgarte - Separating bodies receive no impulse but still receive positional correction", "[Baumgarte]")
{
    BodyManager bodyManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);
    // Already moving apart along the normal.
    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(1.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), 1.0, 1.0});

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.1;

    Baumgarte solver;
    solver.ResolveContacts({contact}, bodyManager);

    // Velocities untouched since velocityAlongNormal > 0 (separating).
    RequireWithinAbs(bodyManager.GetBody(a)->velocity, Vector3(-1.0, 0.0, 0.0), 1e-9);
    RequireWithinAbs(bodyManager.GetBody(b)->velocity, Vector3(1.0, 0.0, 0.0), 1e-9);

    // But positional correction is unconditional on penetration alone.
    REQUIRE(bodyManager.GetBody(a)->position.x < 0.0);
    REQUIRE(bodyManager.GetBody(b)->position.x > 1.0);
}

TEST_CASE("Baumgarte - Unequal masses split positional correction inversely proportional to mass", "[Baumgarte]")
{
    BodyManager bodyManager;
    Handle heavy = MakeHandle(0);
    Handle light = MakeHandle(1);
    // heavy has 4x the mass of light -> 1/4 the invMass -> should move 1/4
    // as far under the same correction.
    bodyManager.AddBody(heavy, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 4.0, 0.25});
    bodyManager.AddBody(light, BodyData{Vector3(2.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});

    Contact contact;
    contact.a = heavy;
    contact.b = light;
    contact.normal = Vector3(1.0, 0.0, 0.0);
    contact.penetration = 0.1;

    Baumgarte solver;
    solver.ResolveContacts({contact}, bodyManager);

    double heavyDisplacement = -bodyManager.GetBody(heavy)->position.x;
    double lightDisplacement = bodyManager.GetBody(light)->position.x - 2.0;

    REQUIRE(heavyDisplacement > 0.0);
    REQUIRE(lightDisplacement > 0.0);
    REQUIRE_THAT(lightDisplacement / heavyDisplacement, WithinAbs(4.0, 1e-6));
}

// ==========================================================================
// Stability: a body dropped onto a static floor should settle, not explode
// ==========================================================================

TEST_CASE("Baumgarte - A sphere dropped onto a static floor settles without exploding or sinking through", "[Baumgarte][stability]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle floor = MakeHandle(0);
    Handle ball = MakeHandle(1);

    const double floorHalfExtent = 0.5;
    const double ballRadius = 0.5;
    const double restHeight = floorHalfExtent + ballRadius; // 1.0

    bodyManager.AddBody(floor, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0e6, 0.0});
    bodyManager.AddBody(ball, BodyData{Vector3(0.0, 3.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(floor, std::make_unique<ShapeCube>(floorHalfExtent));
    shapeManager.AddShape(ball, std::make_unique<ShapeSphere>(ballRadius));

    BroadPhase broadPhase;
    NarrowPhase narrowPhase;
    Baumgarte solver;

    const Vector3 gravity(0.0, -9.8, 0.0);
    const double dt = 0.005;
    const int steps = 2000; // 10 seconds simulated

    double maxAbsVelocityY = 0.0;

    for (int step = 0; step < steps; ++step) {
        BodyData* ballData = bodyManager.GetBody(ball);

        // Euler-Cromer style: integrate velocity first, then position.
        ballData->velocity += gravity * dt;
        ballData->position += ballData->velocity * dt;

        std::vector<CollisionCandidatePair> candidates = broadPhase.FindCandidatePairs(bodyManager, shapeManager);
        std::vector<Contact> contacts;
        narrowPhase.GenerateContacts(candidates, bodyManager, shapeManager, contacts);
        solver.ResolveContacts(contacts, bodyManager);

        REQUIRE(std::isfinite(bodyManager.GetBody(ball)->position.y));
        REQUIRE(std::isfinite(bodyManager.GetBody(ball)->velocity.y));

        // Track peak speed after the first quarter-second (skip the
        // initial free-fall accumulation, which is expected to grow).
        if (step > 50) {
            maxAbsVelocityY = std::max(maxAbsVelocityY, std::abs(bodyManager.GetBody(ball)->velocity.y));
        }
    }

    const BodyData* finalBall = bodyManager.GetBody(ball);

    // Settled near resting height, not sunk through the floor and not
    // left hovering far above it.
    REQUIRE_THAT(finalBall->position.y, WithinAbs(restHeight, 0.1));

    // No runaway energy gain: post-impact speeds should never exceed a
    // generous bound well above the ~7 m/s the initial 2.5 m drop imparts.
    REQUIRE(maxAbsVelocityY < 20.0);

    // Settled: final velocity should be small.
    REQUIRE(std::abs(finalBall->velocity.y) < 1.0);
}
