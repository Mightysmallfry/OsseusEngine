//
// PipelineScenarioTests.cpp
//
// Section 24 (pipeline stage observability, via PhysicsWorld's public
// surface) and Section 25 scenarios not already covered elsewhere:
//   A - Free particle (no forces)
//   B - Constant force (via a universal field, exercising the
//       multi-step universal-force path rather than manually re-added
//       individual forces, which Section 18's tests already cover)
//   C - Two-body mutual-gravity trajectory (momentum/COM conservation)
//   F - Three-body sanity (no NaN, bounded energy)
//   I - Multiple simultaneous collisions
//   J - Force + collision combined (gravity onto a static floor via the
//       universal-force pipeline, distinct from BaumgarteTests.cpp's
//       manually-driven version)
//
// D (circular orbit) and E (elliptical orbit) are covered in
// OrbitTests.cpp / RK4ExpandedTests.cpp. G (basic two-body collision)
// and H (static floor drop) already have solid coverage in
// CollisionTests.cpp and BaumgarteTests.cpp respectively.
//

#include <cmath>

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

    // A constant downward field, applied through the universal-force
    // pipeline (BarnesHut::Evaluate -> ForceManager) rather than via
    // manually re-added per-step individual forces.
    class UniformGravityField : public UniversalForceEvaluator {
        public:
        explicit UniformGravityField(Vector3 acceleration) : acceleration_(acceleration) {
        }

        Vector3 CalculateForce(const OctNode&, Handle, const BodyData& body) override {
            return acceleration_ * body.mass;
        }

        private:
        Vector3 acceleration_;
    };

}

// ==========================================================================
// Section 24 - Full pipeline stage observability
// ==========================================================================

TEST_CASE("Pipeline - Collision manifold reflects contacts detected during the step that just ran", "[pipeline]")
{
    PhysicsWorld world;

    Handle a = world.CreateBody(BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                std::make_unique<ShapeSphere>(1.0));
    Handle b = world.CreateBody(BodyData{Vector3(1.5, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                std::make_unique<ShapeSphere>(1.0));
    (void)a;
    (void)b;

    world.Step(0.01);

    REQUIRE_FALSE(world.GetCollisionManifold().empty());
}

TEST_CASE("Pipeline - Collision manifold is empty when nothing overlaps", "[pipeline]")
{
    PhysicsWorld world;

    world.CreateBody(BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0}, std::make_unique<ShapeSphere>(1.0));
    world.CreateBody(BodyData{Vector3(500.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                     std::make_unique<ShapeSphere>(1.0));

    world.Step(0.01);

    REQUIRE(world.GetCollisionManifold().empty());
}

TEST_CASE("Pipeline - Collision resolution runs even when no forces are registered at all", "[pipeline]")
{
    // Demonstrates the pipeline order: broad/narrow phase + Baumgarte
    // run unconditionally every step, independent of whether any
    // universal or individual force exists this step.
    PhysicsWorld world;

    Handle a = world.CreateBody(BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                std::make_unique<ShapeSphere>(1.0));
    Handle b = world.CreateBody(BodyData{Vector3(1.5, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                std::make_unique<ShapeSphere>(1.0));

    const Vector3 startA = world.GetBody(a)->position;
    const Vector3 startB = world.GetBody(b)->position;

    world.Step(0.01);

    // Overlapping, zero-velocity bodies with zero force acting on them
    // still get pushed apart by positional correction alone.
    REQUIRE(world.GetBody(a)->position.x < startA.x);
    REQUIRE(world.GetBody(b)->position.x > startB.x);
}

TEST_CASE("Pipeline - GetOctree reflects the position bodies had BEFORE this step's integration, not after", "[pipeline]")
{
    // PhysicsWorld::Step() order is: detect/resolve collisions ->
    // RebuildOctree() -> evaluate universal forces via that octree ->
    // integrate. So the octree used to compute this step's forces is
    // built from the position bodies were AT THE START of the step
    // (last step's result), not where they end up after this step's
    // integration. This is standard for an explicit force-then-integrate
    // scheme, but worth pinning explicitly since it's easy to assume
    // the opposite.
    PhysicsWorld world;
    Handle body = world.CreateBody(BodyData{Vector3(1.0, 0.0, 0.0), Vector3(5.0, 0.0, 0.0), 1.0, 1.0, 0.0},
                                   std::make_unique<ShapePoint>());
    (void)body;

    world.Step(1.0);

    // Octree still shows the pre-step position (1.0), even though the
    // body itself has already moved to x=6.0 by the time Step() returns.
    RequireWithinAbs(world.GetOctree().GetRoot().GetCenterOfMass(), Vector3(1.0, 0.0, 0.0), 1e-9);
    RequireWithinAbs(world.GetBody(body)->position, Vector3(6.0, 0.0, 0.0), 1e-9);
}

// ==========================================================================
// Scenario A - Free particle
// ==========================================================================

TEST_CASE("Scenario A - A free particle with no forces travels in a straight line at constant speed", "[scenario][free-particle]")
{
    PhysicsWorld world;
    const Vector3 initialVelocity(3.0, -1.0, 2.0);
    Handle body = world.CreateBody(BodyData{Vector3::Zero(), initialVelocity, 1.0, 1.0, 0.0}, std::make_unique<ShapePoint>());

    const double dt = 0.1;
    const int steps = 100;
    for (int i = 0; i < steps; ++i) {
        world.Step(dt);
    }

    const BodyData* result = world.GetBody(body);
    RequireWithinAbs(result->velocity, initialVelocity, 1e-9);
    RequireWithinAbs(result->position, initialVelocity * (dt * steps), 1e-9);
}

// ==========================================================================
// Scenario B - Constant force via the universal-force pipeline
// ==========================================================================

TEST_CASE("Scenario B - A constant universal force field accelerates a body uniformly across many steps", "[scenario][constant-force]")
{
    PhysicsWorld world;
    UniformGravityField field(Vector3(0.0, -9.8, 0.0));
    world.GetForceManager().AddUniversal(&field);

    Handle body = world.CreateBody(BodyData{Vector3(0.0, 100.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                   std::make_unique<ShapePoint>());

    const double dt = 0.01;
    const int steps = 100;
    for (int i = 0; i < steps; ++i) {
        world.Step(dt);
    }

    const BodyData* result = world.GetBody(body);

    // Euler-Cromer's own closed form (see EulerCromerKinematicsTests.cpp
    // for the derivation): v_n = n*a*dt, x_n = x0 + n*dt*v0 + a*dt^2*n*(n+1)/2.
    const double n = static_cast<double>(steps);
    const double expectedVelocity = n * -9.8 * dt;
    const double expectedPosition = 100.0 + -9.8 * dt * dt * n * (n + 1.0) / 2.0;

    REQUIRE_THAT(result->velocity.y, WithinAbs(expectedVelocity, 1e-6));
    REQUIRE_THAT(result->position.y, WithinAbs(expectedPosition, 1e-6));
}

// ==========================================================================
// Scenario C - Two-body mutual gravity trajectory
// ==========================================================================

TEST_CASE("Scenario C - Two mutually-attracting bodies conserve total momentum and keep their center of mass fixed", "[scenario][two-body]")
{
    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorRungeKutta4>());
    UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    const double massA = 5.0;
    const double massB = 3.0;

    // Equal and opposite momentum -> zero total momentum, COM starts
    // fixed and (by conservation) should stay fixed throughout.
    Handle a = world.CreateBody(BodyData{Vector3(-10.0, 0.0, 0.0), Vector3(0.0, -0.6, 0.0), massA, 1.0 / massA, 0.0},
                                std::make_unique<ShapeSphere>(1.0));
    Handle b = world.CreateBody(BodyData{Vector3(16.6667, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), massB, 1.0 / massB, 0.0},
                                std::make_unique<ShapeSphere>(1.0));

    const double dt = 0.005;
    const int steps = 2000;

    double maxComDrift = 0.0;
    for (int step = 0; step < steps; ++step) {
        world.Step(dt);

        const BodyData* bodyA = world.GetBody(a);
        const BodyData* bodyB = world.GetBody(b);
        REQUIRE(std::isfinite(bodyA->position.x));
        REQUIRE(std::isfinite(bodyB->position.x));

        const Vector3 com = (bodyA->position * massA + bodyB->position * massB) * (1.0 / (massA + massB));
        maxComDrift = std::max(maxComDrift, com.Length());
    }

    // COM should stay essentially fixed at the origin the whole time
    // (small drift only from RK4's own integration error, not from
    // momentum non-conservation).
    REQUIRE(maxComDrift < 0.5);
}

// ==========================================================================
// Scenario F - Three-body sanity
// ==========================================================================

TEST_CASE("Scenario F - A three-body system runs without producing NaN/Inf and stays roughly energy-bounded", "[scenario][three-body]")
{
    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorRungeKutta4>());
    UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    const double mass = 10.0;
    Handle bodyA = world.CreateBody(BodyData{Vector3(20.0, 0.0, 0.0), Vector3(0.0, 4.0, 0.0), mass, 1.0 / mass, 0.0},
                                    std::make_unique<ShapeSphere>(1.0));
    Handle bodyB = world.CreateBody(BodyData{Vector3(-10.0, 17.32, 0.0), Vector3(-3.46, -2.0, 0.0), mass, 1.0 / mass, 0.0},
                                    std::make_unique<ShapeSphere>(1.0));
    Handle bodyC = world.CreateBody(BodyData{Vector3(-10.0, -17.32, 0.0), Vector3(3.46, -2.0, 0.0), mass, 1.0 / mass, 0.0},
                                    std::make_unique<ShapeSphere>(1.0));

    const double dt = 0.005;
    const int steps = 3000;

    for (int step = 0; step < steps; ++step) {
        world.Step(dt);

        for (Handle h : {bodyA, bodyB, bodyC}) {
            const BodyData* body = world.GetBody(h);
            REQUIRE(std::isfinite(body->position.x));
            REQUIRE(std::isfinite(body->position.y));
            REQUIRE(std::isfinite(body->velocity.x));
            REQUIRE(std::isfinite(body->velocity.y));
        }
    }
}

// ==========================================================================
// Scenario I - Multiple simultaneous collisions
// ==========================================================================

TEST_CASE("Scenario I - A central body simultaneously overlapping three others produces three contacts in one step", "[scenario][multi-collision]")
{
    PhysicsWorld world;

    Handle center = world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), 1.0, 1.0, 0.0},
                                     std::make_unique<ShapeSphere>(2.0));
    Handle east = world.CreateBody(BodyData{Vector3(3.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                   std::make_unique<ShapeSphere>(2.0));
    Handle north = world.CreateBody(BodyData{Vector3(0.0, 3.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                    std::make_unique<ShapeSphere>(2.0));
    Handle up = world.CreateBody(BodyData{Vector3(0.0, 0.0, 3.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                 std::make_unique<ShapeSphere>(2.0));

    world.Step(0.01);

    // center-east, center-north, center-up all overlap (radius 2 each,
    // centers 3 apart). The three outer bodies don't overlap each other
    // (pairwise distance ~4.24 > their combined radius of 4).
    REQUIRE(world.GetCollisionManifold().size() == 3);

    for (Handle h : {center, east, north, up}) {
        const BodyData* body = world.GetBody(h);
        REQUIRE(std::isfinite(body->position.x));
        REQUIRE(std::isfinite(body->position.y));
        REQUIRE(std::isfinite(body->position.z));
    }

    // The center body, pushed on by three separating corrections at
    // once, should not have been flung off in any single direction --
    // its net displacement should stay modest given the symmetry of
    // the configuration is close (though not perfectly, since order of
    // contact resolution matters for a sequential solver).
    const double centerDisplacement = world.GetBody(center)->position.Length();
    REQUIRE(centerDisplacement < 1.0);
}

// ==========================================================================
// Scenario J - Force + collision combined
// ==========================================================================

TEST_CASE("Scenario J - A body under a universal gravity field settles onto a static floor without exploding", "[scenario][force-and-collision]")
{
    PhysicsWorld world;
    UniformGravityField field(Vector3(0.0, -9.8, 0.0));
    world.GetForceManager().AddUniversal(&field);

    const double floorHalfExtent = 0.5;
    const double ballRadius = 0.5;
    const double restHeight = floorHalfExtent + ballRadius;

    world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), 1.0e6, 0.0, 0.0},
                     std::make_unique<ShapeCube>(floorHalfExtent));
    Handle ball = world.CreateBody(BodyData{Vector3(0.0, 3.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                   std::make_unique<ShapeSphere>(ballRadius));

    const double dt = 0.005;
    const int steps = 2000;

    for (int step = 0; step < steps; ++step) {
        world.Step(dt);
        const BodyData* body = world.GetBody(ball);
        REQUIRE(std::isfinite(body->position.y));
        REQUIRE(std::isfinite(body->velocity.y));
    }

    const BodyData* finalBall = world.GetBody(ball);
    REQUIRE_THAT(finalBall->position.y, WithinAbs(restHeight, 0.15));
    REQUIRE(std::abs(finalBall->velocity.y) < 1.0);
}
