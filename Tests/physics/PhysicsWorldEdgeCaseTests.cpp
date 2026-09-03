//
// PhysicsWorldEdgeCaseTests.cpp
//
// Section 23 additions not covered by LockstepTests.cpp: dt = 0, very
// small dt, very large dt, and swapping the integrator mid-simulation.
//

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;


TEST_CASE("PhysicsWorld - Stepping with dt = 0 leaves position and velocity unchanged", "[physicsworld][dt]")
{
    PhysicsWorld world;
    Handle body = world.CreateBody(BodyData{Vector3(1.0, 2.0, 3.0), Vector3(4.0, 5.0, 6.0), 1.0, 1.0},
                                   std::make_unique<ShapePoint>());

    world.GetForceManager().Add(body, Vector3(100.0, 0.0, 0.0));
    world.Step(0.0);

    const BodyData* result = world.GetBody(body);
    REQUIRE(result->position == Vector3(1.0, 2.0, 3.0));
    REQUIRE(result->velocity == Vector3(4.0, 5.0, 6.0));
}

TEST_CASE("PhysicsWorld - Stepping with dt = 0 still advances elapsed time by zero, not by a stale or default value", "[physicsworld][dt]")
{
    PhysicsWorld world;
    REQUIRE(world.GetElapsedTime() == 0.0);

    world.Step(0.0);
    REQUIRE(world.GetElapsedTime() == 0.0);

    world.Step(0.5);
    REQUIRE_THAT(world.GetElapsedTime(), WithinAbs(0.5, 1e-12));

    world.Step(0.0);
    REQUIRE_THAT(world.GetElapsedTime(), WithinAbs(0.5, 1e-12));
}

TEST_CASE("PhysicsWorld - A very small dt produces a proportionally small, finite change in state", "[physicsworld][dt]")
{
    PhysicsWorld world;
    Handle body = world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), 1.0, 1.0}, std::make_unique<ShapePoint>());

    world.GetForceManager().Add(body, Vector3(10.0, 0.0, 0.0));
    world.Step(1e-9);

    const BodyData* result = world.GetBody(body);
    REQUIRE(std::isfinite(result->position.x));
    REQUIRE(std::isfinite(result->velocity.x));
    REQUIRE(result->velocity.x > 0.0);
    REQUIRE(result->velocity.x < 1e-6); // proportionally tiny, not zero and not blown up
}

TEST_CASE("PhysicsWorld - A very large dt does not produce NaN or infinite state (even though accuracy is expected to suffer)", "[physicsworld][dt]")
{
    PhysicsWorld world;
    Handle body = world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), 1.0, 1.0}, std::make_unique<ShapePoint>());

    world.GetForceManager().Add(body, Vector3(10.0, 0.0, 0.0));
    world.Step(1000.0);

    const BodyData* result = world.GetBody(body);
    REQUIRE(std::isfinite(result->position.x));
    REQUIRE(std::isfinite(result->velocity.x));
}

TEST_CASE("PhysicsWorld - A very large dt under gravitational orbit still produces finite (if wildly inaccurate) state", "[physicsworld][dt]")
{
    // The interesting failure mode for a large dt with a position
    // dependent force isn't wrong-but-plausible answers, it's NaN from
    // division by a near-zero or negative distance after a body
    // overshoots past its attractor in a single giant step.
    PhysicsWorld world;
    UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), 10000.0, 0.0, 0.0}, std::make_unique<ShapeSphere>(1.0));
    Handle orbiter = world.CreateBody(BodyData{Vector3(100.0, 0.0, 0.0), Vector3(0.0, 10.0, 0.0), 1.0, 1.0, 0.0},
                                      std::make_unique<ShapeSphere>(1.0));

    world.Step(500.0); // absurdly large for this system

    const BodyData* result = world.GetBody(orbiter);
    REQUIRE(std::isfinite(result->position.x));
    REQUIRE(std::isfinite(result->position.y));
    REQUIRE(std::isfinite(result->velocity.x));
    REQUIRE(std::isfinite(result->velocity.y));
}

TEST_CASE("PhysicsWorld - GetElapsedTime accumulates across steps of differing sizes", "[physicsworld][dt]")
{
    PhysicsWorld world;

    world.Step(0.1);
    world.Step(0.2);
    world.Step(0.05);

    REQUIRE_THAT(world.GetElapsedTime(), WithinAbs(0.35, 1e-9));
}

// ==========================================================================
// Integrator replacement mid-simulation
// ==========================================================================

TEST_CASE("PhysicsWorld - SetIntegrator changes GetIntegratorName immediately", "[physicsworld][integrator]")
{
    PhysicsWorld world;
    REQUIRE(world.GetIntegratorName() == "EULER-CROMER");

    world.SetIntegrator(std::make_unique<IntegratorRungeKutta4>());
    REQUIRE(world.GetIntegratorName() == "4TH-ORDER RUNGE-KUTTA");
}

TEST_CASE("PhysicsWorld - Swapping integrators mid-simulation preserves body state rather than resetting it", "[physicsworld][integrator]")
{
    PhysicsWorld world;
    Handle body = world.CreateBody(BodyData{Vector3::Zero(), Vector3(1.0, 0.0, 0.0), 1.0, 1.0},
                                   std::make_unique<ShapePoint>());

    world.Step(0.1); // a few Euler-Cromer steps
    world.Step(0.1);

    const Vector3 positionBeforeSwap = world.GetBody(body)->position;
    const Vector3 velocityBeforeSwap = world.GetBody(body)->velocity;

    world.SetIntegrator(std::make_unique<IntegratorRungeKutta4>());

    // The swap itself, with no Step() yet, must not have reset anything.
    REQUIRE(world.GetBody(body)->position == positionBeforeSwap);
    REQUIRE(world.GetBody(body)->velocity == velocityBeforeSwap);

    world.Step(0.1); // now under RK4

    // Constant velocity, no forces: continues in a straight line
    // regardless of which integrator is driving it.
    REQUIRE_THAT(world.GetBody(body)->position.x, WithinAbs(positionBeforeSwap.x + velocityBeforeSwap.x * 0.1, 1e-9));
}

TEST_CASE("PhysicsWorld - A body under gravity continues its trajectory sensibly across an integrator swap (no discontinuous jump)", "[physicsworld][integrator]")
{
    PhysicsWorld world;
    UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), 10000.0, 0.0, 0.0}, std::make_unique<ShapeSphere>(1.0));
    Handle orbiter = world.CreateBody(BodyData{Vector3(100.0, 0.0, 0.0), Vector3(0.0, 10.0, 0.0), 1.0, 1.0, 0.0},
                                      std::make_unique<ShapeSphere>(1.0));

    const double dt = 0.01;
    for (int i = 0; i < 50; ++i) {
        world.Step(dt);
    }

    const Vector3 positionBeforeSwap = world.GetBody(orbiter)->position;
    const Vector3 velocityBeforeSwap = world.GetBody(orbiter)->velocity;

    world.SetIntegrator(std::make_unique<IntegratorRungeKutta4>());

    for (int i = 0; i < 50; ++i) {
        world.Step(dt);
    }

    const Vector3 positionAfterSwap = world.GetBody(orbiter)->position;

    // No teleport: over 50 more small steps under normal orbital
    // acceleration, position shouldn't have jumped by more than a small
    // multiple of one step's worth of travel at the pre-swap speed.
    const double maxPlausibleTravel = velocityBeforeSwap.Length() * dt * 50 * 3.0;
    REQUIRE((positionAfterSwap - positionBeforeSwap).Length() < maxPlausibleTravel);
    REQUIRE(std::isfinite(positionAfterSwap.x));
    REQUIRE(std::isfinite(positionAfterSwap.y));
}
