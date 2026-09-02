//
// IndividualForceTests.cpp
//
// Section 18 of the test spec: forces added directly via
// ForceManager::Add (as opposed to universal forces evaluated through
// Barnes-Hut) -- specifically whether they accumulate correctly and
// whether they persist across steps or must be re-added.
//
// PhysicsWorld::SyncState() calls forceManager_.Clear() at the very end
// of every Step(), so the expected (and tested) contract is: a manually
// added force affects exactly the step it was added before, and has no
// effect on any subsequent step unless re-added.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;


TEST_CASE("ForceManager - Add accumulates onto any existing force rather than replacing it", "[forcemanager][individual]")
{
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle handle{0, 0};
    bodyManager.AddBody(handle, BodyData{});
    forceManager.Register(handle);

    forceManager.Add(handle, Vector3(1.0, 0.0, 0.0));
    forceManager.Add(handle, Vector3(0.0, 2.0, 0.0));

    REQUIRE(forceManager.Get(handle) == Vector3(1.0, 2.0, 0.0));
}

TEST_CASE("ForceManager - ClearForceOf zeroes only the targeted body's force", "[forcemanager][individual]")
{
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle a{0, 0};
    Handle b{1, 0};
    bodyManager.AddBody(a, BodyData{});
    bodyManager.AddBody(b, BodyData{});
    forceManager.Register(a);
    forceManager.Register(b);

    forceManager.Add(a, Vector3(5.0, 0.0, 0.0));
    forceManager.Add(b, Vector3(3.0, 0.0, 0.0));

    forceManager.ClearForceOf(a);

    REQUIRE(forceManager.Get(a) == Vector3::Zero());
    REQUIRE(forceManager.Get(b) == Vector3(3.0, 0.0, 0.0));
}

TEST_CASE("ForceManager - Clear zeroes every body's force", "[forcemanager][individual]")
{
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle a{0, 0};
    Handle b{1, 0};
    bodyManager.AddBody(a, BodyData{});
    bodyManager.AddBody(b, BodyData{});
    forceManager.Register(a);
    forceManager.Register(b);

    forceManager.Add(a, Vector3(5.0, 0.0, 0.0));
    forceManager.Add(b, Vector3(3.0, 0.0, 0.0));

    forceManager.Clear();

    REQUIRE(forceManager.Get(a) == Vector3::Zero());
    REQUIRE(forceManager.Get(b) == Vector3::Zero());
}

TEST_CASE("PhysicsWorld - A manually added individual force affects the body during the step it was added", "[physicsworld][individual]")
{
    PhysicsWorld world;

    Handle object = world.CreateBody();
    BodyData* body = world.GetBody(object);
    body->invMass = 1.0;

    const Vector3 startVelocity = body->velocity;

    world.GetForceManager().Add(object, Vector3(10.0, 0.0, 0.0));
    world.Step(0.1);

    // A constant +X force on a moving body should have accelerated it
    // in +X (no gravity/EM registered, so this force is the only thing
    // acting on it).
    REQUIRE(world.GetBody(object)->velocity.x > startVelocity.x);
}

TEST_CASE("PhysicsWorld - A manually added individual force does NOT persist to the following step without being re-added", "[physicsworld][individual][regression]")
{
    PhysicsWorld world;

    Handle object = world.CreateBody();
    BodyData* body = world.GetBody(object);
    body->invMass = 1.0;

    world.GetForceManager().Add(object, Vector3(10.0, 0.0, 0.0));
    world.Step(0.1);

    const Vector3 velocityAfterFirstStep = world.GetBody(object)->velocity;

    // Second step: force was not re-added, so with nothing else acting
    // on the body, velocity should be unchanged by this step (no
    // universal forces are registered in this world either).
    world.Step(0.1);

    REQUIRE(world.GetBody(object)->velocity == velocityAfterFirstStep);
}

TEST_CASE("PhysicsWorld - Re-adding the same individual force each step reproduces the same per-step acceleration", "[physicsworld][individual]")
{
    PhysicsWorld world;

    Handle object = world.CreateBody();
    BodyData* body = world.GetBody(object);
    body->invMass = 1.0;

    const Vector3 force(4.0, 0.0, 0.0);
    const double dt = 0.1;

    world.GetForceManager().Add(object, force);
    world.Step(dt);
    const Vector3 velocityAfterStepOne = world.GetBody(object)->velocity;

    world.GetForceManager().Add(object, force);
    world.Step(dt);
    const Vector3 velocityAfterStepTwo = world.GetBody(object)->velocity;

    const Vector3 deltaOne = velocityAfterStepOne;
    const Vector3 deltaTwo = velocityAfterStepTwo - velocityAfterStepOne;

    REQUIRE_THAT(deltaOne.x, WithinAbs(deltaTwo.x, 1e-9));
}

TEST_CASE("PhysicsWorld - Individual forces on different bodies do not cross-contaminate", "[physicsworld][individual]")
{
    PhysicsWorld world;

    Handle a = world.CreateBody();
    world.GetBody(a)->invMass = 1.0;

    Handle b = world.CreateBody();
    world.GetBody(b)->invMass = 1.0;
    // Deliberately far apart so no universal force could be at play
    // even if one were registered (none is here).
    world.GetBody(b)->position = Vector3(1000.0, 1000.0, 1000.0);

    world.GetForceManager().Add(a, Vector3(10.0, 0.0, 0.0));
    world.Step(0.1);

    REQUIRE(world.GetBody(a)->velocity.x > 0.0);
    REQUIRE(world.GetBody(b)->velocity == Vector3::Zero());
}
