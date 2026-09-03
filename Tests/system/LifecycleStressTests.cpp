//
// LifecycleStressTests.cpp
//
// Section 26 of the test spec. LockstepTests.cpp covers correctness of
// individual create/destroy operations at small scale; this file pushes
// volume and interleaving with active stepping, the two things most
// likely to expose slow leaks, index corruption, or stale-handle bugs
// that only show up after many cycles.
//

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

#include <Osseus/Osseus.h>

using namespace osseus;


TEST_CASE("Lifecycle stress - Thousands of sequential create/destroy cycles leave the world in a consistent, empty state", "[lifecycle-stress]")
{
    PhysicsWorld world;

    constexpr int cycles = 5000;
    for (int i = 0; i < cycles; ++i) {
        Handle handle = world.CreateBody(BodyData{Vector3(static_cast<double>(i), 0.0, 0.0), Vector3::Zero(), 1.0, 1.0},
                                         std::make_unique<ShapePoint>());
        REQUIRE(world.GetBody(handle) != nullptr);
        world.DestroyBody(handle);
        REQUIRE(world.GetBody(handle) == nullptr);
    }

    REQUIRE(world.GetObjectCount() == 0);
}

TEST_CASE("Lifecycle stress - A destroyed handle's slot, once reused, never returns the old body's stale data", "[lifecycle-stress][regression]")
{
    PhysicsWorld world;

    Handle first = world.CreateBody(BodyData{Vector3(111.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0}, std::make_unique<ShapePoint>());
    world.DestroyBody(first);

    // Likely (though not guaranteed) to reuse the same underlying slot.
    Handle second = world.CreateBody(BodyData{Vector3(222.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0}, std::make_unique<ShapePoint>());

    REQUIRE(world.GetBody(second)->position.x == 222.0);
    // The stale handle must not resolve to the new body's data.
    REQUIRE(world.GetBody(first) == nullptr);
}

TEST_CASE("Lifecycle stress - Repeated create/step/destroy cycles maintain lockstep between BodyManager, ForceManager, and ShapeManager", "[lifecycle-stress]")
{
    PhysicsWorld world;
    UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    constexpr int cycles = 500;
    for (int i = 0; i < cycles; ++i) {
        Handle a = world.CreateBody(BodyData{Vector3(1.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                    std::make_unique<ShapeSphere>(0.5));
        Handle b = world.CreateBody(BodyData{Vector3(-1.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                    std::make_unique<ShapeSphere>(0.5));

        world.Step(0.01);

        REQUIRE(std::isfinite(world.GetBody(a)->position.x));
        REQUIRE(std::isfinite(world.GetBody(b)->position.x));

        world.DestroyBody(a);
        world.DestroyBody(b);
    }

    REQUIRE(world.GetObjectCount() == 0);
}

TEST_CASE("Lifecycle stress - QueueDestroyBody during an active simulation removes bodies without corrupting the survivors", "[lifecycle-stress]")
{
    PhysicsWorld world;

    std::vector<Handle> handles;
    constexpr int bodyCount = 200;
    for (int i = 0; i < bodyCount; ++i) {
        handles.push_back(world.CreateBody(BodyData{Vector3(static_cast<double>(i), 0.0, 0.0), Vector3::Zero(), 1.0, 1.0},
                                           std::make_unique<ShapePoint>()));
    }

    // Queue every other body for destruction, then step -- SyncState()
    // flushes destructionQueue_ at the end of Step().
    for (int i = 0; i < bodyCount; i += 2) {
        world.QueueDestroyBody(handles[i]);
    }
    world.Step(0.01);

    for (int i = 0; i < bodyCount; ++i) {
        if (i % 2 == 0) {
            REQUIRE(world.GetBody(handles[i]) == nullptr);
        } else {
            REQUIRE(world.GetBody(handles[i]) != nullptr);
            // Survivors' data should be exactly what they started with
            // (no force acting on them, one step at velocity zero).
            REQUIRE(world.GetBody(handles[i])->position.x == static_cast<double>(i));
        }
    }

    REQUIRE(world.GetObjectCount() == bodyCount / 2);
}

TEST_CASE("Lifecycle stress - A large population of bodies can be created, stepped many times, and fully torn down", "[lifecycle-stress]")
{
    PhysicsWorld world;
    UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    std::vector<Handle> handles;
    constexpr int bodyCount = 100;
    for (int i = 0; i < bodyCount; ++i) {
        const double angle = (2.0 * 3.14159265358979 * i) / bodyCount;
        const Vector3 position(50.0 * std::cos(angle), 50.0 * std::sin(angle), 0.0);
        handles.push_back(world.CreateBody(BodyData{position, Vector3::Zero(), 1.0, 1.0, 0.0},
                                           std::make_unique<ShapeSphere>(0.1)));
    }

    constexpr int steps = 200;
    for (int step = 0; step < steps; ++step) {
        world.Step(0.01);
    }

    for (Handle h : handles) {
        const BodyData* body = world.GetBody(h);
        REQUIRE(body != nullptr);
        REQUIRE(std::isfinite(body->position.x));
        REQUIRE(std::isfinite(body->position.y));
    }

    for (Handle h : handles) {
        world.DestroyBody(h);
    }
    REQUIRE(world.GetObjectCount() == 0);

    // World remains usable after a full population teardown.
    Handle survivor = world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), 1.0, 1.0}, std::make_unique<ShapePoint>());
    world.Step(0.01);
    REQUIRE(world.GetBody(survivor) != nullptr);
}
