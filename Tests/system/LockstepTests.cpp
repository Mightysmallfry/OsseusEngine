#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

TEST_CASE("Lockstep - ForceManager remains synchronized with BodyManager when adding bodies")
{
    osseus::Registry registry;
    osseus::BodyManager bodyManager;
    osseus::ForceManager forceManager;

    osseus::Handle handle = registry.CreateHandle();
    osseus::BodyData body{};

    bodyManager.AddBody(handle, body);
    forceManager.Add(handle, osseus::Vector3::Zero());

    REQUIRE(bodyManager.Data().size() == forceManager.NetForces().size());
}

TEST_CASE("Lockstep - ForceManager remains synchronized with BodyManager when adding multiple bodies")
{
    osseus::Registry registry;
    osseus::BodyManager bodyManager;
    osseus::ForceManager forceManager;

    for (int i = 0; i < 5; ++i)
    {
        osseus::Handle handle = registry.CreateHandle();
        osseus::BodyData body{};

        bodyManager.AddBody(handle, body);
        forceManager.Add(handle, osseus::Vector3::Zero());
    }

    REQUIRE(bodyManager.Data().size() == forceManager.NetForces().size());
}

TEST_CASE("Lockstep - ForceManager properly registers a handle")
{
    osseus::Registry registry;
    osseus::ForceManager forceManager;

    osseus::Handle handle = registry.CreateHandle();
    forceManager.Register(handle);

    REQUIRE(forceManager.NetForces().size() > 0);
}

TEST_CASE("Lockstep - BodyManager properly registers a handle")
{
    osseus::Registry registry;
    osseus::BodyManager bodyManager;

    osseus::Handle handle = registry.CreateHandle();
    bodyManager.Register(handle);

    REQUIRE(bodyManager.Data().size() > 0);
}


TEST_CASE("Lockstep - PhysicsWorld maintains lockstep on creation")
{
    osseus::PhysicsWorld world;

    osseus::Handle object = world.CreateBody();
    REQUIRE(world.GetForceManager().NetForces().size() == world.GetBodyManager().Data().size());
    
    osseus::Handle secondObject = world.CreateBody();
    osseus::Handle thirdObject = world.CreateBody();
    
    REQUIRE(world.GetForceManager().NetForces().size() == world.GetBodyManager().Data().size());
}


TEST_CASE("Lockstep - PhysicsWorld maintains lockstep through destruction")
{
    osseus::PhysicsWorld world;

    osseus::Handle object = world.CreateBody();
    REQUIRE(world.GetForceManager().NetForces().size() == world.GetBodyManager().Data().size());

    world.DestroyBody(object);
    
    osseus::Handle secondObject = world.CreateBody();
    world.DestroyBody(secondObject);

    osseus::Handle thirdObject = world.CreateBody();
    
    REQUIRE(world.GetForceManager().NetForces().size() == world.GetBodyManager().Data().size());
}



TEST_CASE("Lockstep - PhysicsWorld typically destroys at end step.")
{
    osseus::PhysicsWorld world;

    osseus::Handle object = world.CreateBody();
    REQUIRE(world.GetBodyManager().Data().size() == 1);

    world.DestroyBody(object);

    REQUIRE(world.GetBodyManager().Data().size() == 0);

    osseus::Handle secondObject = world.CreateBody();
    world.QueueDestroyBody(secondObject);


    REQUIRE(world.GetBodyManager().Data().size() == 1);

    osseus::Handle thirdObject = world.CreateBody();
    world.QueueDestroyBody(thirdObject);
    
    REQUIRE(world.GetBodyManager().Data().size() == 2);

    world.Step(0.01);
    REQUIRE(world.GetBodyManager().Data().size() == 0);
}