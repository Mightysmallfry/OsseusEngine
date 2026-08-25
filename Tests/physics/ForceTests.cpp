#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

TEST_CASE("Universal Forces [Gravity] - Adding Force evaluators rejects duplicates") {
    osseus::PhysicsWorld world;

    osseus::UniversalGravity universalGravity;
    osseus::UniversalGravity gravityDuplicate;


    int goodAdd = world.GetForceManager().AddUniversal(&universalGravity);
    REQUIRE(goodAdd != -1);

    int badAdd = world.GetForceManager().AddUniversal(&gravityDuplicate);
    REQUIRE(badAdd == -1);
}

TEST_CASE("Universal Forces [Gravity] - A single body remains uneffected") {
    osseus::PhysicsWorld world;

    osseus::UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    osseus::Handle object = world.CreateBody();
    osseus::BodyData* body = world.GetBody(object);

    REQUIRE(body->position == osseus::Vector3::Zero());
    REQUIRE(body->velocity == osseus::Vector3::Zero());

    world.Step(0.1);

    REQUIRE(body->position == osseus::Vector3::Zero());
    REQUIRE(body->velocity == osseus::Vector3::Zero());
}

TEST_CASE("Universal Forces [Gravity] - A two bodies along an axis attract as expected") {
    osseus::PhysicsWorld world;
    osseus::UniversalGravity universalGravity;
    world.GetForceManager().AddUniversal(&universalGravity);

    osseus::Handle objectOne = world.CreateBody();
    osseus::BodyData* bodyOne = world.GetBody(objectOne);
    bodyOne->position = osseus::Vector3(10.0, 0.0, 0.0);


    osseus::Handle objectTwo = world.CreateBody();
    osseus::BodyData* bodyTwo = world.GetBody(objectTwo);
    bodyTwo->position = osseus::Vector3::Zero();

    REQUIRE(bodyTwo->position != bodyOne->position);

    world.Step(0.1);

    REQUIRE(bodyOne->position != osseus::Vector3(10.0, 0.0, 0.0));
    REQUIRE(bodyTwo->position != osseus::Vector3::Zero());
    
    REQUIRE(bodyOne->position.x < 10.0);
    REQUIRE(bodyTwo->position.x > 0.0);

}

