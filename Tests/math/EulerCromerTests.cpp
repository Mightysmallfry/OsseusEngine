//
// Created by MightySmallFry on 7/24/2026.
//
//
// Created by MightySmallFry on 7/24/2026.
//
#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

TEST_CASE("Euler-Cromer integrates constant gravity")
{
    const double delta = 0.05;

    osseus::PhysicsWorld world;
    world.SetIntegrator(std::make_unique<osseus::IntegratorEulerCromer>());

    world.AddForce(
        std::make_unique<osseus::ForceGravity>()
    );

    osseus::Handle body = world.CreateBody(
        osseus::BodyData{
            osseus::Vector3(0.0, 10.0, 0.0),
            osseus::Vector3::Zero(),
            1.0
        },
        std::make_unique<osseus::ShapePoint>()
    );

    world.Step(delta);
    auto result = world.GetBody(body);

    REQUIRE(result != nullptr);

    // Gravity should accelerate downward
    REQUIRE(result->velocity.y < 0.0);

    // Body should have moved downward
    REQUIRE(result->position.y < 10.0);
}