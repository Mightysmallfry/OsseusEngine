#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

TEST_CASE("Integrator - Euler-Cromer integrates constant gravity", "[Euler-Cromer]")
{
    const double delta = 0.05;

    osseus::PhysicsWorld world;
    world.SetIntegrator(std::make_unique<osseus::IntegratorEulerCromer>());

   
    osseus::Handle body = world.CreateBody(
        osseus::BodyData{
            osseus::Vector3(0.0, 10.0, 0.0),
            osseus::Vector3::Zero(),
            1.0,
            1.0
        },
        std::make_unique<osseus::ShapePoint>()
    );


    world.GetForceManager().Add(body, osseus::Vector3(0.0, -9.8, 0.0));

    world.Step(delta);

    auto result = world.GetBody(body);

    REQUIRE(result != nullptr);

    // Gravity should accelerate downward
    REQUIRE(result->velocity.y < 0.0);

    // Body should have moved downward
    REQUIRE(result->position.y < 10.0);
}