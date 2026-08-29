//
// Created by MightySmallFry on 7/24/2026.
//
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>
using Catch::Matchers::WithinAbs;

TEST_CASE("Integrator - RK4 integrates constant gravity")
{
    constexpr double delta = 0.05;
    constexpr double gravity = -9.8;

    osseus::PhysicsWorld world;

    world.SetIntegrator(
        std::make_unique<osseus::IntegratorRungeKutta4>()
    );

    const osseus::Handle body = world.CreateBody(
        osseus::BodyData{
            osseus::Vector3(0.0, 10.0, 0.0),
            osseus::Vector3::Zero(),
            1.0,
            1.0
        },
        std::make_unique<osseus::ShapePoint>()
    );

    world.GetForceManager().Add(
        body,
        osseus::Vector3(0.0, gravity, 0.0)
    );

    world.Step(delta);

    const osseus::BodyData* result = world.GetBody(body);

    REQUIRE(result != nullptr);

    const double expectedVelocity = gravity * delta;
    const double expectedPosition =
        10.0 + 0.5 * gravity * delta * delta;

    REQUIRE_THAT(
        result->velocity.y,
        WithinAbs(expectedVelocity, 1e-12)
    );

    REQUIRE_THAT(
        result->position.y,
        WithinAbs(expectedPosition, 1e-12)
    );

    REQUIRE_THAT(result->velocity.x, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(result->velocity.z, WithinAbs(0.0, 1e-12));

    REQUIRE_THAT(result->position.x, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(result->position.z, WithinAbs(0.0, 1e-12));
}

TEST_CASE("Integrator - RK4 preserves constant velocity")
{
    constexpr double delta = 0.1;
    constexpr int steps = 100;
    constexpr double totalTime = delta * steps;

    const osseus::Vector3 initialPosition(1.0, -2.0, 3.0);
    const osseus::Vector3 initialVelocity(4.0, 5.0, -6.0);

    osseus::PhysicsWorld world;
    world.SetIntegrator(
        std::make_unique<osseus::IntegratorRungeKutta4>()
    );

    const osseus::Handle body = world.CreateBody(
        osseus::BodyData{
            initialPosition,
            initialVelocity,
            1.0,
            1.0
        },
        std::make_unique<osseus::ShapePoint>()
    );

    for (int i = 0; i < steps; ++i) {
        world.Step(delta);
    }

    const osseus::BodyData* result = world.GetBody(body);

    REQUIRE(result != nullptr);

    const osseus::Vector3 expectedPosition =
        initialPosition + initialVelocity * totalTime;

    REQUIRE_THAT(result->position.x, WithinAbs(expectedPosition.x, 1e-12));
    REQUIRE_THAT(result->position.y, WithinAbs(expectedPosition.y, 1e-12));
    REQUIRE_THAT(result->position.z, WithinAbs(expectedPosition.z, 1e-12));

    REQUIRE_THAT(result->velocity.x, WithinAbs(initialVelocity.x, 1e-12));
    REQUIRE_THAT(result->velocity.y, WithinAbs(initialVelocity.y, 1e-12));
    REQUIRE_THAT(result->velocity.z, WithinAbs(initialVelocity.z, 1e-12));
}

TEST_CASE("Integrator - RK4 integrates constant acceleration")
{
    constexpr double delta = 0.05;
    constexpr int steps = 20;
    constexpr double totalTime = delta * steps;

    constexpr double acceleration = -9.81;

    const osseus::Vector3 initialPosition(0.0, 10.0, 0.0);
    const osseus::Vector3 initialVelocity(2.0, 3.0, 0.0);

    osseus::PhysicsWorld world;
    world.SetIntegrator(
        std::make_unique<osseus::IntegratorRungeKutta4>()
    );

    const osseus::Handle body = world.CreateBody(
        osseus::BodyData{
            initialPosition,
            initialVelocity,
            1.0,
            1.0
        },
        std::make_unique<osseus::ShapePoint>()
    );

    
    for (int i = 0; i < steps; ++i) {
        world.GetForceManager().Add(body, osseus::Vector3(0.0, -9.81, 0.0));
        world.Step(delta);
    }

    const osseus::BodyData* result = world.GetBody(body);

    REQUIRE(result != nullptr);

    const osseus::Vector3 expectedVelocity(
        initialVelocity.x,
        initialVelocity.y + acceleration * totalTime,
        initialVelocity.z
    );

    const osseus::Vector3 expectedPosition(
        initialPosition.x +
            initialVelocity.x * totalTime,

        initialPosition.y +
            initialVelocity.y * totalTime +
            0.5 * acceleration * totalTime * totalTime,

        initialPosition.z
    );

    REQUIRE_THAT(result->velocity.x, WithinAbs(expectedVelocity.x, 1e-10));
    REQUIRE_THAT(result->velocity.y, WithinAbs(expectedVelocity.y, 1e-10));
    REQUIRE_THAT(result->velocity.z, WithinAbs(expectedVelocity.z, 1e-10));

    REQUIRE_THAT(result->position.x, WithinAbs(expectedPosition.x, 1e-10));
    REQUIRE_THAT(result->position.y, WithinAbs(expectedPosition.y, 1e-10));
    REQUIRE_THAT(result->position.z, WithinAbs(expectedPosition.z, 1e-10));
}

TEST_CASE("Integrator - RK4 preserves a circular orbit")
{
    constexpr double gravitationalConstant = 1.0;
    constexpr double centralMass = 10000.0;
    constexpr double particleMass = 1.0;

    constexpr double orbitRadius = 100.0;
    constexpr double orbitalSpeed = 10.0;

    constexpr double delta = 1.0 / 120.0;
    constexpr int steps = 12000;

    osseus::PhysicsWorld world;

    world.SetIntegrator(
        std::make_unique<osseus::IntegratorRungeKutta4>()
    );

    osseus::UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    const osseus::Handle centralHandle = world.CreateBody(
        osseus::BodyData{
            osseus::Vector3::Zero(),
            osseus::Vector3::Zero(),
            centralMass,
            0.0,
            0.0
        },
        std::make_unique<osseus::ShapeSphere>(20.0)
    );

    const osseus::Handle particleHandle = world.CreateBody(
        osseus::BodyData{
            osseus::Vector3(orbitRadius, 0.0, 0.0),
            osseus::Vector3(0.0, orbitalSpeed, 0.0),
            particleMass,
            1.0 / particleMass,
            0.0
        },
        std::make_unique<osseus::ShapeSphere>(2.0)
    );

    for (int step = 0; step < steps; ++step) {
        world.Step(delta);
    }

    const osseus::BodyData* particle =
        world.GetBody(particleHandle);

    const osseus::BodyData* central =
        world.GetBody(centralHandle);

    REQUIRE(particle != nullptr);
    REQUIRE(central != nullptr);

    const double finalRadius =
        particle->position.Length();

    const double finalEnergy =
        0.5 * particleMass *
            particle->velocity.LengthSquared()
        -
        gravitationalConstant *
            centralMass *
            particleMass /
            finalRadius;

    const double expectedEnergy =
        -gravitationalConstant *
        centralMass *
        particleMass /
        (2.0 * orbitRadius);

    REQUIRE_THAT(
        finalRadius,
        WithinAbs(orbitRadius, 0.1)
    );

    REQUIRE_THAT(
        finalEnergy,
        WithinAbs(expectedEnergy, 1e-3)
    );
}