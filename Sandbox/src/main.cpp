#include <iostream>
#include <memory>
#include <vector>

#include <Osseus/Osseus.h>

void RunIdenticalBodiesSimulation()
{
    constexpr int bodyCount = 100;
    constexpr double deltaTime = 0.01;
    constexpr int simulationSteps = 1000;

    osseus::PhysicsWorld world;

    world.SetIntegrator(
        std::make_unique<osseus::IntegratorRungeKutta4>()
    );

    world.AddForce(
        std::make_unique<osseus::ForceGravity>()
    );

    constexpr double mass = 1.0;
    constexpr double inverseMass = 1.0 / mass;

    for (int i = 0; i < bodyCount; ++i)
    {
        const double x = static_cast<double>(i % 10) * 3.0;
        const double z = static_cast<double>(i / 10) * 3.0;

        world.CreateBody(
            osseus::BodyData{
                osseus::Vector3(x, 10.0, z),
                osseus::Vector3::Zero(),
                inverseMass
            },
            std::make_unique<osseus::ShapeSphere>(1.0)
        );
    }

    for (int step = 0; step < simulationSteps; ++step)
    {
        world.Step(deltaTime);
    }

    std::cout << "Identical-body simulation complete.\n";
}


void RunUniqueBodiesSimulation()
{
    constexpr int bodyCount = 100;
    constexpr double deltaTime = 0.01;
    constexpr int simulationSteps = 1000;

    osseus::PhysicsWorld world;

    world.SetIntegrator(
        std::make_unique<osseus::IntegratorRungeKutta4>()
    );

    world.AddForce(
        std::make_unique<osseus::ForceGravity>()
    );

    for (int i = 0; i < bodyCount; ++i)
    {
        const double mass = 0.5 + static_cast<double>(i) * 0.25;
        const double inverseMass = 1.0 / mass;

        const double x = static_cast<double>(i % 10) * 3.0;
        const double z = static_cast<double>(i / 10) * 3.0;

        const double initialHeight = 5.0 + static_cast<double>(i);
        const double initialVelocity = static_cast<double>(i) * 0.1;

        world.CreateBody(
            osseus::BodyData{
                osseus::Vector3(x, initialHeight, z),
                osseus::Vector3(initialVelocity, 0.0, 0.0),
                inverseMass
            },
            std::make_unique<osseus::ShapeSphere>(
                0.5 + static_cast<double>(i) * 0.01
            )
        );
    }

    for (int step = 0; step < simulationSteps; ++step)
    {
        world.Step(deltaTime);
    }

    std::cout << "Unique-body simulation complete.\n";
}


int main()
{
    std::cout << "Running identical-body simulation...\n";
    RunIdenticalBodiesSimulation();

    std::cout << "Running unique-body simulation...\n";
    RunUniqueBodiesSimulation();

    return 0;
}