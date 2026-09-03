#include "Dashboard.h"
#include "RenderWindow.h"

#include "Osseus/Osseus.h"
#include "Scenarios/Scenarios.h"

#include <SFML/System/Clock.hpp>

#include <memory>

void UpdateTelemetry(double delta, double fps, double frameTimeMs, double physicsTimeMs, sandbox::Telemetry& telemetry,
                     osseus::PhysicsWorld& world) {
    telemetry.bodyCount = world.GetObjectCount();
    telemetry.staticBodyCount = 0;
    telemetry.fps = fps;
    telemetry.frameTimeMs = frameTimeMs;
    telemetry.physicsTimeMs = physicsTimeMs;
    telemetry.simulationTime = world.GetElapsedTime();
    telemetry.deltaTime = delta;
    telemetry.octreeNodes = world.GetOctree().Size();
    telemetry.octreeDepth = world.GetOctree().GetDepth();
    telemetry.collisionCount = world.GetCollisionManifold().size();
    telemetry.resolvedCollisionCount = world.GetCollisionManifold().size();
    telemetry.integrator = world.GetIntegratorName();
}

int main() {
    sandbox::RenderWindow renderWindow(1280, 720, "Osseus");

    sf::Font font;

    if (!font.openFromFile("Sandbox/assets/fonts/JetBrainsMonoNerdFont-Regular.ttf")) {
        return 1;
    }

    osseus::PhysicsWorld world;

    const double delta = 1.0 / 120.0;

    sandbox::Dashboard dashboard(font);

    // std::unique_ptr<sandbox::IScenario> scenario = std::make_unique<sandbox::OrbitScenario>();
    std::unique_ptr<sandbox::IScenario> scenario = std::make_unique<sandbox::NBodyContactlessScenario>();
    // std::unique_ptr<sandbox::IScenario> scenario = std::make_unique<sandbox::BinaryScenario>();

    const sf::FloatRect simulationBounds = dashboard.GetSimulationBounds(renderWindow.Get().getView().getSize());
    scenario->Initialize(world, simulationBounds.size.x, simulationBounds.size.y);

    sf::Clock frameClock;
    sf::Clock fpsClock;
    sf::Clock physicsClock;

    std::size_t frameCount = 0;
    double fps = 0.0;

    while (renderWindow.IsOpen()) {
        renderWindow.HandleEvents();

        // ==================== Frame Timing
        const double frameTimeMs = frameClock.restart().asSeconds() * 1000.0;
        ++frameCount;

        if (fpsClock.getElapsedTime().asSeconds() >= 1.0) {
            const double elapsed = fpsClock.restart().asSeconds();
            fps = static_cast<double>(frameCount) / elapsed;
            frameCount = 0;
        }

        // ==================== Physics
        physicsClock.restart();

        world.Step(delta);
        scenario->Update(world);

        const double physicsTimeMs = physicsClock.getElapsedTime().asSeconds() * 1000.0;

        // ==================== Telemetry
        sandbox::Telemetry telemetry;
        UpdateTelemetry(delta, fps, frameTimeMs, physicsTimeMs, telemetry, world);
        dashboard.SetTelemetry(telemetry);

        // ==================== Rendering
        renderWindow.BeginFrame();
        dashboard.Draw(renderWindow.Get());
        const sf::FloatRect simulationBounds = dashboard.GetSimulationBounds(renderWindow.Get().getView().getSize());
        renderWindow.DrawSimulation(world, *scenario, simulationBounds);

        renderWindow.EndFrame();
    }

    return 0;
}