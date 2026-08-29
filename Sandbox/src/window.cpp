#include "Dashboard.h"
#include "RenderWindow.h"

#include "Osseus/Osseus.h"



void UpdateTelemetry(double delta, sandbox::Telemetry& telemetry, osseus::PhysicsWorld& world) {
    telemetry.bodyCount = world.GetObjectCount();
    telemetry.staticBodyCount = 0;
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
    const double delta = 1.0 / 60.0;
    sandbox::Dashboard dashboard(font);

    // dashboard.SetEvents({
    //     "14:31:02  BODY 0042  COLLISION DETECTED",
    //     "14:31:01  OCTREE     REBUILD 384 NODES",
    //     "14:31:00  PHYSICS    STEP 0007421"
    // });

    while (renderWindow.IsOpen()) {
        renderWindow.HandleEvents();

        sandbox::Telemetry telemetry;

        UpdateTelemetry(delta, telemetry, world);

        dashboard.SetTelemetry(telemetry);

        renderWindow.BeginFrame();
        dashboard.Draw(renderWindow.Get());
        renderWindow.EndFrame();
    }

    return 0;
}