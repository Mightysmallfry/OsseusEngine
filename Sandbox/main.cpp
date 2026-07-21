#include <iostream>
#include <Osseus/Osseus.h>

#include "Osseus/physics/Forces/ForceGravity.h"


// sandbox for the Osseus Engine.
int main() {
    const double delta = 0.05;

    osseus::PhysicsWorld world;

    // Add Force for all particles to be subject to.
    world.AddForce(std::make_unique<osseus::ForceGravity>());

    osseus::Handle capsule = world.CreateBody(osseus::BodyData{
        osseus::Vector3(0.0, 50.0, 0.0), osseus::Vector3(0.0,0.0,0.0), 1.0});

    world.Step(delta);

    if (world.GetBody(capsule)) {
        std::cout << "capsule has been found" << std::endl;
    }

}