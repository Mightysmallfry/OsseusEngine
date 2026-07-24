#include <iostream>
#include <Osseus/Osseus.h>



// sandbox for the Osseus Engine.
int main() {
    const double delta = 0.05;

    osseus::PhysicsWorld world;

    // Add Force for all particles to be subject to.
    world.AddForce(std::make_unique<osseus::ForceGravity>());



    // Create our capusle
    osseus::Handle capsule = world.CreateHandle();
    osseus::BodyData capsuleData = osseus::BodyData {
        osseus::Vector3(0.0,0.0,0.0),
        osseus::Vector3::Zero(),
        1.0
    };

    world.AttachBody(capsule, capsuleData);
    world.AttachShape(capsule, std::make_unique<osseus::ShapePoint>());

    // Consider a builder class
    osseus::Handle ball = world.CreateBody(
        osseus::BodyData{osseus::Vector3{0, 10, 0}, osseus::Vector3{0, 0, 0}, 1.0f },
    std::make_unique<osseus::ShapeSphere>(1.0f)
    );

    // Simulate the world
    world.Step(delta);

    if (world.GetBody(capsule)) {
        std::cout << "capsule has been found" << std::endl;
        std::cout << world.GetBody(capsule)->position << std::endl;
    }

}