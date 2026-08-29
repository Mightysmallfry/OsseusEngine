#include <algorithm>
#include <iostream>
#include <memory>
#include <random>
#include <vector>
#include <numbers>

#include <Osseus/Osseus.h>
#include <SFML/Graphics.hpp>

int main() {
    // ================= SFML Window initialization
    sf::Color particle_color = sf::Color::Cyan;
    const int width = 1200;
    const int height = 1200;
    sf::RenderWindow window(sf::VideoMode({width, height}), "Osseus-Sandbox");

    const int frame_rate = 60;
    window.setFramerateLimit(frame_rate);

    const sf::Font font("Sandbox/assets/fonts/JetBrainsMonoNerdFont-Regular.ttf");

    sf::Clock frameTimer;
    sf::Clock fpsClock;
    sf::Clock physicsTimer;

    int frameCount = 0;
    double accumulator = 0.0;
    double fps = 0.0;
    const float maxFrameDelta = 0.25;
    const int maxPhysicsSteps = 5;

    const double physicsDelta = 1.0 / 120.0; // 120-60 Hz

    sf::Text statisticsText(font, "", 18);
    statisticsText.setFillColor(sf::Color::White);
    statisticsText.setPosition({10.0f, 10.0f});

    // ================= Osseus Initialization
    osseus::PhysicsWorld world;

    const double boundaryRadius = 800.0;
    const double particleRadius = 2.0;
    const double maxDistance = boundaryRadius - particleRadius;

    world.SetIntegrator(std::make_unique<osseus::IntegratorEulerCromer>());

    osseus::UniversalGravity universalGravity;
    world.GetForceManager().AddUniversal(&universalGravity);


    // ================= RNG Initialization
    std::random_device rd;
    std::mt19937 generator(rd());

    const double rangeBound = 300.0;
    std::uniform_real_distribution<double> distribution(-rangeBound, rangeBound);

    // ================= Create static central object

    const double staticRadius = 20.0;
    const double staticMass = 250000.0;

    const double orbitalRadius = 100.0;

    osseus::BodyData staticBody{
        osseus::Vector3::Zero(),
        osseus::Vector3::Zero(),
        staticMass,
        0.0,
        0.0
    };

    osseus::Handle staticHandle =
        world.CreateBody(
            staticBody,
            std::make_unique<osseus::ShapeSphere>(staticRadius)
        );

    // ================= Create particles

    std::vector<osseus::Handle> particleHandles;
    const size_t bodyCount = 1000; // 2000 is current limit
    const double mass = 1.0;
    const double speed = 50.0;

    for (int i = 0; i < bodyCount; i++) {
        double randX;
        double randY;
        do {
            randX = distribution(generator);
            randY = distribution(generator);
        } while (randX * randX + randY * randY <
             (staticRadius + particleRadius) * (staticRadius + particleRadius));

        
        osseus::Vector3 randPosition = osseus::Vector3(randX, randY, 0.0);

        const double dx = randPosition.x;
        const double dy = randPosition.y;

        const double distance = std::sqrt(dx * dx + dy * dy);


        osseus::Vector3 randVelocity(
            -dy / distance * speed,
            dx / distance * speed,
            0.0
        );
        osseus::BodyData body{
            randPosition,            // Position
            randVelocity,            // Velocity
            mass,                    // Mass
            1.0 / mass,              // InvMass
            0.0                      // Charge
        };

        // osseus::BodyData body{
        //     osseus::Vector3(orbitalRadius, 0.0, 0.0),            // Position
        //     osseus::Vector3(0.0, std::sqrt(staticMass / orbitalRadius), 0.0),            // Velocity
        //     mass,                    // Mass
        //     1.0 / mass,              // InvMass
        //     0.0                      // Charge
        // };

        particleHandles.push_back(world.CreateBody(body, std::make_unique<osseus::ShapeSphere>(particleRadius)));
    }

    // run the program as long as the window is open
    while (window.isOpen()) {
        // check all the window's events that were triggered since the last iteration of the loop
        while (const std::optional event = window.pollEvent()) {
            // "close requested" event: we close the window
            if (event->is<sf::Event::Closed>()) {
                window.close();
            } else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {
                if (keyPressed->scancode == sf::Keyboard::Scancode::Escape)
                    window.close();
            }
        }

        ++frameCount;

        if (fpsClock.getElapsedTime().asSeconds() >= 1.0) {
            const double elapsed = fpsClock.restart().asSeconds();

            fps = static_cast<double>(frameCount) / elapsed;
            frameCount = 0;
        }

        const double frameDelta = std::min(frameTimer.restart().asSeconds(), maxFrameDelta);

        accumulator += frameDelta;

        double physicsMs = 0.0;
        int physicsSteps = 0;

        while (accumulator >= physicsDelta && physicsSteps < maxPhysicsSteps) {
            physicsTimer.restart();

            world.Step(physicsDelta);
        
            for (osseus::Handle handle : particleHandles) {
                osseus::BodyData* body = world.GetBody(handle);

                body->position.z = 0.0;

                const double distanceSquared = body->position.LengthSquared();

                if (distanceSquared > maxDistance * maxDistance) {
                    const double distance = std::sqrt(distanceSquared);

                    const osseus::Vector3 normal = body->position / distance;

                    // Put the particle back on the inside surface.
                    body->position = normal * maxDistance;

                    // Reflect the outward velocity.
                    const double velocityAlongNormal = body->velocity.Dot(normal);

                    if (velocityAlongNormal > 0.0) {
                        body->velocity -= normal * (2.0 * velocityAlongNormal);
                    }
                }
            }

            physicsMs += physicsTimer.getElapsedTime().asSeconds() * 1000.0;

            accumulator -= physicsDelta;
            ++physicsSteps;
        }

        // =================== Render Changes

        // clear the window with black
        window.clear(sf::Color::Black);

        // ========= Draw, Step, Repeat

        // Read particle positions
        sf::VertexArray toDrawParticles = sf::VertexArray(sf::PrimitiveType::Points, particleHandles.size());

        for (size_t i = 0; i < particleHandles.size(); ++i) {
            const auto* body = world.GetBody(particleHandles[i]);
            auto position = body->position;

            // Adjust for coordinate systems.
            position.x += width / 2.0;
            position.y = height / 2.0 - position.y;

            toDrawParticles[i] = sf::Vertex{sf::Vector2f(position.x, position.y), particle_color};
        }

        osseus::Vector3 staticPosition = world.GetBody(staticHandle)->position;
        staticPosition.x += width / 2.0;
        staticPosition.y = height / 2.0 - staticPosition.y;
       
        sf::CircleShape staticCircle(staticRadius);
        staticCircle.setFillColor(sf::Color::Yellow);
        staticCircle.setOrigin({staticRadius, staticRadius});
        staticCircle.setPosition({
            static_cast<float>(staticPosition.x),
            static_cast<float>(staticPosition.y)
        });

        window.draw(staticCircle);
        window.draw(toDrawParticles);
        

        const double frameMs = frameDelta * 1000.0;

        const osseus::BodyData* body = world.GetBody(particleHandles[0]);


        std::string statistics = "FPS: " + std::to_string(static_cast<int>(fps)) +
                                 "\nParticle Position: " + world.GetBody(particleHandles[0])->position.ToString() +
                                 "\nPhysics Time: " + std::to_string(physicsMs) +
                                 " ms"
                                 "\nFrame: " +
                                 std::to_string(frameMs) + " ms" +
                                 "\nPhysics: " + std::to_string(static_cast<int>(1.0 / physicsDelta)) + " Hz" +
                                 "\nSteps: " + std::to_string(physicsSteps) +
                                 "\nPhysics Avg: " + std::to_string(physicsSteps > 0 ? physicsMs / physicsSteps : 0.0) +
                                 " ms" + "\nBodies: " + std::to_string(particleHandles.size());

        statisticsText.setString(statistics);
        window.draw(statisticsText);

        // end the current frame
        window.display();
    }

    return 0;
}