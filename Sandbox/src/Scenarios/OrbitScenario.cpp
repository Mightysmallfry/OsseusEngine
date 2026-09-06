#include "Scenarios/OrbitScenario.h"

#include <SFML/Graphics.hpp>

#include <cmath>
#include <memory>
#include <random>

namespace sandbox {

    void OrbitScenario::Initialize(osseus::PhysicsWorld& world, double width, double height) {
        boundaryRadius_ = std::min(width, height) / 2.0;

        world.AddUniversalForce(&universalGravity_);
        // ==================== Central Body

        osseus::BodyData staticBody{osseus::Vector3::Zero(), osseus::Vector3::Zero(), staticMass, 0.0, 0.0};

        const osseus::Handle staticHandle =
            world.CreateBody(staticBody, std::make_unique<osseus::ShapeSphere>(staticRadius_));

        auto staticShape = std::make_unique<sf::CircleShape>(static_cast<float>(staticRadius_));

        staticShape->setOrigin({static_cast<float>(staticRadius_), static_cast<float>(staticRadius_)});

        staticShape->setFillColor(sf::Color::Yellow);

        renderObjects_.push_back({staticHandle, std::move(staticShape)});

        // ==================== Random Number Generator

        std::random_device rd;
        std::mt19937 generator(rd());

        const double rangeBound = boundaryRadius_;

        std::uniform_real_distribution<double> distribution(-rangeBound, rangeBound);

        // ==================== Particles

        const double minimumDistance = staticRadius_ + particleRadius_;

        for (std::size_t i = 0; i < bodyCount; ++i) {
            double randX;
            double randY;

            do {
                randX = distribution(generator);
                randY = distribution(generator);
            } while (randX * randX + randY * randY < minimumDistance * minimumDistance);

            const osseus::Vector3 position{randX, randY, 0.0};
            const double distance = std::sqrt(randX * randX + randY * randY);
            const osseus::Vector3 velocity{-randY / distance * speed, randX / distance * speed, 0.0};
            const osseus::BodyData body{position, velocity, mass, 1.0 / mass, 0.0};

            const osseus::Handle handle =
                world.CreateBody(body, std::make_unique<osseus::ShapeSphere>(particleRadius_));

            auto particleShape = std::make_unique<sf::CircleShape>(static_cast<float>(particleRadius_));

            particleShape->setOrigin({static_cast<float>(particleRadius_), static_cast<float>(particleRadius_)});

            particleShape->setFillColor(sf::Color::Cyan);
            renderObjects_.push_back({handle, std::move(particleShape)});
        }
    }

    void OrbitScenario::Update(osseus::PhysicsWorld& world) {
        const double maxDistance = boundaryRadius_ * 1.5 - particleRadius_;

        for (const RenderObject& object : renderObjects_) {
            osseus::BodyData* body = world.GetBody(object.handle);

            // The central body is static, so there is no boundary
            // handling required for it.
            if (body->invMass == 0.0) {
                continue;
            }

            body->position.z = 0.0;

            const double distanceSquared = body->position.LengthSquared();

            if (distanceSquared > maxDistance * maxDistance) {
                const double distance = std::sqrt(distanceSquared);

                const osseus::Vector3 normal = body->position / distance;

                body->position = normal * maxDistance;

                const double velocityAlongNormal = body->velocity.Dot(normal);

                if (velocityAlongNormal > 0.0) {
                    body->velocity -= normal * (2.0 * velocityAlongNormal);
                }
            }
        }
    }

} // namespace sandbox