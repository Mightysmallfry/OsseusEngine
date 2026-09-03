#include "Scenarios/NBodyContactlessScenario.h"

#include <SFML/Graphics.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <random>

namespace sandbox {

    void NBodyContactlessScenario::Initialize(osseus::PhysicsWorld& world, double width, double height) {
        boundaryRadius_ = std::min(width, height) / 2.0;

        // ==================== World

        world.GetForceManager().AddUniversal(&universalGravity_);
        world.SetCollisionMode(osseus::CollisionMode::ENABLED);
        world.SetIntegrator(std::make_unique<osseus::IntegratorEulerCromer>());

        // ==================== Random Number Generator

        std::random_device rd;
        std::mt19937 generator(rd());

        const double rangeBound = boundaryRadius_;

        std::uniform_real_distribution<double> distribution(-rangeBound, rangeBound);

        // ==================== Particles
        const std::size_t bodyCount = 100;
        const double mass = 100.0;

        const double minimumDistance = particleRadius_;
        const double G = OsseusConstants::GravitationalConstant;

        for (std::size_t i = 0; i < bodyCount; ++i) {
            double randX;
            double randY;
            double distSq;

            do {
                randX = distribution(generator);
                randY = distribution(generator);
                distSq = randX * randX + randY * randY;
            } while (distSq < minimumDistance * minimumDistance || distSq > boundaryRadius_ * boundaryRadius_);

            const double distance = std::sqrt(distSq);
            const double orbitalSpeed = 0;

            const osseus::Vector3 position{randX, randY, 0.0};
            const osseus::Vector3 velocity{0.0, 0.0, 0.0};
            const osseus::BodyData body{position, velocity, mass, 1.0 / mass, 0.0};

            const osseus::Handle handle =
                world.CreateBody(body, std::make_unique<osseus::ShapeSphere>(particleRadius_));

            auto particleShape = std::make_unique<sf::CircleShape>(static_cast<float>(particleRadius_));
            particleShape->setOrigin({static_cast<float>(particleRadius_), static_cast<float>(particleRadius_)});
            particleShape->setFillColor(sf::Color::Cyan);
            renderObjects_.push_back({handle, std::move(particleShape)});
        }
    }

    void NBodyContactlessScenario::Update(osseus::PhysicsWorld& world) {
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