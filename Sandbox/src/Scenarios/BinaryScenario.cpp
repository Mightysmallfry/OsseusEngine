
#include "Scenarios/BinaryScenario.h"

#include <algorithm>
#include <cmath>
#include <memory>

namespace sandbox {

    void BinaryScenario::Initialize(osseus::PhysicsWorld& world, double width, double height) {
        width_ = width;
        height_ = height;

        world.SetIntegrator(std::make_unique<osseus::IntegratorRungeKutta4>());

        world.GetForceManager().AddUniversal(&universalGravity_);

        // ==================== Orbital Configuration

        // orbitalRadius_ = std::min(width_, height_) * 0.15;
        orbitalSpeed_ = std::sqrt(starMass_ / (4.0 * orbitalRadius_));
        

        // ==================== Star 1

        const osseus::BodyData star1Body{osseus::Vector3(-orbitalRadius_, 0.0, 0.0),
                                         osseus::Vector3(0.0, -orbitalSpeed_, 0.0), starMass_, 1.0 / starMass_, 0.0};
        star1_ = world.CreateBody(star1Body, std::make_unique<osseus::ShapeSphere>(starRadius_));
        auto star1Shape = std::make_unique<sf::CircleShape>(static_cast<float>(starRadius_));
        star1Shape->setOrigin({static_cast<float>(starRadius_), static_cast<float>(starRadius_)});
        star1Shape->setFillColor(sf::Color::Yellow);

        renderObjects_.push_back({star1_, std::move(star1Shape)});

        // ==================== Star 2

        const osseus::BodyData star2Body{osseus::Vector3(orbitalRadius_, 0.0, 0.0),
                                         osseus::Vector3(0.0, orbitalSpeed_, 0.0), starMass_, 1.0 / starMass_, 0.0};
        star2_ = world.CreateBody(star2Body, std::make_unique<osseus::ShapeSphere>(starRadius_));
        auto star2Shape = std::make_unique<sf::CircleShape>(static_cast<float>(starRadius_));
        star2Shape->setOrigin({static_cast<float>(starRadius_), static_cast<float>(starRadius_)});
        star2Shape->setFillColor(sf::Color::Yellow);

        renderObjects_.push_back({star2_, std::move(star2Shape)});
    }

    void BinaryScenario::Update(osseus::PhysicsWorld& world) {
        // The binary system is governed entirely
        // by the gravitational force.
    }

} // namespace sandbox