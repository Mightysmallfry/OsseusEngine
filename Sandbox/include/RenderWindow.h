#ifndef OSSEUS_SANDBOX_RENDERWINDOW_H
#define OSSEUS_SANDBOX_RENDERWINDOW_H

#include <Osseus/Osseus.h>
#include <SFML/Graphics.hpp>

#include "IScenario.h"

#include <string>

namespace sandbox {

    class RenderWindow {
        public:
        RenderWindow(unsigned int width, unsigned int height, const std::string& title);

        bool IsOpen() const;
        void HandleEvents();

        void BeginFrame();
        void EndFrame();

        void DrawSimulation(const osseus::PhysicsWorld& world, const IScenario& scenario, const sf::FloatRect& bounds);

        sf::RenderWindow& Get();

        private:
        void HandleResize(const sf::Event::Resized& event);
        void UpdateView(unsigned int width, unsigned int height);

        sf::Vector2f WorldToScreen(const osseus::Vector3& position, const sf::FloatRect& bounds) const;

        sf::RenderWindow window_;
    };

} // namespace sandbox

#endif