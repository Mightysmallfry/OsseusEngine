#include "RenderWindow.h"

namespace sandbox {

    RenderWindow::RenderWindow(unsigned int width, unsigned int height, const std::string& title)
        : window_(sf::VideoMode({width, height}), title, sf::Style::Default) {
        window_.setFramerateLimit(60);
    }

    bool RenderWindow::IsOpen() const {
        return window_.isOpen();
    }

    void RenderWindow::HandleEvents() {
        while (const std::optional event = window_.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window_.close();
            } else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>()) {

                if (keyPressed->scancode == sf::Keyboard::Scancode::Escape) {
                    window_.close();
                }
            } else if (const auto* resized = event->getIf<sf::Event::Resized>()) {

                HandleResize(*resized);
            }
        }
    }

    void RenderWindow::BeginFrame() {
        window_.clear(sf::Color::Black);
    }

    void RenderWindow::EndFrame() {
        window_.display();
    }

    void RenderWindow::DrawSimulation(const osseus::PhysicsWorld& world, const IScenario& scenario,
                                      const sf::FloatRect& bounds) {
        for (const RenderObject& object : scenario.GetRenderObjects()) {

            const osseus::BodyData* body = world.GetBody(object.handle);

            if (body == nullptr || object.shape == nullptr) {
                continue;
            }

            object.shape->setPosition(WorldToScreen(body->position, bounds));

            window_.draw(*object.shape);
        }
    }

    sf::RenderWindow& RenderWindow::Get() {
        return window_;
    }

    void RenderWindow::HandleResize(const sf::Event::Resized& event) {
        UpdateView(event.size.x, event.size.y);
    }

    void RenderWindow::UpdateView(unsigned int width, unsigned int height) {
        const sf::FloatRect visibleArea({0.0f, 0.0f}, {static_cast<float>(width), static_cast<float>(height)});

        window_.setView(sf::View(visibleArea));
    }

    sf::Vector2f RenderWindow::WorldToScreen(const osseus::Vector3& position, const sf::FloatRect& bounds) const {
        return {bounds.position.x + bounds.size.x / 2.0f + static_cast<float>(position.x),
                bounds.position.y + bounds.size.y / 2.0f - static_cast<float>(position.y)};
    }

} // namespace sandbox