#include "RenderWindow.h"

namespace sandbox {

    RenderWindow::RenderWindow(
        unsigned int width,
        unsigned int height,
        const std::string& title
    )
        : window_(
            sf::VideoMode({width, height}),
            title,
            sf::Style::Default
        ) {

        window_.setFramerateLimit(60);

        UpdateView(width, height);
    }

    bool RenderWindow::IsOpen() const {
        return window_.isOpen();
    }

    void RenderWindow::HandleEvents() {
        while (const std::optional event = window_.pollEvent()) {

            if (event->is<sf::Event::Closed>()) {
                window_.close();
                continue;
            }

            if (const auto* resized =
                    event->getIf<sf::Event::Resized>()) {

                HandleResize(*resized);
                continue;
            }

            if (const auto* keyPressed =
                    event->getIf<sf::Event::KeyPressed>()) {

                if (keyPressed->scancode ==
                    sf::Keyboard::Scancode::Escape) {

                    window_.close();
                }
            }
        }
    }

    void RenderWindow::BeginFrame() {
        window_.clear(
            sf::Color(10, 15, 10)
        );
    }

    void RenderWindow::EndFrame() {
        window_.display();
    }

    sf::RenderWindow& RenderWindow::Get() {
        return window_;
    }

    void RenderWindow::HandleResize(
        const sf::Event::Resized& event
    ) {
        UpdateView(
            event.size.x,
            event.size.y
        );
    }

    void RenderWindow::UpdateView(
        unsigned int width,
        unsigned int height
    ) {
        if (width == 0 || height == 0) {
            return;
        }

        const sf::FloatRect viewport{
            {0.0f, 0.0f},
            {static_cast<float>(width),
             static_cast<float>(height)}
        };

        sf::View view(viewport);

        view.setViewport({
            {0.0f, 0.0f},
            {1.0f, 1.0f}
        });

        window_.setView(view);
    }

} // namespace sandbox