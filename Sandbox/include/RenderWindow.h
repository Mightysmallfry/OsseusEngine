#pragma once

#include <SFML/Graphics.hpp>

#include <string>

namespace sandbox {

    class RenderWindow {
    public:
        RenderWindow(
            unsigned int width,
            unsigned int height,
            const std::string& title
        );

        bool IsOpen() const;
        void HandleEvents();

        void BeginFrame();
        void EndFrame();

        sf::RenderWindow& Get();

    private:
        void HandleResize(const sf::Event::Resized& event);
        void UpdateView(unsigned int width, unsigned int height);

        sf::RenderWindow window_;
    };

} // namespace sandbox