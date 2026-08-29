#pragma once

#include <SFML/Graphics.hpp>

#include <string>
#include <vector>

namespace sandbox {

    struct Telemetry {
        std::size_t bodyCount = 0;
        std::size_t staticBodyCount = 0;
        double fps = 0.0;
        double frameTimeMs = 0.0;
        double physicsTimeMs = 0.0;
        double simulationTime = 0.0;
        double deltaTime = 0.0;
        std::size_t octreeNodes = 0;
        std::size_t octreeDepth = 0;
        std::size_t collisionCount = 0;
        std::size_t resolvedCollisionCount = 0;
        std::string integrator = "EULER-CROMER";
    };

    struct DashboardColors {
        sf::Color background{10, 15, 10};
        sf::Color panel{13, 18, 13};
        sf::Color border{32, 40, 32};
        sf::Color borderBright{48, 58, 48};
        sf::Color text{208, 216, 208};
        sf::Color textDim{120, 130, 120};
        sf::Color green{101, 196, 102};
        sf::Color yellow{216, 168, 78};
        sf::Color red{212, 92, 92};
    };

    struct DashboardEvent {
        std::string ts;
        std::string object;
        std::string status;
    };

    class Dashboard {
    public:
        explicit Dashboard(const sf::Font& font);

        void SetTelemetry(const Telemetry& telemetry);
        void SetEvents(const std::vector<std::string>& events);
        void AddEvent(const DashboardEvent event);

        void Draw(sf::RenderWindow& window);

    private:
        struct Layout {
            sf::FloatRect simulation;
            sf::FloatRect status;
            sf::FloatRect simulationInfo;
            sf::FloatRect eventLog;
        };

        Layout CalculateLayout(
            const sf::Vector2f& viewSize
        ) const;

        void DrawHeader(sf::RenderWindow& window);

        void DrawSimulationPanel(
            sf::RenderWindow& window,
            const sf::FloatRect& bounds
        );

        void DrawStatusPanel(
            sf::RenderWindow& window,
            const sf::FloatRect& bounds
        );

        void DrawSimulationInfo(
            sf::RenderWindow& window,
            const sf::FloatRect& bounds
        );

        void DrawEventLog(
            sf::RenderWindow& window,
            const sf::FloatRect& bounds
        );

        void DrawPanel(
            sf::RenderWindow& window,
            const sf::FloatRect& bounds
        );

        void DrawPanelHeader(
            sf::RenderWindow& window,
            const sf::FloatRect& bounds,
            const std::string& title,
            const std::string& status
        );

        void DrawText(
            sf::RenderWindow& window,
            const std::string& string,
            const sf::Vector2f& position,
            unsigned int characterSize,
            const sf::Color& color
        );

        void DrawMetric(
            sf::RenderWindow& window,
            const sf::FloatRect& bounds,
            const std::string& label,
            const std::string& value
        );

        void DrawDataRow(
            sf::RenderWindow& window,
            const sf::FloatRect& bounds,
            const std::string& label,
            const std::string& value
        );

        const sf::Font& font_;
        DashboardColors colors_;
        Telemetry telemetry_;
        std::vector<std::string> events_;
    };

} // namespace sandbox