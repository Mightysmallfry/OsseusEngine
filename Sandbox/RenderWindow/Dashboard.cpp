#include "Dashboard.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace sandbox {

    namespace {

        constexpr float LogicalWidth = 1280.0f;
        constexpr float LogicalHeight = 720.0f;

        constexpr float HeaderHeight = 48.0f;
        constexpr float PanelHeaderHeight = 32.0f;
        constexpr float Padding = 8.0f;

        constexpr float SidebarRatio = 0.28f;
        constexpr float EventLogRatio = 0.20f;
        constexpr float StatusRatio = 0.40f;

        constexpr float MinimumPanelWidth = 80.0f;
        constexpr float MinimumPanelHeight = 40.0f;

        std::string FormatInteger(std::size_t value) {
            return std::to_string(value);
        }

        std::string FormatDouble(double value, int precision) {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(precision) << value;
            return stream.str();
        }

    } // namespace

    Dashboard::Dashboard(const sf::Font& font) : font_(font) {
    }

    void Dashboard::SetTelemetry(const Telemetry& telemetry) {
        telemetry_ = telemetry;
    }

    void Dashboard::SetEvents(const std::vector<std::string>& events) {
        events_ = events;
    }

    void Dashboard::AddEvent(const DashboardEvent event) {
        events_.push_back(event.ts + " " + event.object + " " + event.status);
    }

    void Dashboard::Draw(sf::RenderWindow& window) {
        /*
         * IMPORTANT:
         *
         * getSize() returns the physical/native window size.
         * getView().getSize() returns our logical 1280x720
         * coordinate space.
         */
        const sf::Vector2f viewSize = window.getView().getSize();

        const Layout layout = CalculateLayout(viewSize);

        DrawHeader(window);
        DrawSimulationPanel(window, layout.simulation);
        DrawStatusPanel(window, layout.status);
        DrawSimulationInfo(window, layout.simulationInfo);
        DrawEventLog(window, layout.eventLog);
    }

    Dashboard::Layout Dashboard::CalculateLayout(const sf::Vector2f& viewSize) const {
        const float width = viewSize.x;
        const float height = viewSize.y;

        const float contentTop = HeaderHeight + Padding;
        const float contentBottom = height - Padding;
        const float availableHeight = std::max(0.0f, contentBottom - contentTop);

        /*
         * Event log occupies 20% of the available height
         * and remains anchored to the bottom.
         */
        const float eventLogHeight = std::max(MinimumPanelHeight, availableHeight * EventLogRatio);
        const float eventLogTop = contentBottom - eventLogHeight;

        /*
         * Main dashboard area.
         */
        const float mainTop = contentTop;
        const float mainBottom = eventLogTop - Padding;
        const float mainHeight = std::max(0.0f, mainBottom - mainTop);

        /*
         * Sidebar occupies 28% of the logical width.
         */
        const float sidebarWidth = std::max(MinimumPanelWidth, width * SidebarRatio);
        const float simulationWidth = std::max(MinimumPanelWidth, width - sidebarWidth - Padding * 3.0f);
        const float sidebarX = width - sidebarWidth - Padding;

        Layout layout;

        /*
         * Orbital simulation.
         */
        layout.simulation = {{Padding, mainTop}, {simulationWidth, mainHeight}};

        /*
         * System status.
         */
        const float statusHeight = std::max(MinimumPanelHeight, mainHeight * StatusRatio);
        layout.status = {{sidebarX, mainTop}, {sidebarWidth, statusHeight}};

        /*
         * Simulation information.
         */
        const float simulationInfoTop = mainTop + statusHeight + Padding;
        const float simulationInfoHeight = std::max(0.0f, mainBottom - simulationInfoTop);

        layout.simulationInfo = {{sidebarX, simulationInfoTop}, {sidebarWidth, simulationInfoHeight}};

        /*
         * Event log.
         */
        layout.eventLog = {{Padding, eventLogTop}, {std::max(0.0f, width - Padding * 2.0f), eventLogHeight}};

        return layout;
    }

    void Dashboard::DrawHeader(sf::RenderWindow& window) {
        const sf::Vector2f size = window.getView().getSize();

        constexpr float Edge = 1.0f;

        sf::RectangleShape header;

        header.setPosition({Edge, Edge});
        header.setSize({size.x - Edge * 2.0f, HeaderHeight - Edge});
        header.setFillColor(colors_.panel);

        window.draw(header);

        sf::RectangleShape border;
        border.setPosition({Edge, HeaderHeight - 1.0f});
        border.setSize({size.x - Edge * 2.0f, 1.0f});
        border.setFillColor(colors_.border);
        window.draw(border);

        DrawText(window, "OSSEUS // MONITOR", {16.0f, 15.0f}, 13, colors_.text);
    }

    void Dashboard::DrawSimulationPanel(sf::RenderWindow& window, const sf::FloatRect& bounds) {
        if (bounds.size.x < MinimumPanelWidth || bounds.size.y < MinimumPanelHeight) {
            return;
        }

        DrawPanel(window, bounds);

        DrawPanelHeader(window, bounds, "ORBITAL VIEW", "LIVE");
    }

    void Dashboard::DrawStatusPanel(sf::RenderWindow& window, const sf::FloatRect& bounds) {
        if (bounds.size.x < MinimumPanelWidth || bounds.size.y < MinimumPanelHeight) {
            return;
        }

        DrawPanel(window, bounds);

        DrawPanelHeader(window, bounds, "SYSTEM STATUS", "LIVE");

        const float contentY = bounds.position.y + PanelHeaderHeight;

        const float metricWidth = bounds.size.x / 2.0f;

        DrawMetric(window, {{bounds.position.x, contentY}, {metricWidth, 82.0f}}, "BODIES",
                   FormatInteger(telemetry_.bodyCount));

        DrawMetric(window, {{bounds.position.x + metricWidth, contentY}, {metricWidth, 82.0f}}, "STATIC",
                   FormatInteger(telemetry_.staticBodyCount));

        DrawMetric(window, {{bounds.position.x, contentY + 82.0f}, {metricWidth, 82.0f}}, "FPS",
                   FormatDouble(telemetry_.fps, 0));

        DrawMetric(window, {{bounds.position.x + metricWidth, contentY + 82.0f}, {metricWidth, 82.0f}}, "PHYSICS",
                   FormatDouble(telemetry_.physicsTimeMs, 2) + " ms");
    }

    void Dashboard::DrawSimulationInfo(sf::RenderWindow& window, const sf::FloatRect& bounds) {
        if (bounds.size.x < MinimumPanelWidth || bounds.size.y < MinimumPanelHeight) {
            return;
        }

        DrawPanel(window, bounds);

        DrawPanelHeader(window, bounds, "SIMULATION", "");

        constexpr float rowHeight = 28.0f;

        float rowY = bounds.position.y + PanelHeaderHeight;

        const auto row = [&](const std::string& label, const std::string& value) {
            if (rowY + rowHeight > bounds.position.y + bounds.size.y) {
                return;
            }

            DrawDataRow(window, {{bounds.position.x, rowY}, {bounds.size.x, rowHeight}}, label, value);

            rowY += rowHeight;
        };

        row("TIME", FormatDouble(telemetry_.simulationTime, 3));
        row("DT", FormatDouble(telemetry_.deltaTime, 4));
        row("INTEGRATOR", telemetry_.integrator);
        row("OCTREE NODES", FormatInteger(telemetry_.octreeNodes));
        row("OCTREE DEPTH", FormatInteger(telemetry_.octreeDepth));
        row("COLLISIONS", FormatInteger(telemetry_.collisionCount));
        row("RESOLVED", FormatInteger(telemetry_.resolvedCollisionCount));
    }

    void Dashboard::DrawEventLog(sf::RenderWindow& window, const sf::FloatRect& bounds) {
        if (bounds.size.x < MinimumPanelWidth || bounds.size.y < MinimumPanelHeight) {
            return;
        }

        DrawPanel(window, bounds);
        DrawPanelHeader(window, bounds, "EVENT LOG", "");

        constexpr float rowHeight = 22.0f;
        float rowY = bounds.position.y + PanelHeaderHeight + 4.0f;
        const float availableHeight = bounds.size.y - PanelHeaderHeight - 8.0f;
        const std::size_t maxEvents =
            availableHeight > 0.0f ? static_cast<std::size_t>(availableHeight / rowHeight) : 0;
        const std::size_t start = events_.size() > maxEvents ? events_.size() - maxEvents : 0;
        for (std::size_t i = start; i < events_.size(); ++i) {
            DrawText(window, events_[i], {bounds.position.x + 10.0f, rowY}, 10, colors_.textDim);
            rowY += rowHeight;
        }
    }

    void Dashboard::DrawPanel(sf::RenderWindow& window, const sf::FloatRect& bounds) {
        sf::RectangleShape panel;

        panel.setPosition(bounds.position);
        panel.setSize(bounds.size);
        panel.setFillColor(colors_.panel);
        panel.setOutlineColor(colors_.border);
        panel.setOutlineThickness(1.0f);

        window.draw(panel);
    }

    void Dashboard::DrawPanelHeader(sf::RenderWindow& window, const sf::FloatRect& bounds, const std::string& title,
                                    const std::string& status) {
        sf::RectangleShape line;

        line.setPosition({bounds.position.x, bounds.position.y + PanelHeaderHeight});
        line.setSize({bounds.size.x, 1.0f});
        line.setFillColor(colors_.border);

        window.draw(line);

        DrawText(window, title, {bounds.position.x + 10.0f, bounds.position.y + 10.0f}, 10, colors_.text);
        if (!status.empty()) {
            sf::Text statusText(font_, status, 9);
            statusText.setFillColor(colors_.green);

            const float statusWidth = statusText.getLocalBounds().size.x;

            statusText.setPosition(
                {bounds.position.x + bounds.size.x - statusWidth - 10.0f, bounds.position.y + 10.0f});

            window.draw(statusText);
        }
    }

    void Dashboard::DrawText(sf::RenderWindow& window, const std::string& string, const sf::Vector2f& position,
                             unsigned int characterSize, const sf::Color& color) {
        sf::Text text(font_, string, characterSize);

        text.setPosition(position);
        text.setFillColor(color);

        window.draw(text);
    }

    void Dashboard::DrawMetric(sf::RenderWindow& window, const sf::FloatRect& bounds, const std::string& label,
                               const std::string& value) {
        DrawText(window, label, {bounds.position.x + 10.0f, bounds.position.y + 12.0f}, 9, colors_.textDim);
        DrawText(window, value, {bounds.position.x + 10.0f, bounds.position.y + 34.0f}, 18, colors_.text);
    }

    void Dashboard::DrawDataRow(sf::RenderWindow& window, const sf::FloatRect& bounds, const std::string& label,
                                const std::string& value) {
        DrawText(window, label, {bounds.position.x + 10.0f, bounds.position.y + 7.0f}, 9, colors_.textDim);
        sf::Text valueText(font_, value, 9);
        valueText.setFillColor(colors_.text);
        valueText.setPosition(
            {bounds.position.x + bounds.size.x - valueText.getLocalBounds().size.x - 10.0f, bounds.position.y + 7.0f});

        window.draw(valueText);
    }

} // namespace sandbox