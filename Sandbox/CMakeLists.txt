add_executable(osseus-sandbox
        #src/main.cpp
        src/window.cpp

        RenderWindow/Dashboard.cpp
        RenderWindow/RenderWindow.cpp
        src/Scenarios/OrbitScenario.cpp
        src/Scenarios/BinaryScenario.cpp
        src/Scenarios/NBodyContactlessScenario.cpp
)


find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

target_include_directories(osseus-sandbox
        PUBLIC
        include
        RenderWindow
)

target_link_libraries(
        osseus-sandbox
        PRIVATE
        Osseus
        SFML::Graphics
        SFML::Window
        SFML::System
)