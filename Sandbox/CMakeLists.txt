add_executable(osseus-sandbox
        src/main.cpp
)

find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

target_include_directories(osseus-sandbox
        PUBLIC
        include
)

target_link_libraries(
        osseus-sandbox
        PRIVATE
        Osseus
        SFML::Graphics
        SFML::Window
        SFML::System
)