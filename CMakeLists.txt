cmake_minimum_required(VERSION 4.3)

project(
    OsseusEngine
    VERSION 0.1.0
    DESCRIPTION "A modular C++ physics engine"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_subdirectory(Osseus)
add_subdirectory(Sandbox)
add_subdirectory(Tests)