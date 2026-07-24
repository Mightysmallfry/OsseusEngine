include(FetchContent)

FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.10.0
)

FetchContent_MakeAvailable(Catch2)

add_executable(OsseusTests)

target_sources(OsseusTests
        PRIVATE

        Tests.cpp

        # Math
        math/Vector3Tests.cpp

        # Physics
        # physics/PhysicsWorldTests.cpp

        # System
        # system/RegistryTests.cpp
)

target_link_libraries(
        OsseusTests
        PRIVATE
            Osseus
            Catch2::Catch2WithMain
)

include(Catch)

catch_discover_tests(OsseusTests)