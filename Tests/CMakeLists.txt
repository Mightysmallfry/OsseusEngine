# include(FetchContent)

# FetchContent_Declare(
#         Catch2
#         GIT_REPOSITORY https://github.com/catchorg/Catch2.git
#         GIT_TAG v3.10.0
# )

# FetchContent_MakeAvailable(Catch2)

find_package(Catch2 3 REQUIRED)

add_executable(OsseusTests
        math/RK4Tests.cpp
        math/EulerCromerTests.cpp
        math/QuaternionTests.cpp
        math/CollisionTests.cpp
        math/Vector3Tests.cpp
        physics/ForceTests.cpp
        physics/OctreeTests.cpp
        physics/BarnesHutExclusionTests.cpp
        physics/OrbitTests.cpp
        system/LockstepTests.cpp
        system/Octreerootboundstests.cpp
        )

target_link_libraries(
        OsseusTests
        PRIVATE
            Osseus
            Catch2::Catch2WithMain
)

include(Catch)

catch_discover_tests(OsseusTests)