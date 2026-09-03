# include(FetchContent)

# FetchContent_Declare(
#         Catch2
#         GIT_REPOSITORY https://github.com/catchorg/Catch2.git
#         GIT_TAG v3.10.0
# )

# FetchContent_MakeAvailable(Catch2)

find_package(Catch2 3 REQUIRED)

add_executable(OsseusTests
        # Test that need adjustments
        math/RK4Tests.cpp
        math/EulerCromerTests.cpp
        math/QuaternionTests.cpp
        math/CollisionTests.cpp
        math/Vector3Tests.cpp
        physics/ForceTests.cpp
        # physics/OctreeTests.cpp
        physics/BarnesHutExclusionTests.cpp
        physics/OrbitTests.cpp

        system/LockstepTests.cpp
        system/Octreerootboundstests.cpp        
        
        # Good Tests
        physics/PhysicsWorldEdgeCaseTests.cpp
        physics/SimulationStateTests.cpp

        system/BaumgarteTests.cpp
        system/BroadPhaseTests.cpp
        system/NarrowPhaseTests.cpp
        system/GJKBoundaryTests.cpp
        system/LifecycleStressTests.cpp
        system/OrderAndDeterminismTests.cpp
        system/PipelineScenarioTests.cpp
        system/CoreInvariantsTests.cpp
        system/NumericalEdgeCaseTests.cpp
        
        object-storage/HandleRegistryTests.cpp
        object-storage/SparseSetTests.cpp
        
        manager/ForceManagerTests.cpp
        manager/BodyManagerTests.cpp
        manager/ShapeManagerTests.cpp
        manager/ShapeTests.cpp
        
        data-structure/OctreeTests.cpp
        data-structure/OctreeChargeTests.cpp
        data-structure/AABBTests.cpp
        
        forces/GravityEMTests.cpp
        forces/IndividualForceTests.cpp

        integrators/EulerCromerKinematicsTests.cpp
        integrators/RK4ExpandedTests.cpp
        integrators/IntegratorComparisonTests.cpp
        )

target_link_libraries(
        OsseusTests
        PRIVATE
            Osseus
            Catch2::Catch2WithMain
)

include(Catch)

catch_discover_tests(OsseusTests)