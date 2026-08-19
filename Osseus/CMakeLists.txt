add_library(Osseus)

target_sources(Osseus
        PRIVATE

        # Math
        src/math/Vector3.cpp
        src/math/Solver.cpp
        src/math/Quaternion.cpp
        src/math/Collision/EPA.cpp
        src/math/Collision/GJK.cpp

        # Integrators
        src/math/Integrators/IntegratorEulerCromer.cpp
        src/math/Integrators/IntegratorRungeKutta4.cpp

        # Physics
        src/physics/PhysicsWorld.cpp
        src/physics/Forces/ForceGravity.cpp
        src/physics/Forces/ForceComposite.cpp

        src/physics/Octree.cpp
        src/physics/BarnesHut.cpp

        # System
        src/system/Registry.cpp
        src/system/BodyManager.cpp
        src/system/ForceManager.cpp
        src/system/ShapeManager.cpp
        src/system/BroadPhase.cpp
        src/system/NarrowPhase.cpp
        src/system/SimulationState.cpp

        # Interfaces
        src/interfaces/IIntegrator.cpp
        src/interfaces/IForceEvaluator.cpp
        src/interfaces/IShape.cpp
)

target_include_directories(Osseus
        PUBLIC
            include
)

target_compile_features(Osseus
        PUBLIC
            cxx_std_20
)

target_compile_options(Osseus
        PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)