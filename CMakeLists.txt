cmake_minimum_required(VERSION 4.3)
project(OsseusEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_library(Osseus
        Osseus/src/math/Vector3.cpp
        Osseus/src/physics/PhysicsWorld.cpp
        Osseus/src/physics/PhysicsBody.cpp
        Osseus/include/Osseus/system/PhysicsHandle.h
        Osseus/include/Osseus/system/SparseSet.h
        Osseus/include/Osseus/system/SparseSet.tpp
        Osseus/src/system/Registry.cpp
        Osseus/include/Osseus/system/Registry.h
        Osseus/src/interfaces/IIntegrator.cpp
        Osseus/include/Osseus/interfaces/IIntegrator.h
        Osseus/src/math/Solver.cpp
        Osseus/include/Osseus/math/Solver.h
        Osseus/src/system/BodyManager.cpp
        Osseus/include/Osseus/system/BodyManager.h
        Osseus/src/interfaces/IForceEvaluator.cpp
        Osseus/include/Osseus/interfaces/IForceEvaluator.h
        Osseus/include/Osseus/system/BodyData.h
        Osseus/src/math/Integrators/IntegratorEulerCromer.cpp
        Osseus/include/Osseus/math/Integrators/IntegratorEulerCromer.h
        Osseus/src/math/Integrators/IntegratorRungeKutta4.cpp
        Osseus/include/Osseus/math/Integrators/IntegratorRungeKutta4.h
        Osseus/src/physics/Forces/ForceGravity.cpp
        Osseus/include/Osseus/physics/Forces/ForceGravity.h
        Osseus/src/physics/Forces/ForceComposite.cpp
        Osseus/include/Osseus/physics/Forces/ForceComposite.h
        Osseus/src/system/ShapeManager.cpp
        Osseus/include/Osseus/system/ShapeManager.h
        Osseus/src/interfaces/IShape.cpp
        Osseus/include/Osseus/interfaces/IShape.h
        Osseus/include/Osseus/system/AABB.h
        Osseus/include/Osseus/math/Geometry/ShapeSphere.h
        Osseus/include/Osseus/math/Geometry/ShapePoint.h
        Osseus/include/Osseus/system/CollisionCandidatePair.h
        Osseus/src/system/BroadPhase.cpp
        Osseus/include/Osseus/system/BroadPhase.h
)

target_include_directories(Osseus
        PUBLIC
        Osseus/include
)

add_executable(Sandbox
        Sandbox/main.cpp
)

target_link_libraries(Sandbox PRIVATE Osseus)