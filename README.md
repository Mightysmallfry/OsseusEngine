# OsseusEngine
OsseusEngine is a C++20 physics engine focused on real-time rigid-body and particle simulation. It provides a modular physics architecture with numerical integration, collision detection, spatial acceleration structures, and extensible force systems.

The engine is currently under active development.

## Features
- C++20 physics engine
- Rigid-body and particle simulation
- Multiple numerical integrators
    - Euler-Cromer
    - Runge-Kutta 4 (RK4)
- 3D vector mathematics
- Collision detection and resolution
    - GJK/EPA collision detection
- Octree spatial partitioning
- Barnes-Hut force approximation
- Extensible force system
    - Universal and per-body forces
- Catch2-based physics tests
- SFML-based simulation sandbox

## Building Requirements
- C++20-compatible compiler
- CMake
- Ninja
- vcpkg

The project uses CMake presets and vcpkg for dependency management.

## Project Structure
```
OsseusEngine/
├── Osseus/
│   └── Include/
│   └── Osseus/
│   └── Osseus.h    // Main Include file You Want!
├── Tests/          // Catch2 Test Suite For the Library
├── Sandbox/        // Example Code and Dashboard for view engine telemetry
├── CMakeLists.txt
├── CMakePresets.json
└── vcpkg.json
```

### Architecture
OsseusEngine is organized around a few core systems:

| Component | Responsibility |
|-----------|----------------|
| **PhysicsWorld**  | Owns and coordinates the simulation. |
| **BodyManager**   | Creates, stores, and manages physics bodies. |
| **ForceManager**  | Manages forces applied to individual bodies and universal force evaluators. |
| **Integrators**   | Advance the simulation state using numerical integration methods. |
| **Octree**        | Provides spatial partitioning for efficient spatial queries and force calculations. |
| **Barnes-Hut**    | Approximates long-range forces for large numbers of bodies. |
| **Collision System** | Detects and resolves collisions between supported shapes. |

The architecture is designed so that individual systems can be developed and tested independently.

## Numerical Integration
The engine currently supports multiple integration methods.

### Euler-Cromer
Euler-Cromer is a simple, inexpensive integrator that is particularly useful for simulations where computational cost is important.

### Runge-Kutta 4
RK4 provides significantly greater numerical accuracy by evaluating the system derivative at multiple intermediate states during each timestep.

## Forces
Forces can be applied directly to individual bodies or through universal force evaluators.

Per-body forces can be added directly:

``` C++
world.GetForceManager().Add(
    objectHandle,
    osseus::Vector3(0.0, -9.8, 0.0)
);
```

Universal forces operate across the simulation and are useful for interactions such as gravity or electromagnetic effects.

``` C++
osseus::UniversalGravity gravity;

world.GetForceManager().AddUniversal(&gravity);
```

This separation allows forces that affect specific objects to coexist with forces that govern the entire simulation.

## Spatial Acceleration
OsseusEngine uses an **octree** to organize bodies spatially.

The octree provides the foundation for efficient spatial queries and is used by the Barnes-Hut force solver to approximate long-range interactions.

This allows the engine to avoid evaluating every possible pair of bodies when simulating large systems. Effectively going from `O(N^2)` time complexity to `O(NlogN)` complexity.

## Collision Detection
Collision detection is based on convex-shape algorithms that revolve around the Minkowski difference, including:

- [x] GJK (Gilbert-Johnson-Keerthi)
- [x] EPA (Expanding Polytope Algorithm)

These algorithms provide collision detection and penetration information for supported convex shapes.

## How The Engine works
Most of the compute time, the engine will likely be going through the `PhysicsWorld::Step(delta)` function. Top down, the step function performs the simulation in this order:
- Broad Phase collision finds candidates using Axis-Aligned Bounding Boxes
- Narrow Phase collision confirms which of the broad phase candidates do collide.
- CollisionSolver resolves the collisions and adjusts object properties (position, velocity etc.)
- The Octree is rebuilt since object positions could have changed due to collision.
- Barnes-Hut Evaluates all the bodies in the Octree, applying any universal force currently active.
- Whichever chosen integrator (Euler-Cromer by default) then moves all non-static objects.

### Static/Stationary Objects
Currently how stationary Objects are defined is super poor. Each object has a `BodyData` struct object associated with them in the `BodyManager` of their respective physics world. This `BodyData` object tracks both **Mass** and **InvMass** (inverse of mass). These are not dependent on each other... 

A stationary object has an `InvMass == 0.0`. All the math uses the objects mass so there isn't a problem mathematically but yeah... This is horrible.

### Constants
To keep things fairly fast and reasonable, Osseus does use it's own values for the gravitational constant as well as coulomb's constant.

Currently Osseus uses `1.0` for both constants, this will eventually change to allow the true values.