# OsseusEngine
A physics engine, built with the flexibility for both Science and Game Development



## How it works
Osseus runs a pretty similar process to alot of other engines. You can add whichever bodies you want to a `PhysicsWorld`. Following that the physics world simulates each step by doing the following process.

### Broadphase 
Focuses on broad collision using bounding boxes around shapes and bodies.

### Narrowphase
After receiving any collision candidates from the broadphase, we use the GJK(Gilbert-Johnson-Keerthi)/EPA(Expanding Polytop Algorithm) method of determining precise collisions.

- GJK focuses on if two convex shapes overlap, giving shortest distance between the two in the case of no collision.
- EPA focuses on how much depth and direction of a collision.

> The Support() function handles the Minkowski Difference needed for the narrow phase pipeline, so no explicit construction needed

### Collision Resolution
Any collisions found are numerically resolved and adjustments to trajectories are made.

### Universal Forces
All universal forces that are computationally heavy, and apply to the entire simulation go here. This is primarily for `Coulomb's Law` and the `Universal Gravitational Force` to be applied in particle physics. It uses a `Barnes-Hut` approach with an Octree to maintain an O(NlogN) performance.

These would be the force at a distance laws that effect everything.
- Gravity
- Electromagnetism

### Individual Forces
You can also add forces to individual bodies in which case you can see if there is a force evaluator already made for it. If there isn't feel free to write your own or just manually add the force. These forces are only around for one iteration of the loop so if you want force to be distributed or applied for a set duration, you will have to track the elapsed time by tracking the time `delta` for a number of cycles.

### Integration
The bodies are now pushed through one step of the chosen integrator. That could be something like `4th order Runge-Kutta` for scientific endeavors or the `Euler-Cromer` method for something more akin to video games.

