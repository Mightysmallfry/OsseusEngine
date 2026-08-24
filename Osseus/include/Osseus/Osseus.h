#ifndef OSSEUSENGINE_OSSEUS_H
#define OSSEUSENGINE_OSSEUS_H


// Osseus.h — umbrella header for the Osseus physics engine.
// Includes every public header in the engine so consumers can
// pull in the whole library with a single #include "Osseus.h".

// --- interfaces ---
#include "interfaces/IForceEvaluator.h"
#include "interfaces/IIntegrator.h"
#include "interfaces/IShape.h"

// --- math ---
#include "math/Vector3.h"
#include "math/Quaternion.h"
#include "math/Solver.h"

// --- math / Integrator
#include "math/Integrators/IntegratorEulerCromer.h"
#include "math/Integrators/IntegratorRungeKutta4.h"

// --- math / collision (GJK/EPA, support-function based) ---
#include "math/Collision/Contact.h"
#include "math/Collision/GJK.h"
#include "math/Collision/EPA.h"

// --- math / geometry (concrete convex shapes) ---
#include "math/Geometry/ShapePoint.h"
#include "math/Geometry/ShapeSphere.h"
#include "math/Geometry/ShapeCube.h"

// --- physics ---
#include "physics/PhysicsWorld.h"

#include "physics/Octree.h"
#include "physics/BarnesHut.h"

#include "physics/UniversalForce.h"
#include "physics/Forces/UniversalGravityForce.h"
#include "physics/Forces/UniversalEMForce.h"

// --- system ---
#include "system/AABB.h"
#include "system/Handle.h"
#include "system/Registry.h"
#include "system/SparseSet.h"
#include "system/BodyData.h"
#include "system/BodyManager.h"
#include "system/ShapeManager.h"
#include "system/ForceManager.h"
#include "system/CollisionCandidatePair.h"
#include "system/BroadPhase.h"
#include "system/NarrowPhase.h"
#include "system/SimulationState.h"

#endif // OSSEUSENGINE_OSSEUS_H
