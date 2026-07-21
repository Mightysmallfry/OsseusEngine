#ifndef OSSEUSENGINE_OSSEUS_H
#define OSSEUSENGINE_OSSEUS_H


// Osseus.h — umbrella header for the Osseus physics engine.
// Includes every public header in the engine so consumers can
// pull in the whole library with a single #include "Osseus.h".

// --- interfaces ---
#include "interfaces/IForceEvaluator.h"
#include "interfaces/IIntegrator.h"

// --- math ---
#include "math/Vector3.h"
#include "math/Solver.h"
#include "math/Integrators/IntegratorEulerCromer.h"
#include "math/Integrators/IntegratorRungeKutta4.h"

// --- physics ---
#include "physics/PhysicsBody.h"
#include "physics/PhysicsWorld.h"
#include "physics/Forces/ForceComposite.h"
#include "physics/Forces/ForceGravity.h"

// --- system ---
#include "system/PhysicsHandle.h"
#include "system/Registry.h"
#include "system/SparseSet.h"
#include "system/BodyData.h"
#include "system/BodyManager.h"



#endif // OSSEUSENGINE_OSSEUS_H
