//
// Created by MightySmallFry on 7/16/2026.
//

#ifndef OSSEUSENGINE_PHYSICSWORLD_H
#define OSSEUSENGINE_PHYSICSWORLD_H
#include <memory>
#include <stdint.h>
#include <vector>

#include "Osseus/interfaces/IIntegrator.h"
#include "Osseus/physics/Forces/ForceComposite.h"
#include "Osseus/math/Solver.h"
#include "Osseus/math/Integrators/IntegratorEulerCromer.h"
#include "Osseus/system/BodyManager.h"
#include "Osseus/system/BroadPhase.h"
#include "Osseus/system/Registry.h"

namespace osseus {

    class PhysicsWorld {
    public:
        PhysicsWorld() : integrator(std::make_unique<IntegratorEulerCromer>()) {}

        Handle CreateHandle();

        Handle CreateBody(BodyData bodyData, std::unique_ptr<IShape> shape);
        void DestroyBody(Handle handle);

        void AddForce(std::unique_ptr<IForceEvaluator> force);

        void AttachBody(Handle handle, BodyData bodyData);
        void AttachShape(Handle handle, std::unique_ptr<IShape> shape);


        void SetIntegrator(std::unique_ptr<IIntegrator> newIntegrator);

        void Step(double delta);

        BodyData* GetBody(Handle handle);
        const BodyData* GetBody(Handle handle) const;

    private:
        std::unique_ptr<IIntegrator> integrator;

        Registry registry;
        BodyManager bodyManager;
        ShapeManager shapeManager;
        // ConstraintManager constraintManager;

        ForceComposite forces;
        Solver solver;
        BroadPhase broadPhase;
    };
} // osseus

#endif //OSSEUSENGINE_PHYSICSWORLD_H
