//
// Created by MightySmallFry on 7/16/2026.
//

#ifndef OSSEUSENGINE_PHYSICSWORLD_H
#define OSSEUSENGINE_PHYSICSWORLD_H
#include <memory>
#include <stdint.h>
#include <vector>

#include "Osseus/interfaces/IIntegrator.h"
#include "Osseus/system/BodyManager.h"
#include "Osseus/system/ForceManager.h"
#include "Osseus/system/ShapeManager.h"

#include "Osseus/math/Solver.h"
#include "Osseus/math/Integrators/IntegratorEulerCromer.h"

#include "Osseus/system/BroadPhase.h"
#include "Osseus/system/NarrowPhase.h"
#include "Osseus/system/Registry.h"
#include "Osseus/physics/Octree.h"
#include "Osseus/physics/BarnesHut.h"

namespace osseus {

    class PhysicsWorld {
    public:
        PhysicsWorld() : integrator(std::make_unique<IntegratorEulerCromer>()) {}

        Handle CreateHandle();

        Handle CreateBody();
        Handle CreateBody(BodyData bodyData, std::unique_ptr<IShape> shape);
        
        // Try to queue the destroy body more often
        void DestroyBody(Handle handle); 
        void QueueDestroyBody(Handle handle); 
        
        void SetIntegrator(std::unique_ptr<IIntegrator> newIntegrator);
        
        void Step(double delta);
        
        BodyData* GetBody(Handle handle);
        const BodyData* GetBody(Handle handle) const;
        
        
        void RebuildOctree();
        
        ForceManager& GetForceManager() { return forceManager; }
        BodyManager& GetBodyManager() { return bodyManager; }
        ShapeManager& GetShapeManager() { return shapeManager; }
        
        private:
        void AttachBody(Handle handle, BodyData bodyData);
        void AttachShape(Handle handle, std::unique_ptr<IShape> shape);
        

        void SyncState();

        std::vector<Handle> destructionQueue;

        std::unique_ptr<IIntegrator> integrator;
        
        Registry registry;
        BodyManager bodyManager;
        ForceManager forceManager;
        ShapeManager shapeManager;
        Octree spatialTree;

        BarnesHut barnesHut_;
        BroadPhase broadPhase;
        NarrowPhase narrowPhase;
        Solver solver;

    };
} // osseus

#endif //OSSEUSENGINE_PHYSICSWORLD_H
