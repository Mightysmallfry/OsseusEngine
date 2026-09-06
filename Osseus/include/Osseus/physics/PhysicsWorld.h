#ifndef OSSEUSENGINE_PHYSICSWORLD_H
#define OSSEUSENGINE_PHYSICSWORLD_H

#include <stdint.h>
#include <memory>
#include <vector>

#include "Osseus/interfaces/IIntegrator.h"
#include "Osseus/math/Integrators/IntegratorEulerCromer.h"
#include "Osseus/math/Baumgarte.h"
#include "Osseus/physics/BarnesHut.h"
#include "Osseus/physics/Octree.h"
#include "Osseus/system/BodyManager.h"
#include "Osseus/system/BroadPhase.h"
#include "Osseus/system/ForceManager.h"
#include "Osseus/system/NarrowPhase.h"
#include "Osseus/system/Registry.h"
#include "Osseus/system/ShapeManager.h"
#include "Osseus/system/ApproximationMode.h"

namespace osseus {

    enum class CollisionMode {
        ENABLED,
        DISABLED
    };

    class PhysicsWorld {
        public:
        PhysicsWorld() : integrator_(std::make_unique<IntegratorEulerCromer>()) {
        }

        Handle CreateHandle();

        Handle CreateBody();
        Handle CreateBody(BodyData bodyData);
        Handle CreateBody(BodyData bodyData, std::unique_ptr<IShape> shape);

        // Try to queue the destroy body more often
        void DestroyBody(Handle handle);
        void QueueDestroyBody(Handle handle);

        void SetIntegrator(std::unique_ptr<IIntegrator> newIntegrator);

        void Step(double delta);
        
        void RebuildOctree();
        
        // Force Manager
        Vector3 GetNetForce(Handle handle);
        const Vector3 GetNetForce(Handle handle) const;

        void AddForce(Handle handle, Vector3& force);
        void AddUniversalForce(UniversalForceEvaluator* uForce);
        
        // Body Manager
        BodyData* GetBody(Handle handle);
        const BodyData* GetBody(Handle handle) const;
        

        Octree& GetOctree() {
            return spatialTree_;
        }

        int GetObjectCount() const {
            return objectCount_;
        }

        double GetElapsedTime() const {
            return elapsedTime_;
        }

        std::string GetIntegratorName() {
            return integrator_->GetName();
        }

        const std::vector<Contact>& GetCollisionManifold() {
            return collisionManifold_;
        }

        void SetCollisionMode(CollisionMode mode);

        private:        
        void AttachBody(Handle handle, BodyData bodyData);
        void AttachShape(Handle handle, std::unique_ptr<IShape> shape);
        
        void SyncState();

        // Computes an AABB over the current body positions (with padding),
        // for use as the octree's root bounds. RebuildOctree() must derive
        // fresh bounds from the actual bodies every call rather than
        // relying on a fixed default -- see Octree::SetRootBounds.
        Bounds ComputeWorldBounds() const;
        
        CollisionMode collisionMode_{ CollisionMode::ENABLED };


        std::vector<Handle> destructionQueue_;
        
        std::vector<Contact> collisionManifold_;

        std::unique_ptr<IIntegrator> integrator_;
        
        Registry registry_;
        BodyManager bodyManager_;
        ForceManager forceManager_;
        ShapeManager shapeManager_;
        Octree spatialTree_;
        
        BarnesHut barnesHut_;
        BroadPhase broadPhase_;
        NarrowPhase narrowPhase_;
        Baumgarte baumGarte_;
        
        int objectCount_{ 0 };
        double elapsedTime_{ 0.0 };
    };
} // namespace osseus

#endif // OSSEUSENGINE_PHYSICSWORLD_H