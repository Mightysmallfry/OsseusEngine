//
// Created by MightySmallFry on 7/16/2026.
//

#include <algorithm>
#include <cstddef>

#include "Osseus/physics/PhysicsWorld.h"

namespace osseus {
    Handle PhysicsWorld::CreateHandle() {
        return registry.CreateHandle();
    }

    Handle PhysicsWorld::CreateBody(){
        Handle handle = CreateHandle();
        bodyManager.Register(handle);
        forceManager.Register(handle);
        shapeManager.Register(handle);
        return handle;
    }

    Handle PhysicsWorld::CreateBody(BodyData bodyData, std::unique_ptr<IShape> shape) {
        Handle handle = CreateHandle();
        AttachBody(handle, bodyData);
        AttachShape(handle, std::move(shape));
        forceManager.Register(handle);
        return handle;
    }

    void PhysicsWorld::DestroyBody(Handle handle) {
        if (!registry.IsValid(handle)) { return; }
        bodyManager.RemoveBody(handle);
        shapeManager.RemoveShape(handle);
        registry.Destroy(handle);
        forceManager.ClearForceOf(handle);
    }

    void PhysicsWorld::QueueDestroyBody(Handle handle) {
        if (!registry.IsValid(handle)) { return; }
        destructionQueue.push_back(handle);
    }

    void PhysicsWorld::AttachBody(Handle handle, BodyData bodyData) {
        if (!registry.IsValid(handle)) { return; }
        bodyManager.AddBody(handle, bodyData);
        forceManager.Add(handle, Vector3::Zero());
    }

    void PhysicsWorld::AttachShape(Handle handle, std::unique_ptr<IShape> shape) {
        if (!registry.IsValid(handle)) { return; }
        shapeManager.AddShape(handle, std::move(shape));
    }

    void PhysicsWorld::SetIntegrator(std::unique_ptr<IIntegrator> newIntegrator) {
        integrator = std::move(newIntegrator);
    }

    
    void PhysicsWorld::RebuildOctree() {
        spatialTree.Clear();
        
        // An Octree of Handles.
        for (auto& handle : bodyManager.Handles()){
            BodyData* body = bodyManager.GetBody(handle);
            spatialTree.Insert(handle, body->position, body->mass);
        }
    }



    void PhysicsWorld::Step(double delta) {

        // ============ Detect Collisions ============
        // Broad Phase
        std::vector<CollisionCandidatePair> candidates =
            broadPhase.FindCandidatePairs(bodyManager, shapeManager);

        // Narrow Phase (GJK/EPA via IShape::Support)
        std::vector<Contact> contacts =
            narrowPhase.GenerateContacts(candidates, bodyManager, shapeManager);

        // ============ Resolve Collisions ============
        // Solver
        solver.ResolveContacts(contacts, bodyManager);

        // =========== Apply Universal Forces ===========
        RebuildOctree();
        // Apply Universal Forces Via Barnes Hut
 
        // Build barnes hut evaluation.
        barnesHut_.Evaluate(spatialTree, bodyManager.Handles(), 
            bodyManager.Data(), forceManager);

        // =========== Apply Individual Forces ==========
        // This is done via the public ForceManager

        // ============ Resolve Trajectories ============
        // Integrator
        integrator->Step(bodyManager, forceManager, delta);

        // Sync State
        SyncState();
    }

    BodyData * PhysicsWorld::GetBody(Handle handle) {
        if (!registry.IsValid(handle)) { return nullptr; }
        return bodyManager.GetBody(handle);
    }

    const BodyData* PhysicsWorld::GetBody(Handle handle) const {
        if (!registry.IsValid(handle)) { return nullptr; }
        return bodyManager.GetBody(handle);
    }

    void PhysicsWorld::SyncState(){
        if (destructionQueue.size() > 0){
            for (Handle handle : destructionQueue)
            {
                DestroyBody(handle);
            }
        }
    }

} // osseus