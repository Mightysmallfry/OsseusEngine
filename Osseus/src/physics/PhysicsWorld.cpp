//
// Created by MightySmallFry on 7/16/2026.
//

#include <algorithm>
#include <cstddef>

#include "Osseus/physics/PhysicsWorld.h"

namespace osseus {
    Handle PhysicsWorld::CreateHandle() {
        return registry_.CreateHandle();
    }

    Handle PhysicsWorld::CreateBody() {
        Handle handle = CreateHandle();
        bodyManager_.Register(handle);
        forceManager_.Register(handle);
        shapeManager_.Register(handle);
        return handle;
    }

    Handle PhysicsWorld::CreateBody(BodyData bodyData) {
        Handle handle = CreateHandle();
        AttachBody(handle, bodyData);
        forceManager_.Register(handle);
        shapeManager_.Register(handle);
        return handle;
    }

    Handle PhysicsWorld::CreateBody(BodyData bodyData, std::unique_ptr<IShape> shape) {
        Handle handle = CreateHandle();
        AttachBody(handle, bodyData);
        AttachShape(handle, std::move(shape));
        forceManager_.Register(handle);
        return handle;
    }

    void PhysicsWorld::DestroyBody(Handle handle) {
        if (!registry_.IsValid(handle)) {
            return;
        }
        bodyManager_.RemoveBody(handle);
        shapeManager_.RemoveShape(handle);
        registry_.Destroy(handle);
        forceManager_.ClearForceOf(handle);
    }

    void PhysicsWorld::QueueDestroyBody(Handle handle) {
        if (!registry_.IsValid(handle)) {
            return;
        }
        destructionQueue_.push_back(handle);
    }

    void PhysicsWorld::AttachBody(Handle handle, BodyData bodyData) {
        if (!registry_.IsValid(handle)) {
            return;
        }
        bodyManager_.AddBody(handle, bodyData);
        forceManager_.Add(handle, Vector3::Zero());
    }

    void PhysicsWorld::AttachShape(Handle handle, std::unique_ptr<IShape> shape) {
        if (!registry_.IsValid(handle)) {
            return;
        }
        shapeManager_.AddShape(handle, std::move(shape));
    }

    void PhysicsWorld::SetIntegrator(std::unique_ptr<IIntegrator> newIntegrator) {
        integrator_ = std::move(newIntegrator);
    }

    void PhysicsWorld::RebuildOctree() {
        spatialTree_.Clear();

        // An Octree of Handles.
        for (auto& handle : bodyManager_.Handles()) {
            BodyData* body = bodyManager_.GetBody(handle);
            spatialTree_.Insert(handle, body->position, body->mass, body->charge);
        }
        spatialTree_.UpdateProperties();
    }

    void PhysicsWorld::Step(double delta) {

        // ============ Detect Collisions ============
        // Broad Phase
        std::vector<CollisionCandidatePair> candidates = broadPhase_.FindCandidatePairs(bodyManager_, shapeManager_);

        // Narrow Phase (GJK/EPA via IShape::Support)
        std::vector<Contact> contacts = narrowPhase_.GenerateContacts(candidates, bodyManager_, shapeManager_);

        // ============ Resolve Collisions ============
        // Solver
        solver_.ResolveContacts(contacts, bodyManager_);

        // =========== Apply Universal Forces ===========
        RebuildOctree();
        // Apply Universal Forces Via Barnes Hut

        // Build barnes hut evaluation.
        barnesHut_.Evaluate(spatialTree_, bodyManager_.Handles(), bodyManager_.Data(), forceManager_);

        // =========== Apply Individual Forces ==========
        // This is done via the public ForceManager

        // ============ Resolve Trajectories ============
        // Integrator
        integrator_->Step(bodyManager_, forceManager_, delta);

        // Sync State
        SyncState();
    }

    BodyData* PhysicsWorld::GetBody(Handle handle) {
        if (!registry_.IsValid(handle)) {
            return nullptr;
        }
        return bodyManager_.GetBody(handle);
    }

    const BodyData* PhysicsWorld::GetBody(Handle handle) const {
        if (!registry_.IsValid(handle)) {
            return nullptr;
        }
        return bodyManager_.GetBody(handle);
    }

    void PhysicsWorld::SyncState() {
        if (destructionQueue_.size() > 0) {
            for (Handle handle : destructionQueue_) {
                DestroyBody(handle);
            }
        }
    }

} // namespace osseus