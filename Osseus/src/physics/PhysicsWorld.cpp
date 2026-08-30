#include <algorithm>
#include <cstddef>
#include <limits>

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
        objectCount_++;
        return handle;
    }

    Handle PhysicsWorld::CreateBody(BodyData bodyData) {
        Handle handle = CreateHandle();
        AttachBody(handle, bodyData);
        forceManager_.Register(handle);
        shapeManager_.Register(handle);
        objectCount_++;
        return handle;
    }

    Handle PhysicsWorld::CreateBody(BodyData bodyData, std::unique_ptr<IShape> shape) {
        Handle handle = CreateHandle();
        AttachBody(handle, bodyData);
        AttachShape(handle, std::move(shape));
        forceManager_.Register(handle);
        objectCount_++;
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
        objectCount_--;
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

    Bounds PhysicsWorld::ComputeWorldBounds() const {
        const std::vector<BodyData>& bodies = bodyManager_.Data();

        if (bodies.empty()) {
            return Bounds{};
        }

        Vector3 minPoint(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max());
        Vector3 maxPoint(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                         std::numeric_limits<double>::lowest());

        for (const BodyData& body : bodies) {
            minPoint.x = std::min(minPoint.x, body.position.x);
            minPoint.y = std::min(minPoint.y, body.position.y);
            minPoint.z = std::min(minPoint.z, body.position.z);
            maxPoint.x = std::max(maxPoint.x, body.position.x);
            maxPoint.y = std::max(maxPoint.y, body.position.y);
            maxPoint.z = std::max(maxPoint.z, body.position.z);
        }

        const Vector3 center = (minPoint + maxPoint) * 0.5;
        Vector3 halfSize = (maxPoint - minPoint) * 0.5;

        // Pad so bodies sit strictly inside the bounds rather than exactly
        // on the boundary, and floor the size so a single body (or a
        // cluster of coincident/near-coincident bodies) doesn't collapse
        // the root to a near-zero-volume box.
        constexpr double kPaddingFactor = 1.1;
        constexpr double kMinHalfSize = 1.0;
        halfSize = Vector3(std::max(halfSize.x * kPaddingFactor, kMinHalfSize),
                           std::max(halfSize.y * kPaddingFactor, kMinHalfSize),
                           std::max(halfSize.z * kPaddingFactor, kMinHalfSize));

        return Bounds{center, halfSize};
    }

    void PhysicsWorld::RebuildOctree() {
        spatialTree_.SetRootBounds(ComputeWorldBounds());

        // An Octree of Handles.
        for (auto& handle : bodyManager_.Handles()) {
            BodyData* body = bodyManager_.GetBody(handle);
            spatialTree_.Insert(handle, body->position, body->mass, body->charge);
        }
        spatialTree_.UpdateProperties();
    }

    void PhysicsWorld::Step(double delta) {
        elapsedTime_ += delta;
        // ============ Detect Collisions ============
        // Broad Phase
        std::vector<CollisionCandidatePair> candidates = broadPhase_.FindCandidatePairs(bodyManager_, shapeManager_);

        // Narrow Phase (GJK/EPA via IShape::Support)
        narrowPhase_.GenerateContacts(candidates, bodyManager_, shapeManager_, collisionManifold_);

        // ============ Resolve Collisions ============
        // BaumGarte
        baumGarte_.ResolveContacts(collisionManifold_, bodyManager_);

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

        forceManager_.Clear();
    }

} // namespace osseus