//
// Created by MightySmallFry on 7/16/2026.
//

#include "Osseus/physics/PhysicsWorld.h"

namespace osseus {
    Handle PhysicsWorld::CreateHandle() {
        return registry.CreateHandle();
    }

    Handle PhysicsWorld::CreateBody(BodyData bodyData, std::unique_ptr<IShape> shape) {
        Handle handle = CreateHandle();
        AttachBody(handle, bodyData);
        AttachShape(handle, std::move(shape));
        return handle;
    }

    void PhysicsWorld::DestroyBody(Handle handle) {
        if (!registry.IsValid(handle)) { return; }
        bodyManager.RemoveBody(handle);
        shapeManager.RemoveShape(handle);
        registry.Destroy(handle);
    }

    void PhysicsWorld::AddForce(std::unique_ptr<IForceEvaluator> force) {
        forces.AddForce(std::move(force));
    }

    void PhysicsWorld::AttachBody(Handle handle, BodyData bodyData) {
        if (!registry.IsValid(handle)) { return; }
        bodyManager.AddBody(handle, bodyData);
    }

    void PhysicsWorld::AttachShape(Handle handle, std::unique_ptr<IShape> shape) {
        if (!registry.IsValid(handle)) { return; }
        shapeManager.AddShape(handle, std::move(shape));
    }

    void PhysicsWorld::SetIntegrator(std::unique_ptr<IIntegrator> newIntegrator) {
        integrator = std::move(newIntegrator);
    }

    void PhysicsWorld::Step(double delta) {
        // Broad Phase
        std::vector<CollisionCandidatePair> candidates =
            broadPhase.FindCandidatePairs(bodyManager, shapeManager);
        // Narrow Phase

        // Solver

        // Integrator
        integrator->Step(bodyManager, forces, delta);

        // Sync State
    }

    BodyData * PhysicsWorld::GetBody(Handle handle) {
        if (!registry.IsValid(handle)) { return nullptr; }
        return bodyManager.GetBody(handle);
    }

    const BodyData* PhysicsWorld::GetBody(Handle handle) const {
        if (!registry.IsValid(handle)) { return nullptr; }
        return bodyManager.GetBody(handle);
    }
} // osseus