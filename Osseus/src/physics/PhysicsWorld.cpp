//
// Created by MightySmallFry on 7/16/2026.
//

#include "../../include/Osseus/physics/PhysicsWorld.h"
#include "Osseus/physics/Forces/ForceGravity.h"

namespace osseus {
    Handle PhysicsWorld::CreateBody(BodyData bodyData) {
        Handle handle = registry.CreateHandle();
        bodyManager.bodies.Insert(handle, bodyData);
        return handle;
    }

    void PhysicsWorld::DestroyBody(Handle handle) {
        if (!registry.IsValid(handle)) { return; }
        bodyManager.bodies.Remove(handle);
        registry.Destroy(handle);
    }

    void PhysicsWorld::AddForce(std::unique_ptr<IForceEvaluator> force) {
        forces.AddForce(std::move(force));
    }

    void PhysicsWorld::SetIntegrator(std::unique_ptr<IIntegrator> newIntegrator) {
        integrator = std::move(newIntegrator);
    }

    void PhysicsWorld::Step(double delta) {
        // Broad Phase

        // Narrow Phase

        // Solver

        // Integrator
        integrator->step(bodyManager, forces, delta);

        // Sync State
    }

    BodyData * PhysicsWorld::GetBody(Handle handle) {
        if (!registry.IsValid(handle)) { return nullptr; }
        return bodyManager.bodies.Get(handle);
    }

    const BodyData* PhysicsWorld::GetBody(Handle handle) const {
        if (!registry.IsValid(handle)) { return nullptr; }
        return bodyManager.bodies.Get(handle);
    }
} // osseus