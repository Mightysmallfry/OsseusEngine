//
// Created by MightySmallFry on 7/20/2026.
//

#include "../../../include/Osseus/math/Integrators/IntegratorEulerCromer.h"

namespace osseus {
    void IntegratorEulerCromer::Step(BodyManager& bodyManager, ForceManager& forceManager, double delta) {
        for (Handle handle : bodyManager.Handles()) {
            BodyData* body = bodyManager.GetBody(handle);

            if (body->invMass == 0.0) {
                continue;
            }

            Vector3 acceleration = forceManager.Get(handle) / body->mass;
            body->velocity += acceleration * delta;
            body->position += body->velocity * delta;
        }
    }
} // namespace osseus