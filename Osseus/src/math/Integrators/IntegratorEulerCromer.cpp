//
// Created by MightySmallFry on 7/20/2026.
//

#include "../../../include/Osseus/math/Integrators/IntegratorEulerCromer.h"

namespace osseus {
    void IntegratorEulerCromer::step(BodyManager &manager, const IForceEvaluator &forces, double delta) {
        for (BodyData& body : manager.bodies.Data()) {
            if (body.invMass == 0.0) { continue; }

            Vector3 acceleration = forces.CalculateAcceleration(body);
            body.velocity += acceleration * delta;
            body.position += body.velocity * delta;
        }
    }
} // osseus