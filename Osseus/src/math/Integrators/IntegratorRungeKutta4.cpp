//
// Created by MightySmallFry on 7/20/2026.
//

#include "Osseus/math/Integrators/IntegratorRungeKutta4.h"

namespace osseus {
    void IntegratorRungeKutta4::Step(BodyManager &manager, const IForceEvaluator &forces, double delta) {
        for (BodyData& body : manager.Data()) {
            if (body.invMass == 0.0) { continue; }
            Vector3 x = body.position;
            Vector3 v = body.velocity;

            BodyData state1{ x, v, body.invMass };
            Vector3 k1v = forces.CalculateAcceleration(state1);
            Vector3 k1x = v;

            BodyData state2{ x + k1x * (delta/2), v + k1v * (delta/2), body.invMass };
            Vector3 k2v = forces.CalculateAcceleration(state2);
            Vector3 k2x = v + k1v * (delta/2);

            BodyData state3{ x + k2x * (delta/2), v + k2v * (delta/2), body.invMass };
            Vector3 k3v = forces.CalculateAcceleration(state3);
            Vector3 k3x = v + k2v * (delta/2);

            BodyData state4{ x + k3x * delta, v + k3v * delta, body.invMass };
            Vector3 k4v = forces.CalculateAcceleration(state4);
            Vector3 k4x = v + k3v * delta;

            body.velocity = v + (k1v + k2v*2.0 + k3v*2.0 + k4v) * (delta/6.0);
            body.position = x + (k1x + k2x*2.0 + k3x*2.0 + k4x) * (delta/6.0);
        }
    }
} // osseus