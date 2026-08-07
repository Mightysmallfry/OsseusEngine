#include "Osseus/math/Solver.h"

#include <algorithm>

namespace osseus {
    void Solver::ResolveContacts(const std::vector<Contact>& contacts, BodyManager& bodyManager) const {
        for (const Contact& contact : contacts) {
            BodyData* bodyA = bodyManager.GetBody(contact.a);
            BodyData* bodyB = bodyManager.GetBody(contact.b);
            if (!bodyA || !bodyB) { continue; }

            double invMassSum = bodyA->invMass + bodyB->invMass;
            if (invMassSum <= 0.0) { continue; } // both infinite mass, nothing to resolve

            Vector3 relativeVelocity = bodyB->velocity - bodyA->velocity;
            double velocityAlongNormal = relativeVelocity.Dot(contact.normal);

            if (velocityAlongNormal <= 0.0) {
                double impulseMagnitude = -(1.0 + restitution) * velocityAlongNormal / invMassSum;
                Vector3 impulse = contact.normal * impulseMagnitude;

                bodyA->velocity -= impulse * bodyA->invMass;
                bodyB->velocity += impulse * bodyB->invMass;
            }

            // Positional correction: push the bodies apart directly rather
            // than relying on velocity alone, so residual penetration from
            // a discrete step doesn't accumulate frame over frame.
            double correctionMagnitude = std::max(contact.penetration - positionalSlop, 0.0)
                                          / invMassSum * positionalCorrectionPercent;
            Vector3 correction = contact.normal * correctionMagnitude;

            bodyA->position -= correction * bodyA->invMass;
            bodyB->position += correction * bodyB->invMass;
        }
    }
} // osseus
