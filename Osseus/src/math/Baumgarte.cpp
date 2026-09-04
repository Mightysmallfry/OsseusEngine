#include "Osseus/math/Baumgarte.h"

#include <algorithm>
#include <iostream>

namespace osseus {
    void Baumgarte::ResolveContacts(const std::vector<Contact>& contacts, BodyManager& bodyManager) const {
        for (const Contact& contact : contacts) {
            BodyData* bodyA = bodyManager.GetBody(contact.a);
            BodyData* bodyB = bodyManager.GetBody(contact.b);
            if (!bodyA || !bodyB) {
                std::cerr << "Invalid contact found\n";
                continue;
            }

            double invMassSum = bodyA->invMass + bodyB->invMass;
            if (invMassSum <= 0.0) {
                continue;
            } // both infinite mass, nothing to resolve

            // Check for degenerate faces (near coplanar face from EPA)
            if (contact.normal.LengthSquared() < Vector3::TOLERANCE) {
                std::cerr << "Discarding degenerate contact (zero-length normal)\n";
                continue;
            }

            Vector3 relativeVelocity = bodyB->velocity - bodyA->velocity;
            double velocityAlongNormal = relativeVelocity.Dot(contact.normal);

            if (velocityAlongNormal <= 0.0) {
                double impulseMagnitude = -(1.0 + restitution_) * velocityAlongNormal / invMassSum;
                Vector3 impulse = contact.normal * impulseMagnitude;

                bodyA->velocity -= impulse * bodyA->invMass;
                bodyB->velocity += impulse * bodyB->invMass;
            }

            // Positional correction: push the bodies apart directly rather
            // than relying on velocity alone, so residual penetration from
            // a discrete step doesn't accumulate frame over frame.
            double correctionMagnitude =
                std::max(contact.penetration - positionalSlop_, 0.0) / invMassSum * positionalCorrectionPercent_;
            Vector3 correction = contact.normal * correctionMagnitude;

            bodyA->position -= correction * bodyA->invMass;
            bodyB->position += correction * bodyB->invMass;
        }
    }
} // namespace osseus
