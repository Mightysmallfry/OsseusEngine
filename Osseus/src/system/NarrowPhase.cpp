#include "Osseus/system/NarrowPhase.h"

#include <iostream>

#include "Osseus/math/Collision/EPA.h"
#include "Osseus/math/Collision/GJK.h"

namespace osseus {
    void NarrowPhase::GenerateContacts(const std::vector<CollisionCandidatePair>& candidates,
                                                       BodyManager& bodyManager, ShapeManager& shapeManager,
                                                       std::vector<Contact>& contacts) const {
        contacts.clear();
        contacts.reserve(candidates.size());
                                                   
        for (const CollisionCandidatePair& candidate : candidates) {
            BodyData* bodyA = bodyManager.GetBody(candidate.a);
            BodyData* bodyB = bodyManager.GetBody(candidate.b);
            IShape* shapeA = shapeManager.GetShape(candidate.a);
            IShape* shapeB = shapeManager.GetShape(candidate.b);

            if (!bodyA || !bodyB || !shapeA || !shapeB) {
                std::cerr << "Invalid Collision pairing\n";
                continue;
            }

            GJKSimplex simplex;
            if (!GJK::Intersect(*shapeA, bodyA->position, *shapeB, bodyB->position, simplex)) {
                continue;
            }

            Contact contact = EPA::Resolve(*shapeA, bodyA->position, candidate.a, *shapeB, bodyB->position, candidate.b, simplex);

            // Verify that the contact is non-degenerate
            // Stop the bad contact from entering the manifold
            if (contact.normal.LengthSquared() < Vector3::TOLERANCE){
                std::cerr << "Discarding degenerate EPA result (zero-length normal)\n";
                continue;
            }
            contacts.push_back(contact);
        }
    }
} // namespace osseus
