#include "Osseus/system/NarrowPhase.h"

#include "Osseus/math/Collision/GJK.h"
#include "Osseus/math/Collision/EPA.h"

namespace osseus {
    std::vector<Contact> NarrowPhase::GenerateContacts(const std::vector<CollisionCandidatePair>& candidates,
                                                         BodyManager& bodyManager, ShapeManager& shapeManager) const {
        std::vector<Contact> contacts;
        contacts.reserve(candidates.size());

        for (const CollisionCandidatePair& candidate : candidates) {
            BodyData* bodyA = bodyManager.GetBody(candidate.a);
            BodyData* bodyB = bodyManager.GetBody(candidate.b);
            IShape* shapeA = shapeManager.GetShape(candidate.a);
            IShape* shapeB = shapeManager.GetShape(candidate.b);

            if (!bodyA || !bodyB || !shapeA || !shapeB) { continue; }

            GJKSimplex simplex;
            if (!GJK::Intersect(*shapeA, bodyA->position, *shapeB, bodyB->position, simplex)) {
                continue;
            }

            contacts.push_back(EPA::Resolve(*shapeA, bodyA->position, candidate.a,
                                             *shapeB, bodyB->position, candidate.b, simplex));
        }
        return contacts;
    }
} // osseus
