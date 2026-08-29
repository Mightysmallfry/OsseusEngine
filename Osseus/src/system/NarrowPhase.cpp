#include "Osseus/system/NarrowPhase.h"

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
                continue;
            }

            GJKSimplex simplex;
            if (!GJK::Intersect(*shapeA, bodyA->position, *shapeB, bodyB->position, simplex)) {
                continue;
            }

            contacts.push_back(
                EPA::Resolve(*shapeA, bodyA->position, candidate.a, *shapeB, bodyB->position, candidate.b, simplex));
        }
    }
} // namespace osseus
