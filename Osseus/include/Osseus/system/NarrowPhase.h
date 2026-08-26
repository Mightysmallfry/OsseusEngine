#ifndef OSSEUSENGINE_NARROWPHASE_H
#define OSSEUSENGINE_NARROWPHASE_H
#include <vector>

#include "BodyManager.h"
#include "CollisionCandidatePair.h"
#include "Osseus/math/Collision/Contact.h"
#include "ShapeManager.h"

namespace osseus {
    // Turns broad-phase candidate pairs into real Contacts using
    // GJK/EPA. Because every shape only needs to implement
    // IShape::Support, adding a new convex shape never requires
    // touching this class.
    class NarrowPhase {
        public:
        std::vector<Contact> GenerateContacts(const std::vector<CollisionCandidatePair> &candidates,
                                              BodyManager &bodyManager, ShapeManager &shapeManager) const;
    };
} // namespace osseus

#endif // OSSEUSENGINE_NARROWPHASE_H
