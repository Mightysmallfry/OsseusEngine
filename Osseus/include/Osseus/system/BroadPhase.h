//
// Created by MightySmallFry on 7/21/2026.
//

#ifndef OSSEUSENGINE_BROADPHASE_H
#define OSSEUSENGINE_BROADPHASE_H
#include <vector>

#include "AABB.h"
#include "BodyManager.h"
#include "CollisionCandidatePair.h"
#include "ShapeManager.h"
#include "Osseus/interfaces/IShape.h"

namespace osseus {
    class BroadPhase {
    public:
        std::vector<CollisionCandidatePair> FindCandidatePairs(BodyManager& bodyManager, ShapeManager& shapeManager);

    private:
        bool AABBOverlaps(const AABB& ours, const AABB& other) const;
    };
} // osseus

#endif //OSSEUSENGINE_BROADPHASE_H
