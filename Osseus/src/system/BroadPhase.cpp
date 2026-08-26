//
// Created by MightySmallFry on 7/21/2026.
//

#include "../../include/Osseus/system/BroadPhase.h"

namespace osseus {
    bool BroadPhase::AABBOverlaps(const AABB& ours, const AABB& other) const {
        return ours.min.x <= other.max.x && ours.max.x >= other.min.x && ours.min.y <= other.max.y &&
               ours.max.y >= other.min.y && ours.min.z <= other.max.z && ours.max.z >= other.min.z;
    }

    std::vector<CollisionCandidatePair> BroadPhase::FindCandidatePairs(BodyManager& bodyManager,
                                                                       ShapeManager& shapeManager) {
        std::vector<CollisionCandidatePair> candidatePairs;

        const auto& bodyData = bodyManager.Data();
        const auto& bodyHandles = bodyManager.Handles();

        for (size_t i = 0; i < bodyData.size(); i++) {
            IShape* shapeA = shapeManager.GetShape(bodyHandles[i]);
            if (!shapeA) {
                continue;
            }

            AABB shapeA_aabb = shapeA->ComputeBoundingBox(bodyData[i].position);

            for (size_t j = i + 1; j < bodyData.size(); j++) {
                IShape* shapeB = shapeManager.GetShape(bodyHandles[j]);
                if (!shapeB) {
                    continue;
                }

                AABB shapeB_aabb = shapeB->ComputeBoundingBox(bodyData[j].position);

                if (AABBOverlaps(shapeA_aabb, shapeB_aabb)) {
                    candidatePairs.push_back(CollisionCandidatePair{bodyHandles[i], bodyHandles[j]});
                }
            }
        }
        return candidatePairs;
    }
} // namespace osseus