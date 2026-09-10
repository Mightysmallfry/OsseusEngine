#include "Osseus/system/BroadPhase.h"

#include <algorithm>

namespace osseus {
    bool BroadPhase::AABBOverlaps(const AABB& ours, const AABB& other) const {
        return ours.min.x <= other.max.x && ours.max.x >= other.min.x &&
               ours.min.y <= other.max.y && ours.max.y >= other.min.y &&
               ours.min.z <= other.max.z && ours.max.z >= other.min.z;
    }

    std::vector<CollisionCandidatePair> BroadPhase::FindCandidatePairs(BodyManager& bodyManager,
                                                                       ShapeManager& shapeManager) {
        const auto& bodyData = bodyManager.Data();
        const auto& bodyHandles = bodyManager.Handles();

        struct AABBEntry {
            size_t index;
            AABB bounds;
        };

        std::vector<AABBEntry> entries;
        entries.reserve(bodyData.size());

        for (size_t i = 0; i < bodyData.size(); ++i) {
            IShape* shape = shapeManager.GetShape(bodyHandles[i]);
            if (shape == nullptr) {
                continue;
            }

            entries.push_back({
                i,
                shape->ComputeBoundingBox(bodyData[i].position)
            });
        }

        std::sort(entries.begin(), entries.end(),
                  [](const AABBEntry& a, const AABBEntry& b) {
                      return a.bounds.min.x < b.bounds.min.x;
                  });

        std::vector<CollisionCandidatePair> candidatePairs;
        candidatePairs.reserve(entries.size());

        for (size_t i = 0; i < entries.size(); ++i) {
            const AABBEntry& current = entries[i];

            for (size_t j = i + 1; j < entries.size(); ++j) {
                const AABBEntry& other = entries[j];

                // Since entries are sorted by min.x, nothing after this
                // point can overlap current on the X axis.
                if (other.bounds.min.x > current.bounds.max.x) {
                    break;
                }

                // X overlaps; test the remaining axes.
                if (current.bounds.min.y > other.bounds.max.y ||
                    current.bounds.max.y < other.bounds.min.y) {
                    continue;
                }

                if (current.bounds.min.z > other.bounds.max.z ||
                    current.bounds.max.z < other.bounds.min.z) {
                    continue;
                }

                candidatePairs.push_back({
                    bodyHandles[current.index],
                    bodyHandles[other.index]
                });
            }
        }

        return candidatePairs;
    }
} // namespace osseus