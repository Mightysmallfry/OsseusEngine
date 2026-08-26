#ifndef OSSEUSENGINE_OCTREE_H
#define OSSEUSENGINE_OCTREE_H

#include <array>
#include <memory>
#include <vector>

#include "Osseus/math/Vector3.h"
#include "Osseus/system/Handle.h"

namespace osseus {

    struct Bounds {
        Vector3 center = Vector3::Zero();
        Vector3 halfSize = Vector3::One();

        bool Contains(const Vector3& point) const {
            return std::abs(point.x - center.x) <= halfSize.x && std::abs(point.y - center.y) <= halfSize.y &&
                   std::abs(point.z - center.z) <= halfSize.z;
        }
    };

    struct Entry {
        Handle handle;
        Vector3 position;
        double mass;
        double charge;
    };

    class OctNode {
        public:
        static constexpr double kChargeEpsilon = 1e-9;

        explicit OctNode(const Bounds& bounds, OctNode* parent = nullptr, int depth = 0);

        void Insert(Handle handle, const Vector3& position, double mass, double charge);
        bool Remove(Handle handle);

        bool IsLeaf() const;
        bool IsEmpty() const;
        bool ContainsBody(Handle handle, const Vector3& position) const;

        std::size_t GetBodyCount() const;

        double GetTotalMass() const;
        const Vector3& GetCenterOfMass() const;
        double GetTotalCharge() const;
        const Vector3& GetCenterOfCharge() const;
        const Vector3& GetDipoleMoment() const;

        const Bounds& GetBounds() const;

        const OctNode* GetChild(std::size_t index) const;
        bool HasChild(std::size_t index) const;

        const OctNode* GetParent() const;
        int GetDepth() const;

        void UpdateProperties();

        private:
        static constexpr std::size_t MaxBodiesPerNode = 1;
        static constexpr int MaxDepth = 32;

        std::size_t GetOctantIndex(const Vector3& position) const;
        Bounds ComputeChildBounds(std::size_t octant) const;

        void Subdivide();
        void UpdateMassProperties();

        // TODO: Implement Charge Property Tracking
        void UpdateChargeProperties();

        bool RemoveFromEntries(Handle handle);

        Bounds bounds_;
        OctNode* parent_;
        int depth_;

        std::array<std::unique_ptr<OctNode>, 8> children_;
        std::vector<Entry> entries_;

        std::size_t bodyCount_{0};
        double totalMass_{0.0};
        Vector3 centerOfMass_{Vector3::Zero()};

        double totalCharge_{0.0};
        Vector3 centerOfCharge_{Vector3::Zero()};
        Vector3 dipoleMoment_{Vector3::Zero()};
    };

    class Octree {
        public:
        Octree();
        explicit Octree(const Bounds& rootBounds);

        void Clear();
        void Insert(Entry entry);
        void Insert(Handle handle, const Vector3& position, double mass, double charge);
        void Remove(Handle handle);

        void UpdateProperties();
        const OctNode& GetRoot() const;

        private:
        Bounds rootBounds_;
        std::unique_ptr<OctNode> root_;
    };

} // namespace osseus

#endif // OSSEUSENGINE_OCTREE_H