#ifndef OSSEUSENGINE_OCTREE_H
#define OSSEUSENGINE_OCTREE_H

#include <array>
#include <memory>
#include <vector>

#include "Osseus/system/Handle.h"
#include "Osseus/math/Vector3.h"

namespace osseus {

    struct Bounds {
        Vector3 center = Vector3::Zero();
        Vector3 halfSize = Vector3::One();
    };

    class OctNode {
    public:
        explicit OctNode(const Bounds& bounds, OctNode* parent = nullptr, int depth = 0);

        void Insert(Handle handle, const Vector3& position, double mass);
        bool Remove(Handle handle);
        
        bool IsLeaf() const;
        bool IsEmpty() const;
        bool ContainsBody(Handle handle, const Vector3& position) const;
        
        std::size_t GetBodyCount() const;
        double GetTotalMass() const;
        const Vector3& GetCenterOfMass() const;
        const Bounds& GetBounds() const;

        const OctNode* GetChild(std::size_t index) const;
        bool HasChild(std::size_t index) const;

        const OctNode* GetParent() const;
        int GetDepth() const;

    private:
        struct Entry {
            Handle handle;
            Vector3 position;
            double mass;
        };

        static constexpr std::size_t MaxBodiesPerNode = 1;
        static constexpr int MaxDepth = 32;

        std::size_t GetOctantIndex(const Vector3& position) const;
        Bounds ComputeChildBounds(std::size_t octant) const;

        void Subdivide();
        void UpdateMassProperties();
        bool RemoveFromEntries(Handle handle);

        Bounds bounds;
        OctNode* parent;
        int depth;

        std::array<std::unique_ptr<OctNode>, 8> children;
        std::vector<Entry> entries;

        std::size_t bodyCount;
        double totalMass;
        Vector3 centerOfMass;
    };

    class Octree {
    public:
        Octree();
        explicit Octree(const Bounds& rootBounds);

        void Clear();
        void Insert(Handle handle, const Vector3& position, double mass);
        void Remove(Handle handle);

        const OctNode& GetRoot() const;

    private:
        Bounds rootBounds;
        std::unique_ptr<OctNode> root;
    };

} // osseus

#endif //OSSEUSENGINE_OCTREE_H