#include "Osseus/physics/Octree.h"

namespace osseus {

    // ==================== OctNode ====================

    OctNode::OctNode(const Bounds& bounds, OctNode* parent, int depth)
        : bounds(bounds), parent(parent), depth(depth),
        bodyCount(0), totalMass(0.0), centerOfMass(Vector3::Zero()) {}


    void OctNode::Insert(Handle handle, const Vector3& position, double mass) {
        if (mass <= 0.0) { return; }

        if (IsLeaf()) {
            if (entries.size() < MaxBodiesPerNode || depth >= MaxDepth) {
                entries.push_back(Entry{ handle, position, mass });
                UpdateMassProperties();
                return;
            }
        Subdivide();
    }

    const std::size_t octant = GetOctantIndex(position);
        if (!children[octant]) {
            children[octant] = std::make_unique<OctNode>(ComputeChildBounds(octant), this, depth + 1);
        }
        children[octant]->Insert(handle, position, mass);

        UpdateMassProperties();
    }

    bool OctNode::Remove(Handle handle) {
        if (IsLeaf()) {
            const bool removed = RemoveFromEntries(handle);
            if (removed) { UpdateMassProperties(); }
            return removed;
        }

        for (auto& child : children) {
            if (!child) { continue; }

            if (child->Remove(handle)) {
                if (child->IsEmpty() && child->IsLeaf()) {
                    child.reset();
                }
                UpdateMassProperties();
                return true;
            }
        }
        return false;
    }

    bool OctNode::IsLeaf() const {
        for (const auto& child : children) {
            if (child) { return false; }
        }
        return true;
    }

    bool OctNode::IsEmpty() const {
        return bodyCount == 0;
    }

    std::size_t OctNode::GetBodyCount() const {
        return bodyCount;
    }

    double OctNode::GetTotalMass() const {
        return totalMass;
    }

    const Vector3& OctNode::GetCenterOfMass() const {
        return centerOfMass;
    }

    const Bounds& OctNode::GetBounds() const {
        return bounds;
    }

    const OctNode* OctNode::GetChild(std::size_t index) const {
        return children.at(index).get();
    }

    bool OctNode::HasChild(std::size_t index) const {
        return children.at(index) != nullptr;
    }

    std::size_t OctNode::GetOctantIndex(const Vector3& position) const {
        std::size_t index = 0;
        if (position.x >= bounds.center.x) { index |= 1; }
        if (position.y >= bounds.center.y) { index |= 2; }
        if (position.z >= bounds.center.z) { index |= 4; }
        return index;
    }

    Bounds OctNode::ComputeChildBounds(std::size_t octant) const {
        const Vector3 quarter = bounds.halfSize * 0.5;
        const Vector3 offset(
            (octant & 1) ? quarter.x : -quarter.x,
            (octant & 2) ? quarter.y : -quarter.y,
            (octant & 4) ? quarter.z : -quarter.z
        );
        return Bounds{ bounds.center + offset, quarter };
    }

    void OctNode::Subdivide() {
        std::vector<Entry> existingEntries = std::move(entries);
        entries.clear();

        for (const Entry& entry : existingEntries) {
            const std::size_t octant = GetOctantIndex(entry.position);
            if (!children[octant]) {
                children[octant] = std::make_unique<OctNode>(ComputeChildBounds(octant), this, depth + 1);
            }
            children[octant]->Insert(entry.handle, entry.position, entry.mass);
        }
    }

    const OctNode* OctNode::GetParent() const {
        return parent;
    }

    int OctNode::GetDepth() const {
        return depth;
    }

    void OctNode::UpdateMassProperties() {
        totalMass = 0.0;
        centerOfMass = Vector3::Zero();
        bodyCount = 0;

        if (IsLeaf()) {
            for (const Entry& entry : entries) {
                totalMass += entry.mass;
                centerOfMass += entry.position * entry.mass;
            }
            bodyCount = entries.size();
        } else {
            for (const auto& child : children) {
                if (!child) { continue; }
                totalMass += child->totalMass;
                centerOfMass += child->centerOfMass * child->totalMass;
                bodyCount += child->bodyCount;
            }
        }

        if (totalMass > 0.0) {
            centerOfMass /= totalMass;
        }
    }

    bool OctNode::RemoveFromEntries(Handle handle) {
    for (auto it = entries.begin(); it != entries.end(); ++it) {
        if (it->handle == handle) {
            entries.erase(it);
            return true;
        }
    }
    return false;
}

    // ==================== Octree ====================

    Octree::Octree()
    : rootBounds(), root(std::make_unique<OctNode>(rootBounds)) {}

    Octree::Octree(const Bounds& rootBounds)
    : rootBounds(rootBounds), root(std::make_unique<OctNode>(rootBounds)) {}

    void Octree::Clear() {
        root = std::make_unique<OctNode>(rootBounds);
    }

    void Octree::Insert(Handle handle, const Vector3& position, double mass) {
        root->Insert(handle, position, mass);
    }

    void Octree::Remove(Handle handle) {
        root->Remove(handle);
    }

    const OctNode& Octree::GetRoot() const {
        return *root;
    }

} // osseus