#include "Osseus/physics/Octree.h"

namespace osseus {

    // ==================== OctNode ====================

    OctNode::OctNode(const Bounds& bounds, OctNode* parent, int depth)
        : bounds_(bounds), parent_(parent), depth_(depth) {}


    void OctNode::Insert(Handle handle, const Vector3& position, double mass, double charge) {
        if (mass <= 0.0) { return; }

        if (IsLeaf()) {
            if (entries_.size() < MaxBodiesPerNode || depth_ >= MaxDepth) {
                entries_.push_back(Entry{ handle, position, mass, charge });
                UpdateMassProperties();
                UpdateChargeProperties();
                return;
            }
        Subdivide();
    }

    const std::size_t octant = GetOctantIndex(position);
        if (!children_[octant]) {
            children_[octant] = std::make_unique<OctNode>(ComputeChildBounds(octant), this, depth_ + 1);
        }
        children_[octant]->Insert(handle, position, mass, charge);

        UpdateMassProperties();
        UpdateChargeProperties();
    }

    bool OctNode::Remove(Handle handle) {
        if (IsLeaf()) {
            const bool removed = RemoveFromEntries(handle);
            if (removed) { UpdateMassProperties(); UpdateChargeProperties(); }
            return removed;
        }

        for (auto& child : children_) {
            if (!child) { continue; }

            if (child->Remove(handle)) {
                if (child->IsEmpty() && child->IsLeaf()) {
                    child.reset();
                }
                UpdateMassProperties();
                UpdateChargeProperties();
                return true;
            }
        }
        return false;
    }

    bool OctNode::IsLeaf() const {
        for (const auto& child : children_) {
            if (child) { return false; }
        }
        return true;
    }

    bool OctNode::IsEmpty() const {
        return bodyCount_ == 0;
    }

    bool OctNode::ContainsBody(Handle handle, const Vector3& position) const {
    if (IsEmpty()) { return false; }
    if (IsLeaf()) {
        for (const Entry& entry : entries_) {
            if (entry.handle == handle) { return true; }
        }
        return false;
    }
    const std::size_t octant = GetOctantIndex(position);
    return HasChild(octant) && children_[octant]->ContainsBody(handle, position);
}

    std::size_t OctNode::GetBodyCount() const {
        return bodyCount_;
    }

    double OctNode::GetTotalMass() const {
        return totalMass_;
    }

    const Vector3& OctNode::GetCenterOfMass() const {
        return centerOfMass_;
    }

    double OctNode::GetTotalCharge() const {
        return totalCharge_;
    }

    const Vector3& OctNode::GetCenterOfCharge() const {
        return centerOfCharge_;
    }

    const Vector3& OctNode::GetDipoleMoment() const {
        return dipoleMoment_;
    }

    const Bounds& OctNode::GetBounds() const {
        return bounds_;
    }

    const OctNode* OctNode::GetChild(std::size_t index) const {
        return children_.at(index).get();
    }

    bool OctNode::HasChild(std::size_t index) const {
        return children_.at(index) != nullptr;
    }

    std::size_t OctNode::GetOctantIndex(const Vector3& position) const {
        std::size_t index = 0;
        if (position.x >= bounds_.center.x) { index |= 1; }
        if (position.y >= bounds_.center.y) { index |= 2; }
        if (position.z >= bounds_.center.z) { index |= 4; }
        return index;
    }

    Bounds OctNode::ComputeChildBounds(std::size_t octant) const {
        const Vector3 quarter = bounds_.halfSize * 0.5;
        const Vector3 offset(
            (octant & 1) ? quarter.x : -quarter.x,
            (octant & 2) ? quarter.y : -quarter.y,
            (octant & 4) ? quarter.z : -quarter.z
        );
        return Bounds{ bounds_.center + offset, quarter };
    }

    void OctNode::Subdivide() {
        std::vector<Entry> existingEntries = std::move(entries_);
        entries_.clear();

        for (const Entry& entry : existingEntries) {
            const std::size_t octant = GetOctantIndex(entry.position);
            if (!children_[octant]) {
                children_[octant] = std::make_unique<OctNode>(ComputeChildBounds(octant), this, depth_ + 1);
            }
            children_[octant]->Insert(entry.handle, entry.position, entry.mass, entry.charge);
        }
    }

    const OctNode* OctNode::GetParent() const {
        return parent_;
    }

    int OctNode::GetDepth() const {
        return depth_;
    }

    void OctNode::UpdateMassProperties() {
        totalMass_ = 0.0;
        centerOfMass_ = Vector3::Zero();
        bodyCount_ = 0;

        if (IsLeaf()) {
            for (const Entry& entry : entries_) {
                totalMass_ += entry.mass;
                centerOfMass_ += entry.position * entry.mass;
            }
            bodyCount_ = entries_.size();
        } else {
            for (const auto& child : children_) {
                if (!child) { continue; }
                totalMass_ += child->totalMass_;
                centerOfMass_ += child->centerOfMass_ * child->totalMass_;
                bodyCount_ += child->bodyCount_;
            }
        }

        if (totalMass_ > 0.0) {
            centerOfMass_ /= totalMass_;
        }
    }

    void OctNode::UpdateChargeProperties() {
        totalCharge_ = 0.0;
        centerOfCharge_ = Vector3::Zero();
        dipoleMoment_ = Vector3::Zero();

        if (IsLeaf()) {
            // Pass 1: total charge + charge-weighted center
            Vector3 geometricCenter = Vector3::Zero();
            for (const Entry& entry : entries_) {
                totalCharge_ += entry.charge;
                centerOfCharge_ += entry.position * entry.charge;
                geometricCenter += entry.position;
            }
            if (!entries_.empty()) {
                geometricCenter /= static_cast<double>(entries_.size());
            }

            if (std::abs(totalCharge_) > kChargeEpsilon) {
                centerOfCharge_ /= totalCharge_;
            } else {
                centerOfCharge_ = geometricCenter;
            }

            // Pass 2: dipole moment about the now-finalized centerOfCharge_
            for (const Entry& entry : entries_) {
                dipoleMoment_ += (entry.position - centerOfCharge_) * entry.charge;
            }
       } else {
            // Pass 1: total charge + charge-weighted center from children
            Vector3 geometricCenter = Vector3::Zero();
            int childCount = 0;
            for (const auto& child : children_) {
                if (!child) { continue; }
                totalCharge_ += child->totalCharge_;
                centerOfCharge_ += child->centerOfCharge_ * child->totalCharge_;
                geometricCenter += child->centerOfCharge_;
                ++childCount;
            }
            if (childCount > 0) {
                geometricCenter /= static_cast<double>(childCount);
            }

            if (std::abs(totalCharge_) > kChargeEpsilon) {
                centerOfCharge_ /= totalCharge_;
            } else {
                centerOfCharge_ = geometricCenter;
            }

            // Pass 2: shift each child's dipole moment to the parent's center
            // (parallel-axis correction) and accumulate.
            for (const auto& child : children_) {
                if (!child) { continue; }
                dipoleMoment_ += child->dipoleMoment_
                               + child->totalCharge_ * (child->centerOfCharge_ - centerOfCharge_);
            }
        }

    }

    bool OctNode::RemoveFromEntries(Handle handle) {
    for (auto it = entries_.begin(); it != entries_.end(); ++it) {
        if (it->handle == handle) {
            entries_.erase(it);
            return true;
        }
    }
    return false;
}

    // ==================== Octree ====================

    Octree::Octree()
    : rootBounds_(), root_(std::make_unique<OctNode>(rootBounds_)) {}

    Octree::Octree(const Bounds& rootBounds)
    : rootBounds_(rootBounds), root_(std::make_unique<OctNode>(rootBounds)) {}

    void Octree::Clear() {
        root_ = std::make_unique<OctNode>(rootBounds_);
    }

    void Octree::Insert(Handle handle, const Vector3& position, double mass, double charge) {
        root_->Insert(handle, position, mass, charge);
    }

    void Octree::Remove(Handle handle) {
        root_->Remove(handle);
    }

    const OctNode& Octree::GetRoot() const {
        return *root_;
    }

} // osseus