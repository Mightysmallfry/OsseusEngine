#include <iostream>

#include "Osseus/physics/Octree.h"

namespace osseus {

    // ==================== OctNode ====================

    OctNode::OctNode(const Bounds& bounds, OctNode* parent, int depth)
        : bounds_(bounds), parent_(parent), depth_(depth), deepestDepth_(depth) {
    }

    void OctNode::Insert(Handle handle, const Vector3& position, double mass, double charge) {
        if (mass <= 0.0) {
            std::cerr << "Octree insertion attempted with mass < 0.0\n";
            return;
        }

        const Entry entry{handle, position, mass, charge};
        AccumulateEntry(entry);

        if (IsLeaf()) {
            if (entries_.size() < MaxBodiesPerNode || depth_ >= MaxDepth) {
                entries_.push_back(entry);
                return;
            }
            Subdivide();
        }

        const std::size_t octant = GetOctantIndex(position);
        if (!children_[octant]) {
            children_[octant] = std::make_unique<OctNode>(ComputeChildBounds(octant), 
                this, 
                depth_ + 1);
            IncrementSubtreeCount();
        }
        children_[octant]->Insert(handle, position, mass, charge);
        deepestDepth_ = std::max(deepestDepth_, children_[octant]->GetDeepestDepth());
    }

    bool OctNode::Remove(Handle handle) {
        Entry removed{};
        return RemoveInternal(handle, removed);
    }

    bool OctNode::RemoveInternal(Handle handle, Entry& outRemoved) {
        if (IsLeaf()) {
            if (!RemoveFromEntries(handle, outRemoved)) {
                return false;
            }
            RemoveEntryProperties(outRemoved);
            return true;
        }

        for (auto& child : children_) {
            if (!child) {
                continue;
            }

            if (child->RemoveInternal(handle, outRemoved)) {
                RemoveEntryProperties(outRemoved);
                if (child->IsEmpty() && child->IsLeaf()) {
                    child.reset();
                    DecrementSubtreeCount();
                }
                
                deepestDepth_ = depth_;
                for (const auto& remainingChild : children_) {
                    if (remainingChild) {
                        deepestDepth_ = std::max(
                            deepestDepth_,
                            remainingChild->GetDeepestDepth()
                        );
                    }
                }
                return true;
            }
        }
        return false;
    }

    void OctNode::IncrementSubtreeCount() {
        ++subtreeSize_;

        if (parent_) {
            parent_->IncrementSubtreeCount();
        }
    }

    void OctNode::DecrementSubtreeCount() {
        --subtreeSize_;

        if (parent_) {
            parent_->DecrementSubtreeCount();
        }
    }

    bool OctNode::IsLeaf() const {
        for (const auto& child : children_) {
            if (child) {
                return false;
            }
        }
        return true;
    }

    bool OctNode::IsEmpty() const {
        return bodyCount_ == 0;
    }

    bool OctNode::ContainsBody(Handle handle, const Vector3& position) const {
        if (IsEmpty()) {
            return false;
        }
        if (IsLeaf()) {
            for (const Entry& entry : entries_) {
                if (entry.handle == handle) {
                    return true;
                }
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
        if (position.x >= bounds_.center.x) {
            index |= 1;
        }
        if (position.y >= bounds_.center.y) {
            index |= 2;
        }
        if (position.z >= bounds_.center.z) {
            index |= 4;
        }
        return index;
    }

    Bounds OctNode::ComputeChildBounds(std::size_t octant) const {
        const Vector3 quarter = bounds_.halfSize * 0.5;
        const Vector3 offset((octant & 1) ? quarter.x : -quarter.x, (octant & 2) ? quarter.y : -quarter.y,
                             (octant & 4) ? quarter.z : -quarter.z);
        return Bounds{bounds_.center + offset, quarter};
    }

    void OctNode::Subdivide() {
        std::vector<Entry> existingEntries = std::move(entries_);
        entries_.clear();

        for (const Entry& entry : existingEntries) {
            const std::size_t octant = GetOctantIndex(entry.position);
            if (!children_[octant]) {
                children_[octant] = std::make_unique<OctNode>(ComputeChildBounds(octant), this, depth_ + 1);
                IncrementSubtreeCount();
            }
            children_[octant]->Insert(entry.handle, entry.position, 
                entry.mass, entry.charge);
            
            deepestDepth_ = std::max(deepestDepth_, children_[octant]->GetDeepestDepth());
        }
    }

    const OctNode* OctNode::GetParent() const {
        return parent_;
    }

    int OctNode::GetDepth() const {
        return depth_;
    }

    int OctNode::GetDeepestDepth() const {
        return deepestDepth_;
    }

    int OctNode::GetSubtreeSize() const {
        return subtreeSize_;
    }

    void OctNode::AccumulateEntry(const Entry& entry) {
        // Mass / center of mass. Mass is always positive here (Insert rejects mass <= 0),
        // so there's no cancellation risk and no fallback needed, unlike charge below.
        const double newTotalMass = totalMass_ + entry.mass;
        centerOfMass_ = (centerOfMass_ * totalMass_ + entry.position * entry.mass) / newTotalMass;
        totalMass_ = newTotalMass;

        // Running unweighted position average; used as the centerOfCharge_ fallback
        // when totalCharge_ is at or near zero (charge can cancel, mass cannot).
        geometricCenter_ =
            (geometricCenter_ * static_cast<double>(bodyCount_) + entry.position) / static_cast<double>(bodyCount_ + 1);

        const double oldTotalCharge = totalCharge_;
        const Vector3 oldCenterOfCharge = centerOfCharge_;
        const double newTotalCharge = totalCharge_ + entry.charge;

        Vector3 newCenterOfCharge;
        if (std::abs(newTotalCharge) > kChargeEpsilon) {
            newCenterOfCharge = (oldCenterOfCharge * oldTotalCharge + entry.position * entry.charge) / newTotalCharge;
        } else {
            newCenterOfCharge = geometricCenter_;
        }

        // Parallel-axis shift of the dipole moment onto the new center, plus the new
        // entry's own contribution. This identity holds regardless of whether
        // newCenterOfCharge came from the charge-weighted formula or the geometric
        // fallback above, so it stays exact across the epsilon boundary.
        dipoleMoment_ += oldTotalCharge * (oldCenterOfCharge - newCenterOfCharge) +
                         entry.charge * (entry.position - newCenterOfCharge);

        totalCharge_ = newTotalCharge;
        centerOfCharge_ = newCenterOfCharge;

        ++bodyCount_;
    }

    void OctNode::RemoveEntryProperties(const Entry& entry) {
        // Mass / center of mass (inverse of the AccumulateEntry mass update). Guarded
        // against kChargeEpsilon rather than 0.0: repeated insert/remove cycles can leave
        // a residual below Vector3::operator/'s own 1e-12 zero-check due to floating-point
        // cancellation, which would otherwise throw on a mass that should read as exactly 0.
        const double newTotalMass = totalMass_ - entry.mass;
        if (newTotalMass > kChargeEpsilon) {
            centerOfMass_ = (centerOfMass_ * totalMass_ - entry.position * entry.mass) / newTotalMass;
            totalMass_ = newTotalMass;
        } else {
            centerOfMass_ = Vector3::Zero();
            totalMass_ = 0.0;
        }

        const std::size_t newBodyCount = (bodyCount_ > 0) ? bodyCount_ - 1 : 0;
        if (newBodyCount > 0) {
            geometricCenter_ = (geometricCenter_ * static_cast<double>(bodyCount_) - entry.position) /
                                static_cast<double>(newBodyCount);
        } else {
            geometricCenter_ = Vector3::Zero();
        }

        // Charge / center of charge / dipole moment: exact algebraic inverse of
        // AccumulateEntry, solving for the "without this entry" state from the
        // current ("with this entry") state.
        const double withTotalCharge = totalCharge_;
        const Vector3 withCenterOfCharge = centerOfCharge_;
        const double withoutTotalCharge = totalCharge_ - entry.charge;

        // The weighted-inverse formula is only valid to apply if AccumulateEntry actually
        // used the weighted formula (rather than the geometric fallback) to produce the
        // currently-stored centerOfCharge_ -- i.e. only if withTotalCharge was itself above
        // epsilon at insert time. It also must not divide by a withoutTotalCharge that's near
        // zero. If either check fails, the weighted relationship never held (or is unstable),
        // so fall back to the geometric center, matching what a fresh accumulation would do.
        Vector3 withoutCenterOfCharge;
        if (std::abs(withTotalCharge) > kChargeEpsilon && std::abs(withoutTotalCharge) > kChargeEpsilon) {
            withoutCenterOfCharge =
                (withCenterOfCharge * withTotalCharge - entry.position * entry.charge) / withoutTotalCharge;
        } else {
            withoutCenterOfCharge = geometricCenter_;
        }

        dipoleMoment_ -= withoutTotalCharge * (withoutCenterOfCharge - withCenterOfCharge) +
                         entry.charge * (entry.position - withCenterOfCharge);

        totalCharge_ = withoutTotalCharge;
        centerOfCharge_ = withoutCenterOfCharge;

        bodyCount_ = newBodyCount;
    }

    // TODO: Implementent NaN catches. Something is happening with math and division.
    void OctNode::UpdateProperties() {
        totalMass_ = 0.0;
        centerOfMass_ = Vector3::Zero();
        totalCharge_ = 0.0;
        centerOfCharge_ = Vector3::Zero();
        dipoleMoment_ = Vector3::Zero();
        geometricCenter_ = Vector3::Zero();
        bodyCount_ = 0;
 
        if (IsLeaf()) {
            for (const Entry& entry : entries_) {
                totalMass_ += entry.mass;
                centerOfMass_ += entry.position * entry.mass;
 
                totalCharge_ += entry.charge;
                centerOfCharge_ += entry.position * entry.charge; // weighted sum, normalized below
                geometricCenter_ += entry.position;
 
                ++bodyCount_;
            }
 
            if (totalMass_ > 0.0) {
                centerOfMass_ /= totalMass_;
            }
 
            if (bodyCount_ > 0) {
                geometricCenter_ /= static_cast<double>(bodyCount_);
            }
 
            if (std::abs(totalCharge_) > kChargeEpsilon) {
                centerOfCharge_ /= totalCharge_;
            } else {
                centerOfCharge_ = geometricCenter_;
            }
 
            for (const Entry& entry : entries_) {
                dipoleMoment_ += (entry.position - centerOfCharge_) * entry.charge;
            }
 
            return;
        }
 
        // Internal node: recurse first so each child's aggregates are already correct,
        // then combine bottom-up.
        for (auto& child : children_) {
            if (!child) {
                continue;
            }
 
            child->UpdateProperties();
 
            totalMass_ += child->totalMass_;
            centerOfMass_ += child->centerOfMass_ * child->totalMass_;
 
            totalCharge_ += child->totalCharge_;
            centerOfCharge_ += child->centerOfCharge_ * child->totalCharge_; // weighted sum
            geometricCenter_ += child->geometricCenter_ * static_cast<double>(child->bodyCount_);
 
            bodyCount_ += child->bodyCount_;
        }
 
        if (totalMass_ > 0.0) {
            centerOfMass_ /= totalMass_;
        }
 
        if (bodyCount_ > 0) {
            geometricCenter_ /= static_cast<double>(bodyCount_);
        }
 
        if (std::abs(totalCharge_) > kChargeEpsilon) {
            centerOfCharge_ /= totalCharge_;
        } else {
            centerOfCharge_ = geometricCenter_;
        }
 
        // Parallel-axis theorem: shift each child's dipole onto the shared center, plus
        // the moment of the child's own total charge sitting at its own center.
        for (const auto& child : children_) {
            if (!child) {
                continue;
            }
            dipoleMoment_ += child->dipoleMoment_ + child->totalCharge_ * (child->centerOfCharge_ - centerOfCharge_);
        }
    }

    bool OctNode::RemoveFromEntries(Handle handle, Entry& outRemoved) {
        for (auto it = entries_.begin(); it != entries_.end(); ++it) {
            if (it->handle == handle) {
                outRemoved = *it;
                entries_.erase(it);
                return true;
            }
        }
        return false;
    }

    // ==================== Octree ====================

    Octree::Octree() : rootBounds_(), root_(std::make_unique<OctNode>(rootBounds_)) {
    }

    Octree::Octree(const Bounds& rootBounds) : rootBounds_(rootBounds), root_(std::make_unique<OctNode>(rootBounds)) {
    }

    void Octree::SetRootBounds(const Bounds& rootBounds) {
        rootBounds_ = rootBounds;
        root_ = std::make_unique<OctNode>(rootBounds_);
    }

    void Octree::Clear() {
        root_ = std::make_unique<OctNode>(rootBounds_);
    }

    void Octree::Insert(Entry entry) {
        Insert(entry.handle, entry.position, entry.mass, entry.charge);
    }
    
    void Octree::Insert(Handle handle, const Vector3& position, double mass, double charge) {
        root_->Insert(handle, position, mass, charge);
    }

    void Octree::Remove(Handle handle) {
        root_->Remove(handle);
    }

    size_t Octree::Size() const {
        return root_ ? root_->GetSubtreeSize() : 0;
    }

    int Octree::GetDepth() const {
        return root_ ? root_->GetDeepestDepth() : 0;
    }

    void Octree::UpdateProperties() {
        if (root_) {
            root_->UpdateProperties();
        }
    }

    const OctNode& Octree::GetRoot() const {
        return *root_;
    }

} // namespace osseus