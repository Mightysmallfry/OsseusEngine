#include "Osseus/physics/BarnesHut.h"

#include <algorithm>
#include <cmath>

namespace osseus {

    BarnesHut::BarnesHut(double theta, double G, double softening)
        : theta_(theta), G_(G), softening_(softening) {}

    void BarnesHut::Evaluate(const Octree& tree, const BodyManager& bodyManager, ForceManager& forceManager) const {
        const OctNode& root = tree.GetRoot();

        for (Handle handle : bodyManager.Handles()) {
            const BodyData* body = bodyManager.GetBody(handle);
            if (body == nullptr) { continue; }
            if (body->invMass == 0.0) { continue; } // static bodies don't move; no force needed

            const Vector3 force = CalculateForce(root, handle, body->position, body->mass);
            forceManager.Add(handle, force);
        }
    }

    Vector3 BarnesHut::CalculateForce(const OctNode& node, Handle queryHandle,
                                       const Vector3& queryPosition, double queryMass) const {
        if (node.IsEmpty()) { return Vector3::Zero(); }

        if (node.IsLeaf()) {
            if (node.ContainsBody(queryHandle, queryPosition)) { return Vector3::Zero(); }
            return PointForce(node.GetCenterOfMass(), node.GetTotalMass(), queryPosition, queryMass);
        }

        const Vector3 offset = node.GetCenterOfMass() - queryPosition;
        const double distSq = offset.LengthSquared() + softening_ * softening_;
        const double dist = std::sqrt(distSq);

        const Vector3& halfSize = node.GetBounds().halfSize;
        const double width = 2.0 * std::max({ halfSize.x, halfSize.y, halfSize.z });

        const bool farEnough = dist > 1e-12 && (width / dist) < theta_;

        if (farEnough && !node.ContainsBody(queryHandle, queryPosition)) {
            return PointForce(node.GetCenterOfMass(), node.GetTotalMass(), queryPosition, queryMass);
        }

        Vector3 total = Vector3::Zero();
        for (std::size_t i = 0; i < 8; ++i) {
            if (node.HasChild(i)) {
                total += CalculateForce(*node.GetChild(i), queryHandle, queryPosition, queryMass);
            }
        }
        return total;
    }

    Vector3 BarnesHut::PointForce(const Vector3& sourcePosition, double sourceMass,
                                   const Vector3& queryPosition, double queryMass) const {
        const Vector3 offset = sourcePosition - queryPosition;
        const double distSq = offset.LengthSquared() + softening_ * softening_;
        const double dist = std::sqrt(distSq);
        if (dist <= 1e-12) { return Vector3::Zero(); }

        const double forceMag = (G_ * queryMass * sourceMass) / distSq;
        return offset * (forceMag / dist); // (offset / dist) is the unit direction
    }

    void BarnesHut::SetTheta(double newTheta) { theta_ = newTheta; }
    double BarnesHut::GetTheta() const { return theta_; }

    void BarnesHut::SetG(double newG) { G_ = newG; }
    double BarnesHut::GetG() const { return G_; }

    void BarnesHut::SetSoftening(double newSoftening) { softening_ = newSoftening; }
    double BarnesHut::GetSoftening() const { return softening_; }

} // osseus