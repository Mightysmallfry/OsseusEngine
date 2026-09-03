#include <iostream>

#include "Osseus/physics/Forces/UniversalGravityForce.h"

namespace osseus {

    Vector3 UniversalGravity::CalculateForce(const OctNode& sourceNode, Handle handle, const BodyData& body) {
        if (sourceNode.IsEmpty()) {
            return Vector3::Zero();
        }

        Vector3 queryPosition = body.position;
        double queryMass = body.mass;

        const bool inBounds = sourceNode.GetBounds().Contains(queryPosition);

        if (sourceNode.IsLeaf()) {
            if (inBounds && sourceNode.ContainsBody(handle, queryPosition)) {
                return Vector3::Zero();
            }
            return PointForce(sourceNode.GetCenterOfMass(), sourceNode.GetTotalMass(), queryPosition, queryMass);
        }

        const Vector3 offset = sourceNode.GetCenterOfMass() - queryPosition;
        const double distSq = offset.LengthSquared() + softening_ * softening_;
        const double dist = std::sqrt(distSq);

        const Vector3& halfSize = sourceNode.GetBounds().halfSize;
        const double width = 2.0 * std::max({halfSize.x, halfSize.y, halfSize.z});

        const bool farEnough = dist > 1e-12 && (width / dist) < theta_;

        if (farEnough && !(inBounds && sourceNode.ContainsBody(handle, queryPosition))) {
            return PointForce(sourceNode.GetCenterOfMass(), sourceNode.GetTotalMass(), queryPosition, queryMass);
        }

        Vector3 total = Vector3::Zero();
        for (std::size_t i = 0; i < 8; ++i) {
            if (sourceNode.HasChild(i)) {
                total += CalculateForce(*sourceNode.GetChild(i), handle, body);
            }
        }
        return total;
    }

    Vector3 UniversalGravity::PointForce(const Vector3& sourcePosition, double sourceMass, const Vector3& queryPosition,
                                         double queryMass) {

        if (!sourcePosition.IsFinite() || !queryPosition.IsFinite() || !std::isfinite(sourceMass) ||
            !std::isfinite(queryMass)) {

            std::cerr << "INVALID GRAVITY INPUT\n";
            std::cerr << "Source: " << sourcePosition << " mass=" << sourceMass << "\n";
            std::cerr << "Query: " << queryPosition << " mass=" << queryMass << "\n";

            std::exit(EXIT_FAILURE);
            return Vector3::Zero();
        }

        const Vector3 offset = sourcePosition - queryPosition;
        const double distSq = offset.LengthSquared() + softening_ * softening_;
        const double dist = std::sqrt(distSq);

        if (!std::isfinite(distSq) || !std::isfinite(dist)) {
            std::cerr << "INVALID GRAVITY DISTANCE\n";
            std::cerr << "Source: " << sourcePosition << "\n";
            std::cerr << "Query: " << queryPosition << "\n";
            std::cerr << "Offset: " << offset << "\n";
            std::cerr << "distSq: " << distSq << "\n";
            std::cerr << "dist: " << dist << "\n";
            return Vector3::Zero();
        }

        if (dist <= 1e-12) {
            std::cerr << "Short range point detected\n";
            std::cerr << "Source: " << sourcePosition << " Query: " << queryPosition << "\n";
            return Vector3::Zero();
        }

        const double forceMag = (OsseusConstants::GravitationalConstant * queryMass * sourceMass) / distSq;

        const Vector3 force = offset * (forceMag / dist);

        if (!force.IsFinite()) {
            std::cerr << "GRAVITY PRODUCED INVALID FORCE\n";
            std::cerr << "Source: " << sourcePosition << "\n";
            std::cerr << "Query: " << queryPosition << "\n";
            std::cerr << "Source mass: " << sourceMass << "\n";
            std::cerr << "Query mass: " << queryMass << "\n";
            std::cerr << "Offset: " << offset << "\n";
            std::cerr << "distSq: " << distSq << "\n";
            std::cerr << "dist: " << dist << "\n";
            std::cerr << "forceMag: " << forceMag << "\n";
        }

        return force;
    }

} // namespace osseus