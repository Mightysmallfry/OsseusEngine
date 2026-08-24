#include "Osseus/physics/Forces/UniversalEMForce.h"

namespace osseus {

    Vector3 UniversalElectroMag::CalculateForce(const OctNode& sourceNode, Handle handle, const BodyData& body) {
        if (sourceNode.IsEmpty()) { return Vector3::Zero(); }

        Vector3 queryPosition = body.position;
        double queryCharge = body.charge;

        if (sourceNode.IsLeaf()) {
            if (sourceNode.ContainsBody(handle, queryPosition)) { return Vector3::Zero(); }
            return PointForce(sourceNode.GetCenterOfCharge(), sourceNode.GetTotalCharge(), queryPosition, queryCharge);
        }

        const Vector3 offset = sourceNode.GetCenterOfCharge() - queryPosition;
        const double distSq = offset.LengthSquared() + softening_ * softening_;
        const double dist = std::sqrt(distSq);

        const Vector3& halfSize = sourceNode.GetBounds().halfSize;
        const double width = 2.0 * std::max({ halfSize.x, halfSize.y, halfSize.z });

        const bool farEnough = dist > 1e-12 && (width / dist) < theta_;

        if (farEnough && !sourceNode.ContainsBody(handle, queryPosition)) {
            Vector3 monopole = PointForce(sourceNode.GetCenterOfCharge(), sourceNode.GetTotalCharge(), queryPosition, queryCharge);
            
            Vector3 dipole = DipoleForce(sourceNode.GetDipoleMoment(), 
                sourceNode.GetCenterOfCharge(), 
                queryPosition, queryCharge);

            return monopole + dipole;
        }

        Vector3 total = Vector3::Zero();
        for (std::size_t i = 0; i < 8; ++i) {
            if (sourceNode.HasChild(i)) {
                total += CalculateForce(*sourceNode.GetChild(i), handle, body);
            }
        }
        return total;
    }


    Vector3 UniversalElectroMag::PointForce(const Vector3& sourcePosition, double sourceCharge,
                                   const Vector3& queryPosition, double queryCharge) {
        const Vector3 offset = sourcePosition - queryPosition;
        const double distSq = offset.LengthSquared() + softening_ * softening_;
        const double dist = std::sqrt(distSq);
        if (dist <= 1e-12) { return Vector3::Zero(); }

        const double forceMag = (OsseusConstants::CoulombConstant * queryCharge * sourceCharge) / distSq;
        return offset * (forceMag / dist); // (offset / dist) is the unit direction
    }



    Vector3 UniversalElectroMag::DipoleForce(const Vector3& dipoleMoment, const Vector3& sourcePosition,
                                   const Vector3& queryPosition, double queryCharge) {
        Vector3 offset = queryPosition - sourcePosition;  // note: field point relative to dipole
        const double distSq = offset.LengthSquared() + softening_ * softening_;
        const double dist = std::sqrt(distSq);
        if (dist <= 1e-12) { return Vector3::Zero(); }

        const Vector3 rHat = offset / dist;
        const double pDotR = dipoleMoment.Dot(rHat);
        const Vector3 field = (rHat * (3.0 * pDotR) - dipoleMoment) * (OsseusConstants::CoulombConstant / (distSq * dist));
        return field * queryCharge;

    }

}