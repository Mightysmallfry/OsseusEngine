//
// Created by MightySmallFry on 7/24/2026.
//

#ifndef OSSEUSENGINE_NARROWPHASE_H
#define OSSEUSENGINE_NARROWPHASE_H
#include <vector>

#include "BodyManager.h"
#include "Contact.h"
#include "CollisionCandidatePair.h"
#include "ShapeManager.h"

namespace osseus {
    class NarrowPhase {
    public:
        std::vector<Contact> GenerateContacts(const std::vector<CollisionCandidatePair>& candidates,
                                               BodyManager& bodyManager, ShapeManager& shapeManager) const;

    private:
        enum class ShapeKind { Point, Sphere, Cube };

        // ShapePoint (and any unrecognized shape type) classifies as Point
        // and is treated as a radius-0 sphere by the sphere tests below, so
        // new shapes still collide reasonably until they get real handling.
        ShapeKind Classify(const IShape* shape) const;
        double GetRadius(const IShape* shape) const;

        bool IntersectSphereSphere(const Vector3& posA, double radiusA,
                               const Vector3& posB, double radiusB,
                               Contact& outContact) const;

        // sphereIsA tells the test which body (a or b) the sphere/point
        // belongs to, so the resulting Contact::normal can be built
        // pointing consistently from a to b.
        bool IntersectSphereCube(const Vector3& spherePos, double sphereRadius,
                             const Vector3& cubePos, double cubeHalfExtent,
                             bool sphereIsA, Contact& outContact) const;

        bool IntersectCubeCube(const Vector3& posA, double halfExtentA,
                           const Vector3& posB, double halfExtentB,
                           Contact& outContact) const;
    };
} // osseus

#endif //OSSEUSENGINE_NARROWPHASE_H
