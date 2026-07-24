//
// Created by MightySmallFry on 7/24/2026.
//

#include "../../include/Osseus/system/NarrowPhase.h"

#include <algorithm>
#include <cmath>

#include "Osseus/math/Geometry/ShapeCube.h"
#include "Osseus/math/Geometry/ShapeSphere.h"

namespace osseus {
    NarrowPhase::ShapeKind NarrowPhase::Classify(const IShape* shape) const {
        if (dynamic_cast<const ShapeCube*>(shape)) { return ShapeKind::Cube; }
        if (dynamic_cast<const ShapeSphere*>(shape)) { return ShapeKind::Sphere; }
        return ShapeKind::Point;
    }

    double NarrowPhase::GetRadius(const IShape* shape) const {
        if (const auto* sphere = dynamic_cast<const ShapeSphere*>(shape)) {
            return sphere->GetRadius();
        }
        return 0.0; // ShapePoint, or any unrecognized shape, treated as a point
    }

    bool NarrowPhase::IntersectSphereSphere(const Vector3& posA, double radiusA,
                                        const Vector3& posB, double radiusB,
                                        Contact& outContact) const {
        const double radiusSum = radiusA + radiusB;
        const double distSq = DistanceSquared(posA, posB);

        if (distSq > radiusSum * radiusSum) { return false; }

        const double dist = std::sqrt(distSq);

        // Coincident centers: pick an arbitrary separation axis rather than
        // dividing by zero.
        outContact.normal = (dist > Vector3::TOLERANCE)
            ? (posB - posA) / dist
            : Vector3::UnitY();
        outContact.penetration = radiusSum - dist;
        return true;
    }

    bool NarrowPhase::IntersectSphereCube(const Vector3& spherePos, double sphereRadius,
                                      const Vector3& cubePos, double cubeHalfExtent,
                                      bool sphereIsA, Contact& outContact) const {
        const Vector3 local = spherePos - cubePos;
        const Vector3 clamped(
            std::clamp(local.x, -cubeHalfExtent, cubeHalfExtent),
            std::clamp(local.y, -cubeHalfExtent, cubeHalfExtent),
            std::clamp(local.z, -cubeHalfExtent, cubeHalfExtent));
        const Vector3 closestPoint = cubePos + clamped;

        const Vector3 diff = spherePos - closestPoint;
        const double distSq = diff.LengthSquared();

        if (distSq > sphereRadius * sphereRadius) { return false; }

        const double dist = std::sqrt(distSq);

        // cubeToSphere points away from the cube's surface toward the
        // sphere center. When the center sits exactly on/inside the cube
        // (dist ~ 0, e.g. a fast-moving small sphere that already tunneled
        // to the cube's center) there's no clean surface direction to
        // separate along, so fall back to +Y. Good enough for shallow
        // penetration; this simple solver isn't meant to resolve deep
        // tunneling.
        const Vector3 cubeToSphere = (dist > Vector3::TOLERANCE) ? diff / dist : Vector3::UnitY();

        // Contact::normal always points from a to b.
        outContact.normal = sphereIsA ? -cubeToSphere : cubeToSphere;
        outContact.penetration = sphereRadius - dist;
        return true;
    }

    bool NarrowPhase::IntersectCubeCube(const Vector3& posA, double halfExtentA,
                                    const Vector3& posB, double halfExtentB,
                                    Contact& outContact) const {
        const Vector3 delta = posB - posA;

        const double overlapX = (halfExtentA + halfExtentB) - std::abs(delta.x);
        if (overlapX <= 0.0) { return false; }

        const double overlapY = (halfExtentA + halfExtentB) - std::abs(delta.y);
        if (overlapY <= 0.0) { return false; }

        const double overlapZ = (halfExtentA + halfExtentB) - std::abs(delta.z);
        if (overlapZ <= 0.0) { return false; }

        // Minimum translation axis: push apart along whichever axis has the
        // least overlap, since that's the cheapest way out.
        if (overlapX < overlapY && overlapX < overlapZ) {
            outContact.normal = Vector3(delta.x < 0.0 ? -1.0 : 1.0, 0.0, 0.0);
            outContact.penetration = overlapX;
        } else if (overlapY < overlapZ) {
            outContact.normal = Vector3(0.0, delta.y < 0.0 ? -1.0 : 1.0, 0.0);
            outContact.penetration = overlapY;
        } else {
            outContact.normal = Vector3(0.0, 0.0, delta.z < 0.0 ? -1.0 : 1.0);
            outContact.penetration = overlapZ;
        }
        return true;
    }

    std::vector<Contact> NarrowPhase::GenerateContacts(const std::vector<CollisionCandidatePair>& candidates,
                                                         BodyManager& bodyManager, ShapeManager& shapeManager) const {
        std::vector<Contact> contacts;

        for (const CollisionCandidatePair& pair : candidates) {
            const BodyData* bodyA = bodyManager.GetBody(pair.a);
            const BodyData* bodyB = bodyManager.GetBody(pair.b);
            if (!bodyA || !bodyB) { continue; }

            const IShape* shapeA = shapeManager.GetShape(pair.a);
            const IShape* shapeB = shapeManager.GetShape(pair.b);
            if (!shapeA || !shapeB) { continue; }

            const ShapeKind kindA = Classify(shapeA);
            const ShapeKind kindB = Classify(shapeB);

            Contact contact;
            bool hit = false;

            if (kindA == ShapeKind::Cube && kindB == ShapeKind::Cube) {
                const auto* cubeA = static_cast<const ShapeCube*>(shapeA);
                const auto* cubeB = static_cast<const ShapeCube*>(shapeB);
                hit = IntersectCubeCube(bodyA->position, cubeA->GetHalfExtent(),
                                    bodyB->position, cubeB->GetHalfExtent(), contact);
            } else if (kindA == ShapeKind::Cube || kindB == ShapeKind::Cube) {
                const bool cubeIsA = (kindA == ShapeKind::Cube);
                const auto* cube = static_cast<const ShapeCube*>(cubeIsA ? shapeA : shapeB);
                const IShape* sphereShape = cubeIsA ? shapeB : shapeA;
                const Vector3& cubePos = cubeIsA ? bodyA->position : bodyB->position;
                const Vector3& spherePos = cubeIsA ? bodyB->position : bodyA->position;

                hit = IntersectSphereCube(spherePos, GetRadius(sphereShape),
                                      cubePos, cube->GetHalfExtent(),
                                      /*sphereIsA=*/!cubeIsA, contact);
            } else {
                hit = IntersectSphereSphere(bodyA->position, GetRadius(shapeA),
                                        bodyB->position, GetRadius(shapeB), contact);
            }

            if (hit) {
                contact.a = pair.a;
                contact.b = pair.b;
                contacts.push_back(contact);
            }
        }
        return contacts;
    }
} // osseus
