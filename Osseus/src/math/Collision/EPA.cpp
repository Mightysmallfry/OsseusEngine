#include "Osseus/math/Collision/EPA.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>

// ============================================================================
//  EPA Face Construction
// ============================================================================

namespace osseus {
    EPA::Face EPA::MakeFace(const std::vector<GJKSupportPoint>& polytope, int a, int b, int c) {
        Vector3 pa = polytope[a].point;
        Vector3 pb = polytope[b].point;
        Vector3 pc = polytope[c].point;

        Vector3 normal = (pb - pa).Cross(pc - pa);
        double lengthSq = normal.LengthSquared();
        if (lengthSq <= 1e-12) {
            return Face{a, b, c, Vector3::Zero(), std::numeric_limits<double>::max(), true};
        }

        double invLen = 1.0 / std::sqrt(lengthSq);
        normal *= invLen;
        double distance = normal.Dot(pa);

        return Face{a, b, c, normal, distance, true};
    }

    // ============================================================================
    //  Silhouette Edge Management
    // ============================================================================

    void EPA::AddUniqueEdge(std::vector<std::pair<int, int>>& edges, int a, int b) {
        for (size_t i = 0; i < edges.size(); ++i) {
            if (edges[i].first == b && edges[i].second == a) {
                edges[i] = edges.back();
                edges.pop_back();
                return;
            }
        }

        edges.emplace_back(a, b);
    }

    // ============================================================================
    //  Contact Construction
    // ============================================================================

    Contact EPA::BuildContact(const std::vector<GJKSupportPoint>& polytope, const Face& face, Handle handleA,
                              Handle handleB) {
        const Vector3& pa = polytope[face.a].point;
        const Vector3& pb = polytope[face.b].point;
        const Vector3& pc = polytope[face.c].point;
        Vector3 projected = face.normal * face.distance;

        Vector3 v0 = pb - pa;
        Vector3 v1 = pc - pa;
        Vector3 v2 = projected - pa;

        double d00 = v0.Dot(v0);
        double d01 = v0.Dot(v1);
        double d11 = v1.Dot(v1);
        double d20 = v2.Dot(v0);
        double d21 = v2.Dot(v1);
        double denom = d00 * d11 - d01 * d01;

        double v = 0.0;
        double w = 0.0;
        if (std::abs(denom) > Vector3::TOLERANCE) {
            v = (d11 * d20 - d01 * d21) / denom;
            w = (d00 * d21 - d01 * d20) / denom;
        }
        double u = 1.0 - v - w;

        Contact contact;
        contact.a = handleA;
        contact.b = handleB;
        contact.normal = face.normal;
        contact.penetration = face.distance;
        contact.pointOnA = polytope[face.a].pointA * u + polytope[face.b].pointA * v + polytope[face.c].pointA * w;
        contact.pointOnB = polytope[face.a].pointB * u + polytope[face.b].pointB * v + polytope[face.c].pointB * w;

        return contact;
    }

    // ============================================================================
    //  EPA Main Resolution Loop
    // ============================================================================

    Contact EPA::Resolve(const IShape& shapeA, const Vector3& posA, Handle handleA, const IShape& shapeB,
                         const Vector3& posB, Handle handleB, const GJKSimplex& startingSimplex) {

        // ----------------------------------------
        // Initialize polytope
        // ----------------------------------------
        std::vector<GJKSupportPoint> polytope;
        polytope.reserve(64);
        polytope.insert(polytope.end(),
                        {startingSimplex[0], startingSimplex[1], startingSimplex[2], startingSimplex[3]});

        // ----------------------------------------
        // Fix tetrahedron handedness
        // ----------------------------------------
        if ((polytope[1].point - polytope[0].point)
                .Dot((polytope[2].point - polytope[0].point).Cross(polytope[3].point - polytope[0].point)) > 0.0) {
            std::swap(polytope[2], polytope[3]);
        }

        // ----------------------------------------
        // Build initial faces
        // ----------------------------------------
        std::vector<Face> faces;
        faces.reserve(128);
        faces.insert(faces.end(), {MakeFace(polytope, 0, 1, 2), MakeFace(polytope, 0, 2, 3),
                                   MakeFace(polytope, 0, 3, 1), MakeFace(polytope, 1, 3, 2)});

        std::vector<int> aliveIndices;
        aliveIndices.reserve(128);
        for (int i = 0; i < 4; ++i) {
            faces[i].aliveSlot = i;
            aliveIndices.push_back(i);
        }

        // ----------------------------------------
        // Initialize heap
        // ----------------------------------------
        std::priority_queue<HeapEntry, std::vector<HeapEntry>, std::greater<HeapEntry>> faceQueue;
        for (int i = 0; i < 4; ++i)
            faceQueue.emplace(faces[i].distance, i);

        constexpr int maxIterations = 64;
        constexpr double epsilon = 1e-4;
        constexpr double duplicatePointEpsilonSq = 1e-10;

        Face lastClosest{};
        bool hasClosest = false;

        // ====================================================================
        //  EPA Iteration Loop
        // ====================================================================

        Vector3 lastDir = Vector3::Zero();
        GJKSupportPoint lastSupport;
        bool hasCachedSupport = false;
        constexpr double dirSimilarityThreshold = 0.99999; // cosine similarity ~0.5 degrees

        std::vector<std::pair<int, int>> uniqueEdges;
        uniqueEdges.reserve(64);

        for (int iteration = 0; iteration < maxIterations; ++iteration) {

            // ----------------------------------------
            // Pop closest alive face
            // ----------------------------------------
            hasClosest = false;
            while (!faceQueue.empty()) {
                HeapEntry entry = faceQueue.top();
                faceQueue.pop();

                Face& f = faces[entry.index];
                if (f.alive) {
                    lastClosest = f;
                    hasClosest = true;
                    break;
                }
            }

            if (!hasClosest)
                break;

            // ----------------------------------------
            // Support point in direction of closest normal
            // ----------------------------------------

            Vector3 dir = lastClosest.normal;
            // Check if direction is nearly identical to last direction
            GJKSupportPoint newPoint;
            if (hasCachedSupport && dir.Dot(lastDir) > dirSimilarityThreshold) {
                newPoint = lastSupport; // reuse cached support point
            } else {
                newPoint = GJK::Support(shapeA, posA, shapeB, posB, dir);
                lastSupport = newPoint;
                lastDir = dir;
                hasCachedSupport = true;
            }

            double supportDistance = newPoint.point.Dot(lastClosest.normal);

            if (supportDistance - lastClosest.distance < epsilon)
                break;

            // ----------------------------------------
            // Duplicate point guard
            // ----------------------------------------

            for (const auto& existing : polytope) {
                Vector3 difference = existing.point - newPoint.point;

                if (fabs(difference.x) < epsilon && fabs(difference.y) < epsilon && fabs(difference.z) < epsilon) {
                    if (difference.LengthSquared() < duplicatePointEpsilonSq)
                        return BuildContact(polytope, lastClosest, handleA, handleB);
                }
            }

            polytope.push_back(newPoint);
            int newIndex = static_cast<int>(polytope.size() - 1);

            // ----------------------------------------
            // Silhouette extraction (lazy removal)
            // ----------------------------------------

            uniqueEdges.clear();

            for (size_t slot = 0; slot < aliveIndices.size();) {
                Face& face = faces[aliveIndices[slot]];

                Vector3 faceToPoint = newPoint.point - polytope[face.a].point;

                if (face.normal.Dot(faceToPoint) > epsilon) {
                    AddUniqueEdge(uniqueEdges, face.a, face.b);
                    AddUniqueEdge(uniqueEdges, face.b, face.c);
                    AddUniqueEdge(uniqueEdges, face.c, face.a);

                    face.alive = false;
                    face.aliveSlot = -1;
                    int lastIndex = aliveIndices.back();
                    aliveIndices[slot] = lastIndex;
                    faces[lastIndex].aliveSlot = static_cast<int>(slot);
                    aliveIndices.pop_back();
                } else {
                    ++slot;
                }
            }

            // ----------------------------------------
            // Patch hole with new faces
            // ----------------------------------------
            for (const auto& edge : uniqueEdges) {
                Face newFace = MakeFace(polytope, edge.first, edge.second, newIndex);

                newFace.aliveSlot = static_cast<int>(aliveIndices.size());
                faces.push_back(newFace);
                aliveIndices.push_back(static_cast<int>(faces.size() - 1));
                faceQueue.emplace(newFace.distance, faces.size() - 1);
            }
        }

        // ====================================================================
        //  Final Contact
        // ====================================================================
        return BuildContact(polytope, lastClosest, handleA, handleB);
    }
} // namespace osseus