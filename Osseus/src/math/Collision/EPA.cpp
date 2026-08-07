#include "Osseus/math/Collision/EPA.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace osseus {
    EPA::Face EPA::MakeFace(const std::vector<GJKSupportPoint>& polytope, int a, int b, int c) {
        Vector3 pa = polytope[a].point;
        Vector3 pb = polytope[b].point;
        Vector3 pc = polytope[c].point;

        Vector3 normal = (pb - pa).Cross(pc - pa);
        double lengthSq = normal.LengthSquared();
        if (lengthSq <= 1e-12) {
            // Degenerate (near zero-area) triangle. Its "normal" is
            // meaningless noise, and dotting it against pa can look like
            // a tiny distance even when this face isn't remotely close
            // to the real surface. Keep it out of the closest-face search
            // entirely rather than let it masquerade as a converged hit;
            // the edge-patching step will supersede it as the polytope grows.
            return Face{ a, b, c, Vector3::Zero(), std::numeric_limits<double>::max() };
        }
        normal = normal / std::sqrt(lengthSq);

        double distance = normal.Dot(pa);
        if (distance < 0.0) {
            // Winding put the normal facing the origin - flip both so it
            // always points outward, away from the enclosed origin.
            normal = -normal;
            distance = -distance;
            std::swap(b, c);
        }

        return Face{ a, b, c, normal, distance };
    }

    void EPA::AddUniqueEdge(std::vector<std::pair<int, int>>& edges, int a, int b) {
        auto reversed = std::find(edges.begin(), edges.end(), std::make_pair(b, a));
        if (reversed != edges.end()) {
            // Shared by two faces being removed together - it's an
            // interior edge of the hole, not part of its silhouette.
            edges.erase(reversed);
        } else {
            edges.emplace_back(a, b);
        }
    }

    Contact EPA::BuildContact(const std::vector<GJKSupportPoint>& polytope, const Face& face,
                               Handle handleA, Handle handleB) {
        Vector3 pa = polytope[face.a].point;
        Vector3 pb = polytope[face.b].point;
        Vector3 pc = polytope[face.c].point;
        Vector3 projected = face.normal * face.distance; // closest point on the face's plane to the origin

        // Barycentric coordinates of `projected` within triangle (pa, pb, pc),
        // used to interpolate the real witness points on A and B.
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

    Contact EPA::Resolve(const IShape& shapeA, const Vector3& posA, Handle handleA,
                          const IShape& shapeB, const Vector3& posB, Handle handleB,
                          const GJKSimplex& startingSimplex) {
        std::vector<GJKSupportPoint> polytope = {
            startingSimplex[0], startingSimplex[1], startingSimplex[2], startingSimplex[3]
        };

        std::vector<Face> faces = {
            MakeFace(polytope, 0, 1, 2),
            MakeFace(polytope, 0, 2, 3),
            MakeFace(polytope, 0, 3, 1),
            MakeFace(polytope, 1, 3, 2)
        };

        constexpr int maxIterations = 64;
        constexpr double epsilon = 1e-8;

        Face closest = faces[0];

        for (int iteration = 0; iteration < maxIterations; ++iteration) {
            size_t closestIndex = 0;
            double minDistance = std::numeric_limits<double>::max();
            for (size_t i = 0; i < faces.size(); ++i) {
                if (faces[i].distance < minDistance) {
                    minDistance = faces[i].distance;
                    closestIndex = i;
                }
            }
            closest = faces[closestIndex];

            GJKSupportPoint newPoint = GJK::Support(shapeA, posA, shapeB, posB, closest.normal);
            double supportDistance = newPoint.point.Dot(closest.normal);

            if (supportDistance - closest.distance < epsilon) {
                // The closest face already sits on the Minkowski surface -
                // no support point extends further along its normal.
                break;
            }

            polytope.push_back(newPoint);
            int newIndex = static_cast<int>(polytope.size() - 1);

            // Remove every face visible from the new point, recording the
            // silhouette edges left behind so we can patch the hole.
            std::vector<std::pair<int, int>> uniqueEdges;
            for (auto it = faces.begin(); it != faces.end(); ) {
                Vector3 faceToPoint = newPoint.point - polytope[it->a].point;
                if (it->normal.Dot(faceToPoint) > 0.0) {
                    AddUniqueEdge(uniqueEdges, it->a, it->b);
                    AddUniqueEdge(uniqueEdges, it->b, it->c);
                    AddUniqueEdge(uniqueEdges, it->c, it->a);
                    it = faces.erase(it);
                } else {
                    ++it;
                }
            }

            for (const auto& edge : uniqueEdges) {
                faces.push_back(MakeFace(polytope, edge.first, edge.second, newIndex));
            }
        }

        return BuildContact(polytope, closest, handleA, handleB);
    }
} // osseus
