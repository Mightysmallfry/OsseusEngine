#include "Osseus/math/Collision/GJK.h"

#include <cmath>

namespace osseus {
    GJKSupportPoint GJK::Support(const IShape& shapeA, const Vector3& posA, const IShape& shapeB, const Vector3& posB,
                                 const Vector3& direction) {
        Vector3 pointA = shapeA.Support(posA, direction);
        Vector3 pointB = shapeB.Support(posB, -direction);
        return GJKSupportPoint{pointA - pointB, pointA, pointB};
    }

    bool GJK::Intersect(const IShape& shapeA, const Vector3& posA, const IShape& shapeB, const Vector3& posB,
                        GJKSimplex& outSimplex) {
        Vector3 direction = posB - posA;
        if (direction.LengthSquared() < Vector3::TOLERANCE) {
            direction = Vector3::UnitX();
        }

        GJKSupportPoint support = Support(shapeA, posA, shapeB, posB, direction);
        outSimplex = GJKSimplex{};
        outSimplex.PushFront(support);

        direction = -support.point;

        constexpr int maxIterations = 64;
        for (int i = 0; i < maxIterations; ++i) {
            if (direction.LengthSquared() < Vector3::TOLERANCE) {
                // Degenerate direction (origin sits exactly on a prior
                // support point) - nudge and keep going.
                direction = Vector3::UnitX();
            }

            support = Support(shapeA, posA, shapeB, posB, direction);

            if (support.point.Dot(direction) < 0.0) {
                return false; // new point didn't pass the origin - no overlap
            }

            outSimplex.PushFront(support);

            if (NextSimplex(outSimplex, direction)) {
                return true;
            }
        }
        return false; // failed to converge within budget - treat as a miss
    }

    bool GJK::NextSimplex(GJKSimplex& simplex, Vector3& direction) {
        switch (simplex.Size()) {
        case 2:
            return Line(simplex, direction);
        case 3:
            return Triangle(simplex, direction);
        case 4:
            return Tetrahedron(simplex, direction);
        default:
            return false;
        }
    }

    bool GJK::SameDirection(const Vector3& direction, const Vector3& ao) {
        return direction.Dot(ao) > 0.0;
    }

    Vector3 GJK::ArbitraryPerpendicular(const Vector3& v) {
        Vector3 reference = (std::abs(v.x) < 0.9 * std::sqrt(v.LengthSquared() + Vector3::TOLERANCE))
                                ? Vector3::UnitX()
                                : Vector3::UnitY();
        Vector3 perp = v.Cross(reference);
        if (perp.LengthSquared() < Vector3::TOLERANCE) {
            perp = v.Cross(Vector3::UnitZ());
        }
        return perp;
    }

    bool GJK::Line(GJKSimplex& simplex, Vector3& direction) {
        GJKSupportPoint a = simplex[0];
        GJKSupportPoint b = simplex[1];
        Vector3 ab = b.point - a.point;
        Vector3 ao = -a.point;

        if (SameDirection(ab, ao)) {
            Vector3 candidate = ab.Cross(ao).Cross(ab);
            // ao parallel (or anti-parallel) to ab collapses the usual
            // double-cross to zero - the origin sits exactly on the line.
            // Any direction perpendicular to ab is a valid way to keep
            // searching outward.
            direction = (candidate.LengthSquared() > Vector3::TOLERANCE) ? candidate : ArbitraryPerpendicular(ab);
        } else {
            simplex.Set({a});
            direction = ao;
        }
        return false;
    }

    bool GJK::Triangle(GJKSimplex& simplex, Vector3& direction) {
        GJKSupportPoint a = simplex[0];
        GJKSupportPoint b = simplex[1];
        GJKSupportPoint c = simplex[2];

        Vector3 ab = b.point - a.point;
        Vector3 ac = c.point - a.point;
        Vector3 ao = -a.point;
        Vector3 abc = ab.Cross(ac);

        if (SameDirection(abc.Cross(ac), ao)) {
            if (SameDirection(ac, ao)) {
                simplex.Set({a, c});
                direction = ac.Cross(ao).Cross(ac);
            } else {
                simplex.Set({a, b});
                return Line(simplex, direction);
            }
        } else if (SameDirection(ab.Cross(abc), ao)) {
            simplex.Set({a, b});
            return Line(simplex, direction);
        } else {
            if (SameDirection(abc, ao)) {
                direction = abc;
            } else {
                simplex.Set({a, c, b});
                direction = -abc;
            }
        }
        return false;
    }

    bool GJK::Tetrahedron(GJKSimplex& simplex, Vector3& direction) {
        GJKSupportPoint a = simplex[0];
        GJKSupportPoint b = simplex[1];
        GJKSupportPoint c = simplex[2];
        GJKSupportPoint d = simplex[3];

        Vector3 ab = b.point - a.point;
        Vector3 ac = c.point - a.point;
        Vector3 ad = d.point - a.point;
        Vector3 ao = -a.point;

        Vector3 abc = ab.Cross(ac);
        Vector3 acd = ac.Cross(ad);
        Vector3 adb = ad.Cross(ab);

        if (SameDirection(abc, ao)) {
            simplex.Set({a, b, c});
            return Triangle(simplex, direction);
        }
        if (SameDirection(acd, ao)) {
            simplex.Set({a, c, d});
            return Triangle(simplex, direction);
        }
        if (SameDirection(adb, ao)) {
            simplex.Set({a, d, b});
            return Triangle(simplex, direction);
        }

        return true; // origin is enclosed by the tetrahedron - overlap confirmed
    }
} // namespace osseus
