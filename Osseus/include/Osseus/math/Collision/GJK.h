#ifndef OSSEUSENGINE_GJK_H
#define OSSEUSENGINE_GJK_H
#include <array>
#include <cstddef>
#include <initializer_list>

#include "Osseus/interfaces/IShape.h"
#include "Osseus/math/Vector3.h"

namespace osseus {
    // A point on the Minkowski difference of two shapes, carrying the
    // witness points on each original shape that produced it. GJK only
    // needs `.point`; EPA needs `.pointA`/`.pointB` to reconstruct real
    // contact points once a collision is confirmed.
    struct GJKSupportPoint {
        Vector3 point;  // pointA - pointB, i.e. the Minkowski difference
        Vector3 pointA; // witness point on shape A
        Vector3 pointB; // witness point on shape B
    };

    // Fixed-capacity simplex (never more than a tetrahedron in 3D).
    // Index 0 always holds the most recently added point.
    class GJKSimplex {
        public:
        void PushFront(const GJKSupportPoint& point) {
            points_ = {point, points_[0], points_[1], points_[2]};
            count_ = count_ < 4 ? count_ + 1 : 4;
        }

        void Set(std::initializer_list<GJKSupportPoint> newPoints) {
            count_ = 0;
            for (const auto& p : newPoints) {
                points_[count_++] = p;
            }
        }

        GJKSupportPoint& operator[](size_t index) {
            return points_[index];
        }
        const GJKSupportPoint& operator[](size_t index) const {
            return points_[index];
        }
        size_t Size() const {
            return count_;
        }

        private:
        std::array<GJKSupportPoint, 4> points_{};
        size_t count_{0};
    };

    // Generic Gilbert-Johnson-Keerthi intersection test. It only ever
    // calls IShape::Support, so it works against any pair of convex
    // shapes without knowing what they are - adding a new shape never
    // requires touching this class.
    class GJK {
        public:
        static bool Intersect(const IShape& shapeA, const Vector3& posA, const IShape& shapeB, const Vector3& posB,
                              GJKSimplex& outSimplex);

        // Support point of the Minkowski difference (A - B) along `direction`.
        static GJKSupportPoint Support(const IShape& shapeA, const Vector3& posA, const IShape& shapeB,
                                       const Vector3& posB, const Vector3& direction);

        private:
        static bool NextSimplex(GJKSimplex& simplex, Vector3& direction);
        static bool Line(GJKSimplex& simplex, Vector3& direction);
        static bool Triangle(GJKSimplex& simplex, Vector3& direction);
        static bool Tetrahedron(GJKSimplex& simplex, Vector3& direction);
        static bool SameDirection(const Vector3& direction, const Vector3& ao);

        // Used when the usual double-cross-product direction collapses to
        // zero, which happens whenever the origin sits exactly on the
        // current simplex edge/plane (e.g. two shapes centered on the
        // same axis - a very common case, not a rare edge case).
        static Vector3 ArbitraryPerpendicular(const Vector3& v);
    };
} // namespace osseus

#endif // OSSEUSENGINE_GJK_H
