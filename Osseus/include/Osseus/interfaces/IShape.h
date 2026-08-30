#ifndef OSSEUSENGINE_ISHAPE_H
#define OSSEUSENGINE_ISHAPE_H

#include "Osseus/math/Vector3.h"
#include "Osseus/system/AABB.h"

namespace osseus {
    // Every convex shape in Osseus implements this interface. Adding a
    // brand new shape (capsule, cylinder, convex hull, ...) only ever
    // requires a ComputeBoundingBox and a Support function here -
    // GJK, EPA, and the narrow phase never need to know it exists.
    class IShape {
        public:
        virtual ~IShape() = default;

        // Axis-Aligned Bounding-Box, used by the broad phase.
        virtual AABB ComputeBoundingBox(const Vector3& position) const = 0;

        // Returns the point on the shape's surface (in world space, given
        // the body's world-space position) that is furthest along
        // `direction`. This single function is all GJK/EPA need to test
        // and resolve collisions against any other IShape.
        virtual Vector3 Support(const Vector3& position, const Vector3& direction) const = 0;
    };
} // namespace osseus

#endif // OSSEUSENGINE_ISHAPE_H
