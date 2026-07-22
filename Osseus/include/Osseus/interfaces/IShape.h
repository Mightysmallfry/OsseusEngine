//
// Created by MightySmallFry on 7/21/2026.
//

#ifndef OSSEUSENGINE_ISHAPE_H
#define OSSEUSENGINE_ISHAPE_H
#include "Osseus/math/Vector3.h"
#include "Osseus/system/AABB.h"

namespace osseus {
    class IShape {
    public:
        virtual ~IShape() = default;
        // Axis-Aligned Bounding-Box
        virtual AABB ComputeBoundingBox(const Vector3& position) const = 0;
    };
} // osseus

#endif //OSSEUSENGINE_ISHAPE_H
