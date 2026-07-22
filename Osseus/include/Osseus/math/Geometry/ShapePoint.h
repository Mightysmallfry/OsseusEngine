//
// Created by MightySmallFry on 7/21/2026.
//

#ifndef OSSEUSENGINE_SHAPEPOINT_H
#define OSSEUSENGINE_SHAPEPOINT_H
#include "Osseus/interfaces/IShape.h"

namespace osseus {
    class ShapePoint : public IShape{
    public:
        AABB ComputeBoundingBox(const Vector3& position) const override {
            return AABB{ position, position };
        }
    };
} // osseus

#endif //OSSEUSENGINE_SHAPEPOINT_H
