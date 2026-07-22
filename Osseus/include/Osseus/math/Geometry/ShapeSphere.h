//
// Created by MightySmallFry on 7/21/2026.
//

#ifndef OSSEUSENGINE_CIRCLE_H
#define OSSEUSENGINE_CIRCLE_H
#include "Osseus/interfaces/IShape.h"

namespace osseus {
    class ShapeSphere : public IShape {
    public:
        explicit ShapeSphere(double _radius) : radius(_radius) {}

        AABB ComputeBoundingBox(const Vector3 &position) const override {
            return AABB{ position - Vector3(radius, radius, radius),
                position + Vector3(radius, radius, radius) };
        }

    private:
        double radius{ 1.0 };
    };
} // osseus

#endif //OSSEUSENGINE_CIRCLE_H
