#ifndef OSSEUSENGINE_CIRCLE_H
#define OSSEUSENGINE_CIRCLE_H

#include <cmath>

#include "Osseus/interfaces/IShape.h"

namespace osseus {
    class ShapeSphere : public IShape {
        public:
        explicit ShapeSphere(double _radius) : radius_(_radius) {
        }

        AABB ComputeBoundingBox(const Vector3& position) const override {
            return AABB{position - Vector3(radius_, radius_, radius_), position + Vector3(radius_, radius_, radius_)};
        }

        // Furthest point on a sphere along `direction` is just the
        // center pushed out by one radius along that direction.
        Vector3 Support(const Vector3& position, const Vector3& direction) const override {
            const double lengthSq = direction.LengthSquared();
            const Vector3 unitDirection =
                (lengthSq > Vector3::TOLERANCE) ? direction * (1.0 / std::sqrt(lengthSq)) : Vector3::UnitX();
            return position + unitDirection * radius_;
        }

        double GetRadius() const {
            return radius_;
        }

        private:
        double radius_{1.0};
    };
} // namespace osseus

#endif // OSSEUSENGINE_CIRCLE_H
