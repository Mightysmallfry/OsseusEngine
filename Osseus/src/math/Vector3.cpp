//
// Created by MightySmallFry on 7/16/2026.
//

#include "../../include/Osseus/math/Vector3.h"

#include <cmath>
#include <ostream>
#include <sstream>

namespace osseus {

    // ==================== Unary functions ====================

    // Returns the magnitude or length of a vector.
    // Not constexpr: std::sqrt isn't constexpr on most standard
    // libraries yet (C++26 territory).
    double Vector3::Length() const noexcept {
        return std::sqrt(LengthSquared());
    }

    // Turns a vector into a unit vector.
    // Divides each component by the vector magnitude.
    void Vector3::Normalize() noexcept {
        double length = Length();

        if (length <= TOLERANCE) {
            x = 0.0;
            y = 0.0;
            z = 0.0;
            return;
        }

        x /= length;
        y /= length;
        z /= length;
    }

    // Returns a new unit vector in the direction of this vector,
    // without modifying it.
    Vector3 Vector3::Normalized() const noexcept {
        Vector3 result = *this;
        result.Normalize();
        return result;
    }

    // Return a string with a vector formatted as [X, Y, Z]
    std::string Vector3::ToString() const {
        std::stringstream ss;
        ss << "[" << x << ", " << y << ", " << z << "]";
        return ss.str();
    }

    // ==================== Free functions ====================

    // Returns the distance between two points.
    double Distance(const Vector3& v1, const Vector3& v2) noexcept {
        return (v1 - v2).Length();
    }

    // Returns a unit vector in the direction of the parameter, without
    // mutating the parameter itself.
    Vector3 NormalizedVector(const Vector3& vector) noexcept {
        return vector.Normalized();
    }

    // Inserts a formatted vector into an output stream.
    std::ostream& operator<<(std::ostream& os, const Vector3& vector) {
        return os << "Vector: " << vector.ToString();
    }

} // namespace osseus