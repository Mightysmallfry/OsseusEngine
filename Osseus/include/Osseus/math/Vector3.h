//
// Created by MightySmallFry on 7/16/2026.
//

#ifndef OSSEUSENGINE_VECTOR3_H
#define OSSEUSENGINE_VECTOR3_H

#include <algorithm>
#include <cmath>
#include <iosfwd>
#include <stdexcept>
#include <string>

namespace osseus {

    class Vector3 {
        public:
        double x;
        double y;
        double z;

        static constexpr double TOLERANCE = 1e-9;

        // ==================== Constructors ====================

        constexpr Vector3() noexcept : x(0.0), y(0.0), z(0.0) {
        }

        constexpr Vector3(double x, double y) noexcept : x(x), y(y), z(0.0) {
        }

        constexpr Vector3(double x, double y, double z) noexcept : x(x), y(y), z(z) {
        }

        // Rule of zero: trivial members, compiler-generated copy/move/dtor
        // are already correct and already constexpr-eligible.

        // ==================== Named constants ====================

        static constexpr Vector3 Zero() {
            return {0.0, 0.0, 0.0};
        }
        static constexpr Vector3 One() {
            return {1.0, 1.0, 1.0};
        }
        static constexpr Vector3 UnitX() {
            return {1.0, 0.0, 0.0};
        }
        static constexpr Vector3 UnitY() {
            return {0.0, 1.0, 0.0};
        }
        static constexpr Vector3 UnitZ() {
            return {0.0, 0.0, 1.0};
        }

        // ==================== Arithmetic operators ====================

        [[nodiscard]] constexpr Vector3 operator+(const Vector3& other) const noexcept {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }
        constexpr Vector3& operator+=(const Vector3& other) noexcept {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        [[nodiscard]] constexpr Vector3 operator-(const Vector3& other) const noexcept {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }
        constexpr Vector3& operator-=(const Vector3& other) noexcept {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        [[nodiscard]] constexpr Vector3 operator*(double scalar) const noexcept {
            return Vector3(x * scalar, y * scalar, z * scalar);
        }
        [[nodiscard]] friend constexpr Vector3 operator*(double scalar, const Vector3& v) noexcept {
            return v * scalar;
        }
        constexpr Vector3& operator*=(double scalar) noexcept {
            x *= scalar;
            y *= scalar;
            z *= scalar;
            return *this;
        }

        // Policy: division by zero produces inf/nan (IEEE 754 behavior),
        // Callers who need strict validation should check
        // the scalar themselves before dividing.
        [[nodiscard]] Vector3 operator/(double scalar) const {
            if (std::abs(scalar) < 1e-12) {
                throw std::runtime_error("Cannot divide Vector3 by zero.");
            }

            return Vector3(x / scalar, y / scalar, z / scalar);
        }

        constexpr Vector3& operator/=(double scalar) noexcept {
            x /= scalar;
            y /= scalar;
            z /= scalar;
            return *this;
        }

        [[nodiscard]] constexpr Vector3 operator-() const noexcept {
            return Vector3(-x, -y, -z);
        }
        constexpr void Reverse() noexcept {
            x = -x;
            y = -y;
            z = -z;
        }

        // ==================== Comparison operators ====================

        // Fuzzy equality — see RelDiff below.
        [[nodiscard]] constexpr bool operator==(const Vector3& other) const noexcept {
            return RelDiff(x, other.x) < TOLERANCE && RelDiff(y, other.y) < TOLERANCE &&
                   RelDiff(z, other.z) < TOLERANCE;
        }
        [[nodiscard]] constexpr bool operator!=(const Vector3& other) const noexcept {
            return !(*this == other);
        }

        // ==================== Unary functions ====================

        [[nodiscard]] constexpr double LengthSquared() const noexcept {
            return x * x + y * y + z * z;
        }

        // Not constexpr: std::sqrt isn't constexpr until C++26 on most
        // standard libraries. Defined in Vector3.cpp.
        [[nodiscard]] double Length() const noexcept;

        // Not constexpr, same reason (calls Length()). Defined in Vector3.cpp.
        void Normalize() noexcept;

        // Returns a new unit vector in the direction of this vector,
        // without modifying it. Equivalent to NormalizedVector(*this)
        // but callable as v.Normalized() for chaining.
        // Not constexpr: routes through Normalize() -> Length() -> std::sqrt.
        [[nodiscard]] Vector3 Normalized() const noexcept;

        [[nodiscard]] constexpr double Dot(const Vector3& other) const noexcept {
            return (x * other.x) + (y * other.y) + (z * other.z);
        }

        // Fuzzy comparison, consistent with operator== above — exact
        // `== 0` would fail for legitimately orthogonal vectors carrying
        // floating-point noise.
        [[nodiscard]] constexpr bool IsOrthogonalTo(const Vector3& other) const noexcept {
            return std::abs(Dot(other)) < TOLERANCE;
        }

        [[nodiscard]] constexpr Vector3 Cross(const Vector3& other) const noexcept {
            return Vector3(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
        }

        // Not constexpr: std::stringstream isn't constexpr-friendly.
        // Defined in Vector3.cpp.
        [[nodiscard]] std::string ToString() const;

        private:
        // Relative Difference — used for fuzzy floating-point equality.
        // Ref. Knuth Sec. 4.2.2 pp. 217-8
        [[nodiscard]] static constexpr double RelDiff(double a, double b) noexcept {
            const double diff = std::max(std::abs(a), std::abs(b));
            return diff == 0.0 ? 0.0 : std::abs(a - b) / diff;
        }
    };

    // ==================== Free functions ====================

    [[nodiscard]] constexpr Vector3 CreateVector(const Vector3& start, const Vector3& end) noexcept {
        return end - start;
    }

    [[nodiscard]] constexpr double DistanceSquared(const Vector3& v1, const Vector3& v2) noexcept {
        return (v1 - v2).LengthSquared();
    }

    // Not constexpr: routes through Length() -> std::sqrt.
    [[nodiscard]] double Distance(const Vector3& v1, const Vector3& v2) noexcept;

    // Use this function to get a unit vector of the parameter without
    // changing the parameter. Not constexpr: routes through Normalize().
    [[nodiscard]] Vector3 NormalizedVector(const Vector3& vector) noexcept;

    // Not constexpr: routes through ToString() -> std::stringstream.
    std::ostream& operator<<(std::ostream& os, const Vector3& vector);

} // namespace osseus

#endif // OSSEUSENGINE_VECTOR3_H