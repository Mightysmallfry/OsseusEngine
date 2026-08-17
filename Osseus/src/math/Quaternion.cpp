//
// Created by MightySmallFry on 8/2/2026.
//

#include <Osseus/math/Quaternion.h>

#include <cmath>
#include <ostream>

namespace osseus
{

Quaternion::Quaternion()
    : w(1.0), x(0.0), y(0.0), z(0.0)
{
}

Quaternion::Quaternion(double _w, double _x, double _y, double _z)
    : w(_w), x(_x), y(_y), z(_z)
{
}

Quaternion Quaternion::Identity()
{
    return Quaternion(1.0, 0.0, 0.0, 0.0);
}

Quaternion Quaternion::FromAxisAngle(const Vector3& axis, double angleRadians)
{
    constexpr double epsilon = 1e-12;

    double lengthSquared =
        axis.x * axis.x +
        axis.y * axis.y +
        axis.z * axis.z;

    // Zero-length axis means no rotation.
    if (lengthSquared <= epsilon)
    {
        return Identity();
    }

    double inverseLength = 1.0 / std::sqrt(lengthSquared);

    double halfAngle = angleRadians * 0.5;
    double sinHalfAngle = std::sin(halfAngle);

    return Quaternion(
        std::cos(halfAngle),
        axis.x * inverseLength * sinHalfAngle,
        axis.y * inverseLength * sinHalfAngle,
        axis.z * inverseLength * sinHalfAngle
    );
}

double Quaternion::Magnitude() const
{
    return std::sqrt(MagnitudeSquared());
}

double Quaternion::MagnitudeSquared() const
{
    return w * w +
           x * x +
           y * y +
           z * z;
}

void Quaternion::Normalize()
{
    constexpr double epsilon = 1e-12;

    double magnitudeSquared = MagnitudeSquared();

    if (magnitudeSquared <= epsilon)
    {
        *this = Identity();
        return;
    }

    double inverseMagnitude = 1.0 / std::sqrt(magnitudeSquared);

    w *= inverseMagnitude;
    x *= inverseMagnitude;
    y *= inverseMagnitude;
    z *= inverseMagnitude;
}

Quaternion Quaternion::Normalized() const
{
    Quaternion result(*this);
    result.Normalize();
    return result;
}

Quaternion Quaternion::Conjugate() const
{
    return Quaternion(
        w,
        -x,
        -y,
        -z
    );
}

Quaternion Quaternion::Inverse() const
{
    constexpr double epsilon = 1e-12;

    double magnitudeSquared = MagnitudeSquared();

    if (magnitudeSquared <= epsilon)
    {
        return Identity();
    }

    return Conjugate() / magnitudeSquared;
}

double Quaternion::Dot(const Quaternion& other) const
{
    return w * other.w +
           x * other.x +
           y * other.y +
           z * other.z;
}

Quaternion Quaternion::operator+(const Quaternion& other) const
{
    return Quaternion(
        w + other.w,
        x + other.x,
        y + other.y,
        z + other.z
    );
}

Quaternion Quaternion::operator-(const Quaternion& other) const
{
    return Quaternion(
        w - other.w,
        x - other.x,
        y - other.y,
        z - other.z
    );
}

Quaternion Quaternion::operator*(double scalar) const
{
    return Quaternion(
        w * scalar,
        x * scalar,
        y * scalar,
        z * scalar
    );
}

Quaternion Quaternion::operator/(double scalar) const
{
    constexpr double epsilon = 1e-12;

    if (std::abs(scalar) <= epsilon)
    {
        return Identity();
    }

    return Quaternion(
        w / scalar,
        x / scalar,
        y / scalar,
        z / scalar
    );
}

Quaternion& Quaternion::operator+=(const Quaternion& other)
{
    w += other.w;
    x += other.x;
    y += other.y;
    z += other.z;

    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& other)
{
    w -= other.w;
    x -= other.x;
    y -= other.y;
    z -= other.z;

    return *this;
}

Quaternion& Quaternion::operator*=(double scalar)
{
    w *= scalar;
    x *= scalar;
    y *= scalar;
    z *= scalar;

    return *this;
}

Quaternion& Quaternion::operator/=(double scalar)
{
    constexpr double epsilon = 1e-12;

    if (std::abs(scalar) <= epsilon)
    {
        *this = Identity();
        return *this;
    }

    w /= scalar;
    x /= scalar;
    y /= scalar;
    z /= scalar;

    return *this;
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
    return Quaternion(
        w * other.w -
        x * other.x -
        y * other.y -
        z * other.z,

        w * other.x +
        x * other.w +
        y * other.z -
        z * other.y,

        w * other.y -
        x * other.z +
        y * other.w +
        z * other.x,

        w * other.z +
        x * other.y -
        y * other.x +
        z * other.w
    );
}

Quaternion& Quaternion::operator*=(const Quaternion& other)
{
    *this = *this * other;
    return *this;
}

bool Quaternion::operator==(const Quaternion& other) const
{
    constexpr double epsilon = 1e-10;

    return std::abs(w - other.w) <= epsilon &&
           std::abs(x - other.x) <= epsilon &&
           std::abs(y - other.y) <= epsilon &&
           std::abs(z - other.z) <= epsilon;
}

bool Quaternion::operator!=(const Quaternion& other) const
{
    return !(*this == other);
}

bool Quaternion::IsNormalized(double epsilon) const
{
    return std::abs(MagnitudeSquared() - 1.0) <= epsilon;
}

Quaternion operator*(double scalar, const Quaternion& quaternion)
{
    return quaternion * scalar;
}

std::ostream& operator<<(std::ostream& stream, const Quaternion& quaternion)
{
    stream << "Quaternion("
           << quaternion.w << ", "
           << quaternion.x << ", "
           << quaternion.y << ", "
           << quaternion.z
           << ")";

    return stream;
}

} // namespace osseus