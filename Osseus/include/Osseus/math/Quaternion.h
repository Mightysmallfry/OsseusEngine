//
// Created by MightySmallFry on 8/2/2026
//

#ifndef OSSEUS_QUATERNION_H
#define OSSEUS_QUATERNION_H

#include <iosfwd>

#include "Vector3.h"

namespace osseus
{
    class Quaternion
    {
    public:
        double w;
        double x;
        double y;
        double z;

        // Constructors
        Quaternion();
        Quaternion(double _w, double _x, double _y, double _z);

        // Static factory methods
        static Quaternion Identity();
        static Quaternion FromAxisAngle(const Vector3& axis, double angleRadians);

        // Magnitude
        double Magnitude() const;
        double MagnitudeSquared() const;

        // Normalization
        void Normalize();
        Quaternion Normalized() const;

        // Conjugate & Inverse
        Quaternion Conjugate() const;
        Quaternion Inverse() const;

        // Dot Product
        double Dot(const Quaternion& other) const;

        // Arithmetic
        Quaternion operator+(const Quaternion& other) const;
        Quaternion operator-(const Quaternion& other) const;

        Quaternion operator*(double scalar) const;
        Quaternion operator/(double scalar) const;

        Quaternion& operator+=(const Quaternion& other);
        Quaternion& operator-=(const Quaternion& other);

        Quaternion& operator*=(double scalar);
        Quaternion& operator/=(double scalar);

        // Quaternion multiplication (rotation composition)
        Quaternion operator*(const Quaternion& other) const;
        Quaternion& operator*=(const Quaternion& other);

        // Comparisons
        bool operator==(const Quaternion& other) const;
        bool operator!=(const Quaternion& other) const;

        // Utility
        bool IsNormalized(double epsilon = 1e-8) const;
    };

    // Scalar multiplication
    Quaternion operator*(double scalar, const Quaternion& quaternion);

    // Stream output
    std::ostream& operator<<(std::ostream& stream, const Quaternion& quaternion);

} // namespace osseus

#endif // OSSEUS_QUATERNION_H