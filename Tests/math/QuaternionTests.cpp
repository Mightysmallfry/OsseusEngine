//
// Created by MightySmallFry on 8/2/2026.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <stdexcept>

#include <Osseus/Osseus.h>

using Catch::Matchers::WithinAbs;


TEST_CASE("Quaternion - Default constructor creates identity quaternion")
{
    osseus::Quaternion q;

    REQUIRE(q.w == 1.0);
    REQUIRE(q.x == 0.0);
    REQUIRE(q.y == 0.0);
    REQUIRE(q.z == 0.0);

    REQUIRE(q.IsNormalized());
}


TEST_CASE("Quaternion - Parameterized constructor stores values")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    REQUIRE(q.w == 1.0);
    REQUIRE(q.x == 2.0);
    REQUIRE(q.y == 3.0);
    REQUIRE(q.z == 4.0);
}


TEST_CASE("Quaternion - Identity returns identity quaternion")
{
    osseus::Quaternion q = osseus::Quaternion::Identity();

    REQUIRE(q == osseus::Quaternion(1.0, 0.0, 0.0, 0.0));
}


TEST_CASE("Quaternion - MagnitudeSquared computes correctly")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    REQUIRE(q.MagnitudeSquared() == 30.0);
}


TEST_CASE("Quaternion - Magnitude computes correctly")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    REQUIRE_THAT(
        q.Magnitude(),
        WithinAbs(std::sqrt(30.0), 1e-12)
    );
}


TEST_CASE("Quaternion - Normalize produces unit quaternion")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    q.Normalize();

    REQUIRE_THAT(q.Magnitude(), WithinAbs(1.0, 1e-12));
    REQUIRE(q.IsNormalized());
}


TEST_CASE("Quaternion - Normalized does not modify original")
{
    osseus::Quaternion q(2.0, 0.0, 0.0, 0.0);

    osseus::Quaternion normalized = q.Normalized();

    REQUIRE(q.w == 2.0);

    REQUIRE(normalized.w == 1.0);
    REQUIRE(normalized.x == 0.0);
    REQUIRE(normalized.y == 0.0);
    REQUIRE(normalized.z == 0.0);
}

TEST_CASE("Quaternion - Conjugate negates imaginary components")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    osseus::Quaternion conjugate = q.Conjugate();

    REQUIRE(conjugate.w == 1.0);
    REQUIRE(conjugate.x == -2.0);
    REQUIRE(conjugate.y == -3.0);
    REQUIRE(conjugate.z == -4.0);
}


TEST_CASE("Quaternion - Inverse multiplied by quaternion equals identity")
{
    osseus::Quaternion q(2.0, 3.0, 4.0, 5.0);

    osseus::Quaternion result = q * q.Inverse();

    REQUIRE_THAT(result.w, WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(result.x, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(result.y, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(result.z, WithinAbs(0.0, 1e-12));
}


TEST_CASE("Quaternion - Dot product")
{
    osseus::Quaternion a(1.0, 2.0, 3.0, 4.0);
    osseus::Quaternion b(5.0, 6.0, 7.0, 8.0);

    REQUIRE(a.Dot(b) == 70.0);
}


TEST_CASE("Quaternion - Addition")
{
    osseus::Quaternion a(1.0, 2.0, 3.0, 4.0);
    osseus::Quaternion b(5.0, 6.0, 7.0, 8.0);

    REQUIRE(
        a + b ==
        osseus::Quaternion(6.0, 8.0, 10.0, 12.0)
    );
}


TEST_CASE("Quaternion - Subtraction")
{
    osseus::Quaternion a(5.0, 6.0, 7.0, 8.0);
    osseus::Quaternion b(1.0, 2.0, 3.0, 4.0);

    REQUIRE(
        a - b ==
        osseus::Quaternion(4.0, 4.0, 4.0, 4.0)
    );
}


TEST_CASE("Quaternion - Scalar multiplication")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    REQUIRE(
        q * 2.0 ==
        osseus::Quaternion(2.0, 4.0, 6.0, 8.0)
    );
}


TEST_CASE("Quaternion - Scalar multiplication is commutative")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    REQUIRE(q * 3.0 == 3.0 * q);
}


TEST_CASE("Quaternion - Scalar division")
{
    osseus::Quaternion q(2.0, 4.0, 6.0, 8.0);

    REQUIRE(
        q / 2.0 ==
        osseus::Quaternion(1.0, 2.0, 3.0, 4.0)
    );
}



TEST_CASE("Quaternion - Compound addition")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    q += osseus::Quaternion(4.0, 3.0, 2.0, 1.0);

    REQUIRE(q == osseus::Quaternion(5.0, 5.0, 5.0, 5.0));
}


TEST_CASE("Quaternion - Compound subtraction")
{
    osseus::Quaternion q(5.0, 5.0, 5.0, 5.0);

    q -= osseus::Quaternion(1.0, 2.0, 3.0, 4.0);

    REQUIRE(q == osseus::Quaternion(4.0, 3.0, 2.0, 1.0));
}


TEST_CASE("Quaternion - Compound scalar multiplication")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    q *= 3.0;

    REQUIRE(q == osseus::Quaternion(3.0, 6.0, 9.0, 12.0));
}


TEST_CASE("Quaternion - Compound scalar division")
{
    osseus::Quaternion q(3.0, 6.0, 9.0, 12.0);

    q /= 3.0;

    REQUIRE(q == osseus::Quaternion(1.0, 2.0, 3.0, 4.0));
}


TEST_CASE("Quaternion - Multiplication with identity")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    REQUIRE(q * osseus::Quaternion::Identity() == q);
    REQUIRE(osseus::Quaternion::Identity() * q == q);
}


TEST_CASE("Quaternion - Multiplication is not commutative")
{
    osseus::Quaternion a(1.0, 2.0, 3.0, 4.0);
    osseus::Quaternion b(5.0, 6.0, 7.0, 8.0);

    REQUIRE((a * b) != (b * a));
}


TEST_CASE("Quaternion - Multiplication is associative")
{
    osseus::Quaternion a(1.0, 2.0, 3.0, 4.0);
    osseus::Quaternion b(2.0, 3.0, 4.0, 5.0);
    osseus::Quaternion c(3.0, 4.0, 5.0, 6.0);

    osseus::Quaternion left = (a * b) * c;
    osseus::Quaternion right = a * (b * c);

    REQUIRE_THAT(left.w, WithinAbs(right.w, 1e-10));
    REQUIRE_THAT(left.x, WithinAbs(right.x, 1e-10));
    REQUIRE_THAT(left.y, WithinAbs(right.y, 1e-10));
    REQUIRE_THAT(left.z, WithinAbs(right.z, 1e-10));
}


TEST_CASE("Quaternion - Conjugate property")
{
    osseus::Quaternion q(1.0, 2.0, 3.0, 4.0);

    osseus::Quaternion result = q * q.Conjugate();

    REQUIRE_THAT(result.x, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(result.y, WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(result.z, WithinAbs(0.0, 1e-12));

    REQUIRE_THAT(
        result.w,
        WithinAbs(q.MagnitudeSquared(), 1e-12)
    );
}


TEST_CASE("Quaternion - Equality operator")
{
    osseus::Quaternion a(1.0, 2.0, 3.0, 4.0);
    osseus::Quaternion b(1.0, 2.0, 3.0, 4.0);

    REQUIRE(a == b);
}


TEST_CASE("Quaternion - Inequality operator")
{
    osseus::Quaternion a(1.0, 2.0, 3.0, 4.0);
    osseus::Quaternion b(1.0, 2.0, 3.0, 5.0);

    REQUIRE(a != b);
}


TEST_CASE("Quaternion - Identity quaternion is normalized")
{
    REQUIRE(osseus::Quaternion::Identity().IsNormalized());
}


TEST_CASE("Quaternion - Random normalized quaternion remains normalized")
{
    osseus::Quaternion q(0.3, -1.5, 7.2, 4.1);

    osseus::Quaternion normalized = q.Normalized();

    REQUIRE(normalized.IsNormalized());
}

TEST_CASE("Quaternion - Division by zero returns identity fallback")
{
    osseus::Quaternion q(2.0, 4.0, 6.0, 8.0);

    osseus::Quaternion result = q / 0.0;

    REQUIRE(result == osseus::Quaternion::Identity());
}


TEST_CASE("Quaternion - Division by near zero returns identity fallback")
{
    osseus::Quaternion q(2.0, 4.0, 6.0, 8.0);

    osseus::Quaternion result = q / 1e-15;

    REQUIRE(result == osseus::Quaternion::Identity());
}


TEST_CASE("Quaternion - Divide assignment by zero returns identity fallback")
{
    osseus::Quaternion q(2.0, 4.0, 6.0, 8.0);

    q /= 0.0;

    REQUIRE(q == osseus::Quaternion::Identity());
}


TEST_CASE("Quaternion - Inverse of zero quaternion returns identity fallback")
{
    osseus::Quaternion q(0.0, 0.0, 0.0, 0.0);

    osseus::Quaternion inverse = q.Inverse();

    REQUIRE(inverse == osseus::Quaternion::Identity());
}


TEST_CASE("Quaternion - Normalize zero quaternion returns identity fallback")
{
    osseus::Quaternion q(0.0, 0.0, 0.0, 0.0);

    q.Normalize();

    REQUIRE(q == osseus::Quaternion::Identity());
}


TEST_CASE("Quaternion - Valid quaternion normalization remains normalized")
{
    osseus::Quaternion q(2.0, 0.0, 0.0, 0.0);

    osseus::Quaternion normalized = q.Normalized();

    REQUIRE(normalized.IsNormalized());
}