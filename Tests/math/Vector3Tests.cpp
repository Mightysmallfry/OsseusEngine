//
// Created by MightySmallFry on 7/24/2026.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <stdexcept>

#include <Osseus/Osseus.h>

using Catch::Matchers::WithinAbs;


TEST_CASE("Vector3 - Default constructor initializes to zero")
{
    osseus::Vector3 vector;

    REQUIRE(vector.x == 0.0);
    REQUIRE(vector.y == 0.0);
    REQUIRE(vector.z == 0.0);
}


TEST_CASE("Vector3 - Two-dimensional constructor initializes Z to zero")
{
    osseus::Vector3 vector(2.0, 5.0);

    REQUIRE(vector.x == 2.0);
    REQUIRE(vector.y == 5.0);
    REQUIRE(vector.z == 0.0);
}


TEST_CASE("Vector3 - Three-dimensional constructor initializes all components")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    REQUIRE(vector.x == 1.0);
    REQUIRE(vector.y == 2.0);
    REQUIRE(vector.z == 3.0);
}


TEST_CASE("Vector3 - Addition")
{
    osseus::Vector3 a(1.0, 2.0, 3.0);
    osseus::Vector3 b(4.0, 5.0, 6.0);

    osseus::Vector3 result = a + b;

    REQUIRE(result == osseus::Vector3(5.0, 7.0, 9.0));
}


TEST_CASE("Vector3 - Subtraction")
{
    osseus::Vector3 a(5.0, 7.0, 9.0);
    osseus::Vector3 b(1.0, 2.0, 3.0);

    osseus::Vector3 result = a - b;

    REQUIRE(result == osseus::Vector3(4.0, 5.0, 6.0));
}


TEST_CASE("Vector3 - Scalar multiplication")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    osseus::Vector3 result = vector * 2.0;

    REQUIRE(result == osseus::Vector3(2.0, 4.0, 6.0));
}


TEST_CASE("Vector3 - Scalar multiplication is commutative")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    REQUIRE(vector * 2.0 == 2.0 * vector);
}


TEST_CASE("Vector3 - Scalar division")
{
    osseus::Vector3 vector(2.0, 4.0, 6.0);

    osseus::Vector3 result = vector / 2.0;

    REQUIRE(result == osseus::Vector3(1.0, 2.0, 3.0));
}


TEST_CASE("Vector3 - Division by Zero throws")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    REQUIRE_THROWS_AS(
        vector / 0.0,
        std::runtime_error
    );
}


TEST_CASE("Vector3 - Dot product")
{
    osseus::Vector3 a(1.0, 2.0, 3.0);
    osseus::Vector3 b(4.0, 5.0, 6.0);

    REQUIRE(a.Dot(b) == 32.0);
}


TEST_CASE("Vector3 - Dot product of perpendicular vectors is zero")
{
    osseus::Vector3 xAxis(1.0, 0.0, 0.0);
    osseus::Vector3 yAxis(0.0, 1.0, 0.0);

    REQUIRE(xAxis.Dot(yAxis) == 0.0);
}


TEST_CASE("Vector3 - Cross product")
{
    osseus::Vector3 a(1.0, 0.0, 0.0);
    osseus::Vector3 b(0.0, 1.0, 0.0);

    osseus::Vector3 result = a.Cross(b);

    REQUIRE(result == osseus::Vector3(0.0, 0.0, 1.0));
}


TEST_CASE("Vector3 - Cross product is anti-commutative")
{
    osseus::Vector3 a(1.0, 2.0, 3.0);
    osseus::Vector3 b(4.0, 5.0, 6.0);

    REQUIRE(a.Cross(b) == -(b.Cross(a)));
}

TEST_CASE("Vector3 - LengthSquared")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    REQUIRE(vector.LengthSquared() == 14.0);
}


TEST_CASE("Vector3 - Length")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    REQUIRE_THAT(
        vector.Length(),
        WithinAbs(std::sqrt(14.0), 1e-12)
    );
}


TEST_CASE("Vector3 - Normalize produces unit vector")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    vector.Normalize();

    REQUIRE_THAT(
        vector.Length(),
        WithinAbs(1.0, 1e-12)
    );
}


TEST_CASE("Vector3 - Normalized does not modify original")
{
    osseus::Vector3 vector(2.0, 0.0, 0.0);

    osseus::Vector3 normalized = vector.Normalized();

    REQUIRE(vector == osseus::Vector3(2.0, 0.0, 0.0));
    REQUIRE(normalized == osseus::Vector3(1.0, 0.0, 0.0));
}

TEST_CASE("Vector3 - Normalize zero vector remains zero")
{
    osseus::Vector3 vector;

    vector.Normalize();

    REQUIRE(vector == osseus::Vector3::Zero());
}

TEST_CASE("Vector3 - Compound addition")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    vector += osseus::Vector3(4.0, 5.0, 6.0);

    REQUIRE(vector == osseus::Vector3(5.0, 7.0, 9.0));
}


TEST_CASE("Vector3 - Compound subtraction")
{
    osseus::Vector3 vector(5.0, 7.0, 9.0);

    vector -= osseus::Vector3(1.0, 2.0, 3.0);

    REQUIRE(vector == osseus::Vector3(4.0, 5.0, 6.0));
}


TEST_CASE("Vector3 - Compound scalar multiplication")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    vector *= 2.0;

    REQUIRE(vector == osseus::Vector3(2.0, 4.0, 6.0));
}


TEST_CASE("Vector3 - Compound scalar division")
{
    osseus::Vector3 vector(2.0, 4.0, 6.0);

    vector /= 2.0;

    REQUIRE(vector == osseus::Vector3(1.0, 2.0, 3.0));
}


TEST_CASE("Vector3 - Equality operator")
{
    osseus::Vector3 a(1.0, 2.0, 3.0);
    osseus::Vector3 b(1.0, 2.0, 3.0);

    REQUIRE(a == b);
}


TEST_CASE("Vector3 - Inequality operator")
{
    osseus::Vector3 a(1.0, 2.0, 3.0);
    osseus::Vector3 b(1.0, 2.0, 4.0);

    REQUIRE(a != b);
}


TEST_CASE("Vector3 - Zero returns zero vector")
{
    osseus::Vector3 zero = osseus::Vector3::Zero();

    REQUIRE(zero == osseus::Vector3(0.0, 0.0, 0.0));
}