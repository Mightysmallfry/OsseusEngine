//
// Created by MightySmallFry on 7/24/2026.
//
#include <catch2/catch_test_macros.hpp>

#include <Osseus/math/Vector3.h>


TEST_CASE("Default constructor initializes to zero")
{
    osseus::Vector3 vector;

    REQUIRE(vector.x == 0.0);
    REQUIRE(vector.y == 0.0);
    REQUIRE(vector.z == 0.0);
}


TEST_CASE("Two-dimensional constructor initializes Z to zero")
{
    osseus::Vector3 vector(2.0, 5.0);

    REQUIRE(vector.x == 2.0);
    REQUIRE(vector.y == 5.0);
    REQUIRE(vector.z == 0.0);
}


TEST_CASE("Three-dimensional constructor initializes all components")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    REQUIRE(vector.x == 1.0);
    REQUIRE(vector.y == 2.0);
    REQUIRE(vector.z == 3.0);
}


TEST_CASE("Vector addition")
{
    osseus::Vector3 a(1.0, 2.0, 3.0);
    osseus::Vector3 b(4.0, 5.0, 6.0);

    osseus::Vector3 result = a + b;

    REQUIRE(result.x == 5.0);
    REQUIRE(result.y == 7.0);
    REQUIRE(result.z == 9.0);
}


TEST_CASE("Vector subtraction")
{
    osseus::Vector3 a(5.0, 7.0, 9.0);
    osseus::Vector3 b(1.0, 2.0, 3.0);

    osseus::Vector3 result = a - b;

    REQUIRE(result.x == 4.0);
    REQUIRE(result.y == 5.0);
    REQUIRE(result.z == 6.0);
}


TEST_CASE("Scalar multiplication")
{
    osseus::Vector3 vector(1.0, 2.0, 3.0);

    osseus::Vector3 result = vector * 2.0;

    REQUIRE(result.x == 2.0);
    REQUIRE(result.y == 4.0);
    REQUIRE(result.z == 6.0);
}

TEST_CASE("Dot product")
{
    osseus::Vector3 a(1.0, 2.0, 3.0);
    osseus::Vector3 b(4.0, 5.0, 6.0);

    REQUIRE(a.Dot(b) == 32.0);
}


TEST_CASE("Cross product")
{
    osseus::Vector3 a(1.0, 0.0, 0.0);
    osseus::Vector3 b(0.0, 1.0, 0.0);

    osseus::Vector3 result = a.Cross(b);

    REQUIRE(result.x == 0.0);
    REQUIRE(result.y == 0.0);
    REQUIRE(result.z == 1.0);
}