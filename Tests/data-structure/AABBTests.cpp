#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>


TEST_CASE("AABB - Default construction yields zero min and max", "[aabb]")
{
    osseus::AABB box{};

    REQUIRE(box.min.x == 0.0);
    REQUIRE(box.min.y == 0.0);
    REQUIRE(box.min.z == 0.0);

    REQUIRE(box.max.x == 0.0);
    REQUIRE(box.max.y == 0.0);
    REQUIRE(box.max.z == 0.0);
}


TEST_CASE("AABB - Stores the minimum and maximum it was constructed with", "[aabb]")
{
    osseus::AABB box{osseus::Vector3(-1.0, -2.0, -3.0), osseus::Vector3(4.0, 5.0, 6.0)};

    REQUIRE(box.min == osseus::Vector3(-1.0, -2.0, -3.0));
    REQUIRE(box.max == osseus::Vector3(4.0, 5.0, 6.0));
}


TEST_CASE("AABB - Degenerate box collapsed on a single axis stores equal min and max on that axis", "[aabb][degenerate]")
{
    // Zero-thickness slab: collapsed on Y only.
    osseus::AABB box{osseus::Vector3(-1.0, 2.0, -1.0), osseus::Vector3(1.0, 2.0, 1.0)};

    REQUIRE(box.min.y == box.max.y);
    REQUIRE(box.min.x != box.max.x);
    REQUIRE(box.min.z != box.max.z);
}


TEST_CASE("AABB - Point-sized box has identical min and max on every axis", "[aabb][degenerate]")
{
    osseus::AABB box{osseus::Vector3(3.0, -4.0, 5.0), osseus::Vector3(3.0, -4.0, 5.0)};

    REQUIRE(box.min == box.max);
    REQUIRE(box.min == osseus::Vector3(3.0, -4.0, 5.0));
}


TEST_CASE("AABB - Negative coordinates are stored without transformation", "[aabb]")
{
    osseus::AABB box{osseus::Vector3(-100.0, -50.0, -25.0), osseus::Vector3(-10.0, -5.0, -1.0)};

    REQUIRE(box.min == osseus::Vector3(-100.0, -50.0, -25.0));
    REQUIRE(box.max == osseus::Vector3(-10.0, -5.0, -1.0));

    // min is still numerically less than max on every axis even though
    // both are negative.
    REQUIRE(box.min.x < box.max.x);
    REQUIRE(box.min.y < box.max.y);
    REQUIRE(box.min.z < box.max.z);
}


TEST_CASE("AABB - Very large coordinates are stored without precision loss at this magnitude", "[aabb]")
{
    const double large = 1.0e12;

    osseus::AABB box{osseus::Vector3(-large, -large, -large), osseus::Vector3(large, large, large)};

    REQUIRE(box.min == osseus::Vector3(-large, -large, -large));
    REQUIRE(box.max == osseus::Vector3(large, large, large));
}