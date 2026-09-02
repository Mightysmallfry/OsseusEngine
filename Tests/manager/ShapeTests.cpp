#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using Catch::Matchers::WithinAbs;


// ==================== ShapePoint ====================

TEST_CASE("ShapePoint - Support in +X returns the point's own position", "[shape][point]")
{
    osseus::ShapePoint point;
    const osseus::Vector3 position(3.0, -1.0, 2.0);

    REQUIRE(point.Support(position, osseus::Vector3::UnitX()) == position);
}


TEST_CASE("ShapePoint - Support in -X returns the point's own position", "[shape][point]")
{
    osseus::ShapePoint point;
    const osseus::Vector3 position(3.0, -1.0, 2.0);

    REQUIRE(point.Support(position, -osseus::Vector3::UnitX()) == position);
}


TEST_CASE("ShapePoint - Support in +Y returns the point's own position", "[shape][point]")
{
    osseus::ShapePoint point;
    const osseus::Vector3 position(3.0, -1.0, 2.0);

    REQUIRE(point.Support(position, osseus::Vector3::UnitY()) == position);
}


TEST_CASE("ShapePoint - Support in -Y returns the point's own position", "[shape][point]")
{
    osseus::ShapePoint point;
    const osseus::Vector3 position(3.0, -1.0, 2.0);

    REQUIRE(point.Support(position, -osseus::Vector3::UnitY()) == position);
}


TEST_CASE("ShapePoint - Support in +Z returns the point's own position", "[shape][point]")
{
    osseus::ShapePoint point;
    const osseus::Vector3 position(3.0, -1.0, 2.0);

    REQUIRE(point.Support(position, osseus::Vector3::UnitZ()) == position);
}


TEST_CASE("ShapePoint - Support in -Z returns the point's own position", "[shape][point]")
{
    osseus::ShapePoint point;
    const osseus::Vector3 position(3.0, -1.0, 2.0);

    REQUIRE(point.Support(position, -osseus::Vector3::UnitZ()) == position);
}


TEST_CASE("ShapePoint - Support in an arbitrary direction still returns the point's own position", "[shape][point]")
{
    osseus::ShapePoint point;
    const osseus::Vector3 position(3.0, -1.0, 2.0);

    REQUIRE(point.Support(position, osseus::Vector3(1.0, 1.0, 1.0)) == position);
    REQUIRE(point.Support(position, osseus::Vector3(-5.0, 2.0, 0.3)) == position);
}


TEST_CASE("ShapePoint - Support with a zero direction returns the point's own position", "[shape][point]")
{
    osseus::ShapePoint point;
    const osseus::Vector3 position(3.0, -1.0, 2.0);

    REQUIRE(point.Support(position, osseus::Vector3::Zero()) == position);
}


TEST_CASE("ShapePoint - Bounding box collapses to the point itself", "[shape][point][aabb]")
{
    osseus::ShapePoint point;
    const osseus::Vector3 position(3.0, -1.0, 2.0);

    const osseus::AABB box = point.ComputeBoundingBox(position);

    REQUIRE(box.min == position);
    REQUIRE(box.max == position);
}


// ==================== ShapeSphere ====================

TEST_CASE("ShapeSphere - Radius is stored and retrievable", "[shape][sphere]")
{
    osseus::ShapeSphere sphere(2.5);

    REQUIRE(sphere.GetRadius() == 2.5);
}


TEST_CASE("ShapeSphere - Support in +X sits one radius from center along +X", "[shape][sphere]")
{
    osseus::ShapeSphere sphere(2.0);
    const osseus::Vector3 position(1.0, 1.0, 1.0);

    const osseus::Vector3 support = sphere.Support(position, osseus::Vector3::UnitX());

    REQUIRE(support == position + osseus::Vector3(2.0, 0.0, 0.0));
}


TEST_CASE("ShapeSphere - Support in -X sits one radius from center along -X", "[shape][sphere]")
{
    osseus::ShapeSphere sphere(2.0);
    const osseus::Vector3 position(1.0, 1.0, 1.0);

    const osseus::Vector3 support = sphere.Support(position, -osseus::Vector3::UnitX());

    REQUIRE(support == position + osseus::Vector3(-2.0, 0.0, 0.0));
}


TEST_CASE("ShapeSphere - Support in +Y sits one radius from center along +Y", "[shape][sphere]")
{
    osseus::ShapeSphere sphere(2.0);
    const osseus::Vector3 position(1.0, 1.0, 1.0);

    const osseus::Vector3 support = sphere.Support(position, osseus::Vector3::UnitY());

    REQUIRE(support == position + osseus::Vector3(0.0, 2.0, 0.0));
}


TEST_CASE("ShapeSphere - Support in -Y sits one radius from center along -Y", "[shape][sphere]")
{
    osseus::ShapeSphere sphere(2.0);
    const osseus::Vector3 position(1.0, 1.0, 1.0);

    const osseus::Vector3 support = sphere.Support(position, -osseus::Vector3::UnitY());

    REQUIRE(support == position + osseus::Vector3(0.0, -2.0, 0.0));
}


TEST_CASE("ShapeSphere - Support in +Z sits one radius from center along +Z", "[shape][sphere]")
{
    osseus::ShapeSphere sphere(2.0);
    const osseus::Vector3 position(1.0, 1.0, 1.0);

    const osseus::Vector3 support = sphere.Support(position, osseus::Vector3::UnitZ());

    REQUIRE(support == position + osseus::Vector3(0.0, 0.0, 2.0));
}


TEST_CASE("ShapeSphere - Support in -Z sits one radius from center along -Z", "[shape][sphere]")
{
    osseus::ShapeSphere sphere(2.0);
    const osseus::Vector3 position(1.0, 1.0, 1.0);

    const osseus::Vector3 support = sphere.Support(position, -osseus::Vector3::UnitZ());

    REQUIRE(support == position + osseus::Vector3(0.0, 0.0, -2.0));
}


TEST_CASE("ShapeSphere - Support in an arbitrary direction lies exactly one radius from center", "[shape][sphere]")
{
    osseus::ShapeSphere sphere(3.0);
    const osseus::Vector3 position(5.0, -2.0, 0.0);
    const osseus::Vector3 direction(1.0, 2.0, -3.0);

    const osseus::Vector3 support = sphere.Support(position, direction);
    const double distanceFromCenter = (support - position).Length();

    REQUIRE_THAT(distanceFromCenter, WithinAbs(3.0, 1e-9));

    // Support direction from center must be parallel (same unit vector) to
    // the requested direction.
    const osseus::Vector3 unitDirection = direction.Normalized();
    const osseus::Vector3 unitSupportOffset = (support - position).Normalized();
    REQUIRE_THAT(unitSupportOffset.x, WithinAbs(unitDirection.x, 1e-9));
    REQUIRE_THAT(unitSupportOffset.y, WithinAbs(unitDirection.y, 1e-9));
    REQUIRE_THAT(unitSupportOffset.z, WithinAbs(unitDirection.z, 1e-9));
}


TEST_CASE("ShapeSphere - Support with a zero direction falls back to +X rather than NaN", "[shape][sphere][regression]")
{
    osseus::ShapeSphere sphere(4.0);
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    const osseus::Vector3 support = sphere.Support(position, osseus::Vector3::Zero());

    REQUIRE(support == position + osseus::Vector3(4.0, 0.0, 0.0));
}


TEST_CASE("ShapeSphere - Support translates with the body's position", "[shape][sphere]")
{
    osseus::ShapeSphere sphere(1.0);
    const osseus::Vector3 positionA(0.0, 0.0, 0.0);
    const osseus::Vector3 positionB(10.0, -5.0, 3.0);

    const osseus::Vector3 supportA = sphere.Support(positionA, osseus::Vector3::UnitX());
    const osseus::Vector3 supportB = sphere.Support(positionB, osseus::Vector3::UnitX());

    REQUIRE(supportB == supportA + (positionB - positionA));
}


TEST_CASE("ShapeSphere - Different radii produce proportionally different support distances", "[shape][sphere]")
{
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    osseus::ShapeSphere small(1.0);
    osseus::ShapeSphere large(5.0);

    const double smallDistance = (small.Support(position, osseus::Vector3::UnitX()) - position).Length();
    const double largeDistance = (large.Support(position, osseus::Vector3::UnitX()) - position).Length();

    REQUIRE_THAT(smallDistance, WithinAbs(1.0, 1e-9));
    REQUIRE_THAT(largeDistance, WithinAbs(5.0, 1e-9));
}


TEST_CASE("ShapeSphere - Bounding box extends exactly one radius on every side of center", "[shape][sphere][aabb]")
{
    osseus::ShapeSphere sphere(2.5);
    const osseus::Vector3 position(1.0, 2.0, 3.0);

    const osseus::AABB box = sphere.ComputeBoundingBox(position);

    REQUIRE(box.min == position - osseus::Vector3(2.5, 2.5, 2.5));
    REQUIRE(box.max == position + osseus::Vector3(2.5, 2.5, 2.5));
}


// ==================== ShapeCube ====================

TEST_CASE("ShapeCube - Half-extent is stored and retrievable", "[shape][cube]")
{
    osseus::ShapeCube cube(1.5);

    REQUIRE(cube.GetHalfExtent() == 1.5);
}


TEST_CASE("ShapeCube - Support along +X selects the +X face corner", "[shape][cube]")
{
    osseus::ShapeCube cube(1.0);
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    const osseus::Vector3 support = cube.Support(position, osseus::Vector3::UnitX());

    REQUIRE(support == osseus::Vector3(1.0, 1.0, 1.0));
}


TEST_CASE("ShapeCube - Support along -X selects the -X face corner", "[shape][cube]")
{
    osseus::ShapeCube cube(1.0);
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    const osseus::Vector3 support = cube.Support(position, -osseus::Vector3::UnitX());

    REQUIRE(support == osseus::Vector3(-1.0, 1.0, 1.0));
}


TEST_CASE("ShapeCube - Support along +Y selects the +Y face corner", "[shape][cube]")
{
    osseus::ShapeCube cube(1.0);
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    const osseus::Vector3 support = cube.Support(position, osseus::Vector3::UnitY());

    REQUIRE(support == osseus::Vector3(1.0, 1.0, 1.0));
}


TEST_CASE("ShapeCube - Support along -Y selects the -Y face corner", "[shape][cube]")
{
    osseus::ShapeCube cube(1.0);
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    const osseus::Vector3 support = cube.Support(position, -osseus::Vector3::UnitY());

    REQUIRE(support == osseus::Vector3(1.0, -1.0, 1.0));
}


TEST_CASE("ShapeCube - Support along +Z selects the +Z face corner", "[shape][cube]")
{
    osseus::ShapeCube cube(1.0);
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    const osseus::Vector3 support = cube.Support(position, osseus::Vector3::UnitZ());

    REQUIRE(support == osseus::Vector3(1.0, 1.0, 1.0));
}


TEST_CASE("ShapeCube - Support along -Z selects the -Z face corner", "[shape][cube]")
{
    osseus::ShapeCube cube(1.0);
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    const osseus::Vector3 support = cube.Support(position, -osseus::Vector3::UnitZ());

    REQUIRE(support == osseus::Vector3(1.0, 1.0, -1.0));
}


TEST_CASE("ShapeCube - Support along a diagonal direction selects the matching diagonal corner", "[shape][cube]")
{
    osseus::ShapeCube cube(1.0);
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    // Every-axis-negative diagonal should select the (-,-,-) corner.
    const osseus::Vector3 support = cube.Support(position, osseus::Vector3(-1.0, -1.0, -1.0));

    REQUIRE(support == osseus::Vector3(-1.0, -1.0, -1.0));
}


TEST_CASE("ShapeCube - Support handles negative-direction components independently per axis", "[shape][cube]")
{
    osseus::ShapeCube cube(1.0);
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    // Mixed-sign direction: +X, -Y, +Z.
    const osseus::Vector3 support = cube.Support(position, osseus::Vector3(1.0, -1.0, 1.0));

    REQUIRE(support == osseus::Vector3(1.0, -1.0, 1.0));
}


TEST_CASE("ShapeCube - Support translates with the body's position", "[shape][cube]")
{
    osseus::ShapeCube cube(1.0);
    const osseus::Vector3 position(5.0, -3.0, 2.0);

    const osseus::Vector3 support = cube.Support(position, osseus::Vector3::UnitX());

    REQUIRE(support == osseus::Vector3(6.0, -2.0, 3.0));
}


TEST_CASE("ShapeCube - Different half-extents scale the support point accordingly", "[shape][cube]")
{
    const osseus::Vector3 position(0.0, 0.0, 0.0);

    osseus::ShapeCube small(0.5);
    osseus::ShapeCube large(3.0);

    REQUIRE(small.Support(position, osseus::Vector3::UnitX()).x == 0.5);
    REQUIRE(large.Support(position, osseus::Vector3::UnitX()).x == 3.0);
}


TEST_CASE("ShapeCube - Bounding box extends exactly the half-extent on every side of center", "[shape][cube][aabb]")
{
    osseus::ShapeCube cube(1.5);
    const osseus::Vector3 position(1.0, 2.0, 3.0);

    const osseus::AABB box = cube.ComputeBoundingBox(position);

    REQUIRE(box.min == position - osseus::Vector3(1.5, 1.5, 1.5));
    REQUIRE(box.max == position + osseus::Vector3(1.5, 1.5, 1.5));
}
