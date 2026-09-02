#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;

namespace {
    constexpr double kEpsilon = 1e-4;
}

// ==========================================================================
// Boundary cases
// ==========================================================================

TEST_CASE("GJK - Spheres separated by radius minus epsilon are reported as intersecting", "[gjk][boundary]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;

    // Radii sum to 2.0; centers just under that distance apart.
    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(2.0 - kEpsilon, 0.0, 0.0), simplex);

    REQUIRE(hit);
}

TEST_CASE("GJK - Spheres separated by radius plus epsilon are reported as separated", "[gjk][boundary]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    GJKSimplex simplex;

    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(2.0 + kEpsilon, 0.0, 0.0), simplex);

    REQUIRE_FALSE(hit);
}

TEST_CASE("GJK - Cubes separated by half-extent sum minus epsilon are reported as intersecting", "[gjk][boundary]")
{
    ShapeCube a(0.5);
    ShapeCube b(0.5);
    GJKSimplex simplex;

    // Half-extents sum to 1.0; centers just under that distance apart.
    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(1.0 - kEpsilon, 0.0, 0.0), simplex);

    REQUIRE(hit);
}

TEST_CASE("GJK - Cubes separated by half-extent sum plus epsilon are reported as separated", "[gjk][boundary]")
{
    ShapeCube a(0.5);
    ShapeCube b(0.5);
    GJKSimplex simplex;

    bool hit = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(1.0 + kEpsilon, 0.0, 0.0), simplex);

    REQUIRE_FALSE(hit);
}

// ==========================================================================
// Translation invariance
// ==========================================================================

TEST_CASE("GJK - Translating both overlapping shapes by the same vector preserves the collision result", "[gjk][translation]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    const Vector3 shift(37.0, -12.0, 8.5);

    GJKSimplex simplexOriginal;
    bool hitOriginal = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(1.5, 0.0, 0.0), simplexOriginal);

    GJKSimplex simplexShifted;
    bool hitShifted =
        GJK::Intersect(a, Vector3(0.0, 0.0, 0.0) + shift, b, Vector3(1.5, 0.0, 0.0) + shift, simplexShifted);

    REQUIRE(hitOriginal);
    REQUIRE(hitShifted);
}

TEST_CASE("GJK - Translating both separated shapes by the same vector preserves the collision result", "[gjk][translation]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    const Vector3 shift(-40.0, 15.0, 3.0);

    GJKSimplex simplexOriginal;
    bool hitOriginal = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(5.0, 0.0, 0.0), simplexOriginal);

    GJKSimplex simplexShifted;
    bool hitShifted =
        GJK::Intersect(a, Vector3(0.0, 0.0, 0.0) + shift, b, Vector3(5.0, 0.0, 0.0) + shift, simplexShifted);

    REQUIRE_FALSE(hitOriginal);
    REQUIRE_FALSE(hitShifted);
}

// ==========================================================================
// Symmetry: GJK(A, B) must agree with GJK(B, A)
// ==========================================================================

TEST_CASE("GJK - Overlapping spheres report the same result regardless of argument order", "[gjk][symmetry]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);

    GJKSimplex simplexAB;
    GJKSimplex simplexBA;

    bool hitAB = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(1.5, 0.0, 0.0), simplexAB);
    bool hitBA = GJK::Intersect(b, Vector3(1.5, 0.0, 0.0), a, Vector3(0.0, 0.0, 0.0), simplexBA);

    REQUIRE(hitAB == hitBA);
    REQUIRE(hitAB);
}

TEST_CASE("GJK - Separated cubes report the same result regardless of argument order", "[gjk][symmetry]")
{
    ShapeCube a(0.5);
    ShapeCube b(0.5);

    GJKSimplex simplexAB;
    GJKSimplex simplexBA;

    bool hitAB = GJK::Intersect(a, Vector3(0.0, 0.0, 0.0), b, Vector3(5.0, 0.0, 0.0), simplexAB);
    bool hitBA = GJK::Intersect(b, Vector3(5.0, 0.0, 0.0), a, Vector3(0.0, 0.0, 0.0), simplexBA);

    REQUIRE(hitAB == hitBA);
    REQUIRE_FALSE(hitAB);
}

TEST_CASE("GJK - Mismatched shape types (sphere vs cube) report the same result regardless of argument order", "[gjk][symmetry]")
{
    ShapeSphere sphere(1.0);
    ShapeCube cube(0.5);

    GJKSimplex simplexSphereFirst;
    GJKSimplex simplexCubeFirst;

    bool hitSphereFirst =
        GJK::Intersect(sphere, Vector3(0.0, 0.0, 0.0), cube, Vector3(1.0, 0.0, 0.0), simplexSphereFirst);
    bool hitCubeFirst =
        GJK::Intersect(cube, Vector3(1.0, 0.0, 0.0), sphere, Vector3(0.0, 0.0, 0.0), simplexCubeFirst);

    REQUIRE(hitSphereFirst == hitCubeFirst);
}

// ==========================================================================
// Minkowski support
// ==========================================================================

TEST_CASE("GJK - Support returns the Minkowski difference of each shape's own support point", "[gjk][support]")
{
    ShapeSphere a(1.0);
    ShapeSphere b(1.0);
    const Vector3 posA(0.0, 0.0, 0.0);
    const Vector3 posB(3.0, 0.0, 0.0);
    const Vector3 direction = Vector3::UnitX();

    GJKSupportPoint support = GJK::Support(a, posA, b, posB, direction);

    // A's own support along +X, B's support along the opposite (-X)
    // direction (GJK always queries B along -direction internally).
    const Vector3 expectedPointA = a.Support(posA, direction);
    const Vector3 expectedPointB = b.Support(posB, -direction);

    REQUIRE(support.pointA == expectedPointA);
    REQUIRE(support.pointB == expectedPointB);
    REQUIRE(support.point == expectedPointA - expectedPointB);
}

TEST_CASE("GJK - Simplex starts empty before any point is pushed", "[gjk][simplex]")
{
    GJKSimplex simplex;

    REQUIRE(simplex.Size() == 0);
}

TEST_CASE("GJK - Simplex tracks up to four points and keeps the newest at index zero", "[gjk][simplex]")
{
    GJKSimplex simplex;

    GJKSupportPoint p0{Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)};
    GJKSupportPoint p1{Vector3(1.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0)};

    simplex.PushFront(p0);
    REQUIRE(simplex.Size() == 1);
    REQUIRE(simplex[0].point == p0.point);

    simplex.PushFront(p1);
    REQUIRE(simplex.Size() == 2);
    REQUIRE(simplex[0].point == p1.point);
    REQUIRE(simplex[1].point == p0.point);
}
