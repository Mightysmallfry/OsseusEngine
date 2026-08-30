#include <algorithm>
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

namespace {

    // ========================================================================
    // Construction
    // ========================================================================

    TEST_CASE(
        "Octree - Default construction creates an empty root",
        "[Octree][Construction]"
    )
    {
        osseus::Octree tree;

        const osseus::OctNode& root = tree.GetRoot();

        REQUIRE(root.IsLeaf());
        REQUIRE(root.IsEmpty());
        REQUIRE(root.GetBodyCount() == 0);
        REQUIRE(root.GetTotalMass() == 0.0);
        REQUIRE(root.GetCenterOfMass() == osseus::Vector3::Zero());
        REQUIRE(root.GetBounds().center == osseus::Vector3::Zero());
        REQUIRE(root.GetBounds().halfSize == osseus::Vector3::One());
    }

    TEST_CASE(
        "Octree - Explicit bounds are assigned to the root",
        "[Octree][Construction]"
    )
    {
        const osseus::Bounds bounds{
            osseus::Vector3(1.0, 2.0, 3.0),
            osseus::Vector3(10.0, 20.0, 30.0)
        };

        osseus::Octree tree(bounds);

        const osseus::Bounds& actual = tree.GetRoot().GetBounds();

        REQUIRE(actual.center == bounds.center);
        REQUIRE(actual.halfSize == bounds.halfSize);
    }

    TEST_CASE(
        "Octree - Root starts without children",
        "[Octree][Construction][OctNode]"
    )
    {
        osseus::Octree tree;

        const osseus::OctNode& root = tree.GetRoot();

        for (std::size_t index = 0; index < 8; ++index)
        {
            REQUIRE_FALSE(root.HasChild(index));
            REQUIRE(root.GetChild(index) == nullptr);
        }
    }

    TEST_CASE(
        "Octree - Invalid child index throws",
        "[Octree][Construction][OctNode][Validation]"
    )
    {
        osseus::Octree tree;

        const osseus::OctNode& root = tree.GetRoot();

        REQUIRE_THROWS(root.GetChild(8));
        REQUIRE_THROWS(root.HasChild(8));
    }

    TEST_CASE(
        "Octree - Root has no parent",
        "[Octree][Parent]"
    )
    {
        osseus::Octree tree;

        const osseus::OctNode& root = tree.GetRoot();

        REQUIRE(root.GetParent() == nullptr);
        REQUIRE(root.GetDepth() == 0);
    }

    // ========================================================================
    // Insertion
    // ========================================================================

    TEST_CASE(
        "Octree - Inserting a body makes the tree non-empty",
        "[Octree][Insert]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(1.0, 1.0, 1.0),
            1.0,
            0.0
        );

        const osseus::OctNode& root = tree.GetRoot();

        REQUIRE_FALSE(root.IsEmpty());
        REQUIRE(root.GetBodyCount() == 1);
    }

    TEST_CASE(
        "Octree - Multiple bodies are counted",
        "[Octree][Insert]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(100.0, 100.0, 100.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(10.0, 10.0, 10.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-10.0, 10.0, 10.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{2, 0},
            osseus::Vector3(10.0, -10.0, 10.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{3, 0},
            osseus::Vector3(10.0, 10.0, -10.0),
            1.0,
            0.0
        );

        REQUIRE(tree.GetRoot().GetBodyCount() == 4);
    }

    TEST_CASE(
        "Octree - Zero mass bodies are excluded",
        "[Octree][Insert][Static]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(1.0, 1.0, 1.0),
            0.0,
            0.0
        );

        REQUIRE(tree.GetRoot().IsEmpty());
        REQUIRE(tree.GetRoot().GetBodyCount() == 0);
        REQUIRE(tree.GetRoot().GetTotalMass() == 0.0);
    }

    TEST_CASE(
        "Octree - Negative mass bodies are excluded",
        "[Octree][Insert][Static][Validation]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(1.0, 1.0, 1.0),
            -5.0,
            0.0
        );

        REQUIRE(tree.GetRoot().IsEmpty());
        REQUIRE(tree.GetRoot().GetBodyCount() == 0);
    }

    TEST_CASE(
        "Octree - Static and dynamic bodies are distinguished",
        "[Octree][Insert][Static]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(100.0, 100.0, 100.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(10.0, 10.0, 10.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-10.0, -10.0, -10.0),
            0.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{2, 0},
            osseus::Vector3(10.0, -10.0, 10.0),
            2.0,
            0.0
        );

        REQUIRE(tree.GetRoot().GetBodyCount() == 2);
        REQUIRE(tree.GetRoot().GetTotalMass() == 3.0);
    }

    // ========================================================================
    // Mass properties
    // ========================================================================

    TEST_CASE(
        "Octree - Total mass is accumulated",
        "[Octree][Properties][Mass]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(100.0, 100.0, 100.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(10.0, 10.0, 10.0),
            2.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-10.0, 10.0, 10.0),
            3.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{2, 0},
            osseus::Vector3(10.0, -10.0, 10.0),
            5.0,
            0.0
        );

        tree.UpdateProperties();

        REQUIRE(tree.GetRoot().GetTotalMass() == 10.0);
    }

    TEST_CASE(
        "Octree - Single body center of mass equals its position",
        "[Octree][Properties][Mass]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        const osseus::Vector3 position(2.0, -3.0, 4.0);

        tree.Insert(
            osseus::Handle{0, 0},
            position,
            5.0,
            0.0
        );

        tree.UpdateProperties();

        REQUIRE(tree.GetRoot().GetCenterOfMass() == position);
    }

    TEST_CASE(
        "Octree - Center of mass is mass weighted",
        "[Octree][Properties][Mass]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(5.0, 0.0, 0.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-5.0, 0.0, 0.0),
            3.0,
            0.0
        );

        tree.UpdateProperties();

        const osseus::Vector3 expected(-2.5, 0.0, 0.0);

        REQUIRE(tree.GetRoot().GetCenterOfMass() == expected);
    }

    TEST_CASE(
        "Octree - Symmetric equal masses have origin center of mass",
        "[Octree][Properties][Mass]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(5.0, 5.0, 5.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-5.0, -5.0, -5.0),
            1.0,
            0.0
        );

        tree.UpdateProperties();

        REQUIRE(
            tree.GetRoot().GetCenterOfMass()
            == osseus::Vector3::Zero()
        );
    }

    // ========================================================================
    // Subdivision
    // ========================================================================

    TEST_CASE(
        "Octree - Bodies in different octants cause subdivision",
        "[Octree][Subdivide]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        REQUIRE(tree.GetRoot().IsLeaf());

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(5.0, 5.0, 5.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-5.0, -5.0, -5.0),
            1.0,
            0.0
        );

        REQUIRE_FALSE(tree.GetRoot().IsLeaf());
        REQUIRE(tree.GetRoot().HasChild(0));
        REQUIRE(tree.GetRoot().HasChild(7));
    }

    TEST_CASE(
        "Octree - Bodies sharing an octant are recursively subdivided",
        "[Octree][Subdivide]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(100.0, 100.0, 100.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(1.0, 1.0, 1.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(1.5, 1.5, 1.5),
            1.0,
            0.0
        );

        REQUIRE(tree.GetRoot().GetBodyCount() == 2);
        REQUIRE_FALSE(tree.GetRoot().IsLeaf());
    }

    TEST_CASE(
        "Octree - Center boundary resolves deterministically",
        "[Octree][Subdivide][Boundary]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3::Zero(),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-5.0, -5.0, -5.0),
            1.0,
            0.0
        );

        REQUIRE_FALSE(tree.GetRoot().IsLeaf());
        REQUIRE(tree.GetRoot().GetBodyCount() == 2);
    }

    TEST_CASE(
        "Octree - Coincident bodies terminate subdivision",
        "[Octree][Subdivide][Regression]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        const osseus::Vector3 position(1.0, 1.0, 1.0);

        tree.Insert(osseus::Handle{0, 0}, position, 1.0, 0.0);
        tree.Insert(osseus::Handle{1, 0}, position, 1.0, 0.0);
        tree.Insert(osseus::Handle{2, 0}, position, 1.0, 0.0);

        REQUIRE(tree.GetRoot().GetBodyCount() == 3);
        REQUIRE(tree.GetRoot().GetTotalMass() == 3.0);
    }

    // ========================================================================
    // Parent relationships
    // ========================================================================

    TEST_CASE(
        "Octree - Child nodes reference their parent",
        "[Octree][Parent][Subdivide]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(5.0, 5.0, 5.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-5.0, -5.0, -5.0),
            1.0,
            0.0
        );

        const osseus::OctNode& root = tree.GetRoot();

        const osseus::OctNode* positive = root.GetChild(7);
        const osseus::OctNode* negative = root.GetChild(0);

        REQUIRE(positive != nullptr);
        REQUIRE(negative != nullptr);

        REQUIRE(positive->GetParent() == &root);
        REQUIRE(negative->GetParent() == &root);
    }

    TEST_CASE(
        "Octree - Child depth is one greater than parent depth",
        "[Octree][Parent][Subdivide]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(5.0, 5.0, 5.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-5.0, -5.0, -5.0),
            1.0,
            0.0
        );

        const osseus::OctNode& root = tree.GetRoot();
        const osseus::OctNode* child = root.GetChild(7);

        REQUIRE(child != nullptr);
        REQUIRE(root.GetDepth() == 0);
        REQUIRE(child->GetDepth() == 1);
    }

    TEST_CASE(
        "Octree - Root reference remains stable across insertions",
        "[Octree][Parent][Regression]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        const osseus::OctNode* rootBefore = &tree.GetRoot();

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(5.0, 5.0, 5.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-5.0, -5.0, -5.0),
            1.0,
            0.0
        );

        REQUIRE(rootBefore == &tree.GetRoot());
    }

    // ========================================================================
    // Removal
    // ========================================================================

    TEST_CASE(
        "Octree - Removing the only body makes the tree empty",
        "[Octree][Remove]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        const osseus::Handle handle{0, 0};

        tree.Insert(
            handle,
            osseus::Vector3(1.0, 1.0, 1.0),
            1.0,
            0.0
        );

        tree.Remove(handle);

        REQUIRE(tree.GetRoot().IsEmpty());
        REQUIRE(tree.GetRoot().GetBodyCount() == 0);
    }

    TEST_CASE(
        "Octree - Removing one body preserves the remaining bodies",
        "[Octree][Remove]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(100.0, 100.0, 100.0)
        });

        const osseus::Handle a{0, 0};
        const osseus::Handle b{1, 0};
        const osseus::Handle c{2, 0};

        tree.Insert(a, osseus::Vector3(10.0, 10.0, 10.0), 1.0, 0.0);
        tree.Insert(b, osseus::Vector3(-10.0, 10.0, 10.0), 1.0, 0.0);
        tree.Insert(c, osseus::Vector3(10.0, -10.0, 10.0), 1.0, 0.0);

        tree.Remove(b);

        REQUIRE(tree.GetRoot().GetBodyCount() == 2);
        REQUIRE_FALSE(tree.GetRoot().IsEmpty());
    }

    TEST_CASE(
        "Octree - Removing an unknown handle is safe",
        "[Octree][Remove][Regression]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(100.0, 100.0, 100.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(10.0, 10.0, 10.0),
            1.0,
            0.0
        );

        tree.Remove(osseus::Handle{99, 0});

        REQUIRE(tree.GetRoot().GetBodyCount() == 1);
    }

    // ========================================================================
    // Clear
    // ========================================================================

    TEST_CASE(
        "Octree - Clear removes all bodies",
        "[Octree][Clear]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(100.0, 100.0, 100.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(10.0, 10.0, 10.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-10.0, 10.0, 10.0),
            1.0,
            0.0
        );

        tree.Clear();

        const osseus::OctNode& root = tree.GetRoot();

        REQUIRE(root.IsLeaf());
        REQUIRE(root.IsEmpty());
        REQUIRE(root.GetBodyCount() == 0);
        REQUIRE(root.GetTotalMass() == 0.0);
        REQUIRE(root.GetCenterOfMass() == osseus::Vector3::Zero());
    }

    TEST_CASE(
        "Octree - Tree can be reused after Clear",
        "[Octree][Clear][Regression]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(100.0, 100.0, 100.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(10.0, 10.0, 10.0),
            1.0,
            0.0
        );

        tree.Clear();

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-10.0, -10.0, -10.0),
            2.0,
            0.0
        );

        REQUIRE(tree.GetRoot().GetBodyCount() == 1);
        REQUIRE(tree.GetRoot().GetTotalMass() == 2.0);
    }

    TEST_CASE(
        "Octree - Clear restores root parent and depth",
        "[Octree][Clear][Parent]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(5.0, 5.0, 5.0),
            1.0,
            0.0
        );

        tree.Insert(
            osseus::Handle{1, 0},
            osseus::Vector3(-5.0, -5.0, -5.0),
            1.0,
            0.0
        );

        tree.Clear();

        REQUIRE(tree.GetRoot().GetParent() == nullptr);
        REQUIRE(tree.GetRoot().GetDepth() == 0);
    }

    // ========================================================================
    // Root bounds
    // ========================================================================

    TEST_CASE(
        "Octree - SetRootBounds updates root bounds",
        "[Octree][Bounds]"
    )
    {
        osseus::Octree tree;

        const osseus::Bounds bounds{
            osseus::Vector3(100.0, 200.0, 300.0),
            osseus::Vector3(500.0, 600.0, 700.0)
        };

        tree.SetRootBounds(bounds);

        const osseus::Bounds& actual = tree.GetRoot().GetBounds();

        REQUIRE(actual.center == bounds.center);
        REQUIRE(actual.halfSize == bounds.halfSize);
    }

    // ========================================================================
    // Size / depth
    // ========================================================================

    TEST_CASE(
        "Octree - Size tracks inserted nodes",
        "[Octree][Size]"
    )
    {
        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(10.0, 10.0, 10.0)
        });

        const std::size_t initialSize = tree.Size();

        tree.Insert(
            osseus::Handle{0, 0},
            osseus::Vector3(5.0, 5.0, 5.0),
            1.0,
            0.0
        );

        REQUIRE(tree.Size() >= initialSize);

        tree.Clear();

        REQUIRE(tree.Size() == initialSize);
    }

    TEST_CASE(
        "Octree - Root depth is zero",
        "[Octree][Depth]"
    )
    {
        osseus::Octree tree;

        REQUIRE(tree.GetRoot().GetDepth() == 0);
        REQUIRE(tree.GetDepth() == 0);
    }

    // ========================================================================
    // Important Barnes-Hut / gravity regression
    // ========================================================================

    TEST_CASE(
        "Octree - Root bounds prevent self-force contamination for distant bodies",
        "[Octree][BarnesHut][Gravity][Regression]"
    )
    {
        constexpr double mass = 125000.0;
        constexpr double orbitalRadius = 200.0;

        osseus::Octree tree;

        tree.SetRootBounds({
            osseus::Vector3::Zero(),
            osseus::Vector3(1000.0, 1000.0, 1000.0)
        });

        const osseus::Handle handleA{0, 0};
        const osseus::Handle handleB{1, 0};

        const osseus::Vector3 positionA(
            -orbitalRadius,
            0.0,
            0.0
        );

        const osseus::Vector3 positionB(
            orbitalRadius,
            0.0,
            0.0
        );

        tree.Insert(handleA, positionA, mass, 0.0);
        tree.Insert(handleB, positionB, mass, 0.0);
        tree.UpdateProperties();

        osseus::BodyData bodyA;
        bodyA.position = positionA;
        bodyA.mass = mass;

        osseus::UniversalGravity gravity;

        const osseus::Vector3 force =
            gravity.CalculateForce(
                tree.GetRoot(),
                handleA,
                bodyA
            );

        const double expected =
            OsseusConstants::GravitationalConstant
            * mass
            * mass
            / (4.0 * orbitalRadius * orbitalRadius);

        const double magnitude = force.Length();

        REQUIRE_THAT(
            magnitude,
            Catch::Matchers::WithinRel(expected, 1e-9)
        );

        REQUIRE(force.x > 0.0);
    }

}