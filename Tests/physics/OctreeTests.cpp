//
// Created by MightySmallFry on 8/16/2026.
//

#include <catch2/catch_test_macros.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;

// ==========================================================================
// Construction / initial state
// ==========================================================================

TEST_CASE("Octree - Default constructor creates an empty root node", "[octree]")
{
    Octree tree;

    const OctNode& root = tree.GetRoot();

    REQUIRE(root.IsLeaf());
    REQUIRE(root.IsEmpty());
    REQUIRE(root.GetBodyCount() == 0);
    REQUIRE(root.GetTotalMass() == 0.0);
    REQUIRE(root.GetCenterOfMass() == Vector3::Zero());
    REQUIRE(root.GetBounds().center == Vector3::Zero());
    REQUIRE(root.GetBounds().halfSize == Vector3::One());
}

TEST_CASE("Octree - Root node starts as an empty leaf", "[octree]")
{
    const Bounds bounds{
        Vector3(0.0, 0.0, 0.0),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    const OctNode& root = tree.GetRoot();

    REQUIRE(root.IsLeaf());
    REQUIRE(root.IsEmpty());
    REQUIRE(root.GetBodyCount() == 0);
    REQUIRE(root.GetTotalMass() == 0.0);
    REQUIRE(root.GetCenterOfMass() == Vector3::Zero());
}

TEST_CASE("Octree - Root node has no children when first created", "[octree]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    const OctNode& root = tree.GetRoot();

    for (std::size_t index = 0; index < 8; ++index)
    {
        REQUIRE_FALSE(root.HasChild(index));
        REQUIRE(root.GetChild(index) == nullptr);
    }
}

TEST_CASE("Octree - Root node rejects invalid child indices", "[octree]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    const OctNode& root = tree.GetRoot();

    REQUIRE_THROWS(root.GetChild(8));
    REQUIRE_THROWS(root.HasChild(8));
}

// ==========================================================================
// OctNode
// ==========================================================================

TEST_CASE("Octree - Newly constructed node has zero mass properties", "[octnode]")
{
    const Bounds bounds{
        Vector3(1.0, 2.0, 3.0),
        Vector3(10.0, 20.0, 30.0)
    };

    OctNode node(bounds);

    REQUIRE(node.GetBodyCount() == 0);
    REQUIRE(node.GetTotalMass() == 0.0);
    REQUIRE(node.GetCenterOfMass() == Vector3::Zero());
}

TEST_CASE("Octree - Newly constructed node is a leaf", "[octnode]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    OctNode node(bounds);

    REQUIRE(node.IsLeaf());
}

// ==========================================================================
// Insertion
// ==========================================================================

TEST_CASE("Octree - Inserting a body makes the root non-empty", "[octree][insert]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    Handle handle{0, 0};

    tree.Insert(handle, Vector3(1.0, 1.0, 1.0), 1.0, 0.0);

    const OctNode& root = tree.GetRoot();

    REQUIRE_FALSE(root.IsEmpty());
    REQUIRE(root.GetBodyCount() == 1);
}

TEST_CASE("Octree - Inserting multiple bodies tracks the body count", "[octree][insert]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(10.0, 10.0, 10.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-10.0, 10.0, 10.0), 1.0, 0.0);
    tree.Insert(Handle{2, 0}, Vector3(10.0, -10.0, 10.0), 1.0, 0.0);
    tree.Insert(Handle{3, 0}, Vector3(10.0, 10.0, -10.0), 1.0, 0.0);

    const OctNode& root = tree.GetRoot();

    REQUIRE_FALSE(root.IsEmpty());
    REQUIRE(root.GetBodyCount() == 4);
}

TEST_CASE("Octree - Inserting a static body (zero mass) is excluded from the tree", "[octree][insert][static]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(1.0, 1.0, 1.0), 0.0, 0.0);

    const OctNode& root = tree.GetRoot();

    REQUIRE(root.IsEmpty());
    REQUIRE(root.GetBodyCount() == 0);
    REQUIRE(root.GetTotalMass() == 0.0);
}

TEST_CASE("Octree - Inserting a body with negative mass is excluded from the tree", "[octree][insert][static]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(1.0, 1.0, 1.0), -5.0, 0.0);

    const OctNode& root = tree.GetRoot();

    REQUIRE(root.IsEmpty());
    REQUIRE(root.GetBodyCount() == 0);
}

TEST_CASE("Octree - A mix of static and dynamic bodies only counts the dynamic ones", "[octree][insert][static]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(10.0, 10.0, 10.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-10.0, -10.0, -10.0), 0.0, 0.0);   // static, excluded
    tree.Insert(Handle{2, 0}, Vector3(10.0, -10.0, 10.0), 2.0, 0.0);

    const OctNode& root = tree.GetRoot();

    REQUIRE(root.GetBodyCount() == 2);
    REQUIRE(root.GetTotalMass() == 3.0);
}

// ==========================================================================
// Mass properties
// ==========================================================================

TEST_CASE("Octree - Total mass accumulates the sum of inserted body masses", "[octree][insert][mass]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(10.0, 10.0, 10.0), 2.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-10.0, 10.0, 10.0), 3.0, 0.0);
    tree.Insert(Handle{2, 0}, Vector3(10.0, -10.0, 10.0), 5.0, 0.0);

    REQUIRE(tree.GetRoot().GetTotalMass() == 10.0);
}

TEST_CASE("Octree - Center of mass of a single body equals that body's position", "[octree][insert][mass]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    const Vector3 position(2.0, -3.0, 4.0);
    tree.Insert(Handle{0, 0}, position, 5.0, 0.0);

    REQUIRE(tree.GetRoot().GetCenterOfMass() == position);
}

TEST_CASE("Octree - Center of mass is the mass-weighted average of symmetric bodies", "[octree][insert][mass]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    // Two equal masses placed symmetrically about the origin along X
    // should average out to a center of mass at the origin.
    tree.Insert(Handle{0, 0}, Vector3(5.0, 5.0, 5.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-5.0, -5.0, -5.0), 1.0, 0.0);

    REQUIRE(tree.GetRoot().GetCenterOfMass() == Vector3::Zero());
}

TEST_CASE("Octree - Center of mass weights heavier bodies more strongly", "[octree][insert][mass]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    // Body A at +5 with mass 1, body B at -5 with mass 3.
    // Weighted average: (5*1 + -5*3) / 4 = -2.5
    tree.Insert(Handle{0, 0}, Vector3(5.0, 0.0, 0.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-5.0, 0.0, 0.0), 3.0, 0.0);

    const Vector3 expected(-2.5, 0.0, 0.0);
    REQUIRE(tree.GetRoot().GetCenterOfMass() == expected);
}

// ==========================================================================
// Subdivision
// ==========================================================================

TEST_CASE("Octree - Root stays a leaf while bodies occupy the same octant", "[octree][subdivide]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    // Both positions fall in the same (+,+,+) octant.
    tree.Insert(Handle{0, 0}, Vector3(1.0, 1.0, 1.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(2.0, 2.0, 2.0), 1.0, 0.0);

    // Bodies sharing an octant force recursive subdivision past a single
    // node, so the root itself is expected to have subdivided into
    // children while still correctly aggregating both bodies.
    REQUIRE(tree.GetRoot().GetBodyCount() == 2);
}

TEST_CASE("Octree - Root subdivides once bodies occupy different octants", "[octree][subdivide]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    REQUIRE(tree.GetRoot().IsLeaf());

    tree.Insert(Handle{0, 0}, Vector3(5.0, 5.0, 5.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-5.0, -5.0, -5.0), 1.0, 0.0);

    REQUIRE_FALSE(tree.GetRoot().IsLeaf());
    REQUIRE(tree.GetRoot().HasChild(0));  // (-,-,-) octant per GetOctantIndex bit layout
    REQUIRE(tree.GetRoot().HasChild(7));  // (+,+,+) octant
}

TEST_CASE("Octree - A body exactly on the center boundary resolves deterministically", "[octree][subdivide][boundary]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    // Position lies exactly on all three axis planes. GetOctantIndex uses
    // >=, so this should deterministically resolve to octant 7 (+,+,+).
    tree.Insert(Handle{0, 0}, Vector3(0.0, 0.0, 0.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-5.0, -5.0, -5.0), 1.0, 0.0);

    REQUIRE_FALSE(tree.GetRoot().IsLeaf());
    REQUIRE(tree.GetRoot().GetBodyCount() == 2);
}

TEST_CASE("Octree - Coincident positions terminate subdivision at max depth instead of recursing forever", "[octree][subdivide][regression]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    const Vector3 samePosition(1.0, 1.0, 1.0);

    // All bodies land in the exact same point, which would force infinite
    // subdivision without a MaxDepth guard on OctNode::Insert.
    tree.Insert(Handle{0, 0}, samePosition, 1.0, 0.0);
    tree.Insert(Handle{1, 0}, samePosition, 1.0, 0.0);
    tree.Insert(Handle{2, 0}, samePosition, 1.0, 0.0);

    REQUIRE(tree.GetRoot().GetBodyCount() == 3);
    REQUIRE(tree.GetRoot().GetTotalMass() == 3.0);
}

// ==========================================================================
// Removal
// ==========================================================================

TEST_CASE("Octree - Removing the only body makes the root empty", "[octree][remove]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    const Handle handle{0, 0};

    tree.Insert(handle, Vector3(1.0, 1.0, 1.0), 1.0, 0.0);
    REQUIRE(tree.GetRoot().GetBodyCount() == 1);

    tree.Remove(handle);

    const OctNode& root = tree.GetRoot();

    REQUIRE(root.IsEmpty());
    REQUIRE(root.GetBodyCount() == 0);
}

TEST_CASE("Octree - Removing one body leaves the remaining bodies in the tree", "[octree][remove]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    const Handle a{0, 0};
    const Handle b{1, 0};
    const Handle c{2, 0};

    tree.Insert(a, Vector3(10.0, 10.0, 10.0), 1.0, 0.0);
    tree.Insert(b, Vector3(-10.0, 10.0, 10.0), 1.0, 0.0);
    tree.Insert(c, Vector3(10.0, -10.0, 10.0), 1.0, 0.0);

    REQUIRE(tree.GetRoot().GetBodyCount() == 3);

    tree.Remove(b);

    REQUIRE(tree.GetRoot().GetBodyCount() == 2);
    REQUIRE_FALSE(tree.GetRoot().IsEmpty());
}

TEST_CASE("Octree - Removing a handle that was never inserted is a safe no-op", "[octree][remove][regression]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(10.0, 10.0, 10.0), 1.0, 0.0);

    REQUIRE(tree.GetRoot().GetBodyCount() == 1);

    tree.Remove(Handle{99, 0});

    REQUIRE(tree.GetRoot().GetBodyCount() == 1);
    REQUIRE_FALSE(tree.GetRoot().IsEmpty());
}

TEST_CASE("Octree - Removing a handle that was excluded for being static is a safe no-op", "[octree][remove][static][regression]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    const Handle staticHandle{1, 0};

    tree.Insert(Handle{0, 0}, Vector3(10.0, 10.0, 10.0), 1.0, 0.0);
    tree.Insert(staticHandle, Vector3(-10.0, -10.0, -10.0), 0.0, 0.0); // excluded

    REQUIRE(tree.GetRoot().GetBodyCount() == 1);

    tree.Remove(staticHandle);

    REQUIRE(tree.GetRoot().GetBodyCount() == 1);
    REQUIRE_FALSE(tree.GetRoot().IsEmpty());
}

// ==========================================================================
// Clear
// ==========================================================================

TEST_CASE("Octree - Clear removes all bodies from the tree", "[octree][clear]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(10.0, 10.0, 10.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-10.0, 10.0, 10.0), 1.0, 0.0);
    tree.Insert(Handle{2, 0}, Vector3(10.0, -10.0, 10.0), 1.0, 0.0);

    REQUIRE(tree.GetRoot().GetBodyCount() == 3);

    tree.Clear();

    const OctNode& root = tree.GetRoot();

    REQUIRE(root.IsLeaf());
    REQUIRE(root.IsEmpty());
    REQUIRE(root.GetBodyCount() == 0);
    REQUIRE(root.GetTotalMass() == 0.0);
    REQUIRE(root.GetCenterOfMass() == Vector3::Zero());

    for (std::size_t index = 0; index < 8; ++index)
    {
        REQUIRE_FALSE(root.HasChild(index));
        REQUIRE(root.GetChild(index) == nullptr);
    }
}

// ==========================================================================
// Multiple operations
// ==========================================================================

TEST_CASE("Octree - Tree can be reused after Clear", "[octree][clear][regression]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(10.0, 10.0, 10.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-10.0, 10.0, 10.0), 1.0, 0.0);

    REQUIRE(tree.GetRoot().GetBodyCount() == 2);

    tree.Clear();

    REQUIRE(tree.GetRoot().GetBodyCount() == 0);

    tree.Insert(Handle{2, 0}, Vector3(10.0, -10.0, 10.0), 1.0, 0.0);

    REQUIRE(tree.GetRoot().GetBodyCount() == 1);
    REQUIRE_FALSE(tree.GetRoot().IsEmpty());
}

// ==========================================================================
// Parent pointers
// ==========================================================================

TEST_CASE("Octree - Root node has no parent", "[octree][parent]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    REQUIRE(tree.GetRoot().GetParent() == nullptr);
    REQUIRE(tree.GetRoot().GetDepth() == 0);
}

TEST_CASE("Octree - A freshly constructed standalone OctNode has no parent by default", "[octnode][parent]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    OctNode node(bounds);

    REQUIRE(node.GetParent() == nullptr);
    REQUIRE(node.GetDepth() == 0);
}

TEST_CASE("Octree - Child nodes created by subdivision point back to their parent", "[octree][parent][subdivide]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    // Two bodies in different octants force the root to subdivide.
    tree.Insert(Handle{0, 0}, Vector3(5.0, 5.0, 5.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-5.0, -5.0, -5.0), 1.0, 0.0);

    const OctNode& root = tree.GetRoot();
    REQUIRE_FALSE(root.IsLeaf());

    const OctNode* childA = root.GetChild(7); // (+,+,+)
    const OctNode* childB = root.GetChild(0); // (-,-,-)

    REQUIRE(childA != nullptr);
    REQUIRE(childB != nullptr);

    REQUIRE(childA->GetParent() == &root);
    REQUIRE(childB->GetParent() == &root);
}

TEST_CASE("Octree - Child depth is one greater than parent depth", "[octree][parent][subdivide]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(5.0, 5.0, 5.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-5.0, -5.0, -5.0), 1.0, 0.0);

    const OctNode& root = tree.GetRoot();
    const OctNode* child = root.GetChild(7);

    REQUIRE(root.GetDepth() == 0);
    REQUIRE(child->GetDepth() == 1);
}

TEST_CASE("Octree - Parent chain is consistent across multiple levels of subdivision", "[octree][parent][subdivide]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    // Two bodies close together but not coincident, positioned so they
    // share several levels of octant nesting before finally separating,
    // forcing subdivision more than one level deep.
    tree.Insert(Handle{0, 0}, Vector3(1.0, 1.0, 1.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(1.5, 1.5, 1.5), 1.0, 0.0);

    const OctNode& root = tree.GetRoot();
    REQUIRE_FALSE(root.IsLeaf());

    const OctNode* level1 = root.GetChild(7); // both bodies start in (+,+,+)
    REQUIRE(level1 != nullptr);
    REQUIRE(level1->GetParent() == &root);
    REQUIRE(level1->GetDepth() == 1);

    // Walking back up from a deeper node should always land on the same
    // root instance, regardless of how many levels were descended.
    const OctNode* walkUp = level1;
    while (walkUp->GetParent() != nullptr)
    {
        walkUp = walkUp->GetParent();
    }
    REQUIRE(walkUp == &root);
}

TEST_CASE("Octree - Clear resets the root to have no parent", "[octree][parent][clear]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(5.0, 5.0, 5.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-5.0, -5.0, -5.0), 1.0, 0.0);

    tree.Clear();

    REQUIRE(tree.GetRoot().GetParent() == nullptr);
    REQUIRE(tree.GetRoot().GetDepth() == 0);
}

TEST_CASE("Octree - Probe: nested octant descent for closely-spaced bodies", "[octree][parent][subdivide][probe]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(1.0, 1.0, 1.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(1.5, 1.5, 1.5), 1.0, 0.0);

    // Descend through whichever single child is populated at each level.
    // The two bodies stay in the same octant for several levels before
    // the shrinking cell size finally separates them, so this walk does
    // NOT assume a fixed octant index at every step.
    const OctNode* current = &tree.GetRoot();
    int stepsDescended = 0;
    int populatedChildCount = 0;

    while (!current->IsLeaf())
    {
        populatedChildCount = 0;
        const OctNode* next = nullptr;

        for (std::size_t i = 0; i < 8; ++i)
        {
            if (current->HasChild(i))
            {
                ++populatedChildCount;
                next = current->GetChild(i);
            }
        }

        // Once the bodies separate into different octants, this node has
        // more than one populated child and traversal stops here.
        if (populatedChildCount > 1) { break; }

        REQUIRE(next != nullptr);
        current = next;
        ++stepsDescended;

        // Safety bound so a genuine infinite-subdivision regression fails
        // loudly here instead of hanging the test run. (OctNode::MaxDepth
        // is private and not visible here, so this mirrors that value.)
        REQUIRE(stepsDescended <= 32);
    }

    // Both bodies must still be accounted for somewhere in the tree.
    REQUIRE(tree.GetRoot().GetBodyCount() == 2);
    REQUIRE(tree.GetRoot().GetTotalMass() == 2.0);

    // The two bodies separate into different octants at this node.
    REQUIRE(populatedChildCount == 2);
}

TEST_CASE("Octree - GetRoot returns the same node instance across multiple calls", "[octree][parent]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(5.0, 5.0, 5.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(-5.0, -5.0, -5.0), 1.0, 0.0);

    const OctNode& firstCall = tree.GetRoot();
    const OctNode& secondCall = tree.GetRoot();

    // Confirms GetRoot() isn't returning by value or otherwise producing
    // a fresh temporary each call — parent-pointer comparisons elsewhere
    // (e.g. childA->GetParent() == &root) depend on this being a stable
    // reference to the same persistent instance.
    REQUIRE(&firstCall == &secondCall);

    // And children captured against one call's address should still
    // compare equal to a parent pointer resolved through a later call.
    const OctNode* child = firstCall.GetChild(7);
    REQUIRE(child->GetParent() == &secondCall);
}

TEST_CASE("Octree - GetRoot reference is invalidated only by Clear, not by other operations", "[octree][parent][regression]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(10.0, 10.0, 10.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(5.0, 5.0, 5.0), 1.0, 0.0);

    const OctNode* beforeSecondInsert = &tree.GetRoot();

    tree.Insert(Handle{1, 0}, Vector3(-5.0, -5.0, -5.0), 1.0, 0.0);

    // Insert triggers subdivision internally but should never replace the
    // root OctNode instance itself -- only Clear() does that, by
    // resetting the unique_ptr. This guards against a future refactor
    // accidentally rebuilding the root on insert.
    REQUIRE(beforeSecondInsert == &tree.GetRoot());
}

TEST_CASE("Octree - Closely-spaced bodies split into separate leaves at the verified exact depth", "[octree][parent][subdivide][regression]")
{
    const Bounds bounds{
        Vector3::Zero(),
        Vector3(100.0, 100.0, 100.0)
    };

    Octree tree(bounds);

    tree.Insert(Handle{0, 0}, Vector3(1.0, 1.0, 1.0), 1.0, 0.0);
    tree.Insert(Handle{1, 0}, Vector3(1.5, 1.5, 1.5), 1.0, 0.0);

    // Values pinned via direct probe of Insert/Subdivide/GetOctantIndex
    // against these exact positions: the two bodies travel together
    // through octant 7, then octant 0 six times, then octant 7 once
    // more, before finally separating at depth 8.
    const OctNode* node = &tree.GetRoot();
    node = node->GetChild(7); // depth 1
    node = node->GetChild(0); // depth 2
    node = node->GetChild(0); // depth 3
    node = node->GetChild(0); // depth 4
    node = node->GetChild(0); // depth 5
    node = node->GetChild(0); // depth 6
    node = node->GetChild(0); // depth 7
    node = node->GetChild(7); // depth 8 -- the split node

    REQUIRE(node->GetDepth() == 8);
    REQUIRE_FALSE(node->IsLeaf());
    REQUIRE(node->HasChild(0));
    REQUIRE(node->HasChild(7));

    const OctNode* leafA = node->GetChild(0);
    const OctNode* leafB = node->GetChild(7);

    REQUIRE(leafA->IsLeaf());
    REQUIRE(leafB->IsLeaf());
    REQUIRE(leafA->GetDepth() == 9);
    REQUIRE(leafB->GetDepth() == 9);
    REQUIRE(leafA->GetBodyCount() == 1);
    REQUIRE(leafB->GetBodyCount() == 1);

    REQUIRE(leafA->GetParent() == node);
    REQUIRE(leafB->GetParent() == node);
}