#include <catch2/catch_test_macros.hpp>

#include <memory>

#include <Osseus/Osseus.h>

using namespace osseus;

namespace {
    Handle MakeHandle(uint32_t index) {
        return Handle{index, 0};
    }
}

// ==========================================================================
// AABB overlap cases
// ==========================================================================

TEST_CASE("BroadPhase - Touching (edge-to-edge) AABBs are reported as a candidate pair", "[broadphase][boundary]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);

    // Two unit half-extent cubes with centers exactly 1.0 apart touch
    // face-to-face; the broad phase treats touching as an overlap since
    // AABBOverlaps uses <= / >=.
    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(1.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(a, std::make_unique<ShapeCube>(0.5));
    shapeManager.AddShape(b, std::make_unique<ShapeCube>(0.5));

    BroadPhase broadPhase;
    std::vector<CollisionCandidatePair> candidates = broadPhase.FindCandidatePairs(bodyManager, shapeManager);

    REQUIRE(candidates.size() == 1);
}

TEST_CASE("BroadPhase - Identical (fully coincident) AABBs are reported as a candidate pair", "[broadphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);

    bodyManager.AddBody(a, BodyData{Vector3(2.0, 2.0, 2.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(2.0, 2.0, 2.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    std::vector<CollisionCandidatePair> candidates = broadPhase.FindCandidatePairs(bodyManager, shapeManager);

    REQUIRE(candidates.size() == 1);
}

TEST_CASE("BroadPhase - One AABB fully contained inside another is reported as a candidate pair", "[broadphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle outer = MakeHandle(0);
    Handle inner = MakeHandle(1);

    bodyManager.AddBody(outer, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(inner, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(outer, std::make_unique<ShapeSphere>(10.0));
    shapeManager.AddShape(inner, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    std::vector<CollisionCandidatePair> candidates = broadPhase.FindCandidatePairs(bodyManager, shapeManager);

    REQUIRE(candidates.size() == 1);
}

TEST_CASE("BroadPhase - AABBs separated on only the X axis do not overlap", "[broadphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);

    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(10.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    REQUIRE(broadPhase.FindCandidatePairs(bodyManager, shapeManager).empty());
}

TEST_CASE("BroadPhase - AABBs separated on only the Y axis do not overlap", "[broadphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);

    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(0.0, 10.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    REQUIRE(broadPhase.FindCandidatePairs(bodyManager, shapeManager).empty());
}

TEST_CASE("BroadPhase - AABBs separated on only the Z axis do not overlap", "[broadphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);

    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(0.0, 0.0, 10.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    REQUIRE(broadPhase.FindCandidatePairs(bodyManager, shapeManager).empty());
}

// ==========================================================================
// Object types
// ==========================================================================

TEST_CASE("BroadPhase - Two static bodies (zero invMass) still produce a candidate pair when overlapping", "[broadphase][static]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);

    // invMass of 0.0 marks a static body; the broad phase doesn't care
    // about mass at all, only bounding boxes.
    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0e6, 0.0});
    bodyManager.AddBody(b, BodyData{Vector3(0.5, 0.0, 0.0), Vector3::Zero(), 1.0e6, 0.0});
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    REQUIRE(broadPhase.FindCandidatePairs(bodyManager, shapeManager).size() == 1);
}

TEST_CASE("BroadPhase - A mix of static and dynamic overlapping bodies still produces a candidate pair", "[broadphase][static]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle staticFloor = MakeHandle(0);
    Handle dynamicBall = MakeHandle(1);

    bodyManager.AddBody(staticFloor, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0e6, 0.0});
    bodyManager.AddBody(dynamicBall, BodyData{Vector3(0.0, 0.5, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(staticFloor, std::make_unique<ShapeCube>(1.0));
    shapeManager.AddShape(dynamicBall, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    REQUIRE(broadPhase.FindCandidatePairs(bodyManager, shapeManager).size() == 1);
}

TEST_CASE("BroadPhase - A body with no registered shape is silently skipped rather than crashing", "[broadphase][regression]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle withShape = MakeHandle(0);
    Handle withoutShape = MakeHandle(1);

    bodyManager.AddBody(withShape, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(withoutShape, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(withShape, std::make_unique<ShapeSphere>(1.0));
    // withoutShape intentionally never registered with the shape manager.

    BroadPhase broadPhase;
    std::vector<CollisionCandidatePair> candidates = broadPhase.FindCandidatePairs(bodyManager, shapeManager);

    REQUIRE(candidates.empty());
}

TEST_CASE("BroadPhase - A destroyed body no longer participates in candidate pairs", "[broadphase][lifecycle]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);

    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(0.5, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));

    bodyManager.RemoveBody(b);

    BroadPhase broadPhase;
    REQUIRE(broadPhase.FindCandidatePairs(bodyManager, shapeManager).empty());
}

TEST_CASE("BroadPhase - Multiple bodies produce every overlapping pair exactly once", "[broadphase]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;

    // A cluster of three mutually-overlapping spheres plus one far away.
    Handle a = MakeHandle(0);
    Handle b = MakeHandle(1);
    Handle c = MakeHandle(2);
    Handle far = MakeHandle(3);

    bodyManager.AddBody(a, BodyData{Vector3(0.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(b, BodyData{Vector3(0.5, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(c, BodyData{Vector3(0.0, 0.5, 0.0), Vector3::Zero(), 1.0, 1.0});
    bodyManager.AddBody(far, BodyData{Vector3(100.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0});

    shapeManager.AddShape(a, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(b, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(c, std::make_unique<ShapeSphere>(1.0));
    shapeManager.AddShape(far, std::make_unique<ShapeSphere>(1.0));

    BroadPhase broadPhase;
    std::vector<CollisionCandidatePair> candidates = broadPhase.FindCandidatePairs(bodyManager, shapeManager);

    // a-b, a-c, b-c overlap; nothing involves `far`.
    REQUIRE(candidates.size() == 3);
    for (const CollisionCandidatePair& pair : candidates) {
        REQUIRE(pair.a.index != far.index);
        REQUIRE(pair.b.index != far.index);
    }
}

// ==========================================================================
// Critical invariant: no false negatives
// ==========================================================================

TEST_CASE("BroadPhase - Never misses a pair that GJK would confirm as truly colliding", "[broadphase][invariant]")
{
    BodyManager bodyManager;
    ShapeManager shapeManager;

    // A spread of overlapping/separated/touching spheres and cubes,
    // deliberately not all pairwise-overlapping, so the invariant check
    // below is non-trivial in both directions.
    struct Placement {
        Handle handle;
        Vector3 position;
        bool isCube;
        double extent; // radius or half-extent
    };

    const std::vector<Placement> placements = {
        {MakeHandle(0), Vector3(0.0, 0.0, 0.0), false, 1.0},
        {MakeHandle(1), Vector3(1.5, 0.0, 0.0), false, 1.0},
        {MakeHandle(2), Vector3(-3.0, 0.0, 0.0), true, 0.5},
        {MakeHandle(3), Vector3(0.0, 5.0, 0.0), true, 0.5},
        {MakeHandle(4), Vector3(0.0, 0.3, 0.0), false, 1.0},
        {MakeHandle(5), Vector3(50.0, 50.0, 50.0), false, 1.0},
    };

    for (const Placement& p : placements) {
        bodyManager.AddBody(p.handle, BodyData{p.position, Vector3::Zero(), 1.0, 1.0});
        if (p.isCube) {
            shapeManager.AddShape(p.handle, std::make_unique<ShapeCube>(p.extent));
        } else {
            shapeManager.AddShape(p.handle, std::make_unique<ShapeSphere>(p.extent));
        }
    }

    BroadPhase broadPhase;
    std::vector<CollisionCandidatePair> candidates = broadPhase.FindCandidatePairs(bodyManager, shapeManager);

    auto containsPair = [&](Handle a, Handle b) {
        for (const CollisionCandidatePair& pair : candidates) {
            if ((pair.a.index == a.index && pair.b.index == b.index) ||
                (pair.a.index == b.index && pair.b.index == a.index)) {
                return true;
            }
        }
        return false;
    };

    // Ground truth via direct GJK on every pair.
    for (size_t i = 0; i < placements.size(); ++i) {
        for (size_t j = i + 1; j < placements.size(); ++j) {
            const Placement& p1 = placements[i];
            const Placement& p2 = placements[j];

            IShape* shapeA = shapeManager.GetShape(p1.handle);
            IShape* shapeB = shapeManager.GetShape(p2.handle);

            GJKSimplex simplex;
            bool actuallyColliding = GJK::Intersect(*shapeA, p1.position, *shapeB, p2.position, simplex);

            if (actuallyColliding) {
                REQUIRE(containsPair(p1.handle, p2.handle));
            }
            // False positives (broad phase says yes, GJK says no) are
            // explicitly allowed by the spec and not checked here.
        }
    }
}
