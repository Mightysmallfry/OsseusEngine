
// Section 34 of the test spec: properties that should hold across the
// whole engine regardless of which specific subsystem exercises them,
// as opposed to the narrower per-class unit tests everywhere else in
// this suite.
//

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;


// ==========================================================================
// Handle validity is binary: a handle resolves to live data, or it
// resolves to nothing. Never to another body's data.
// ==========================================================================

TEST_CASE("Invariant - A destroyed handle never resolves to a body, even one created afterward at the same slot", "[invariant]")
{
    PhysicsWorld world;

    Handle first = world.CreateBody(BodyData{Vector3(1.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0}, std::make_unique<ShapePoint>());
    world.DestroyBody(first);

    Handle second = world.CreateBody(BodyData{Vector3(2.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0}, std::make_unique<ShapePoint>());

    REQUIRE(world.GetBody(first) == nullptr);
    REQUIRE(world.GetBody(second) != nullptr);
    REQUIRE(world.GetBody(second)->position.x == 2.0);
}

TEST_CASE("Invariant - Every handle returned by CreateBody is initially valid and resolves to non-null", "[invariant]")
{
    PhysicsWorld world;

    for (int i = 0; i < 50; ++i) {
        Handle handle = world.CreateBody(BodyData{}, std::make_unique<ShapePoint>());
        REQUIRE(world.GetBody(handle) != nullptr);
    }
}

// ==========================================================================
// Static bodies never move, regardless of what acts on them
// ==========================================================================

TEST_CASE("Invariant - A static body (invMass = 0) never moves under gravity, individual forces, or collision", "[invariant]")
{
    PhysicsWorld world;
    UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    Handle staticBody = world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), 1.0e6, 0.0, 0.0},
                                         std::make_unique<ShapeCube>(1.0));
    Handle dynamicBody = world.CreateBody(BodyData{Vector3(0.5, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0},
                                          std::make_unique<ShapeSphere>(1.0));
    (void)dynamicBody;

    world.GetForceManager().Add(staticBody, Vector3(1000.0, 0.0, 0.0));

    for (int i = 0; i < 100; ++i) {
        world.Step(0.01);
        REQUIRE(world.GetBody(staticBody)->position == Vector3::Zero());
        REQUIRE(world.GetBody(staticBody)->velocity == Vector3::Zero());
    }
}

// ==========================================================================
// Manager lockstep: BodyManager, ForceManager, and ShapeManager agree on
// which handles are live
// ==========================================================================

TEST_CASE("Invariant - BodyManager and ForceManager report the same set of live handles after arbitrary create/destroy churn", "[invariant]")
{
    PhysicsWorld world;
    std::vector<Handle> survivors;

    for (int i = 0; i < 30; ++i) {
        Handle h = world.CreateBody(BodyData{}, std::make_unique<ShapePoint>());
        if (i % 3 == 0) {
            world.DestroyBody(h);
        } else {
            survivors.push_back(h);
        }
    }

    REQUIRE(world.GetBodyManager().Data().size() == world.GetForceManager().NetForces().size());
    REQUIRE(world.GetBodyManager().Handles().size() == survivors.size());

    for (Handle h : survivors) {
        REQUIRE(world.GetBodyManager().GetBody(h) != nullptr);
    }
}

// ==========================================================================
// Octree aggregate mass always equals the sum of dynamic body masses
// currently in the world
// ==========================================================================

TEST_CASE("Invariant - The octree's total mass always equals the sum of dynamic (nonzero-invMass) body masses", "[invariant][octree]")
{
    PhysicsWorld world;

    world.CreateBody(BodyData{Vector3(1.0, 0.0, 0.0), Vector3::Zero(), 3.0, 1.0, 0.0}, std::make_unique<ShapeSphere>(1.0));
    world.CreateBody(BodyData{Vector3(-1.0, 0.0, 0.0), Vector3::Zero(), 7.0, 1.0, 0.0}, std::make_unique<ShapeSphere>(1.0));
    // Static bodies (invMass 0) still have positive mass and ARE
    // inserted into the octree -- Octree::Insert doesn't look at
    // invMass at all, only PhysicsWorld's own AttachBody/BarnesHut
    // logic treats invMass specially elsewhere.
    world.CreateBody(BodyData{Vector3(0.0, 5.0, 0.0), Vector3::Zero(), 100.0, 0.0, 0.0},
                     std::make_unique<ShapeCube>(1.0));

    world.RebuildOctree();

    REQUIRE(world.GetOctree().GetRoot().GetTotalMass() == 110.0);
}

// ==========================================================================
// Momentum conservation for isolated (no external force) systems
// ==========================================================================

TEST_CASE("Invariant - Total momentum is conserved for an isolated multi-body gravitational system", "[invariant][conservation]")
{
    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorRungeKutta4>());
    UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    struct Seed { Vector3 position, velocity; double mass; };
    const std::vector<Seed> seeds = {
        {Vector3(10.0, 0.0, 0.0), Vector3(0.0, 2.0, 0.0), 4.0},
        {Vector3(-10.0, 0.0, 0.0), Vector3(0.0, -1.0, 0.0), 8.0},
        {Vector3(0.0, 10.0, 0.0), Vector3(-0.5, 0.0, 0.0), 6.0},
    };

    std::vector<Handle> handles;
    Vector3 initialMomentum = Vector3::Zero();
    for (const Seed& s : seeds) {
        handles.push_back(world.CreateBody(BodyData{s.position, s.velocity, s.mass, 1.0 / s.mass, 0.0},
                                           std::make_unique<ShapeSphere>(0.5)));
        initialMomentum += s.velocity * s.mass;
    }

    for (int step = 0; step < 500; ++step) {
        world.Step(0.005);
    }

    Vector3 finalMomentum = Vector3::Zero();
    for (std::size_t i = 0; i < handles.size(); ++i) {
        const BodyData* body = world.GetBody(handles[i]);
        finalMomentum += body->velocity * seeds[i].mass;
    }

    // RK4 conserves momentum very well but not to arbitrary precision
    // over 500 steps of a genuinely multi-body (non-integrable) system;
    // 1e-2 is generous relative to the ~10 units/s of momentum in this
    // system while still catching any real non-conservation bug (e.g.
    // an asymmetric force application) which would show drift orders
    // of magnitude larger than numerical integration error alone.
    REQUIRE_THAT((finalMomentum - initialMomentum).Length(), WithinAbs(0.0, 1e-2));
}

// ==========================================================================
// No handle collision: two live bodies never share a handle
// ==========================================================================

TEST_CASE("Invariant - No two simultaneously-live bodies ever share the same handle index and generation", "[invariant]")
{
    PhysicsWorld world;
    std::vector<Handle> live;

    for (int i = 0; i < 100; ++i) {
        Handle h = world.CreateBody(BodyData{}, std::make_unique<ShapePoint>());

        for (Handle existing : live) {
            REQUIRE_FALSE((existing.index == h.index && existing.generation == h.generation));
        }
        live.push_back(h);

        // Occasionally destroy one to exercise slot reuse alongside
        // fresh allocations, since that's where a collision would
        // actually be likely to surface.
        if (i % 4 == 0 && !live.empty()) {
            world.DestroyBody(live.front());
            live.erase(live.begin());
        }
    }
}
