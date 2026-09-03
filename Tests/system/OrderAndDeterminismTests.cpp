//
// OrderAndDeterminismTests.cpp
//
// Sections 27-28 of the test spec: whether the order bodies are created
// in affects the physics result, and whether an identical simulation
// run twice produces identical output.
//
// Both were empirically probed with standalone programs before writing
// any assertions here (2-body and 3-body permutations, mutual gravity,
// hundreds of steps) and came back bit-for-bit identical across every
// creation order tried. That's a stronger, more useful regression
// guard than a loose tolerance would be, so the tests below assert
// exact equality rather than "close enough" -- if a future change
// (e.g. an order-dependent floating-point summation, or an
// insertion-order-sensitive octree tie-break) breaks that property,
// these tests are meant to catch it precisely.
//

#include <catch2/catch_test_macros.hpp>

#include <vector>

#include <Osseus/Osseus.h>

using namespace osseus;

namespace {

    struct BodySpec {
        Vector3 position;
        Vector3 velocity;
        double mass;
    };

    // Runs the same gravitational system, creating bodies in whatever
    // order `specs` lists them in, and returns each body's final state
    // in the SAME order as `specs` (not creation/handle order), so
    // results from different creation orders can be compared apples to
    // apples.
    std::vector<BodyData> RunSystem(const std::vector<BodySpec>& specs, double dt, int steps) {
        PhysicsWorld world;
        UniversalGravity gravity;
        world.GetForceManager().AddUniversal(&gravity);

        std::vector<Handle> handles;
        handles.reserve(specs.size());
        for (const BodySpec& spec : specs) {
            handles.push_back(world.CreateBody(
                BodyData{spec.position, spec.velocity, spec.mass, 1.0 / spec.mass, 0.0}, std::make_unique<ShapeSphere>(1.0)));
        }

        for (int i = 0; i < steps; ++i) {
            world.Step(dt);
        }

        std::vector<BodyData> results;
        results.reserve(specs.size());
        for (Handle h : handles) {
            results.push_back(*world.GetBody(h));
        }
        return results;
    }

}

// ==========================================================================
// Section 27 - Order independence
// ==========================================================================

TEST_CASE("Order independence - Two mutually-attracting bodies reach identical final state regardless of creation order", "[order-independence]")
{
    BodySpec bodyA{Vector3(10.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), 5.0};
    BodySpec bodyB{Vector3(-10.0, 0.0, 0.0), Vector3(0.0, -1.0, 0.0), 5.0};

    // Same logical system, but created in each order. Results are
    // returned matched to `specs` order, so index 0 is always "bodyA"
    // and index 1 is always "bodyB" regardless of which was created
    // (and assigned a handle) first internally.
    std::vector<BodyData> forwardOrder = RunSystem({bodyA, bodyB}, 0.01, 500);
    std::vector<BodyData> reverseOrder = RunSystem({bodyB, bodyA}, 0.01, 500);

    REQUIRE(forwardOrder[0].position == reverseOrder[1].position);
    REQUIRE(forwardOrder[1].position == reverseOrder[0].position);
    REQUIRE(forwardOrder[0].velocity == reverseOrder[1].velocity);
    REQUIRE(forwardOrder[1].velocity == reverseOrder[0].velocity);
}

TEST_CASE("Order independence - A three-body system reaches identical final state under every creation order permutation", "[order-independence]")
{
    BodySpec bodyA{Vector3(20.0, 0.0, 0.0), Vector3(0.0, 4.0, 0.0), 10.0};
    BodySpec bodyB{Vector3(-10.0, 17.32, 0.0), Vector3(-3.46, -2.0, 0.0), 10.0};
    BodySpec bodyC{Vector3(-10.0, -17.32, 0.0), Vector3(3.46, -2.0, 0.0), 10.0};

    constexpr double dt = 0.005;
    constexpr int steps = 800;

    // {A, B, C} order -> index 0/1/2 == A/B/C.
    std::vector<BodyData> orderABC = RunSystem({bodyA, bodyB, bodyC}, dt, steps);
    // {C, A, B} order -> matched back so index 0/1/2 still == A/B/C.
    std::vector<BodyData> orderCAB_raw = RunSystem({bodyC, bodyA, bodyB}, dt, steps);
    std::vector<BodyData> orderCAB = {orderCAB_raw[1], orderCAB_raw[2], orderCAB_raw[0]};
    // {B, C, A} order, matched the same way.
    std::vector<BodyData> orderBCA_raw = RunSystem({bodyB, bodyC, bodyA}, dt, steps);
    std::vector<BodyData> orderBCA = {orderBCA_raw[2], orderBCA_raw[0], orderBCA_raw[1]};

    for (std::size_t i = 0; i < 3; ++i) {
        REQUIRE(orderABC[i].position == orderCAB[i].position);
        REQUIRE(orderABC[i].position == orderBCA[i].position);
        REQUIRE(orderABC[i].velocity == orderCAB[i].velocity);
        REQUIRE(orderABC[i].velocity == orderBCA[i].velocity);
    }
}

// ==========================================================================
// Section 28 - Determinism
// ==========================================================================

TEST_CASE("Determinism - Running the identical simulation twice produces bit-identical final state", "[determinism]")
{
    BodySpec bodyA{Vector3(10.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), 5.0};
    BodySpec bodyB{Vector3(-10.0, 0.0, 0.0), Vector3(0.0, -1.0, 0.0), 5.0};

    std::vector<BodyData> runOne = RunSystem({bodyA, bodyB}, 0.01, 500);
    std::vector<BodyData> runTwo = RunSystem({bodyA, bodyB}, 0.01, 500);

    REQUIRE(runOne[0].position == runTwo[0].position);
    REQUIRE(runOne[1].position == runTwo[1].position);
    REQUIRE(runOne[0].velocity == runTwo[0].velocity);
    REQUIRE(runOne[1].velocity == runTwo[1].velocity);
}

TEST_CASE("Determinism - A collision scenario run twice from identical initial conditions produces identical outcomes", "[determinism]")
{
    auto runCollision = []() {
        PhysicsWorld world;
        Handle a = world.CreateBody(BodyData{Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), 1.0, 1.0, 0.0},
                                    std::make_unique<ShapeSphere>(1.0));
        Handle b = world.CreateBody(BodyData{Vector3(5.0, 0.0, 0.0), Vector3(-1.0, 0.0, 0.0), 1.0, 1.0, 0.0},
                                    std::make_unique<ShapeSphere>(1.0));

        for (int i = 0; i < 300; ++i) {
            world.Step(0.01);
        }

        return std::make_pair(*world.GetBody(a), *world.GetBody(b));
    };

    auto [firstA, firstB] = runCollision();
    auto [secondA, secondB] = runCollision();

    REQUIRE(firstA.position == secondA.position);
    REQUIRE(firstB.position == secondB.position);
    REQUIRE(firstA.velocity == secondA.velocity);
    REQUIRE(firstB.velocity == secondB.velocity);
}

TEST_CASE("Determinism - Interleaving unrelated body creation/destruction before a run does not change its outcome", "[determinism]")
{
    // A determinism failure mode specific to handle-recycling engines:
    // if stale slot data ever leaked through, two runs that reuse
    // memory differently (because of unrelated churn beforehand) could
    // diverge even though the bodies under test are logically identical.
    auto runWithChurn = [](int churnCycles) {
        PhysicsWorld world;

        for (int i = 0; i < churnCycles; ++i) {
            Handle junk = world.CreateBody(BodyData{Vector3(999.0, 999.0, 999.0), Vector3::Zero(), 1.0, 1.0},
                                           std::make_unique<ShapePoint>());
            world.DestroyBody(junk);
        }

        UniversalGravity gravity;
        world.GetForceManager().AddUniversal(&gravity);
        Handle a = world.CreateBody(BodyData{Vector3(10.0, 0.0, 0.0), Vector3(0.0, 1.0, 0.0), 5.0, 0.2, 0.0},
                                    std::make_unique<ShapeSphere>(1.0));
        Handle b = world.CreateBody(BodyData{Vector3(-10.0, 0.0, 0.0), Vector3(0.0, -1.0, 0.0), 5.0, 0.2, 0.0},
                                    std::make_unique<ShapeSphere>(1.0));

        for (int i = 0; i < 500; ++i) {
            world.Step(0.01);
        }

        return std::make_pair(*world.GetBody(a), *world.GetBody(b));
    };

    auto [noChurnA, noChurnB] = runWithChurn(0);
    auto [churnedA, churnedB] = runWithChurn(37);

    REQUIRE(noChurnA.position == churnedA.position);
    REQUIRE(noChurnB.position == churnedB.position);
}
