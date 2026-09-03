//
// SimulationStateTests.cpp
//
// Section 22 of the test spec -- previously untested entirely.
//
// TWO FINDINGS surfaced while writing these, both left unfixed (that's
// a call for whoever triages this, not something to paper over here):
//
// 1. SimulationState::Resize() is declared in the header but has NO
//    implementation anywhere in the codebase (confirmed via grep across
//    Osseus/src and Osseus/include). Calling it would fail to link.
//    None of the tests below call it for exactly that reason.
//
// 2. SimulationState's copy constructor only copies bodyData_,
//    handles_, and netForces_ -- it silently drops universalForces_ and
//    leaves octree_ default-constructed (empty), unlike CopyFrom()
//    which populates and rebuilds both. A test below pins this as
//    CURRENT BEHAVIOR so a fix shows up as a visible, intentional test
//    change.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;

namespace {
    void RequireWithinAbs(const Vector3& actual, const Vector3& expected, double tolerance) {
        REQUIRE_THAT(actual.x, WithinAbs(expected.x, tolerance));
        REQUIRE_THAT(actual.y, WithinAbs(expected.y, tolerance));
        REQUIRE_THAT(actual.z, WithinAbs(expected.z, tolerance));
    }
}


TEST_CASE("SimulationState - CopyFrom captures every body's data verbatim", "[simulationstate]")
{
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle a{0, 0};
    Handle b{1, 0};
    bodyManager.AddBody(a, BodyData{Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3), 5.0, 0.2, 1.5});
    bodyManager.AddBody(b, BodyData{Vector3(-4.0, 0.0, 2.0), Vector3::Zero(), 2.0, 0.5, -1.0});
    forceManager.Register(a);
    forceManager.Register(b);
    forceManager.Add(a, Vector3(7.0, 0.0, 0.0));

    SimulationState state(bodyManager, forceManager);

    REQUIRE(state.GetHandles().size() == 2);
    REQUIRE(state.GetBodies().size() == 2);
    REQUIRE(state.GetNetForces().size() == 2);

    RequireWithinAbs(state.GetBodies()[a.index].position, Vector3(1.0, 2.0, 3.0), 1e-9);
    RequireWithinAbs(state.GetBodies()[b.index].position, Vector3(-4.0, 0.0, 2.0), 1e-9);
    RequireWithinAbs(state.GetNetForces()[a.index], Vector3(7.0, 0.0, 0.0), 1e-9);
}


TEST_CASE("SimulationState - CopyFrom rebuilds an octree whose aggregate mass matches the source bodies", "[simulationstate][octree]")
{
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle a{0, 0};
    Handle b{1, 0};
    bodyManager.AddBody(a, BodyData{Vector3(5.0, 0.0, 0.0), Vector3::Zero(), 3.0, 1.0, 0.0});
    bodyManager.AddBody(b, BodyData{Vector3(-5.0, 0.0, 0.0), Vector3::Zero(), 7.0, 1.0, 0.0});

    SimulationState state(bodyManager, forceManager);

    REQUIRE(state.GetOctree().GetRoot().GetTotalMass() == 10.0);
}


TEST_CASE("SimulationState - RebuildOctree reflects bodyData_ as currently held, not as it was at construction", "[simulationstate][octree]")
{
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle a{0, 0};
    bodyManager.AddBody(a, BodyData{Vector3(1.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0});

    SimulationState state(bodyManager, forceManager);
    REQUIRE(state.GetOctree().GetRoot().GetCenterOfMass() == Vector3(1.0, 0.0, 0.0));

    // Mutate the state's own copy of the body data directly (this is
    // exactly what RK4's per-stage evaluation does between stages) and
    // rebuild.
    state.GetBodies()[a.index].position = Vector3(9.0, 0.0, 0.0);
    state.RebuildOctree();

    REQUIRE(state.GetOctree().GetRoot().GetCenterOfMass() == Vector3(9.0, 0.0, 0.0));
}


TEST_CASE("SimulationState - CopyFrom makes an independent copy; mutating the source BodyManager afterward does not affect it", "[simulationstate][independence]")
{
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle a{0, 0};
    bodyManager.AddBody(a, BodyData{Vector3(1.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0});

    SimulationState state(bodyManager, forceManager);

    bodyManager.GetBody(a)->position = Vector3(999.0, 999.0, 999.0);

    RequireWithinAbs(state.GetBodies()[a.index].position, Vector3(1.0, 0.0, 0.0), 1e-9);
}


TEST_CASE("SimulationState - Mutating a SimulationState's copied bodies does not affect the source BodyManager", "[simulationstate][independence]")
{
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle a{0, 0};
    bodyManager.AddBody(a, BodyData{Vector3(1.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0});

    SimulationState state(bodyManager, forceManager);
    state.GetBodies()[a.index].position = Vector3(999.0, 999.0, 999.0);

    RequireWithinAbs(bodyManager.GetBody(a)->position, Vector3(1.0, 0.0, 0.0), 1e-9);
}


TEST_CASE("SimulationState - CopyFrom called again on the same instance fully overwrites the previous snapshot", "[simulationstate]")
{
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle a{0, 0};
    bodyManager.AddBody(a, BodyData{Vector3(1.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0});

    SimulationState state(bodyManager, forceManager);
    REQUIRE(state.GetBodies().size() == 1);

    Handle b{1, 0};
    bodyManager.AddBody(b, BodyData{Vector3(2.0, 0.0, 0.0), Vector3::Zero(), 1.0, 1.0, 0.0});

    state.CopyFrom(bodyManager, forceManager);

    REQUIRE(state.GetBodies().size() == 2);
    REQUIRE(state.GetOctree().GetRoot().GetTotalMass() == 2.0);
}


TEST_CASE("SimulationState - CURRENT BEHAVIOR: the copy constructor does not rebuild the octree from the copied body data", "[simulationstate][regression]")
{
    // Documented gap: SimulationState(const SimulationState&) only
    // copies bodyData_/handles_/netForces_, not universalForces_, and
    // never calls RebuildOctree(). A freshly copy-constructed state's
    // octree is therefore left at its default-constructed (empty)
    // state even though GetBodies() reports real data.
    BodyManager bodyManager;
    ForceManager forceManager(bodyManager);

    Handle a{0, 0};
    bodyManager.AddBody(a, BodyData{Vector3(5.0, 0.0, 0.0), Vector3::Zero(), 3.0, 1.0, 0.0});

    SimulationState original(bodyManager, forceManager);
    REQUIRE(original.GetOctree().GetRoot().GetTotalMass() == 3.0); // original is correctly populated

    SimulationState copy(original);

    // ACTUAL: the body data made it across...
    REQUIRE(copy.GetBodies().size() == 1);
    RequireWithinAbs(copy.GetBodies()[a.index].position, Vector3(5.0, 0.0, 0.0), 1e-9);

    // ...but the octree did not, and reports no mass until something
    // explicitly calls RebuildOctree() on the copy.
    REQUIRE(copy.GetOctree().GetRoot().GetTotalMass() == 0.0);

    // Calling RebuildOctree() explicitly on the copy recovers correct
    // aggregates, confirming the gap is specifically "the copy
    // constructor doesn't call it," not a deeper data problem.
    copy.RebuildOctree();
    REQUIRE(copy.GetOctree().GetRoot().GetTotalMass() == 3.0);
}
