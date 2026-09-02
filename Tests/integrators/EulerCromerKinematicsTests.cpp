//
// EulerCromerKinematicsTests.cpp
//
// Section 19 of the test spec. math/EulerCromerTests.cpp has exactly one
// loose test (falling body, direction-only assertions); everything else
// here was untested.
//
// Euler-Cromer is semi-implicit: velocity is updated first, then
// position is updated using the NEW velocity, not the old one. For
// constant acceleration this makes its result NOT equal to the
// continuous kinematics formula (x = x0 + v0*t + 0.5*a*t^2) -- it has a
// deterministic, closed-form discretization offset instead. Per the
// probe-then-pin approach used throughout this suite, the "matches
// analytic" tests below are pinned against that closed form (derived in
// KinematicsClosedForm below), not the continuous formula, so the
// tolerance can be tight (1e-9) instead of a loose bound that would
// hide a real regression.
//

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;

namespace {

    // Closed-form position/velocity after `steps` fixed-size Euler-Cromer
    // steps under a CONSTANT acceleration `a`, starting from x0/v0.
    //
    // Per step:  v_{k+1} = v_k + a*dt
    //            x_{k+1} = x_k + v_{k+1}*dt
    //
    // v_n = v0 + n*a*dt                              (exact, telescopes)
    // x_n = x0 + dt * sum_{k=1}^{n} v_k
    //     = x0 + n*dt*v0 + a*dt^2 * n*(n+1)/2
    struct KinematicsClosedForm {
        double position;
        double velocity;
    };

    KinematicsClosedForm PredictConstantAcceleration(double x0, double v0, double a, double dt, int steps) {
        const double n = static_cast<double>(steps);
        KinematicsClosedForm result;
        result.velocity = v0 + n * a * dt;
        result.position = x0 + n * dt * v0 + a * dt * dt * n * (n + 1.0) / 2.0;
        return result;
    }

    void RequireWithinAbs(const Vector3& actual, const Vector3& expected, double tolerance) {
        REQUIRE_THAT(actual.x, WithinAbs(expected.x, tolerance));
        REQUIRE_THAT(actual.y, WithinAbs(expected.y, tolerance));
        REQUIRE_THAT(actual.z, WithinAbs(expected.z, tolerance));
    }

}


TEST_CASE("Euler-Cromer - Zero acceleration preserves velocity exactly over many steps", "[Euler-Cromer][kinematics]")
{
    constexpr double dt = 0.05;
    constexpr int steps = 200;

    const Vector3 initialPosition(1.0, -2.0, 3.0);
    const Vector3 initialVelocity(4.0, 5.0, -6.0);

    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorEulerCromer>());

    Handle body = world.CreateBody(BodyData{initialPosition, initialVelocity, 1.0, 1.0}, std::make_unique<ShapePoint>());

    for (int i = 0; i < steps; ++i) {
        world.Step(dt);
    }

    const BodyData* result = world.GetBody(body);
    REQUIRE(result != nullptr);

    // Zero acceleration: no discretization offset at all, this matches
    // continuous kinematics exactly.
    const Vector3 expectedPosition = initialPosition + initialVelocity * (dt * steps);

    RequireWithinAbs(result->velocity, initialVelocity, 1e-9);
    RequireWithinAbs(result->position, expectedPosition, 1e-9);
}


TEST_CASE("Euler-Cromer - Zero initial velocity under constant acceleration matches the scheme's own closed form", "[Euler-Cromer][kinematics]")
{
    constexpr double dt = 0.05;
    constexpr int steps = 40;
    constexpr double acceleration = -9.8;

    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorEulerCromer>());

    Handle body =
        world.CreateBody(BodyData{Vector3(0.0, 10.0, 0.0), Vector3::Zero(), 1.0, 1.0}, std::make_unique<ShapePoint>());

    for (int i = 0; i < steps; ++i) {
        world.GetForceManager().Add(body, Vector3(0.0, acceleration, 0.0));
        world.Step(dt);
    }

    const BodyData* result = world.GetBody(body);
    REQUIRE(result != nullptr);

    KinematicsClosedForm expected = PredictConstantAcceleration(10.0, 0.0, acceleration, dt, steps);

    REQUIRE_THAT(result->velocity.y, WithinAbs(expected.velocity, 1e-9));
    REQUIRE_THAT(result->position.y, WithinAbs(expected.position, 1e-9));
}


TEST_CASE("Euler-Cromer - Nonzero initial velocity under constant acceleration matches the scheme's own closed form", "[Euler-Cromer][kinematics]")
{
    constexpr double dt = 0.02;
    constexpr int steps = 75;
    constexpr double acceleration = -9.8;
    constexpr double initialVelocityY = 15.0; // thrown upward

    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorEulerCromer>());

    Handle body = world.CreateBody(BodyData{Vector3(0.0, 0.0, 0.0), Vector3(0.0, initialVelocityY, 0.0), 1.0, 1.0},
                                   std::make_unique<ShapePoint>());

    for (int i = 0; i < steps; ++i) {
        world.GetForceManager().Add(body, Vector3(0.0, acceleration, 0.0));
        world.Step(dt);
    }

    const BodyData* result = world.GetBody(body);
    REQUIRE(result != nullptr);

    KinematicsClosedForm expected = PredictConstantAcceleration(0.0, initialVelocityY, acceleration, dt, steps);

    REQUIRE_THAT(result->velocity.y, WithinAbs(expected.velocity, 1e-9));
    REQUIRE_THAT(result->position.y, WithinAbs(expected.position, 1e-9));
}


TEST_CASE("Euler-Cromer - Single step position update uses the NEWLY updated velocity, not the pre-step velocity (semi-implicit ordering)", "[Euler-Cromer][semi-implicit]")
{
    // This is the property that actually distinguishes Euler-Cromer from
    // plain (explicit) Euler: v is advanced first, then x uses that
    // advanced v. If a future change accidentally reordered this into
    // explicit Euler, this test is the one that would catch it -- the
    // two schemes predict different positions after a single step
    // whenever acceleration is nonzero.
    constexpr double dt = 0.1;
    constexpr double acceleration = -10.0;
    constexpr double initialVelocityY = 0.0;
    constexpr double initialPositionY = 0.0;

    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorEulerCromer>());

    Handle body = world.CreateBody(
        BodyData{Vector3(0.0, initialPositionY, 0.0), Vector3(0.0, initialVelocityY, 0.0), 1.0, 1.0},
        std::make_unique<ShapePoint>());

    world.GetForceManager().Add(body, Vector3(0.0, acceleration, 0.0));
    world.Step(dt);

    const BodyData* result = world.GetBody(body);
    REQUIRE(result != nullptr);

    const double newVelocity = initialVelocityY + acceleration * dt;
    const double semiImplicitPosition = initialPositionY + newVelocity * dt; // uses NEW velocity
    const double explicitEulerPosition = initialPositionY + initialVelocityY * dt; // would use OLD velocity

    REQUIRE_THAT(result->velocity.y, WithinAbs(newVelocity, 1e-12));
    REQUIRE_THAT(result->position.y, WithinAbs(semiImplicitPosition, 1e-12));

    // And explicitly NOT the explicit-Euler answer (they differ here
    // since acceleration is nonzero: -0.1 vs 0.0).
    REQUIRE(std::abs(result->position.y - explicitEulerPosition) > 1e-6);
}


TEST_CASE("Euler-Cromer - A 2D projectile (combined X velocity and Y acceleration) matches the closed form on both axes independently", "[Euler-Cromer][kinematics]")
{
    constexpr double dt = 0.02;
    constexpr int steps = 100;
    constexpr double gravity = -9.8;
    constexpr double initialVelocityX = 20.0;
    constexpr double initialVelocityY = 15.0;

    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorEulerCromer>());

    Handle body = world.CreateBody(BodyData{Vector3::Zero(), Vector3(initialVelocityX, initialVelocityY, 0.0), 1.0, 1.0},
                                   std::make_unique<ShapePoint>());

    for (int i = 0; i < steps; ++i) {
        world.GetForceManager().Add(body, Vector3(0.0, gravity, 0.0));
        world.Step(dt);
    }

    const BodyData* result = world.GetBody(body);
    REQUIRE(result != nullptr);

    // X has zero acceleration -> exact continuous kinematics.
    const double expectedX = initialVelocityX * (dt * steps);

    // Y has constant acceleration -> the scheme's own closed form.
    KinematicsClosedForm expectedY = PredictConstantAcceleration(0.0, initialVelocityY, gravity, dt, steps);

    REQUIRE_THAT(result->position.x, WithinAbs(expectedX, 1e-9));
    REQUIRE_THAT(result->velocity.x, WithinAbs(initialVelocityX, 1e-9));

    REQUIRE_THAT(result->position.y, WithinAbs(expectedY.position, 1e-9));
    REQUIRE_THAT(result->velocity.y, WithinAbs(expectedY.velocity, 1e-9));
}
