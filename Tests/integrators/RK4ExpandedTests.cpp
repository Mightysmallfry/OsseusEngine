//
// RK4ExpandedTests.cpp
//
// Section 20 additions: a harmonic oscillator (the textbook case for
// exercising per-stage re-evaluation of a position-dependent force --
// directly related to the "silent reuse of t0 acceleration" bug fixed
// earlier in RK4) and an elliptical orbit (the existing RK4Tests.cpp
// only covers the circular case).
//

#include <algorithm>
#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;

namespace {

    // A simple global restoring force F = -k * position, anchored at the
    // origin. Deliberately ignores the source octree node entirely --
    // it isn't an N-body force, it's a controlled, analytically-solvable
    // test system (simple harmonic motion) used purely to verify that
    // the integrator re-evaluates a position-dependent force at each of
    // its internal stages rather than reusing the force computed at the
    // start of the step.
    class SpringForce : public UniversalForceEvaluator {
        public:
        explicit SpringForce(double springConstant) : springConstant_(springConstant) {
        }

        Vector3 CalculateForce(const OctNode&, Handle, const BodyData& body) override {
            return body.position * -springConstant_;
        }

        private:
        double springConstant_;
    };

}


TEST_CASE("RK4 - A harmonic oscillator matches the analytic solution closely (position-dependent force re-evaluated per stage)", "[RK4][harmonic-oscillator]")
{
    constexpr double springConstant = 4.0;
    constexpr double mass = 1.0;
    const double omega = std::sqrt(springConstant / mass); // 2.0

    constexpr double dt = 0.01;
    constexpr int steps = 200; // 2.0 seconds; period is 2*pi/omega ~= 3.14s

    const Vector3 initialPosition(5.0, 0.0, 0.0);

    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorRungeKutta4>());

    SpringForce spring(springConstant);
    world.GetForceManager().AddUniversal(&spring);

    Handle body = world.CreateBody(BodyData{initialPosition, Vector3::Zero(), mass, 1.0 / mass},
                                   std::make_unique<ShapePoint>());

    for (int i = 0; i < steps; ++i) {
        world.Step(dt);
    }

    const BodyData* result = world.GetBody(body);
    REQUIRE(result != nullptr);

    const double t = dt * steps;
    const double expectedX = initialPosition.x * std::cos(omega * t);
    const double expectedVx = -initialPosition.x * omega * std::sin(omega * t);

    REQUIRE_THAT(result->position.x, WithinAbs(expectedX, 1e-3));
    REQUIRE_THAT(result->velocity.x, WithinAbs(expectedVx, 1e-3));

    // Motion stays on the axis it started on.
    REQUIRE_THAT(result->position.y, WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(result->position.z, WithinAbs(0.0, 1e-9));
}


TEST_CASE("RK4 - A harmonic oscillator conserves mechanical energy far better than a single-stage method would", "[RK4][harmonic-oscillator]")
{
    // Regression guard specifically for the "reuse t0 acceleration"
    // failure mode: if RK4 silently degraded to evaluating the force
    // only once per step (Euler-like), the energy drift over many
    // periods would be dramatically larger than what's asserted here.
    constexpr double springConstant = 4.0;
    constexpr double mass = 1.0;

    constexpr double dt = 0.01;
    constexpr int steps = 2000; // several full periods

    const Vector3 initialPosition(5.0, 0.0, 0.0);

    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorRungeKutta4>());

    SpringForce spring(springConstant);
    world.GetForceManager().AddUniversal(&spring);

    Handle body = world.CreateBody(BodyData{initialPosition, Vector3::Zero(), mass, 1.0 / mass},
                                   std::make_unique<ShapePoint>());

    const double initialEnergy =
        0.5 * mass * 0.0 /* v0 = 0 */ + 0.5 * springConstant * initialPosition.LengthSquared();

    double maxEnergyError = 0.0;
    for (int i = 0; i < steps; ++i) {
        world.Step(dt);
        const BodyData* result = world.GetBody(body);
        REQUIRE(std::isfinite(result->position.x));

        const double energy =
            0.5 * mass * result->velocity.LengthSquared() + 0.5 * springConstant * result->position.LengthSquared();
        maxEnergyError = std::max(maxEnergyError, std::abs(energy - initialEnergy));
    }

    REQUIRE(maxEnergyError < 1e-2);
}


TEST_CASE("RK4 - An elliptical orbit (sub-circular speed) stays bounded, oscillates in radius, and conserves energy and angular momentum tightly", "[RK4][orbit][elliptical]")
{
    constexpr double gravitationalConstant = 1.0;
    constexpr double centralMass = 10000.0;
    constexpr double particleMass = 1.0;

    constexpr double startRadius = 100.0;
    constexpr double circularSpeed = 10.0; // sqrt(GM / r)
    constexpr double orbitalSpeed = 7.0;   // sub-circular -> elliptical, starts at aphelion

    constexpr double dt = 1.0 / 120.0;
    constexpr int steps = 4200; // a bit over one full period of the resulting ellipse

    PhysicsWorld world;
    world.SetIntegrator(std::make_unique<IntegratorRungeKutta4>());

    UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), centralMass, 0.0, 0.0},
                     std::make_unique<ShapeSphere>(5.0));

    const Handle particleHandle = world.CreateBody(
        BodyData{Vector3(startRadius, 0.0, 0.0), Vector3(0.0, orbitalSpeed, 0.0), particleMass, 1.0 / particleMass, 0.0},
        std::make_unique<ShapeSphere>(1.0));

    REQUIRE(orbitalSpeed < circularSpeed); // sanity: this really is sub-circular

    const double initialEnergy =
        0.5 * particleMass * orbitalSpeed * orbitalSpeed - gravitationalConstant * centralMass * particleMass / startRadius;
    const double initialAngularMomentum = startRadius * orbitalSpeed; // r x v, planar, starts perpendicular

    double minRadius = startRadius;
    double maxRadius = startRadius;
    double maxEnergyError = 0.0;
    double maxAngularMomentumError = 0.0;

    for (int step = 0; step < steps; ++step) {
        world.Step(dt);

        const BodyData* particle = world.GetBody(particleHandle);
        REQUIRE(particle != nullptr);
        REQUIRE(std::isfinite(particle->position.x));
        REQUIRE(std::isfinite(particle->position.y));

        const double radius = particle->position.Length();
        minRadius = std::min(minRadius, radius);
        maxRadius = std::max(maxRadius, radius);

        const double energy = 0.5 * particleMass * particle->velocity.LengthSquared() -
                              gravitationalConstant * centralMass * particleMass / radius;
        maxEnergyError = std::max(maxEnergyError, std::abs(energy - initialEnergy));

        const double angularMomentum =
            particle->position.x * particle->velocity.y - particle->position.y * particle->velocity.x;
        maxAngularMomentumError = std::max(maxAngularMomentumError, std::abs(angularMomentum - initialAngularMomentum));
    }

    // Sub-circular speed at the starting radius means the body starts
    // at aphelion and swings in closer -> perihelion should be
    // meaningfully below the starting radius, and aphelion should stay
    // close to it (energy-conserving bound orbit, not an escape).
    REQUIRE(minRadius < startRadius * 0.9);
    REQUIRE(maxRadius <= startRadius * 1.01);

    // RK4 should conserve both far tighter than Euler-Cromer does over
    // the same problem.
    REQUIRE(maxEnergyError < 1e-2);
    REQUIRE(maxAngularMomentumError < 1e-2);
}
