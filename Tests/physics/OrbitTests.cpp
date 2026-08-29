#include <algorithm>
#include <cmath>
#include <memory>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using Catch::Matchers::WithinAbs;

TEST_CASE("Euler-Cromer preserves a circular orbit", "[IntegratorEulerCromer][orbit]")
{
    osseus::PhysicsWorld world;

    world.SetIntegrator(
        std::make_unique<osseus::IntegratorEulerCromer>()
    );

    osseus::UniversalGravity gravity;
    world.GetForceManager().AddUniversal(&gravity);

    // ======================================================================
    // Physical constants
    //
    // G = 1
    // M = 10000
    // r = 100
    //
    // Circular orbital velocity:
    //
    //     v = sqrt(GM / r)
    //       = sqrt(10000 / 100)
    //       = 10
    //
    // Specific orbital energy:
    //
    //     E = v² / 2 - GM / r
    //       = 50 - 100
    //       = -50
    // ======================================================================

    constexpr double gravitationalConstant = 1.0;
    constexpr double centralMass = 10000.0;
    constexpr double particleMass = 1.0;

    constexpr double orbitRadius = 100.0;
    constexpr double orbitalSpeed = 10.0;

    constexpr double delta = 1.0 / 120.0;
    constexpr int steps = 12000;

    constexpr double expectedEnergy =
        -gravitationalConstant * centralMass * particleMass
        / (2.0 * orbitRadius);

    // ======================================================================
    // Create static central body
    // ======================================================================

    const osseus::Handle centralHandle =
        world.CreateBody(
            osseus::BodyData{
                osseus::Vector3::Zero(),
                osseus::Vector3::Zero(),
                centralMass,
                0.0,
                0.0
            },
            std::make_unique<osseus::ShapeSphere>(20.0)
        );

    // ======================================================================
    // Create orbiting particle
    //
    // Position is on +X.
    // Velocity is perpendicular to the radius, along +Y.
    // ======================================================================

    const osseus::Handle particleHandle =
        world.CreateBody(
            osseus::BodyData{
                osseus::Vector3(orbitRadius, 0.0, 0.0),
                osseus::Vector3(0.0, orbitalSpeed, 0.0),
                particleMass,
                1.0 / particleMass,
                0.0
            },
            std::make_unique<osseus::ShapeSphere>(2.0)
        );

    // ======================================================================
    // Validate analytical initial conditions
    // ======================================================================

    const osseus::BodyData* particle =
        world.GetBody(particleHandle);

    REQUIRE(particle != nullptr);

    const double expectedOrbitalSpeed =
        std::sqrt(
            gravitationalConstant
            * centralMass
            / orbitRadius
        );

    REQUIRE_THAT(
        particle->velocity.Length(),
        WithinAbs(expectedOrbitalSpeed, 1e-12)
    );

    const double initialKineticEnergy =
        0.5
        * particleMass
        * particle->velocity.LengthSquared();

    const double initialPotentialEnergy =
        -gravitationalConstant
        * centralMass
        * particleMass
        / particle->position.Length();

    const double initialEnergy =
        initialKineticEnergy
        + initialPotentialEnergy;

    REQUIRE_THAT(
        initialEnergy,
        WithinAbs(expectedEnergy, 1e-12)
    );

    // ======================================================================
    // Simulate
    // ======================================================================

    double maximumRadiusError = 0.0;
    double maximumEnergyError = 0.0;

    for (int step = 0; step < steps; ++step) {
        world.Step(delta);

        particle = world.GetBody(particleHandle);
        const osseus::BodyData* central =
            world.GetBody(centralHandle);

        REQUIRE(particle != nullptr);
        REQUIRE(central != nullptr);

        // --------------------------------------------------------------
        // State must remain finite.
        // --------------------------------------------------------------

        REQUIRE(std::isfinite(particle->position.x));
        REQUIRE(std::isfinite(particle->position.y));
        REQUIRE(std::isfinite(particle->position.z));

        REQUIRE(std::isfinite(particle->velocity.x));
        REQUIRE(std::isfinite(particle->velocity.y));
        REQUIRE(std::isfinite(particle->velocity.z));

        // --------------------------------------------------------------
        // The simulation is intentionally planar.
        // --------------------------------------------------------------

        REQUIRE_THAT(
            particle->position.z,
            WithinAbs(0.0, 1e-12)
        );

        REQUIRE_THAT(
            particle->velocity.z,
            WithinAbs(0.0, 1e-12)
        );

        // --------------------------------------------------------------
        // Central body must remain static.
        // --------------------------------------------------------------

        REQUIRE_THAT(
            central->position.x,
            WithinAbs(0.0, 1e-12)
        );

        REQUIRE_THAT(
            central->position.y,
            WithinAbs(0.0, 1e-12)
        );

        REQUIRE_THAT(
            central->position.z,
            WithinAbs(0.0, 1e-12)
        );

        REQUIRE_THAT(
            central->velocity.Length(),
            WithinAbs(0.0, 1e-12)
        );

        // --------------------------------------------------------------
        // Measure orbital radius.
        // --------------------------------------------------------------

        const double radius =
            particle->position.Length();

        const double radiusError =
            std::abs(radius - orbitRadius);

        maximumRadiusError =
            std::max(maximumRadiusError, radiusError);

        // --------------------------------------------------------------
        // Measure total mechanical energy.
        //
        // For a static central mass:
        //
        //     E = K + U
        //
        // Euler-Cromer does not conserve this exactly. The important
        // property is that the error remains bounded and small.
        // --------------------------------------------------------------

        const double kineticEnergy =
            0.5
            * particleMass
            * particle->velocity.LengthSquared();

        const double potentialEnergy =
            -gravitationalConstant
            * centralMass
            * particleMass
            / radius;

        const double totalEnergy =
            kineticEnergy + potentialEnergy;

        const double energyError =
            std::abs(totalEnergy - initialEnergy);

        maximumEnergyError =
            std::max(maximumEnergyError, energyError);
    }

    // ======================================================================
    // Regression criteria
    // ======================================================================

    // Euler-Cromer should keep the particle close to the intended
    // circular orbit for the entire 100-second simulation.
    REQUIRE(maximumRadiusError < 0.1);

    // Energy should remain tightly bounded around the analytical value.
    REQUIRE(maximumEnergyError < 1e-3);

    // The final state should still be approximately the expected orbit.
    particle = world.GetBody(particleHandle);

    const double finalRadius =
        particle->position.Length();

    const double finalEnergy =
        0.5
        * particleMass
        * particle->velocity.LengthSquared()
        - gravitationalConstant
        * centralMass
        * particleMass
        / finalRadius;

    REQUIRE_THAT(
        finalRadius,
        WithinAbs(orbitRadius, 0.1)
    );

    REQUIRE_THAT(
        finalEnergy,
        WithinAbs(expectedEnergy, 1e-3)
    );
}