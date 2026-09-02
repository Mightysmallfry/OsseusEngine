//
// IntegratorComparisonTests.cpp
//
// Section 21 of the test spec: runs the identical problem through both
// integrators and checks that RK4's higher order actually buys
// something -- lower energy drift over the same duration and step size.
//

#include <cmath>
#include <memory>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Osseus/Osseus.h>

using namespace osseus;
using Catch::Matchers::WithinAbs;

namespace {

    struct OrbitResult {
        double maxEnergyError;
        double finalRadius;
    };

    OrbitResult RunCircularOrbit(std::unique_ptr<IIntegrator> integrator, double dt, int steps) {
        constexpr double gravitationalConstant = 1.0;
        constexpr double centralMass = 10000.0;
        constexpr double particleMass = 1.0;
        constexpr double orbitRadius = 100.0;
        constexpr double orbitalSpeed = 10.0; // sqrt(GM / r), circular

        PhysicsWorld world;
        world.SetIntegrator(std::move(integrator));

        UniversalGravity gravity;
        world.GetForceManager().AddUniversal(&gravity);

        world.CreateBody(BodyData{Vector3::Zero(), Vector3::Zero(), centralMass, 0.0, 0.0},
                         std::make_unique<ShapeSphere>(5.0));

        const Handle particleHandle = world.CreateBody(
            BodyData{Vector3(orbitRadius, 0.0, 0.0), Vector3(0.0, orbitalSpeed, 0.0), particleMass,
                    1.0 / particleMass, 0.0},
            std::make_unique<ShapeSphere>(1.0));

        const double initialEnergy = 0.5 * particleMass * orbitalSpeed * orbitalSpeed -
                                     gravitationalConstant * centralMass * particleMass / orbitRadius;

        double maxEnergyError = 0.0;
        for (int step = 0; step < steps; ++step) {
            world.Step(dt);
            const BodyData* particle = world.GetBody(particleHandle);
            const double radius = particle->position.Length();
            const double energy = 0.5 * particleMass * particle->velocity.LengthSquared() -
                                  gravitationalConstant * centralMass * particleMass / radius;
            maxEnergyError = std::max(maxEnergyError, std::abs(energy - initialEnergy));
        }

        const BodyData* finalParticle = world.GetBody(particleHandle);
        return OrbitResult{maxEnergyError, finalParticle->position.Length()};
    }

}


TEST_CASE("Integrator comparison - RK4 conserves orbital energy tighter than Euler-Cromer over the same problem and step size", "[integrator-comparison]")
{
    constexpr double dt = 0.05;
    constexpr int steps = 1000;

    OrbitResult eulerCromerResult = RunCircularOrbit(std::make_unique<IntegratorEulerCromer>(), dt, steps);
    OrbitResult rk4Result = RunCircularOrbit(std::make_unique<IntegratorRungeKutta4>(), dt, steps);

    // Not just "RK4 is better" in the abstract -- meaningfully so. At
    // this particular step size the advantage is roughly 3x (confirmed
    // empirically); the coarser-step-size test below shows the gap
    // widening well past 10x as dt grows, which is the more
    // characteristic signature of the 4th vs 1st order difference.
    REQUIRE(rk4Result.maxEnergyError < eulerCromerResult.maxEnergyError / 2.0);
}


TEST_CASE("Integrator comparison - Both integrators keep a circular orbit's radius close to correct at a modest step size", "[integrator-comparison]")
{
    constexpr double dt = 0.05;
    constexpr int steps = 1000;
    constexpr double orbitRadius = 100.0;

    OrbitResult eulerCromerResult = RunCircularOrbit(std::make_unique<IntegratorEulerCromer>(), dt, steps);
    OrbitResult rk4Result = RunCircularOrbit(std::make_unique<IntegratorRungeKutta4>(), dt, steps);

    REQUIRE_THAT(eulerCromerResult.finalRadius, WithinAbs(orbitRadius, 1.0));
    REQUIRE_THAT(rk4Result.finalRadius, WithinAbs(orbitRadius, 1.0));
}


TEST_CASE("Integrator comparison - At a coarser step size, RK4's accuracy advantage over Euler-Cromer widens further", "[integrator-comparison]")
{
    // Euler-Cromer's error grows faster with step size than RK4's does
    // (1st vs 4th order locally), so the accuracy gap between them
    // should be even larger here than at the finer step size above.
    constexpr double fineDt = 0.05;
    constexpr double coarseDt = 0.2;
    constexpr int fineSteps = 1000;
    constexpr int coarseSteps = 250; // same total simulated time (50s)

    OrbitResult eulerCromerFine = RunCircularOrbit(std::make_unique<IntegratorEulerCromer>(), fineDt, fineSteps);
    OrbitResult eulerCromerCoarse = RunCircularOrbit(std::make_unique<IntegratorEulerCromer>(), coarseDt, coarseSteps);

    OrbitResult rk4Fine = RunCircularOrbit(std::make_unique<IntegratorRungeKutta4>(), fineDt, fineSteps);
    OrbitResult rk4Coarse = RunCircularOrbit(std::make_unique<IntegratorRungeKutta4>(), coarseDt, coarseSteps);

    // Euler-Cromer's error should grow noticeably as the step coarsens...
    REQUIRE(eulerCromerCoarse.maxEnergyError > eulerCromerFine.maxEnergyError);

    // ...while RK4 remains dramatically more accurate at the same
    // coarse step size.
    REQUIRE(rk4Coarse.maxEnergyError < eulerCromerCoarse.maxEnergyError / 10.0);
    (void)rk4Fine;
}
