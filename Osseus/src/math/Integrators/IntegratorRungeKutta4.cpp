#include "Osseus/math/Integrators/IntegratorRungeKutta4.h"

namespace osseus {

    DerivativeState IntegratorRungeKutta4::DerivativeOf(SimulationState& state) {
        DerivativeState derivative;

        size_t stateSize = state.GetBodies().size();
        derivative.velocities.resize(stateSize);
        derivative.accelerations.resize(stateSize);

        std::vector<BodyData>& bodies = state.GetBodies();
        std::vector<Vector3>& netForces = state.GetNetForces();

        for (size_t i = 0; i < state.GetHandles().size(); i++) {
            derivative.velocities[i] = bodies[i].velocity;
            derivative.accelerations[i] = netForces[i] / bodies[i].mass;
        }

        return derivative;
    }

    void IntegratorRungeKutta4::EvaluateStageForces(SimulationState& state,
                                                     const std::vector<Vector3>& individualForces,
                                                     const ForceManager& universalTemplate) {
        // Fresh copy just to reuse the registered universal-force evaluator
        // list; its net-force values get fully overwritten below.
        ForceManager stageForces = universalTemplate;
        stageForces.NetForces() = individualForces;

        barnesHut_.Evaluate(state.GetOctree(), state.GetHandles(), state.GetBodies(), stageForces);

        state.GetNetForces() = stageForces.NetForces();
    }

    void IntegratorRungeKutta4::Step(BodyManager& bodyManager, ForceManager& forceManager, double delta) {

        SimulationState state1(bodyManager, forceManager);
        SimulationState state2 = state1;
        SimulationState state3 = state1;
        SimulationState state4 = state1;

        size_t stateSize = state1.GetBodies().size();

        // state1's net forces (copied from forceManager) already contain
        // individual + universal(t0), since PhysicsWorld::Step evaluates
        // universal forces before handing off to the integrator. Isolate
        // the universal(t0) portion so we can subtract it back out and
        // recover the individual (non-positional) contribution alone -
        // that's the part that should carry forward unchanged into every
        // stage, while the universal portion gets re-evaluated fresh each
        // time against that stage's rebuilt octree.
        ForceManager universalOnly = forceManager;
        universalOnly.Clear();
        barnesHut_.Evaluate(state1.GetOctree(), state1.GetHandles(), state1.GetBodies(), universalOnly);

        std::vector<Vector3> individualForces(stateSize);
        for (std::size_t i = 0; i < stateSize; ++i) {
            individualForces[i] = state1.GetNetForces()[i] - universalOnly.NetForces()[i];
        }

        DerivativeState k1 = DerivativeOf(state1);
        for (std::size_t i = 0; i < stateSize; ++i) {
            state2.GetBodies()[i].position += k1.velocities[i] * (delta * 0.5);
            state2.GetBodies()[i].velocity += k1.accelerations[i] * (delta * 0.5);
        }
        state2.RebuildOctree();
        EvaluateStageForces(state2, individualForces, universalOnly);

        DerivativeState k2 = DerivativeOf(state2);
        for (std::size_t i = 0; i < stateSize; ++i) {
            state3.GetBodies()[i].position += k2.velocities[i] * (delta * 0.5);
            state3.GetBodies()[i].velocity += k2.accelerations[i] * (delta * 0.5);
        }
        state3.RebuildOctree();
        EvaluateStageForces(state3, individualForces, universalOnly);

        DerivativeState k3 = DerivativeOf(state3);
        for (std::size_t i = 0; i < stateSize; ++i) {
            state4.GetBodies()[i].position += k3.velocities[i] * delta;
            state4.GetBodies()[i].velocity += k3.accelerations[i] * delta;
        }
        state4.RebuildOctree();
        EvaluateStageForces(state4, individualForces, universalOnly);

        DerivativeState k4 = DerivativeOf(state4);

        std::vector<BodyData>& bodies = bodyManager.Data();

        for (std::size_t i = 0; i < bodies.size(); ++i) {
            bodies[i].position +=
                (k1.velocities[i] + 2.0 * k2.velocities[i] + 2.0 * k3.velocities[i] + k4.velocities[i]) * (delta / 6.0);

            bodies[i].velocity +=
                (k1.accelerations[i] + 2.0 * k2.accelerations[i] + 2.0 * k3.accelerations[i] + k4.accelerations[i]) *
                (delta / 6.0);
        }
    }
} // namespace osseus