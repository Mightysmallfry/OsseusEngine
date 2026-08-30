#ifndef OSSEUSENGINE_INTEGRATORRUNGEKUTTA4_H
#define OSSEUSENGINE_INTEGRATORRUNGEKUTTA4_H

#include "Osseus/interfaces/IIntegrator.h"
#include "Osseus/physics/BarnesHut.h"
#include "Osseus/system/SimulationState.h"

namespace osseus {

    struct DerivativeState {
        std::vector<Vector3> velocities;
        std::vector<Vector3> accelerations;
    };

    class IntegratorRungeKutta4 : public IIntegrator {
        public:
        void Step(BodyManager& bodyManager, ForceManager& forceManager, double delta) override;
        std::string GetName() { return "4TH-ORDER RUNGE-KUTTA"; }

        private:
        DerivativeState DerivativeOf(SimulationState& state);

        void CopyState(SimulationState& state, const BodyManager& bodyManager);

        // Re-evaluates universal (position-dependent) forces against this
        // stage's rebuilt octree, on top of the constant individual-force
        // contribution, and syncs the result into the stage's SimulationState.
        void EvaluateStageForces(SimulationState& state, const std::vector<Vector3>& individualForces,
                                  const ForceManager& universalTemplate);

        BarnesHut barnesHut_;
    };
} // namespace osseus

#endif // OSSEUSENGINE_INTEGRATORRUNGEKUTTA4_H