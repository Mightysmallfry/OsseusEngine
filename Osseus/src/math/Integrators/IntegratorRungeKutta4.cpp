//
// Created by MightySmallFry on 7/20/2026.
//

#include "Osseus/math/Integrators/IntegratorRungeKutta4.h"

namespace osseus {

    DerivativeState IntegratorRungeKutta4::DerivativeOf(SimulationState& state){
        DerivativeState derivative;

        size_t stateSize = state.GetBodies().size();
        derivative.velocities.resize(stateSize);
        derivative.accelerations.resize(stateSize);

        std::vector<BodyData>& bodies = state.GetBodies();
        std::vector<Vector3>& netForces = state.GetNetForces();

        for (size_t i = 0; i < state.GetHandles().size(); i++){
            derivative.velocities[i] = bodies[i].velocity;
            derivative.accelerations[i] =  netForces[i] / bodies[i].mass;
        }

        return derivative;
    }
        
    void IntegratorRungeKutta4::Step(BodyManager &bodyManager, ForceManager &forceManager, double delta) {
     
        SimulationState state1(bodyManager, forceManager);
        SimulationState state2 = state1;
        SimulationState state3 = state1;
        SimulationState state4 = state1;
        size_t stateSize = state1.GetBodies().size();

        
        DerivativeState k1 = DerivativeOf(state1);
        for (std::size_t i = 0; i < stateSize; ++i)
        {
            state2.GetBodies()[i].position += k1.velocities[i] * (delta * 0.5);
            state2.GetBodies()[i].velocity += k1.accelerations[i] * (delta * 0.5);
        }
        state2.RebuildOctree();
        
        barnesHut_.Evaluate(
            state2.GetOctree(), 
            state2.GetHandles(), 
            state2.GetBodies(), 
            state2.GetNetForces());
    
        DerivativeState k2 = DerivativeOf(state2);
        for (std::size_t i = 0; i < stateSize; ++i)
        {
            state3.GetBodies()[i].position += k2.velocities[i] * (delta * 0.5);
            state3.GetBodies()[i].velocity += k2.accelerations[i] * (delta * 0.5);
        }
        state3.RebuildOctree();

        barnesHut_.Evaluate(
            state3.GetOctree(),
            state3.GetHandles(),
            state3.GetBodies(),
            state3.GetNetForces()
        );

        DerivativeState k3 = DerivativeOf(state3);    
        for (std::size_t i = 0; i < stateSize; ++i)
        {
            state4.GetBodies()[i].position += k3.velocities[i] * delta;
            state4.GetBodies()[i].velocity += k3.accelerations[i] * delta;
        }

        state4.RebuildOctree();

        barnesHut_.Evaluate(
            state4.GetOctree(),
            state4.GetHandles(),
            state4.GetBodies(),
            state4.GetNetForces()
        );

        DerivativeState k4 = DerivativeOf(state4);

    
        std::vector<BodyData>& bodies = bodyManager.Data();

        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            bodies[i].position +=
                (k1.velocities[i]
                + 2.0 * k2.velocities[i]
                + 2.0 * k3.velocities[i]
                + k4.velocities[i]) * (delta / 6.0);

            bodies[i].velocity +=
                (k1.accelerations[i]
                + 2.0 * k2.accelerations[i]
                + 2.0 * k3.accelerations[i]
                + k4.accelerations[i]) * (delta / 6.0);
        }


    }
} // osseus