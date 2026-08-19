//
// Created by MightySmallFry on 7/20/2026.
//

#include "Osseus/math/Integrators/IntegratorRungeKutta4.h"

namespace osseus {

    void IntegratorRungeKutta4::CopyState(SimulationState& state, const BodyManager& bodyManager)
    {
        state.Resize(bodyManager.Data().size());

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);
            
            if (body == nullptr)
            {
                continue;
            }

            state.GetPosition(handle) = body->position;
            state.GetVelocity(handle) = body->velocity;
        }
    }


    
    void IntegratorRungeKutta4::Step(BodyManager &bodyManager, ForceManager &forceManager, double delta) {
     
        /*
        * State at the beginning of the timestep.
        */
        SimulationState state1(bodyManager);

        /*
        * Temporary RK4 states.
        */
        SimulationState state2;
        SimulationState state3;
        SimulationState state4;

        state2.CopyState(state1);
        state3.CopyState(state1);
        state4.CopyState(state1);

        
        /*
        * Force managers for each RK4 stage.
        *
        * These contain independent force accumulators but
        * reference the same universal force evaluators.
        */
        ForceManager force1 = forceManager;
        ForceManager force2 = forceManager;
        ForceManager force3 = forceManager;
        ForceManager force4 = forceManager;

        /*
        * RK4 derivatives.
        */
        SimulationState k1(bodyManager);
        SimulationState k2(bodyManager);
        SimulationState k3(bodyManager);
        SimulationState k4(bodyManager);
        
        /*
        * The k states are only derivative containers, so their
        * Octrees aren't needed.
        */
        
          /*
         * ---------------------------------------------------------
         * Stage 1
         * ---------------------------------------------------------
         */

        barnesHut_.Evaluate(
            state1.GetOctree(),
            state1,
            force1
        );

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr || body->invMass == 0.0)
            {
                continue;
            }

            k1.GetPosition(handle) =
                state1.GetVelocity(handle);

            k1.GetVelocity(handle) =
                force1.Get(handle) * body->invMass;
        }

        /*
         * ---------------------------------------------------------
         * Stage 2
         * ---------------------------------------------------------
         */

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr || body->invMass == 0.0)
            {
                continue;
            }

            state2.GetPosition(handle) =
                state1.GetPosition(handle)
                + k1.GetPosition(handle) * (delta / 2.0);

            state2.GetVelocity(handle) =
                state1.GetVelocity(handle)
                + k1.GetVelocity(handle) * (delta / 2.0);
        }

        state2.RebuildOctree(bodyManager);

        barnesHut_.Evaluate(
            state2.GetOctree(),
            state2,
            force2
        );

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr || body->invMass == 0.0)
            {
                continue;
            }

            k2.GetPosition(handle) =
                state2.GetVelocity(handle);

            k2.GetVelocity(handle) =
                force2.Get(handle) * body->invMass;
        }

        /*
         * ---------------------------------------------------------
         * Stage 3
         * ---------------------------------------------------------
         */

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr || body->invMass == 0.0)
            {
                continue;
            }

            state3.GetPosition(handle) =
                state1.GetPosition(handle)
                + k2.GetPosition(handle) * (delta / 2.0);

            state3.GetVelocity(handle) =
                state1.GetVelocity(handle)
                + k2.GetVelocity(handle) * (delta / 2.0);
        }

        state3.RebuildOctree(bodyManager);

        barnesHut_.Evaluate(
            state3.GetOctree(),
            state3,
            force3
        );

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr || body->invMass == 0.0)
            {
                continue;
            }

            k3.GetPosition(handle) =
                state3.GetVelocity(handle);

            k3.GetVelocity(handle) =
                force3.Get(handle) * body->invMass;
        }

        /*
         * ---------------------------------------------------------
         * Stage 4
         * ---------------------------------------------------------
         */

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr || body->invMass == 0.0)
            {
                continue;
            }

            state4.GetPosition(handle) =
                state1.GetPosition(handle)
                + k3.GetPosition(handle) * delta;

            state4.GetVelocity(handle) =
                state1.GetVelocity(handle)
                + k3.GetVelocity(handle) * delta;
        }

        state4.RebuildOctree(bodyManager);

        barnesHut_.Evaluate(
            state4.GetOctree(),
            state4,
            force4
        );

        for (Handle handle : bodyManager.Handles())
        {
            const BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr || body->invMass == 0.0)
            {
                continue;
            }

            k4.GetPosition(handle) =
                state4.GetVelocity(handle);

            k4.GetVelocity(handle) =
                force4.Get(handle) * body->invMass;
        }

        /*
         * ---------------------------------------------------------
         * Final RK4 combination
         * ---------------------------------------------------------
         */

        for (Handle handle : bodyManager.Handles())
        {
            BodyData* body = bodyManager.GetBody(handle);

            if (body == nullptr || body->invMass == 0.0)
            {
                continue;
            }

            body->position =
                state1.GetPosition(handle)
                + (
                    k1.GetPosition(handle)
                    + k2.GetPosition(handle) * 2.0
                    + k3.GetPosition(handle) * 2.0
                    + k4.GetPosition(handle)
                ) * (delta / 6.0);

            body->velocity =
                state1.GetVelocity(handle)
                + (
                    k1.GetVelocity(handle)
                    + k2.GetVelocity(handle) * 2.0
                    + k3.GetVelocity(handle) * 2.0
                    + k4.GetVelocity(handle)
                ) * (delta / 6.0);
        }
        
    }
} // osseus