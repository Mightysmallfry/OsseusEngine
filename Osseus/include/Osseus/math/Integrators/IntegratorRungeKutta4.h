//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_INTEGRATORRUNGEKUTTA4_H
#define OSSEUSENGINE_INTEGRATORRUNGEKUTTA4_H

#include "Osseus/interfaces/IIntegrator.h"
#include "Osseus/system/SimulationState.h"
#include "Osseus/physics/BarnesHut.h"

namespace osseus {
    class IntegratorRungeKutta4 : public IIntegrator{

    public:
        void Step(BodyManager &bodyManager, ForceManager &forceManager, double delta) override;

    private:
        void CopyState(SimulationState& state, const BodyManager& bodyManager);

        BarnesHut barnesHut_;
    };
} // osseus

#endif //OSSEUSENGINE_INTEGRATORRUNGEKUTTA4_H
