//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_INTEGRATORRUNGEKUTTA4_H
#define OSSEUSENGINE_INTEGRATORRUNGEKUTTA4_H
#include "Osseus/interfaces/IIntegrator.h"

namespace osseus {
    class IntegratorRungeKutta4 : IIntegrator{
    public:
        void step(BodyManager &manager, const IForceEvaluator &forces, double delta) override;
    };
} // osseus

#endif //OSSEUSENGINE_INTEGRATORRUNGEKUTTA4_H
