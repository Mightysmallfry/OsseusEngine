//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_INTEGRATOREULERCROMER_H
#define OSSEUSENGINE_INTEGRATOREULERCROMER_H
#include "Osseus/interfaces/IIntegrator.h"

namespace osseus {
    class IntegratorEulerCromer : public IIntegrator {
    public:
        void Step(BodyManager &bodymanager, ForceManager &forceManager, double delta) override;
    };
} // osseus

#endif //OSSEUSENGINE_INTEGRATOREULERCROMER_H
