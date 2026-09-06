#ifndef OSSEUSENGINE_INTEGRATOREULERCROMER_H
#define OSSEUSENGINE_INTEGRATOREULERCROMER_H

#include "Osseus/interfaces/IIntegrator.h"

namespace osseus {
    class IntegratorEulerCromer : public IIntegrator {
        public:
        void Step(BodyManager& bodymanager, ForceManager& forceManager, double delta) override;
        std::string GetName() override { return "EULER-CROMER"; }
    };
} // namespace osseus

#endif // OSSEUSENGINE_INTEGRATOREULERCROMER_H
