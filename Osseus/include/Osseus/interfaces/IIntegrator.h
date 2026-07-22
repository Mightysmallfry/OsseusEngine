//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_INTEGRATOR_H
#define OSSEUSENGINE_INTEGRATOR_H
#include "IForceEvaluator.h"
#include "Osseus/system/BodyManager.h"

namespace osseus {
    class IIntegrator {
    public:
        virtual ~IIntegrator() = default;
        virtual void Step(BodyManager& manager, const IForceEvaluator& forces, double delta) = 0;
    };
} // osseus

#endif //OSSEUSENGINE_INTEGRATOR_H
