#ifndef OSSEUSENGINE_INTEGRATOR_H
#define OSSEUSENGINE_INTEGRATOR_H
#include "IForceEvaluator.h"
#include "Osseus/system/BodyManager.h"
#include "Osseus/system/ForceManager.h"

namespace osseus {
    class IIntegrator {
        public:
        virtual ~IIntegrator() = default;
        virtual void Step(BodyManager& bodyManager, ForceManager& forceManager, double delta) = 0;
    };
} // namespace osseus

#endif // OSSEUSENGINE_INTEGRATOR_H
