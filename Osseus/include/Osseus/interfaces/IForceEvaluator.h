//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_FORCEEVALUATOR_H
#define OSSEUSENGINE_FORCEEVALUATOR_H
#include "Osseus/math/Vector3.h"
#include "Osseus/system/BodyData.h"

namespace osseus {
    class IForceEvaluator {
    public:
        virtual ~IForceEvaluator() = default;
        virtual Vector3 CalculateAcceleration(const BodyData& body) const = 0;
    };
} // osseus

#endif //OSSEUSENGINE_FORCEEVALUATOR_H
