#ifndef OSSEUSENGINE_FORCEEVALUATOR_H
#define OSSEUSENGINE_FORCEEVALUATOR_H
#include "Osseus/math/Vector3.h"
#include "Osseus/system/BodyData.h"

namespace osseus {
    class IForceEvaluator {
       public:
        virtual ~IForceEvaluator() = default;
    };
}  // namespace osseus

#endif  // OSSEUSENGINE_FORCEEVALUATOR_H
