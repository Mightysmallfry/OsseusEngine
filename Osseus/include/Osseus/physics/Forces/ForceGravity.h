//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_FORCEGRAVITY_H
#define OSSEUSENGINE_FORCEGRAVITY_H
#include "Osseus/interfaces/IForceEvaluator.h"

namespace osseus {
    class ForceGravity : public IForceEvaluator {
    public:
        explicit ForceGravity(const Vector3 &newGravity = Vector3(0.0, -9.81, 0.0))
            : gravity(newGravity) {}

        Vector3 CalculateAcceleration(const BodyData &body) const override;

        void SetGravity(const Vector3 &newGravity);
        Vector3 GetGravity() const { return gravity; }

    private:
        Vector3 gravity;
    };
} // osseus

#endif //OSSEUSENGINE_FORCEGRAVITY_H
