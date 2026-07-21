//
// Created by MightySmallFry on 7/21/2026.
//

#ifndef OSSEUSENGINE_FORCECOMPOSITE_H
#define OSSEUSENGINE_FORCECOMPOSITE_H
#include <memory>
#include <vector>

#include "Osseus/interfaces/IForceEvaluator.h"

namespace osseus {
    class ForceComposite : public IForceEvaluator {
    public:
        Vector3 CalculateAcceleration(const BodyData &body) const override;

        void AddForce(std::unique_ptr<IForceEvaluator> force);
    private:
        std::vector<std::unique_ptr<IForceEvaluator>> source_forces;
    };
} // osseus

#endif //OSSEUSENGINE_FORCECOMPOSITE_H
