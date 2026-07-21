//
// Created by MightySmallFry on 7/21/2026.
//

#include "../../../include/Osseus/physics/Forces/ForceComposite.h"

namespace osseus {
    Vector3 ForceComposite::CalculateAcceleration(const BodyData &body) const {
        Vector3 netAcceleration{ 0.0f, 0.0f, 0.0f };
        for (const auto &force : source_forces) {
            netAcceleration += force->CalculateAcceleration(body);
        }
        return netAcceleration;
    }

    void ForceComposite::AddForce(std::unique_ptr<IForceEvaluator> force) {
        source_forces.push_back(std::move(force));
    }
} // osseus