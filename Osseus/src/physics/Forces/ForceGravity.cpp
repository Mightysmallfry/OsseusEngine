//
// Created by MightySmallFry on 7/20/2026.
//

#include "../../../include/Osseus/physics/Forces/ForceGravity.h"

namespace osseus {
    Vector3 ForceGravity::CalculateAcceleration(const BodyData &body) const {
        return gravity;
    }

    void ForceGravity::SetGravity(const Vector3 &newGravity) {
        gravity = newGravity;
    }
} // osseus