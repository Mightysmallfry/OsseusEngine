//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_BODYDATA_H
#define OSSEUSENGINE_BODYDATA_H
#include "Osseus/math/Vector3.h"

namespace osseus {
    struct BodyData {
        Vector3 position;
        Vector3 velocity;
        double invMass;     // Inverse Mass 1/kg, 0 == inf
    };
}

#endif //OSSEUSENGINE_BODYDATA_H
