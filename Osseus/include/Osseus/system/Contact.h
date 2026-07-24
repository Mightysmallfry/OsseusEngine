//
// Created by MightySmallFry on 7/24/2026.
//

#ifndef OSSEUSENGINE_CONTACT_H
#define OSSEUSENGINE_CONTACT_H
#include "PhysicsHandle.h"
#include "Osseus/math/Vector3.h"

namespace osseus {
    struct Contact {
        Handle a;
        Handle b;
        Vector3 normal;      // Points from body a toward body b
        double penetration;  // Overlap depth along normal
    };
} // osseus

#endif //OSSEUSENGINE_CONTACT_H
