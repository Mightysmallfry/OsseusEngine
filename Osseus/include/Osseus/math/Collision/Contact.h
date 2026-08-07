#ifndef OSSEUSENGINE_CONTACT_H
#define OSSEUSENGINE_CONTACT_H
#include "Osseus/math/Vector3.h"
#include "Osseus/system/PhysicsHandle.h"

namespace osseus {
    struct Contact {
        Handle a;
        Handle b;
        Vector3 normal;             // unit vector, points from body A toward body B
        Vector3 pointOnA;           // world-space witness point on A's surface
        Vector3 pointOnB;           // world-space witness point on B's surface
        double penetration{ 0.0 };  // depth of overlap along normal, >= 0
    };
} // osseus

#endif //OSSEUSENGINE_CONTACT_H
