#ifndef OSSEUSENGINE_AABB_H
#define OSSEUSENGINE_AABB_H

#include "Osseus/math/Vector3.h"

namespace osseus {
    struct AABB {
        Vector3 min;
        Vector3 max;
    };
} // namespace osseus

#endif // OSSEUSENGINE_AABB_H
