//
// Created by MightySmallFry on 7/20/2026.
//

#ifndef OSSEUSENGINE_BODYMANAGER_H
#define OSSEUSENGINE_BODYMANAGER_H
#include "BodyData.h"
#include "SparseSet.h"
#include "Osseus/math/Vector3.h"

namespace osseus {
    class BodyManager {
    public:
        SparseSet<BodyData> bodies;
    };
} // osseus

#endif //OSSEUSENGINE_BODYMANAGER_H
