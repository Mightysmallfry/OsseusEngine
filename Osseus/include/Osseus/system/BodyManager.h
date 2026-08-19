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
        void AddBody(Handle handle, BodyData bodyData);
        void RemoveBody(Handle handle);

        BodyData* GetBody(Handle handle);
        const BodyData* GetBody(Handle handle) const;

        

        std::vector<BodyData>& Data() { return bodies.Data(); }
        const std::vector<BodyData>& Data() const { return bodies.Data(); }
        const std::vector<Handle>& Handles() const { return bodies.Handles(); }

    private:
        SparseSet<BodyData> bodies;
    };
} // osseus

#endif //OSSEUSENGINE_BODYMANAGER_H
