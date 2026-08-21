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

        void Register(Handle handle);


        BodyData* GetBody(Handle handle);
        const BodyData* GetBody(Handle handle) const;

        

        std::vector<BodyData>& Data() { return bodies_.Data(); }
        const std::vector<BodyData>& Data() const { return bodies_.Data(); }
        const std::vector<Handle>& Handles() const { return bodies_.Handles(); }

    private:
        SparseSet<BodyData> bodies_;
    };
} // osseus

#endif //OSSEUSENGINE_BODYMANAGER_H
