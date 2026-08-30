#ifndef OSSEUSENGINE_BODYMANAGER_H
#define OSSEUSENGINE_BODYMANAGER_H

#include "BodyData.h"
#include "Osseus/math/Vector3.h"
#include "SparseSet.h"

namespace osseus {
    class BodyManager {
        public:
        void AddBody(Handle handle, BodyData bodyData);
        void RemoveBody(Handle handle);

        void Register(Handle handle);

        BodyData* GetBody(Handle handle);
        const BodyData* GetBody(Handle handle) const;

        std::vector<BodyData>& Data() {
            return bodies_.Data();
        }
        const std::vector<BodyData>& Data() const {
            return bodies_.Data();
        }
        const std::vector<Handle>& Handles() const {
            return bodies_.Handles();
        }
        
        private:
        SparseSet<BodyData> bodies_;

    };
} // namespace osseus

#endif // OSSEUSENGINE_BODYMANAGER_H
