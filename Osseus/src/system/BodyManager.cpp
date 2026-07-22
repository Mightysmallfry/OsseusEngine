//
// Created by MightySmallFry on 7/20/2026.
//

#include "../../include/Osseus/system/BodyManager.h"

namespace osseus {
    void BodyManager::AddBody(Handle handle, BodyData bodyData) {
        bodies.Insert(handle, bodyData);
    }

    void BodyManager::RemoveBody(Handle handle) {
        bodies.Remove(handle);
    }

    BodyData* BodyManager::GetBody(Handle handle) {
        return bodies.Get(handle);
    }

    const BodyData* BodyManager::GetBody(Handle handle) const {
        return bodies.Get(handle);
    }
} // osseus