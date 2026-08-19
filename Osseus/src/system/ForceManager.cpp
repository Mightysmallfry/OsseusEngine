#ifndef OSSEUSENGINE_FORCEMANAGER_CPP
#define OSSEUSENGINE_FORCEMANAGER_CPP

#include <algorithm>

#include "Osseus/system/ForceManager.h"



namespace osseus {

    void ForceManager::Add(Handle handle, const Vector3& force){
        forces_[handle.index] += force;
    }

    const Vector3& ForceManager::Get(Handle handle) const {
        return forces_[handle.index];
    }

    void ForceManager::Resize(std::size_t count) {
        forces_.resize(count);
    }

    void ForceManager::Clear() {
        std::fill(forces_.begin(), forces_.end(), Vector3::Zero());  
    }

}

#endif